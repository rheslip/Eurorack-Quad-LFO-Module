/*

Quad LFO Oct 2018
uses 1khz interrupt driven MCP4822 DAC and a DDS algorithm to generate up and down ramps
waveforms are all derived from the linear ramps
MCP4822 DAC outputs are RC lowpass filtered to about 60hz to minimize aliasing in the output
LFO range is about 30hz - 5 minutes per cycle. max time can be made MUCH longer by adjusting MIN_DELTA lower

Oct 11/18 - initial version based on my Ardcore dual LFO code
Oct 12/18 - new version that implements "shift" pot functions for waveform selection and output level adjustment
Oct 13/18 - added wavetables from Mutable Instruments Peaks and table lookup/interpolation for exponential and quartic waveforms
Dec 4/18 - changed sinewave to use LUT vs trig function call

0-5V pot inputs are connected to analog inputs as follows:
A0 = up ramp 1, + shift= waveform select 1
A1 = down ramp 1, + shift= output level 1
A2 = up ramp 2 , + shift= waveform select 2
A3 = down ramp 2, + shift= output level 2
A4 = up ramp 3, + shift= waveform select 3
A5 = down ramp 3 , + shift= output level 3
A6 = up ramp 4, + shift= waveform select 4
A7 = down ramp 4, + shift= output level 4

D5,D6,D9,D10 - pwm leds
D13 - SCLK
D11 - MOSI
D4 - CS MCP4822 DAC 1
D8 - CS MCP4822 DAC 2
D2 - "shift" button ie alternate pot function select. pressing the shift button for 5 seconds saves settings in eeprom
D0,D1 = serial I/O, unused other than for debug
D7 = unused

*/

#include "SPI.h"
#include <EEPROM.h>
#include "lut.h"

//Setup pin variables
const byte dac1CS = 4;
const byte dac2CS = 8;
const byte LED1 = 5; // led 1 - shows output level
const byte LED2 = 6; // led 2
const byte LED3 = 9; // led 3
const byte LED4 = 10; // led 4
const byte BUTTON1 = 2;   // UI button active low
const byte BUTTON2 = 4;   // UI button active low
const byte LFO1A = A0;  // pots for LFO 1
const byte LFO1B = A1;
const byte LFO2A = A2;  // pots for LFO 2
const byte LFO2B = A3;
const byte LFO3A = A4;  // pots for LFO 3
const byte LFO3B = A5;
const byte LFO4A = A6;  // pots for LFO 4
const byte LFO4B = A7;

// maximum/minimum increments for the DDS which set the fastest/slowest LFO rates
//#define MAX_DELTA 2047 // max DDS ramp increment 
//#define RANGE 162  // 1024/log(MAX_DELTA) 8khz sampling
#define RANGE 142  // 1024/log(MAX_DELTA) for 1khz sampling. gives max LFO rate about 30hz
#define MIN_DELTA 1000 // minimum value for DDS adder - avoids rediculously slow ramps

#define RAMP 0 // wave shape indexes
#define SINE 1
#define QUARTIC 2
#define EXPO 3  // 
#define RANDOM1 4
#define PULSE 5
#define WDIV 190 // divide pot reading by this to get wave shape index

struct lfodata {
  byte wave; // waveform
  long rate1; // rate for first section of waveform
  long rate2; // rate for second section of waveform
  unsigned amplitude; // output level 0-1024
  long acc; // bottom 20 are accumulator for DDS algorithm, top 12 used for waveform generation
  bool phase; // flags first or second section of waveform
  unsigned int dacout; // current DAC value
  unsigned int scaledout; // scaled DAC output
  }lfo[4];

// a/d values from pots
// pots are used for two or more parameters so we don't change the values till
// there is a significant movement of the pots when the pots are "locked"
// this prevents a waveform or level change ("shift" parameters) from changing the ramp times when the shift button is released

#define MIN_POT_CHANGE 25 // pot reading must change by this in order to register
#define POT_AVERAGING 5  // A/D average over this many readings
unsigned param[8]; // pot settings for normal parameters
unsigned shiftparam[8]; // pot settings for "shift" parameters
unsigned pot[8]; // pot A/D readings
bool potlock[8]; // when pots are locked it means they must change by MIN_POT_CHANGE to register

#define DEBOUNCE 20 // button debounce time in ms
#define SAVEPRESS 5000 // long press time is ms to save state in EEPROM
long buttontimer=0; // UI button debounce
bool shift=0; // indicates UI button pressed


// flag all pot values as locked ie they have to change more than MIN_POT_CHANGE to register

void lockpots(void) {
  for (int i=0; i<8;++i) potlock[i]=1;
}


void setup() {
  Serial.begin(9600);
  //Start DAC Comms
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
//  SPI.usingInterrupt(TIMER2_COMPA_vect); // tells SPI routines to disable interrupts during SPI transactions inside the ISR

  //Configure Pins
  pinMode(dac1CS, OUTPUT); //DAC CS
  digitalWrite(dac1CS, HIGH);
  pinMode(dac2CS, OUTPUT); //DAC CS
  digitalWrite(dac2CS, HIGH);
  pinMode(LED1, OUTPUT); // waveform 1
  pinMode(LED2, OUTPUT); // waveform 2
  pinMode(LED3, OUTPUT); // waveform 3
  pinMode(LED4, OUTPUT); // waveform 4
  pinMode(BUTTON1, INPUT_PULLUP); //

  // initialize data structures
  // read LFO parameters from EEPROM
  int eeAddress = 0; //EEPROM address to start reading from

  for (int i=0; i<4;++i) {
//    lfo[i].wave=RAMP;
    EEPROM.get(eeAddress++, lfo[i].wave); // byte variable
    lfo[i].rate1=0x0fffffff;  // these will get overwritten by pot settings anyway
    lfo[i].rate2=0x0fffffff;
//    lfo[i].amplitude=1023;
    EEPROM.get(eeAddress, lfo[i].amplitude);
    lfo[i].amplitude&=0x3ff; // in case there are bad values in eeprom
    eeAddress+=2;  // int variable
  }
  for (int i=0; i<8;++i) {
    pot[i]=0;
    param[i]=0;
    shiftparam[i]=0;
    potlock[i]=0;
  }
   
  cli();//stop interrupts
//set timer2 interrupt at 1kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 249;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 64 prescaler
  TCCR2B |= (1 << CS22);   

  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);


  sei();//allow interrupts
}

// SPI DAC code runs at 1khz. ie about 1ms execution time maximum
// this is not optimized but it does work
// if you need to use other SPI devices you need to use the SPI.usinginterrupt stuff and probably SPI transactions
// this code can be shortened up a lot by ditching digitalwrite, write the SPI registers directly
// I can see on the scope there is some jitter on the timing. could be the arduino 1ms interrupt
// basic waveform is a ramp - up ramp time set by one pot, down ramp time by the second
// ramp implemented with DDS algorithm. 24 bit accumulator - top 12 bits are the ramp and low 16 are the fractional increment
// other waveforms are derived from the ramp values (0-4097)

ISR(TIMER2_COMPA_vect){//timer1 interrupt 1kHz 

  bool phasechange; // indicates ramp has changed direction
  unsigned a,x,y,delta,out; 
  unsigned char i;
  
  for (i=0; i<4; ++i) {
    phasechange=0;  
    
    if (lfo[i].phase == 0) lfo[i].acc+=lfo[i].rate1; // ramp up for first part
        else lfo[i].acc-=lfo[i].rate2; // ramp down for second part

    if (lfo[i].acc >= 0x0fffffff) {  // test for accumulator overflow
      lfo[i].phase = 1 ;// ramp down once we hit the top
      lfo[i].acc=0x0fffffff; // set to max for next DAC output
      phasechange=1;
    }
    if (lfo[i].acc <=0) { // test for accumulator underflow
      lfo[i].phase = 0; //ramp up when we hit bottom
      lfo[i].acc=0; // set to 0 min for start of up ramp
      phasechange=1;
    }

    switch (lfo[i].wave) {
      case RAMP:
        lfo[i].dacout=lfo[i].acc >> 16; // get top 16 bits of accumulator. low 12 bits go to DAC
        break;    
      case PULSE: // variable duty cycle pulse
        if (lfo[i].phase) lfo[i].dacout=4095;
        else lfo[i].dacout=0;
        break;
      case RANDOM1:  // random value every time we hit max or min ramp
        if (phasechange) lfo[i].dacout=random(4096);
        break;
      // table lookup for exponential and quartic
      case EXPO:
        a=lfo[i].acc >> 16; // our 12 bit ramp value range 0-4095
        if (lfo[i].phase) a=4095-a; // reverse the lookup for 2nd half
        x=(a>>4)&0xff; // 256 word lookup table - it has 16 bit values
          // note that x+1 will index off the end of the table so I made the table 257 long
        delta=(lut_env_expo[x+1]-lut_env_expo[x])>>4; // interpolate between the table values - 16 steps
        y=lut_env_expo[x]; // lookup base value from table
        out=(y+(a&0xf)*delta)>>4;
        if (lfo[i].phase) out=4095-out; // invert the value for 2nd half
        lfo[i].dacout=out; // add interpolated value to base and scale to 12 bits
        break; 
      case QUARTIC:
        a=lfo[i].acc >> 16; // our 12 bit ramp value range 0-4095
        if (lfo[i].phase) a=4095-a; // reverse the lookup for 2nd half
        x=(a>>4)&0xff; // 256 word lookup table - it has 16 bit values
          // note that x+1 will index off the end of the table so I made the table 257 long
        delta=(lut_env_quartic[x+1]-lut_env_quartic[x])>>4; // interpolate between the table values - 16 steps
        y=lut_env_quartic[x]; // lookup base value from table
        out=(y+(a&0xf)*delta)>>4; // interpolated output = base value + steps*delta. scale the 16 bit table value to 12 bits
        if (lfo[i].phase) out=4095-out; // invert the value for 2nd half
        lfo[i].dacout=out; 
        break;         
      default:  // in case there are bogus values in EEPROM

      case SINE:
        a=lfo[i].acc >> 16; // our 12 bit ramp value range 0-4095
        if (lfo[i].phase) a=4095-a; // reverse the lookup for 2nd half
        x=(a>>4)&0xff; // 256 word lookup table - it has 16 bit values
          // note that x+1 will index off the end of the table so I made the table 257 long
        delta=(lut_raised_cosine[x+1]-lut_raised_cosine[x])>>4; // interpolate between the table values - 16 steps
        y=lut_raised_cosine[x]; // lookup base value from table
        out=(y+(a&0xf)*delta)>>4; // interpolated output = base value + steps*delta. scale the 16 bit table value to 12 bits
        if (lfo[i].phase) out=4095-out; // invert the value for 2nd half
        lfo[i].dacout=out;     
       // the trig version - pot phase is wrong with this code 
       // double rads=PI/2+PI*(double(lfo[i].acc)/double(0xfffffff)); // convert accumulator value to radians
       // PI/2 shift above gives us a tilted sinewave when rate1 and rate2 are different
      //  lfo[i].dacout=2048+2047*(sin(rads)); // scale and offset the DAC output
        break;
    }

  // scale the output
    long scaledout=((long)lfo[i].dacout*(long)lfo[i].amplitude)/1024; // use long int math here or it will overflow
    lfo[i].scaledout=(unsigned) scaledout; // save it for LED display

 // send the output value to the appropriate DAC
    byte data = scaledout >> 8;
    data = data & B00001111; // mask off dac data
    data = data | B00010000; // DAC A, 4.096v mode
    if (i&1) data = data | B10000000; // select DAC channel B
    
    if (i <2) digitalWrite(dac1CS, LOW); 
    else   digitalWrite(dac2CS, LOW); 
    
    SPI.transfer(data);  // first byte is control and top 4 bits of DAC output
    SPI.transfer((byte)scaledout);  // low 8 bits of DAC output   
    if (i <2) digitalWrite(dac1CS, HIGH); 
    else   digitalWrite(dac2CS, HIGH); 
  }
}

// in the main loop we sample the pot values, check the "shift" button, update the LFO parameters and update the LEDs

void loop() {
  if (digitalRead(BUTTON1)==0) { // button is pressed
    if (((millis() - buttontimer) > DEBOUNCE) && (shift==0)) {
      shift=1; // has been pressed long enough for debounce so flag button as active
      lockpots(); // pot values are locked till a significant change is made
    }
    if ((millis() - buttontimer) > SAVEPRESS) { // if button has been pressed for a long time we save the parameters to EEPROM
 //     Serial.println("long press"); // we have a long button press
      bool locked=1;
      for (int k=0; k<8;++k) if (potlock[k] !=1) locked =0; // make sure pots have not been moved. this avoids saving when user is setting waveform or output level
      if (locked) {
         int eeAddress = 0; //EEPROM address to start writing at
         for (int m=0; m<4;++m) {
            EEPROM.put(eeAddress++, lfo[m].wave); // byte variable
            EEPROM.put(eeAddress, lfo[m].amplitude);
            eeAddress+=2;  // int variable
         }

         analogWrite(LED1,0); // flash the LEDs to show state save
         analogWrite(LED2,0);
         analogWrite(LED3,0);
         analogWrite(LED4,0);
         delay(1000); 
         analogWrite(LED1,0xff); // flash the LEDs to show state save
         analogWrite(LED2,0xff);
         analogWrite(LED3,0xff);
         analogWrite(LED4,0xff);
         delay(1000);  
         while(digitalRead(BUTTON1)==0); // loop till button is released so we don't keep writing the eeprom              
      }
    }
  }
  else { // button is not pressed
    if (shift) { // button was just released
      shift=0;
      lockpots(); // pot values are locked till a significant change is made
    }
    else buttontimer=millis(); // button not pressed so reset the button timer
  }
  
// sample the pots. potlock means apply hysteresis so we only change when the pot is moved significantly
  for (int i=0; i<8;++i) {
    int val=0;
    for (int j=0; j<POT_AVERAGING;++j) val+=analogRead(i); // read the A/D a few times and average for a more stable value
    val=val/POT_AVERAGING;
    if (potlock[i]) {
      int delta=pot[i]-val;  // this needs to be done outside of the abs() function - see arduino abs() docs
      if (abs(delta) > MIN_POT_CHANGE) {
        potlock[i]=0;   // flag pot no longer locked
        pot[i]=val; // save the new reading
      }
    }
    else pot[i]=val; // pot is unlocked so save the reading
  }

  if (shift) { // set alternate parameters - waveform and level
    for (int i=0; i<4;++i) {
      if (potlock[i*2]==0) lfo[i].wave=pot[i*2]/WDIV;
      if (potlock[i*2+1]==0) lfo[i].amplitude=pot[i*2+1];      
    }
  }
  else { // no shift button - set rate parameters
    double t;
    for (int i=0; i<4;++i) {
      if (potlock[i*2]==0) {
        t=double(1024-pot[i*2])/RANGE;
        lfo[i].rate1=(long)(pow(10,t))+MIN_DELTA; // exponential pot response for ramp rates for a very large time range
      }
      if (potlock[i*2+1]==0) {
        t=double(1024-pot[i*2+1])/RANGE;
        lfo[i].rate2=(long)(pow(10,t))+MIN_DELTA;     
      }
    }    
  }
// note - I used white LEDs on the prototype. they light with very low currents so they work even with the very small scaled output values
// for other LEDs some scaling may be needed or the LEDs will be off at lower output levels
  analogWrite(LED1,lfo[0].scaledout>>4); // show ramp state on leds
  analogWrite(LED2,lfo[1].scaledout>>4); 
  analogWrite(LED3,lfo[2].scaledout>>4); // show ramp state on leds
  analogWrite(LED4,lfo[3].scaledout>>4);

}







