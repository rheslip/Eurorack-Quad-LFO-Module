Quad LFO code for "Ardcore like" Eurorack module

R Heslip Oct 2018

A simple module that generates four LFO outputs from approx 0.003Hz to 30Hz. Five waveforms are available - ramp, sine, "mirrored" exponential, exponential, random, and pulse. Two pots per channel
 are used to set the rise time and fall time of the waveform which allows a lot of waveshape variation. A "shift" button allows waveform selection and adjusting the output level. Four LEDs visually indicate the output signal. I used
  white LEDs which show even very low output levels very well. I used 10K series resistors for the white LEDs - they are very efficient.
 
 The hardware consists of a 16Mhz 5V Arduino pro mini with 8 pots, 4 LEDs and one button.
 The code uses 1khz interrupt driven MCP4822 DAC and a DDS algorithm to generate up and down ramps.
 The other waveforms are derived from the linear ramps via lookup tables.
 MCP4822 DAC outputs are RC lowpass filtered to about 60hz to minimize aliasing in the output. A quad opamp should be used to buffer the DAC outputs and can also amplify the 4.096V maximum output if higher levels are needed.

Pots are connected across the 5V and ground. Wipers are connected to analog inputs as follows:

A0 = up ramp 1, + shift= waveform select 1

A1 = down ramp 1, + shift= output level 1

A2 = up ramp 2 , + shift= waveform select 2

A3 = down ramp 2, + shift= output level 2

A4 = up ramp 3, + shift= waveform select 3

A5 = down ramp 3 , + shift= output level 3

A6 = up ramp 4, + shift= waveform select 4

A7 = down ramp 4, + shift= output level 4

D5,D6,D9,D10 - pwm led drivers (use an appropriate series resistor)

D13 - SCLK to MCP4822's

D11 - MOSI to MCP4822's

D4 - CS MCP4822 DAC 1

D8 - CS MCP4822 DAC 2

D2 - "shift" button input used as alternate pot function select. Holding the shift button for 5 seconds saves settings in eeprom

D0,D1 = serial I/O, unused other than for debug

D7 = unused


Oct 11/18 - initial version based on my Ardcore dual LFO code

Oct 12/18 - new version that implements "shift" pot functions for waveform selection and output level adjustment

Oct 13/18 - added wavetables from Mutable Instruments Peaks and table lookup/interpolation for exponential and quartic waveforms

Dec 4/18 - changed sinewave to use LUT vs trig function call


