/* Updated PulsePosition Library for Teensy 3.1
 * Copyright (c) 2014, Robert Collins http://www.rcollins.org
 *
 * Inspired by original PulsePosition library:
 * High resolution input and output of PPM encoded signals
 * http://www.pjrc.com/teensy/td_libs_PulsePosition.html
 * Copyright (c) 2014, Paul Stoffregen, paul@pjrc.com
 *
 * This library borrowed significant portions of the code from the original library.
 * What portions weren't borrowed were entirely rewritten.
 *
 * The new library will support multiple hardware (FTM)
 * timers.  This version of Pulse Position library is entirely table driven:
 * all of the support for input and output is defined by the PPMPins data
 * registration table.  Should future versions of Teensy be created that
 * add or delete other PPM-capable pins, then the only modifications necessary
 * will be the addition or deletion of table entries in PPMPins.
 *
 * Boiler Plate BS:
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*---------------------------------------------------------------------
 * Overview
 * PPM -- Pulse Position Modulation library.  Reads and writes PPM
 * 			stream data.  PPM signals are often used by remote control
 * 			transmitters and receivers to packetize multiple servo
 * 			control streams into a single packet of data.
 *
 * 			This library will simultaneously read and write up to
 * 			twelve PPM streams.  Only a single instance of the library
 * 			is required to support simultaneous PPM input and output
 * 			streams.
 *
 * 			The PPM library is driven by a 48 MHz clock.  Each 'tick'
 * 			is equivalent to 0.020833 uS (20.833 nano-seconds).
 *
 * 			More information about PPM may be found here:
 * 			http://www.omegaco.demon.co.uk/mectnpdf/mectn004.pdf
 *
 * Features:
 * 	- Single instance to support all in/out PPM streams
 *  - Read/Write up to 12 simultaneous input/output PPM streams
 * 	- Each in/out stream is polarity selectable
 * 	- 48 Mhz to 96 Mhz clock resolution
 *
 * HOWTO:
 * Usage: Sending and Receiving PPM Encoded Signals
 *
 * PPM myPPM;
 * Create a PPM object. Create only one instance of the PPM library.
 * The PPM library is designed to handle multiple simultaneous inputs and
 * outputs.  Therefore, create only one instance of the PPM library.
 *
 * Public Methods:  (Each described below)
 * 		PPM(void);
 *		void 	PPMInput(int polarity, int numIOs, ...);
 *		void 	PPMOutput(int polarity, int numIOs, ...);
 *		bool 	PPMRegisterInput(int polarity, int rxPin);
 *		bool 	PPMRegisterOutput(int polarity, uint8_t framePin, int txPin);
 *		int		dataAvl(int rxPin);
 *		float	dataRead(int rxPin, uint8_t channel);
 *		bool 	dataWrite(int txPin, uint8_t channel, float microseconds);
 * 		char	*getVersion(void);
 *		void 	PPMDebug(bool endis);
 *		friend 	void ftm0_isr(void);
 *		friend 	void ftm1_isr(void);
 *		friend 	void ftm2_isr(void);
 *
 * Public Method:  myPPM.PPMInput(POLARITY, numIOs, rxPin [, rxPin, rxPin, ...])
 * 		Valid input pins:  [3, 4, 5, 6, 9, 10, 20, 21, 22, 23, 25, 32]
 * 		Initializes a PPM input stream with one or more input pins.  Configures
 * 		each rxPin with appropriate polarity, timer channel, and interrupts.
 * 		Each rxPin will be ready to receive PPM pulses.  More than
 * 		one input pin may be specified.  PPMInput may also be called more than
 * 		once, should some pins have different polarity than others.
 * 		Example(s):
 * 			myPPM.PPMInput(RISING, 2, 4, 6); - Rising edge, two input pins [4, 6]
 * 			myPPM.PPMInput(FALLING, 1, 22); - Falling edge, one input pin [22]
 *
 * Public Method:   bool PPMRegisterInput(int polarity, int rxPin);
 * 		This is the "worker" function called by PPMInput().  For each pin in the
 * 		argument list of PPMInput, PPMRegisterInput is called.  This is the function
 * 		that actually sets up the appropriate polarity, timer channel, and interrupts.
 * 		Calling this method directly is allowed.
 * 		Example(s):
 * 			myPPM.PPMRegisterInput(RISING, 4); - Rising edge, input pin [4].
 * 			myPPM.PPMRegisterInput(FALLING, 9); - Falling edge, input pin [9].
 *
 * Public Method:  int myPPM.dataAvl(rxPin);
 * 		Returns the number of channels received, or -1 when no new data is available.
 * 		rxPin is a required argument to specify which input stream to read.
 * 		Example(s):
 *			num = myPPM.dataAvl(PPM_INPUT_10);
 *			if (num > 0) {
 * 			...
 * 			}
 *
 * Public Method:  float myPPM.dataRead(rxPin, channel);
 *		Returns channel data from the specific rxPin stream.
 *		rxPin is a required argument to specify which input stream to read.
 *		channel is a required argument to specify which channel of the rxPin
 *		input stream to return.
 *		The returned value is a float representing the number of microseconds
 *		between rising edges. The input "channel" ranges from 1 to the number of
 *		channels indicated by available(). Reading the last channel causes the
 *		data to be cleared (available will return zero until a new frame of PPM
 *		data arrives).
 * 		Example(s):
 *			num = myPPM.dataAvl(PPM_INPUT_10);
 *			if (num > 0) {
 *				for (i=1; i <= num; i++) {
 *					float val = myPPM.dataRead(PPM_INPUT_10, i);
 * 					Serial.println(val);
 * 				}
 * 			}
 *
 * Public Method:  myPPM.PPMOutput(POLARITY, numIOs, txPin [, txPin, txPin, ...])
 * 		Valid output pins:  [3, 4, 5, 6, 9, 10, 20, 21, 22, 23, 25, 32]
 * 		Initializes a PPM output stream with one or more output pins.  Configures
 * 		each txPin with appropriate polarity, timer channel, and interrupts.
 * 		Each txPin will be ready to send PPM pulses.  More than
 * 		one output pin may be specified.  PPMOutput may also be called more than
 * 		once, should some pins have different polarity than others.
 * 		Example(s):
 * 			myPPM.PPMOutput(RISING, 2, 4, 6); - Rising edge, two output pins [4, 6]
 * 			myPPM.PPMOutput(FALLING, 1, 22); - Falling edge, one output pin [22]
 *
 * Public Method:   bool PPMRegisterOutput(int polarity, int txPin);
 * 		This is the "worker" function called by PPMOutput().  For each pin in the
 * 		argument list of PPMOutput, PPMRegisterOutput is called.  This is the function
 * 		that actually sets up the appropriate polarity, timer channel, and interrupts.
 * 		Calling this method directly is allowed.
 * 		Example(s):
 * 			myPPM.PPMRegisterOutput(RISING, 4); - Rising edge, output pin [4].
 * 			myPPM.PPMRegisterOutput(FALLING, 9); - Falling edge, output pin [9].
 *
 * Public Method:  bool dataWrite(int txPin, uint8_t channel, float microseconds);
 *		This function populates the output stream of txPin array with channel
 * 		and timing data.  This function will overwrite any previous channel
 * 		data and replace with new data.  The next time an output frame is
 * 		generated, the data in the output channel array will be converted
 * 		to the appropriate timing register data to create a pulse wave of
 * 		the appropriate polarity and duration.
 * 		Example(s):
 *			float val = myPPM.dataRead(PPM_INPUT_03, i); - Reads from input stream
 *			myPPM.dataWrite(PPM_OUTPUT_32, (i % 16) + 1, val); - Writes it to a different output stream
 *
 * Public Method:  char *getVersion(void);
 * 		This function returns the PPM library version number string.
 * 		Example(s):
 * 			Serial.print("TeensyPPM Library Version Number:  ");
 *			Serial.println(myPPM.getVersion());
 *
 *  Public Method:  void PPMDebug(bool endis);
 * 		This function is provided to enable and disable debugging.
 * 		Currently no debugging exists in the shipping version of this
 * 		module.  The end user is responsible for writing their own
 * 		debugging code and may use this function to enable and disable
 * 		the function.
 *
 * Public Method:  friend void ftm0_isr(void);
 * Public Method:  friend void ftm1_isr(void);
 * Public Method:  friend void ftm2_isr(void);
 * 		Interrupt Service Routines for the FlexTimer Modules.
 * ------------------------------------------------------------------*/

#include "PPM.h"

// Timing parameters, in microseconds.
// The shortest time allowed between any 2 rising edges.  This should be at
// least double TX_PULSE_WIDTH.
#define TX_MINIMUM_SIGNAL   300.0

// The longest time allowed between any 2 rising edges for a normal signal.
#define TX_MAXIMUM_SIGNAL  2500.0

// The default signal to send if nothing has been written.
#define TX_DEFAULT_SIGNAL  1500.0

// When transmitting with a single pin, the minimum space signal that marks
// the end of a frame.  Single wire receivers recognize the end of a frame
// by looking for a gap longer than the maximum data size.  When viewing the
// waveform on an oscilloscope, set the trigger "holdoff" time to slightly
// less than TX_MINIMUM_SPACE, for the most reliable display.  This parameter
// is not used when transmitting with 2 pins.
#define TX_MINIMUM_SPACE   5000.0

// The minimum total frame size.  Some servo motors or other devices may not
// work with pulses the repeat more often than 50 Hz.  To allow transmission
// as fast as possible, set this to the same as TX_MINIMUM_SIGNAL.
#define TX_MINIMUM_FRAME  20000.0

// The length of all transmitted pulses.  This must be longer than the worst
// case interrupt latency, which depends on how long any other library may
// disable interrupts.  This must also be no more than half TX_MINIMUM_SIGNAL.
// Most libraries disable interrupts for no more than a few microseconds.
// The OneWire library is a notable exception, so this may need to be lengthened
// if a library that imposes unusual interrupt latency is in use.
#define TX_PULSE_WIDTH      100.0

// When receiving, any time between rising edges longer than this will be
// treated as the end-of-frame marker.
#define RX_MINIMUM_SPACE   3500.0

// convert from microseconds to I/O clock ticks
#define CLOCKS_PER_MICROSECOND ((double)F_BUS / 1000000.0)
#define TX_MINIMUM_SIGNAL_CLOCKS  (uint32_t)(TX_MINIMUM_SIGNAL * CLOCKS_PER_MICROSECOND)
#define TX_MAXIMUM_SIGNAL_CLOCKS  (uint32_t)(TX_MAXIMUM_SIGNAL * CLOCKS_PER_MICROSECOND)
#define TX_DEFAULT_SIGNAL_CLOCKS  (uint32_t)(TX_DEFAULT_SIGNAL * CLOCKS_PER_MICROSECOND)
#define TX_MINIMUM_SPACE_CLOCKS   (uint32_t)(TX_MINIMUM_SPACE * CLOCKS_PER_MICROSECOND)
#define TX_MINIMUM_FRAME_CLOCKS   (uint32_t)(TX_MINIMUM_FRAME * CLOCKS_PER_MICROSECOND)
#define TX_PULSE_WIDTH_CLOCKS     (uint32_t)(TX_PULSE_WIDTH * CLOCKS_PER_MICROSECOND)
#define RX_MINIMUM_SPACE_CLOCKS   (uint32_t)(RX_MINIMUM_SPACE * CLOCKS_PER_MICROSECOND)

/*---------------------------------------------------------------------
 * Class:		<Static Data>
 * Data:		Version
 *
 * Synopsis:    PPM Version Number
 *
 * Defined in: 	PPM.h
 *
 * ------------------------------------------------------------------*/
	char 	*Version	= PPM_VERSION;

/*---------------------------------------------------------------------
 * Class:		<Static Data>
 * Data:		PPMPins	: PPM PIN registration table
 *
 * Synopsis:    The following table contains all of the valid PPM
 * 				Input and Output capable pins.  The table contains a
 * 				pointer to its hardware registers, and localized data.
 * 				The localized data within this structure allows the
 * 				interrupt service routine to function without calling
 * 				a C++ compatible object.
 *
 * Defined in: 	PPM.h
 *
 * ------------------------------------------------------------------*/
PPMPinStruct PPMPins[] = {
	/*- Flex Timer Base Registers				Flex Timer Channel Registers					Pin#	Previous	Write	Pin		FTM			Available	Channel		Debug		*/
	/*- Flex Timer Base Registers				Flex Timer Channel Registers					Pin#	Value		Index	MUX		IRQ			Flag		Enabled		Enabled		*/
	{(struct FlexTimerBase_Struct *)&FTM0_SC,	(struct FlexTimerChannel_Struct *)&FTM0_C0SC,	22,		0,			255,	4,		IRQ_FTM0,	false,		false,		false},
	{(struct FlexTimerBase_Struct *)&FTM0_SC,	(struct FlexTimerChannel_Struct *)&FTM0_C1SC,	23,		0,			255,	4,		IRQ_FTM0,	false,		false,		false},
	{(struct FlexTimerBase_Struct *)&FTM0_SC,	(struct FlexTimerChannel_Struct *)&FTM0_C2SC,	9,		0,			255,	4,		IRQ_FTM0,	false,		false,		false},
	{(struct FlexTimerBase_Struct *)&FTM0_SC,	(struct FlexTimerChannel_Struct *)&FTM0_C3SC,	10,		0,			255,	4,		IRQ_FTM0,	false,		false,		false},
	{(struct FlexTimerBase_Struct *)&FTM0_SC,	(struct FlexTimerChannel_Struct *)&FTM0_C4SC,	6,		0,			255,	4,		IRQ_FTM0,	false,		false,		false},
	{(struct FlexTimerBase_Struct *)&FTM0_SC,	(struct FlexTimerChannel_Struct *)&FTM0_C5SC,	20,		0,			255,	4,		IRQ_FTM0,	false,		false,		false},
	{(struct FlexTimerBase_Struct *)&FTM0_SC,	(struct FlexTimerChannel_Struct *)&FTM0_C6SC,	21,		0,			255,	4,		IRQ_FTM0,	false,		false,		false},
	{(struct FlexTimerBase_Struct *)&FTM0_SC,	(struct FlexTimerChannel_Struct *)&FTM0_C7SC,	5,		0,			255,	4,		IRQ_FTM0,	false,		false,		false},
	{(struct FlexTimerBase_Struct *)&FTM1_SC,	(struct FlexTimerChannel_Struct *)&FTM1_C0SC,	3,		0,			255,	3,		IRQ_FTM1,	false,		false,		false},
	{(struct FlexTimerBase_Struct *)&FTM1_SC,	(struct FlexTimerChannel_Struct *)&FTM1_C1SC,	4,		0,			255,	3,		IRQ_FTM1,	false,		false,		false},
	{(struct FlexTimerBase_Struct *)&FTM2_SC,	(struct FlexTimerChannel_Struct *)&FTM2_C0SC,	32,		0,			255,	3,		IRQ_FTM2,	false,		false,		false},
	{(struct FlexTimerBase_Struct *)&FTM2_SC,	(struct FlexTimerChannel_Struct *)&FTM2_C1SC,	25,		0,			255,	3,		IRQ_FTM2,	false,		false,		false}
};

/*---------------------------------------------------------------------
 * Class:		<Static Data>
 * Data:		Static data.  Can be viewed from both PPM objects
 * 				and interrupt service routines.
 * ------------------------------------------------------------------*/
uint32_t	PPM::FTM0_OF_Cnt = 0;
uint32_t	PPM::FTM1_OF_Cnt = 0;
uint32_t	PPM::FTM2_OF_Cnt = 0;
bool		PPM::FTM0_OF_Inc = false;
bool		PPM::FTM1_OF_Inc = false;
bool		PPM::FTM2_OF_Inc = false;
bool		PPM::debugEnabled = true;	/* Reserved for USER debug	*/
PPM			*PPM::thisPtr;				/* Currently unused			*/

/*---------------------------------------------------------------------
 * Class:		PPM
 * Function:	PPM : Class constructor
 *
 * Syntax:		PPM myPPM;
 *
 * Synopsis:    Instances the PPM library.
 *
 * Arguments:
 * 		None
 *
 * Returns:
 * 		None
 * ------------------------------------------------------------------*/
PPM::PPM(void) {
}

/*---------------------------------------------------------------------
 * Class:		PPM
 * Function:	PPMInput : Initialize PPM Input
 *
 * Syntax:		PPMInput(polarity, numIOs, rxPin, rxPin, rxPin, ...)
 *
 * Valid input pins:  [3, 4, 5, 6, 9, 10, 20, 21, 22, 23, 25, 32]
 *
 * Synopsis:    Initializes PPM input.  This function accepts a variable
 * 				length of arguments that represent the number of PPM
 * 				input channels.
 *
 * Arguments:
 * 		polarity	Polarity of the PPM input stream (RISING, FALLING).
 * 		numIOs		Number of PPM Input channels.  This is a mandatory
 * 					input to tell the PPMInput function how many input
 * 					channels to receive.
 * 		...			Variable number of arguments, each representing a
 * 					differnt PPM input pin.
 *
 * Returns:
 * 		None
 * ------------------------------------------------------------------*/
void PPM::PPMInput(int polarity, int numIOs, ...) {
	int	x, rxPin; //, numIOs;
	bool res;
	va_list args;
	va_start(args,numIOs);
	for(x = 0; x < numIOs; x ++) {
		rxPin = va_arg(args, int);
		res = PPMRegisterInput(polarity, rxPin);
	}
	va_end(args);
	thisPtr = this;
}

/*---------------------------------------------------------------------
 * Class:		PPM
 * Function:	PPMOutput : Initialize PPM Output
 *
 * Syntax:		PPMOutput(polarity, numIOs, rxPin, rxPin, rxPin, ...)
 *
 * Valid output pins:  [3, 4, 5, 6, 9, 10, 20, 21, 22, 23, 25, 32]
 *
 * Synopsis:    Initializes PPM Output.  This function accepts a variable
 * 				length of arguments that represent the number of PPM
 * 				output channels.
 *
 * Arguments:
 * 		polarity	Polarity of the PPM output stream (RISING, FALLING).
 * 		numIOs		Number of PPM Output channels.  This is a mandatory
 * 					input to tell the PPMOutput function how many input
 * 					channels to receive.
 * 		...			Variable number of arguments, each representing a
 * 					differnt PPM Output pin.
 *
 * Returns:
 * 		None
 * ------------------------------------------------------------------*/
void PPM::PPMOutput(int polarity, int numIOs, ...) {
	int	x, txPin; //, numIOs;
	bool res;
	va_list args;
	va_start(args,numIOs);
	for(x = 0; x < numIOs; x ++) {
		txPin = va_arg(args, int);
		res = PPMRegisterOutput(polarity, FRAMEPIN_NULL, txPin);
	}
	va_end(args);
	thisPtr = this;
}

/*---------------------------------------------------------------------
 * Class:		PPM
 * Function:	PPMRegisterInput : Registers a PIN as a PPM Input device
 *
 * Syntax:		PPMRegisterInput(rxPin, polarity)
 *
 * Synopsis:    Registers a PIN as a PPM Input device:
 * 				1) Verifies the PIN in the PPMPins registration table
 * 				2) Programs the appropriate FTM hardware registers
 * 				3) Sets the PINMUX function to use Flex Timer Module (FTM)
 * 				4) Prioritizes and enables the FTM interrupt
 *
 * Arguments:
 * 		polarity	Polarity of the PPM input stream (RISING, FALLING).
 * 		rxPin		Teensy 3.1 compatible PIN assignment
 *
 * Returns:
 * 		true	: PPM input successfully registered
 * 		false	: Could not find the PIN number in the PPMPins
 * 					registration table
 * ------------------------------------------------------------------*/
bool PPM::PPMRegisterInput(int polarity, int rxPin) {
	int x;
	bool results = false;

	for(x = 0; x < sizeof(PPMPins) / sizeof(PPMPins[0]); x++) {
		if(rxPin == PPMPins[x].rtxPin) {
			PPMPins[x].enabled = true;		/* Channel enabled				*/
			/*---------------------------------------------------------------------
			 * Only program the FTM base registers if necessary.  This is a slight
			 * optimization.
			 * ------------------------------------------------------------------*/
			if(PPMPins[x].FTMBase->MOD != 0xFFFF || (PPMPins[x].FTMBase->SC & !FTM_SC_TOF_VAL(FTM_SC_TOF_INT)) != FTM_SC_REG_INIT) {
				PPMPins[x].FTMBase->SC = 0;			/* Clear interrupt, enable register writes, diesable clocks	*/
				PPMPins[x].FTMBase->CNT = 0;		/* Reset COUNT register										*/
				PPMPins[x].FTMBase->MOD = 0xFFFF;	/* Trigger overflow interrupts every 0xFFFF clocks	   		*/
			/*---------------------------------------------------------------------
			 * Set the FTM (Master) Status and Control as follows
			 * 		(See PPM.h for 'FTM_SC_REG_INIT definition)
			 * 	TOIE	: Enable overflow interrupt
			 * 	CLKS	: Use System Clock
			 * 	PS		: CLK / 1
			 * ------------------------------------------------------------------*/
				PPMPins[x].FTMBase->SC = FTM_SC_REG_INIT;
				PPMPins[x].FTMBase->MODE = 0;		/* Disable register writes									*/
			}
			/*---------------------------------------------------------------------
			 * Set the FTM Channel Status and Control as follows:
			 * 		(See PPM.h for 'FTM_SnSC_INPUT_REG_INIT definition)
			 * 	CHIE	: Interrupt Enable
			 * 	MSx		: Input mode
			 * 	ELSx	: Rising/Falling polarity
			 * 	DMA		: Disabled
			 * ------------------------------------------------------------------*/
			PPMPins[x].pinSet			= FTM_CnSC_INPUT_REG_INIT;
			PPMPins[x].FTMChannel->CnSC = PPMPins[x].pinSet;

			/*---------------------------------------------------------------------
			 * Set Pin Control Register as follows (Register: PORTx_PCRn):
			 * 	MUX		: Pin Mux Control, sets pin definition according to table
			 * 				in K20 Manual, Section 10.3.1,
			 * 				"K20 Signal Multiplexing and Pin Assignments"
			 * 	DSE		: Drive Strength Enable (HIGH driver strength)
			 * 	SRE		: Slew Rate Control (SLOW slew rate for digital output)
			 * ------------------------------------------------------------------*/
			*portConfigRegister(rxPin) = PORT_PCR_MUX(PPMPins[x].pinMux) | PORT_PCR_DSE | PORT_PCR_SRE;

			/*---------------------------------------------------------------------
			 * Set Nested Vector Interrupt Control
			 * ------------------------------------------------------------------*/
			NVIC_SET_PRIORITY(PPMPins[x].ftmIRQ, 32);
			NVIC_ENABLE_IRQ(PPMPins[x].ftmIRQ);

			/*---------------------------------------------------------------------
			 * Returns SUCCESS from function
			 * ------------------------------------------------------------------*/
			results = true;
		}
	}
	return results;
}

/*---------------------------------------------------------------------
 * Class:		PPM
 * Function:	PPMRegisterOutput : Registers a PIN as a PPM Output device
 *
 * Syntax:		PPMRegisterOutput(txPin, polarity)
 *
 * Synopsis:    Registers a PIN as a PPM Output device:
 * 				1) Verifies the PIN in the PPMPins registration table
 * 				2) Programs the appropriate FTM hardware registers
 * 				3) Sets the PINMUX function to use Flex Timer Module (FTM)
 * 				4) Prioritizes and enables the FTM interrupt
 *
 * Arguments:
 * 		txPin		Teensy 3.1 compatible PIN assignment
 * 		polarity	Polarity of the PPM input stream (RISING, FALLING).
 *
 * Returns:
 * 		true	: PPM Output successfully registered
 * 		false	: Could not find the PIN number in the PPMPins
 * 					registration table
 * ------------------------------------------------------------------*/
bool PPM::PPMRegisterOutput(int polarity, uint8_t framePin, int txPin) {
	int x, y;
	bool results = false;

	for(x = 0; x < sizeof(PPMPins) / sizeof(PPMPins[0]); x++) {
		if(txPin == PPMPins[x].rtxPin) {
			PPMPins[x].enabled = true;		/* Channel enabled				*/
			/*---------------------------------------------------------------------
			 * Initialize the output buffer
			 * ------------------------------------------------------------------*/
			PPMPins[x].plsWidth[0] = TX_MINIMUM_FRAME_CLOCKS;
			for (y = 1; y <= PULSEPOSITION_MAXCHANNELS; y++)
			{
				PPMPins[x].plsWidth[y] = TX_DEFAULT_SIGNAL_CLOCKS;
			}

			/*---------------------------------------------------------------------
			 * Configure the framePin.  Some configurations may want a 2-wire mode.
			 * In this case, a framePin is required.  A framePin is used to indicate
			 * when a new data frame begins.  Instead of inserting an extra delay,
			 * the framePin is used to begin a new frame.  Using a framePin is
			 * intended for connecting 1 or 2 74HCT164 chips.  The 74HCT164 is an
			 * 8-bit, digital in/parallel out shift register.  I have no idea what
			 * that means, but the specs for it may be found here:
			 * http://www.nxp.com/documents/data_sheet/74HC_HCT164.pdf
			 * --------------------------------------------------------------------
			 * To determine if this output stream will use a framePin, we called
			 * this function with the framePin value, or a dummy value to indicate
			 * that a framePin wasn't use.
			 * ------------------------------------------------------------------*/
			if (framePin < NUM_DIGITAL_PINS) {
				PPMPins[x].framePinReg = portOutputRegister(framePin);
				pinMode(framePin, OUTPUT);
				*PPMPins[x].framePinReg = 1;
			} else {
				PPMPins[x].framePinReg = NULL;
			}

			/*---------------------------------------------------------------------
			 * Initialize state variables
			 * ------------------------------------------------------------------*/
			PPMPins[x].state 		= OUTPUT_STATE_HIGH;
			PPMPins[x].wrIndx		= 0;
			PPMPins[x].nChannels	= 0;

			/*---------------------------------------------------------------------
			 * Only program the FTM base registers if necessary.  This is a slight
			 * optimization.
			 * ------------------------------------------------------------------*/
			if(PPMPins[x].FTMBase->MOD != 0xFFFF || (PPMPins[x].FTMBase->SC & !FTM_SC_TOF_VAL(FTM_SC_TOF_INT)) != FTM_SC_REG_INIT) {
				PPMPins[x].FTMBase->SC = 0;			/* Clear interrupt, enable register writes, diesable clocks	*/
				PPMPins[x].FTMBase->CNT = 0;		/* Reset COUNT register										*/
				PPMPins[x].FTMBase->MOD = 0xFFFF;	/* Trigger overflow interrupts every 0xFFFF clocks	   		*/

			/*---------------------------------------------------------------------
			 * Set the FTM (Master) Status and Control as follows
			 * 		(See PPM.h for 'FTM_SC_REG_INIT definition)
			 * 	TOIE	: Enable overflow interrupt
			 * 	CLKS	: Use System Clock
			 * 	PS		: CLK / 1
			 * ------------------------------------------------------------------*/
				PPMPins[x].FTMBase->SC = FTM_SC_REG_INIT;
				PPMPins[x].FTMBase->MODE = 0;		/* Disable register writes									*/
			}

			/*---------------------------------------------------------------------
			 * Set timer initial value
			 * ------------------------------------------------------------------*/
			PPMPins[x].FTMChannel->CnV	= OUTPUT_COUNTER_INIT;

			/*---------------------------------------------------------------------
			 * Set the FTM Channel Status and Control as follows:
			 * 		(See PPM.h for 'FTM_SnSC_OUTPUT_REG_INIT definition)
			 * 	CHIE	: Interrupt Enable
			 * 	MSx		: Output mode
			 * 	ELSx	: Rising/Falling polarity
			 * 	DMA		: Disabled
			 * ------------------------------------------------------------------*/
			PPMPins[x].pinSet			= FTM_CnSC_OUTPUT_REG_SET;
			PPMPins[x].pinClear			= FTM_CnSC_OUTPUT_REG_CLEAR;
			PPMPins[x].FTMChannel->CnSC = PPMPins[x].pinSet;

			/*---------------------------------------------------------------------
			 * Set Pin Control Register as follows (Register: PORTx_PCRn):
			 * 	MUX		: Pin Mux Control, sets pin definition according to table
			 * 				in K20 Manual, Section 10.3.1,
			 * 				"K20 Signal Multiplexing and Pin Assignments"
			 * 	DSE		: Drive Strength Enable (HIGH driver strength)
			 * 	SRE		: Slew Rate Control (SLOW slew rate for digital output)
			 * ------------------------------------------------------------------*/
			*portConfigRegister(txPin) = PORT_PCR_MUX(PPMPins[x].pinMux) | PORT_PCR_DSE | PORT_PCR_SRE;

			/*---------------------------------------------------------------------
			 * Set Nested Vector Interrupt Control
			 * ------------------------------------------------------------------*/
			NVIC_SET_PRIORITY(PPMPins[x].ftmIRQ, 32);
			NVIC_ENABLE_IRQ(PPMPins[x].ftmIRQ);

			/*---------------------------------------------------------------------
			 * Returns SUCCESS from function
			 * ------------------------------------------------------------------*/
			results = true;
		}
	}
	return results;
}

/*---------------------------------------------------------------------
 * Class:		PPM
 * Function:	dataAvl : Query function to determine if any PPM input
 * 						is available.
 *
 * Syntax:		dataAvl(rxPin)
 *
 * Synopsis:    Returns the number of channels to indicate whether
 * 				a PPM input stream is available for reading.
 *
 * Arguments:
 * 		rxPin	Teensy 3.1 compatible PIN assignment
 *
 * Returns:
 * 		<num>	Number of channels available to read.
 * 		-1		No data available.
 * ------------------------------------------------------------------*/
int PPM::dataAvl(int rxPin)
{
	int x;
	uint32_t total;
	bool flag;
	for(x = 0; x < sizeof(PPMPins) / sizeof(PPMPins[0]); x++) {
		if(PPMPins[x].rtxPin == rxPin) {
			__disable_irq();
			flag = PPMPins[x].avlFlag;
			total = PPMPins[x].nChannels;
			__enable_irq();
			if (flag) return total;
		}
	}
	return -1;
}

/*---------------------------------------------------------------------
 * Class:		PPM
 * Function:	dataRead : Read PPM input stream data by PIN/Channel.
 *
 * Syntax:		dataRead(rxPin, Channel#)
 *
 * Synopsis:    Returns an individual PPM input channel item.
 *
 * Arguments:
 * 		rxPin		Teensy 3.1 compatible PIN assignment
 * 		Channel#	PPM channel #
 *
 * Returns:
 * 		<data>	PPM Input channel data
 * 		-1		No data available.
 * ------------------------------------------------------------------*/
float PPM::dataRead(int rxPin, uint8_t channel)
{
	int x;
	uint32_t total, index, value=0;

	for(x = 0; x < sizeof(PPMPins) / sizeof(PPMPins[0]); x++) {
		if(PPMPins[x].rtxPin == rxPin) {
			if (channel == 0) return 0.0;
			index = channel - 1;
			__disable_irq();
			total = PPMPins[x].nChannels;
			if (index < total) value = PPMPins[x].plsBuffer[index];
			if (channel >= total) PPMPins[x].avlFlag = false;
			__enable_irq();
			return (float)value / (float)CLOCKS_PER_MICROSECOND;
		}
	}
	return -1.0;
}

/*---------------------------------------------------------------------
 * Class:		PPM
 * Function:	dataWrite : Write PPM output stream data by PIN/Channel.
 *
 * Syntax:		dataRead(txPin, Channel#, MicroSeconds)
 *
 * Synopsis:    Returns an individual PPM input channel item.
 *
 * Arguments:
 * 		txPin			Teensy 3.1 compatible PIN assignment
 * 		Channel#		PPM channel #
 * 		MicroSeconds	# Microseconds between rising/falling edges.
 *
 * Returns:
 * 		true			PPM Output Success
 * 		false			PPM Output Failure
 * ------------------------------------------------------------------*/
bool PPM::dataWrite(int txPin, uint8_t channel, float microseconds)
{
	int x;
	uint32_t i, sum, space, clocks, num_channels;

	for(x = 0; x < sizeof(PPMPins) / sizeof(PPMPins[0]); x++) {
		if(PPMPins[x].rtxPin == txPin) {
			if (channel < 1 || channel > PULSEPOSITION_MAXCHANNELS) return false;
			if (microseconds < TX_MINIMUM_SIGNAL || microseconds > TX_MAXIMUM_SIGNAL) return false;
			clocks = microseconds * CLOCKS_PER_MICROSECOND;
			num_channels = PPMPins[x].nChannels;
			if (channel > num_channels) num_channels = channel;
			sum = clocks;
			for (i=1; i < channel; i++) sum += PPMPins[x].plsWidth[i];
			for (i=channel+1; i <= num_channels; i++) sum += PPMPins[x].plsWidth[i];
			if (sum < TX_MINIMUM_FRAME_CLOCKS - TX_MINIMUM_SPACE_CLOCKS) {
				space = TX_MINIMUM_FRAME_CLOCKS - sum;
			} else {
				if (PPMPins[x].framePinReg) {
					space = TX_PULSE_WIDTH_CLOCKS;
				} else {
					space = TX_MINIMUM_SPACE_CLOCKS;
				}
			}
			__disable_irq();
			PPMPins[x].plsWidth[0] = space;
			PPMPins[x].plsWidth[channel] = clocks;
			PPMPins[x].nChannels = num_channels;
			__enable_irq();
			return true;
		}
	}
	return false;
}

/*---------------------------------------------------------------------
 * Class:		PPM
 * Function:	getVersion : Read PPM library version number
 *
 * Syntax:		char * getVersion()
 *
 * Synopsis:    Returns an individual PPM input channel item.
 *
 * Arguments:
 * 		rxPin		Teensy 3.1 compatible PIN assignment
 * 		Channel#	PPM channel #
 *
 * Returns:
 * 		<data>	PPM Input channel data
 * 		-1		No data available.
 * ------------------------------------------------------------------*/
char *PPM::getVersion(void) {
	return Version;
}


/*---------------------------------------------------------------------
 * Class:		<Owned by Hardware>
 * Function:	fmt0_isr : FlexTimer Module-0 Interrupt Service Routine
 *
 * Syntax:		<Called by hardware, no software entry points.>
 *
 * Synopsis:	Services FTM0 interrupts.
 *
 * Arguments:
 *		None
 *
 * Returns:
 * 		Nothing
 * ------------------------------------------------------------------*/
void ftm0_isr(void)
{
	int x;
	uint32_t val;

	if (FTM0_SC & FTM_SC_TOF_VAL(FTM_SC_TOF_INT)) {
		FTM0_SC ^= FTM_SC_TOF_VAL(FTM_SC_TOF_INT); 		/* Clear overflow interrupt		*/
		PPM::FTM0_OF_Cnt++;								/* Increment overflow counter	*/
		PPM::FTM0_OF_Inc = true;						/* Set overflow occurred flag	*/
	}

	for(x = 0; x < sizeof(PPMPins) / sizeof(PPMPins[0]); x++) {
		/*---------------------------------------------------------------------
		 * Service the interrupt IF all of the conditions are met:
		 * 	1. Looking for IRQ on FTM0
		 * 	2. This channel has been enabled
		 * 	3. Interrupt flag is really set
		 * ------------------------------------------------------------------*/
		val = PPMPins[x].FTMChannel->CnSC;
		if((PPMPins[x].ftmIRQ == IRQ_FTM0) && PPMPins[x].enabled && (val & FTM_CnSC_CHF_VAL(FTM_CnSC_CHF_INT))) {

			/*---------------------------------------------------------------------
			 * Check for input or output interrupt by looking at the channel mode
			 * 	select flags.
			 * ------------------------------------------------------------------*/
 			switch ( val & FTM_CnSC_MSx_VAL(FTM_CnSC_MSx_MASK))
			{
				case FTM_CnSC_MSx_VAL(FTM_CnSC_MSx_INPUT) :
					FTMx_INPUT_ISR(x, &PPM::FTM0_OF_Cnt, &PPM::FTM0_OF_Inc);
					break;

				case FTM_CnSC_MSx_VAL(FTM_CnSC_MSx_OUTPUT) :
					FTMx_OUTPUT_ISR(x);
					break;

			   default:
					break;
			}
		}
	}
	PPM::FTM0_OF_Inc = false;
}


/*---------------------------------------------------------------------
 * Class:		<Owned by Hardware>
 * Function:	fmt1_isr : FlexTimer Module-1 Interrupt Service Routine
 *
 * Syntax:		<Called by hardware, no software entry points.>
 *
 * Synopsis:	Services FTM1 interrupts.
 *
 * Arguments:
 *		None
 *
 * Returns:
 * 		Nothing
 * ------------------------------------------------------------------*/
void ftm1_isr(void)
{
	int x;
	uint32_t val, count;

	if (FTM1_SC & FTM_SC_TOF_VAL(FTM_SC_TOF_INT)) {
		FTM1_SC ^= FTM_SC_TOF_VAL(FTM_SC_TOF_INT); 		/* Clear overflow interrupt		*/
		PPM::FTM1_OF_Cnt++;								/* Increment overflow counter	*/
		PPM::FTM1_OF_Inc = true;						/* Set overflow occurred flag	*/
	}

	for(x = 0; x < sizeof(PPMPins) / sizeof(PPMPins[0]); x++) {
		/*---------------------------------------------------------------------
		 * Service the interrupt IF all of the conditions are met:
		 * 	1. Looking for IRQ on FTM1
		 * 	2. This channel has been enabled
		 * 	3. Interrupt flag is really set
		 * ------------------------------------------------------------------*/
		val = PPMPins[x].FTMChannel->CnSC;
		if((PPMPins[x].ftmIRQ == IRQ_FTM1) && PPMPins[x].enabled && (val & FTM_CnSC_CHF_VAL(FTM_CnSC_CHF_INT))) {

			/*---------------------------------------------------------------------
			 * Check for input or output interrupt by looking at the channel mode
			 * 	select flags.
			 * ------------------------------------------------------------------*/
 			switch ( val & FTM_CnSC_MSx_VAL(FTM_CnSC_MSx_MASK))
			{
				case FTM_CnSC_MSx_VAL(FTM_CnSC_MSx_INPUT) :
					FTMx_INPUT_ISR(x, &PPM::FTM1_OF_Cnt, &PPM::FTM1_OF_Inc);
					break;

				case FTM_CnSC_MSx_VAL(FTM_CnSC_MSx_OUTPUT) :
					FTMx_OUTPUT_ISR(x);
					break;

			   default:
					break;
			}
		}
	}
	PPM::FTM1_OF_Inc = false;
}

/*---------------------------------------------------------------------
 * Class:		<Owned by Hardware>
 * Function:	fmt2_isr : FlexTimer Module-2 Interrupt Service Routine
 *
 * Syntax:		<Called by hardware, no software entry points.>
 *
 * Synopsis:	Services FTM2 interrupts.
 *
 * Arguments:
 *		None
 *
 * Returns:
 * 		Nothing
 * ------------------------------------------------------------------*/
void ftm2_isr(void)
{
	int x;
	uint32_t val, count;

	if (FTM2_SC & FTM_SC_TOF_VAL(FTM_SC_TOF_INT)) {
		FTM2_SC ^= FTM_SC_TOF_VAL(FTM_SC_TOF_INT); 		/* Clear overflow interrupt		*/
		PPM::FTM2_OF_Cnt++;								/* Increment overflow counter	*/
		PPM::FTM2_OF_Inc = true;						/* Set overflow occurred flag	*/
	}

	for(x = 0; x < sizeof(PPMPins) / sizeof(PPMPins[0]); x++) {
		/*---------------------------------------------------------------------
		 * Service the interrupt IF all of the conditions are met:
		 * 	1. Looking for IRQ on FTM2
		 * 	2. This channel has been enabled
		 * 	3. Interrupt flag is really set
		 * ------------------------------------------------------------------*/
		val = PPMPins[x].FTMChannel->CnSC;
		if((PPMPins[x].ftmIRQ == IRQ_FTM2) && PPMPins[x].enabled && (val & FTM_CnSC_CHF_VAL(FTM_CnSC_CHF_INT))) {

			/*---------------------------------------------------------------------
			 * Check for input or output interrupt by looking at the channel mode
			 * 	select flags.
			 * ------------------------------------------------------------------*/
 			switch ( val & FTM_CnSC_MSx_VAL(FTM_CnSC_MSx_MASK))
			{
				case FTM_CnSC_MSx_VAL(FTM_CnSC_MSx_INPUT) :
					FTMx_INPUT_ISR(x, &PPM::FTM2_OF_Cnt, &PPM::FTM2_OF_Inc);
					break;

				case FTM_CnSC_MSx_VAL(FTM_CnSC_MSx_OUTPUT) :
					FTMx_OUTPUT_ISR(x);
					break;

			   default:
					break;
			}
		}
	}
	PPM::FTM2_OF_Inc = false;
}

/*---------------------------------------------------------------------
 * Class:		<Owned by Hardware>
 * Function:	FTMx_INPUT_ISR	: Common PPM input ISR.
 *
 * Syntax:		FTMx_INPUT_ISR( x, *FTMx_OF_Cnt, *FTMx_OF_Inc );
 *
 * Synopsis:	This is called  by all of the various FTMx input ISR
 * 				routines.  Each ISR passes the variables and pointers
 * 				necessary to service the interrupt in this common
 * 				subroutine.

 *
 * Arguments:
 *		x				:	Index into PPMPins registration table.
 * 		*FTMx_OF_Cnt	:	Pointer to FTMx specific overflow counter.
 * 		*FTMx_OF_Inc	:	Pointer to FTMx specific overflow flag.
 *
 * Returns:
 * 		Nothing
 * ------------------------------------------------------------------*/
void FTMx_INPUT_ISR(int x, uint32_t *FTMx_OF_Cnt, bool *FTMx_OF_Inc) {
	uint32_t val, count;
	val = PPMPins[x].FTMChannel->CnV;					/* Get current count	*/
	PPMPins[x].FTMChannel->CnSC = PPMPins[x].pinSet;	/* I don't think this is needed.  Here to mimic original PulsePosition.cpp library	*/
	count = *FTMx_OF_Cnt;								/* Capture running count of overflow interrupts	*/

	/*---------------------------------------------------------------------
	 * This needs a little explaining.
	 *
	 * The ISR was programmed to handle two sources of interrupt:
	 *	1) TOIE, and
	 * 	2) Rising/Falling edges.
	 *
	 * TOIE runs independent and doesn't stop when the Rising/Falling
	 * interrupt occurs. However, when RISING/FALLING interrupt occurs,
	 * the counter value is immediately latched on the FTMx_CnV register.
	 * Even though count was latched, TOIE is still running. The line of
	 * code below is designed to prevent a race condition. It handles
	 * the case where TOIE interrupt occurs after the count was latched
	 * AND before you get a chance to service the RISING/FALLING interrupt.
	 * ------------------------------------------------------------------*/
	if (val > 0xE000 && *FTMx_OF_Inc) count--;
	val |= (count << 16);
	count = val - PPMPins[x].pVal;
	PPMPins[x].pVal = val;
	if (count >= RX_MINIMUM_SPACE_CLOCKS) {
		if (PPMPins[x].wrIndx < 255) {
			for (int i=0; i < PPMPins[x].wrIndx; i++) {
				PPMPins[x].plsBuffer[i] = PPMPins[x].plsWidth[i];
			}
			PPMPins[x].nChannels = PPMPins[x].wrIndx;
			PPMPins[x].avlFlag = true;
		}
		PPMPins[x].wrIndx = 0;
	} else {
		if (PPMPins[x].wrIndx < PULSEPOSITION_MAXCHANNELS) {
			PPMPins[x].plsWidth[PPMPins[x].wrIndx++] = count;
		}
	}
}

/*---------------------------------------------------------------------
 * Class:		<Owned by Hardware>
 * Function:	FTMx_OUTPUT_ISR	: Common PPM output ISR.
 *
 * Syntax:		FTMx_OUTPUT_ISR( x );
 *
 * Synopsis:	This is called  by all of the various FTMx output ISR
 * 				routines.  Each ISR passes the variables and pointers
 * 				necessary to service the interrupt in this common
 * 				subroutine.

 *
 * Arguments:
 *		x				:	Index into PPMPins registration table.
 *
 * Returns:
 * 		Nothing
 * ------------------------------------------------------------------*/
void FTMx_OUTPUT_ISR(int x) {
	PPMPins[x].FTMBase->MODE = 0;
	if (PPMPins[x].state == OUTPUT_STATE_HIGH) {
		// pin was just set high, schedule it to go low
		PPMPins[x].FTMChannel->CnV 	+= 	TX_PULSE_WIDTH_CLOCKS;
		PPMPins[x].FTMChannel->CnSC	=	PPMPins[x].pinClear;
		PPMPins[x].state 			= 	OUTPUT_STATE_LOW;
	} else {
		// pin just went low
		uint32_t width, channel;
		if (PPMPins[x].state == OUTPUT_STATE_LOW) {
			channel = PPMPins[x].wrIndx;
			if (channel == 0) {
				PPMPins[x].nChannelsBuffer = PPMPins[x].nChannels;
				for (uint32_t i=0; i <= PPMPins[x].nChannelsBuffer; i++) {
					PPMPins[x].plsBuffer[i] = PPMPins[x].plsWidth[i];
				}
			}
			width = PPMPins[x].plsBuffer[channel] - TX_PULSE_WIDTH_CLOCKS;
			if (++channel > PPMPins[x].nChannelsBuffer) {
				channel = 0;
			}
			if (PPMPins[x].framePinReg) {
				if (channel == 1) {
					*PPMPins[x].framePinReg = 1;
				} else {
					*PPMPins[x].framePinReg = 0;
				}
			}
			PPMPins[x].wrIndx = channel;
		} else {
			width = PPMPins[x].plsRemaining;
		}
		if (width <= 60000) {
			PPMPins[x].FTMChannel->CnV += width;
			PPMPins[x].FTMChannel->CnSC = PPMPins[x].pinSet;
			PPMPins[x].state = OUTPUT_STATE_HIGH;
		} else {
			PPMPins[x].FTMChannel->CnV += 58000;
			PPMPins[x].FTMChannel->CnSC	=	PPMPins[x].pinClear;
			PPMPins[x].plsRemaining = width - 58000;
			PPMPins[x].state = OUTPUT_STATE_RESIDUAL;
		}
	}
}
