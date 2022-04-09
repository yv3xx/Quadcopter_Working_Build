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

#include <Arduino.h>
#include <FlexTimer.h>

#define	PPM_VERSION					"1.0.0"

#define PULSEPOSITION_MAXCHANNELS	16
#define	OUTPUT_STATE_HIGH			0
#define OUTPUT_STATE_LOW			1
#define	OUTPUT_STATE_RESIDUAL		2

#define	FRAMEPIN_NULL				255
#define	OUTPUT_COUNTER_INIT			200

#define FTM_SC_REG_INIT					FTM_SC_TOIE_VAL(FTM_SC_TOIE_IE)			| FTM_SC_CLKS_VAL(FTM_SC_CLKS_SYSTEM)	| FTM_SC_PS_VAL(FTM_SC_PS_1)
#define	FTM_CnSC_INPUT_REG_INIT			FTM_CnSC_CHIE_VAL(FTM_CnSC_CHIE_ENABLE) | FTM_CnSC_MSx_VAL(FTM_CnSC_MSx_INPUT)	| ((polarity == FALLING)	? FTM_CnSC_ELSx_VAL(FTM_CnSC_ELSx_INPUT_FALLING)	: FTM_CnSC_ELSx_VAL(FTM_CnSC_ELSx_INPUT_RISING))	| FTM_CnSC_DMA_VAL(FTM_DMA_DISA)
#define FTM_CnSC_OUTPUT_REG_INIT        FTM_CnSC_CHIE_VAL(FTM_CnSC_CHIE_ENABLE) | FTM_CnSC_MSx_VAL(FTM_CnSC_MSx_OUTPUT)	| ((polarity == FALLING)	? FTM_CnSC_ELSx_VAL(FTM_CnSC_ELSx_OUTPUT_CLEAR)		: FTM_CnSC_ELSx_VAL(FTM_CnSC_ELSx_OUTPUT_SET))		| FTM_CnSC_DMA_VAL(FTM_DMA_DISA)
#define FTM_CnSC_OUTPUT_REG_SET			FTM_CnSC_CHIE_VAL(FTM_CnSC_CHIE_ENABLE) | FTM_CnSC_MSx_VAL(FTM_CnSC_MSx_OUTPUT)	| ((polarity == FALLING)	? FTM_CnSC_ELSx_VAL(FTM_CnSC_ELSx_OUTPUT_CLEAR)		: FTM_CnSC_ELSx_VAL(FTM_CnSC_ELSx_OUTPUT_SET))		| FTM_CnSC_DMA_VAL(FTM_DMA_DISA)
#define FTM_CnSC_OUTPUT_REG_CLEAR		FTM_CnSC_CHIE_VAL(FTM_CnSC_CHIE_ENABLE) | FTM_CnSC_MSx_VAL(FTM_CnSC_MSx_OUTPUT)	| ((polarity == FALLING)	? FTM_CnSC_ELSx_VAL(FTM_CnSC_ELSx_OUTPUT_SET)		: FTM_CnSC_ELSx_VAL(FTM_CnSC_ELSx_OUTPUT_CLEAR))	| FTM_CnSC_DMA_VAL(FTM_DMA_DISA)

struct PPMPinStruct
{
	FlexTimerBase_Struct	*FTMBase;
	FlexTimerChannel_Struct	*FTMChannel;
	int					rtxPin;
	uint32_t			pVal;				/* Previous value		*/
	uint8_t				wrIndx;				/* IN:write_index, OUT:current_channel		*/
	uint8_t				pinMux;				/* PinMux configuration	*/
	uint8_t				ftmIRQ;				/* FlexTimer IRQ Enable	*/
	bool				avlFlag;			/* Available flag		*/
	bool				enabled;			/* Channel enabled		*/
	bool				debug;				/* Debug Enabled		*/
	uint8_t				state;				/* Output State Flag	*/
	uint8_t				nChannels;			/* Total number of channels	*/
	uint8_t				nChannelsBuffer;	/* 'total_channels_buffer'	*/
	volatile uint8_t 	*framePinReg;		/* OUT: Frame Pin Register	*/
	uint32_t			pinSet;				/* Value programmed in FTMx_CnSC register to SET	*/
	uint32_t			pinClear;			/* Value programmed in FTMx_CnSC register to CLEAR	*/
	uint32_t			plsRemaining;		/* OUT: Pulse width remaining	*/
	uint32_t 			plsWidth[PULSEPOSITION_MAXCHANNELS+1];
	uint32_t 			plsBuffer[PULSEPOSITION_MAXCHANNELS+1];
};

void FTMx_INPUT_ISR(int x, uint32_t *FMTx_OF_Cnt, bool *FMTx_OF_Inc);
void FTMx_OUTPUT_ISR(int x);

class PPM
{
	public:
		PPM(void);
		void PPMInput(int polarity, int numIOs, ...);
		bool PPMRegisterInput(int polarity, int rxPin);
		void PPMOutput(int polarity, int numIOs, ...);
		bool PPMRegisterOutput(int polarity, uint8_t framePin, int txPin);
		bool dataWrite(int txPin, uint8_t channel, float microseconds);
		void PPMDebug(bool endis);
		int		dataAvl(int rxPin);
		float	dataRead(int rxPin, uint8_t channel);
		void ShowFTM(FlexTimerBase_Struct *FTM0, FlexTimerBase_Struct *FTM1, FlexTimerBase_Struct *FTM2);
		char	*getVersion(void);
		friend void ftm0_isr(void);
		friend void ftm1_isr(void);
		friend void ftm2_isr(void);
	private:
		static bool debugEnabled;
		static bool FTM0_OF_Inc;
		static bool FTM1_OF_Inc;
		static bool FTM2_OF_Inc;
		static uint32_t FTM0_OF_Cnt;
		static uint32_t FTM1_OF_Cnt;
		static uint32_t FTM2_OF_Cnt;
		static PPM *thisPtr;
};


