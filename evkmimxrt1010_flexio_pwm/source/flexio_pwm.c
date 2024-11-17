/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_flexio.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

// **EDMA**
#include "fsl_edma.h"
#include "fsl_dmamux.h"
// **EDMA**

//////////////////////////////
#include "fsl_common.h"
#include "fsl_iomuxc.h"
/////////////////////////////

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_TIME_DELAY_FOR_DUTY_CYCLE_UPDATE (2000000U)
#define DEMO_FLEXIO_BASEADDR                  FLEXIO1
// The description appears wrong, FLEXIO_IO26 is muxed to GPIO_AD_14 in the RM p. 190
#define DEMO_FLEXIO_OUTPUTPIN                 (26U) /* Select FLEXIO1_FLEXIO05 as PWM output */
#define DEMO_FLEXIO_TIMER_CH                  (0U)  /* Flexio timer0 used */

/* Select USB1 PLL (480 MHz) as flexio clock source */
#define FLEXIO_CLOCK_SELECT (3U)
/* Clock pre divider for flexio clock source */
#define FLEXIO_CLOCK_PRE_DIVIDER (4U)
/* Clock divider for flexio clock source */
#define FLEXIO_CLOCK_DIVIDER (7U)
#define DEMO_FLEXIO_CLOCK_FREQUENCY \
    (CLOCK_GetFreq(kCLOCK_Usb1PllClk) / (FLEXIO_CLOCK_PRE_DIVIDER + 1U) / (FLEXIO_CLOCK_DIVIDER + 1U))
/* FLEXIO output PWM frequency */
#define DEMO_FLEXIO_FREQUENCY (48000U)
#define FLEXIO_MAX_FREQUENCY (DEMO_FLEXIO_CLOCK_FREQUENCY / 2U)
#define FLEXIO_MIN_FREQUENCY (DEMO_FLEXIO_CLOCK_FREQUENCY / 512U)

/* flexio timer number */
#define FLEXIO_TIMER_CHANNELS (8)

/*******************************************************************************
 *  ++ LED App ++ Definitions
 ******************************************************************************/
#define LED_APP
#define LED_APP_DMA_CHANNEL_0	(0)
#define LED_APP_TCD_QUEUE_SIZE  (4U)

/*******************************************************************************
 *  ++ LED App ++ Variables
 ******************************************************************************/
/* Allocate TCD memory poll */
AT_QUICKACCESS_SECTION_DATA_ALIGN(edma_tcd_t ledAppTcdMemoryPoolPtr[LED_APP_TCD_QUEUE_SIZE + 1], sizeof(edma_tcd_t));

//------------------------------------------------------------------------------

/*******************************************************************************
 *  --Triantaduo-- Definitions
 ******************************************************************************/

#undef HASWHITE

#ifndef LEDCOUNT
#define LEDCOUNT 10
#endif

/*******************************************************************************
 * --Triantaduo-- Variables
 ******************************************************************************/

#ifdef HASWHITE
    /*volatile*/ uint32_t frameBuffer[2][32*LEDCOUNT]={{0},{0}};  /* If the collection of channels has white, size the frame buffer for 32 bits per pixel */
#else
    /*volatile*/ uint32_t frameBuffer[2][24*LEDCOUNT]={{0},{0}};  /* If no white, size the frame buffer for 24 bits per pixel */
#endif

uint8_t m_activeBuffer = 0;
const uint32_t zeros[40]={0};
const uint32_t ones=0xFFFFFFFF;

//------------------------------------------------------------------------------

/*******************************************************************************
 *  **EDMA** Definitions
 ******************************************************************************/
#define EXAMPLE_DMA    DMA0
#define EXAMPLE_DMAMUX DMAMUX
#define BUFFER_LENGTH      8U
#define HALF_BUFFER_LENGTH (BUFFER_LENGTH / 2U)
#define TCD_QUEUE_SIZE     2U

/*******************************************************************************
 * **EDMA** Variables
 ******************************************************************************/
edma_handle_t g_EDMA_Handle;
volatile bool g_Transfer_Done = false;

AT_NONCACHEABLE_SECTION_INIT(uint32_t srcAddr[BUFFER_LENGTH])  = {0x01U, 0x02U, 0x03U, 0x04U,
                                                                 0x05U, 0x06U, 0x07U, 0x08U};
AT_NONCACHEABLE_SECTION_INIT(uint32_t destAddr[BUFFER_LENGTH]) = {0x00U, 0x00U, 0x00U, 0x00U,
                                                                  0x00U, 0x00U, 0x00U, 0x00U};
/* Allocate TCD memory poll */
AT_QUICKACCESS_SECTION_DATA_ALIGN(edma_tcd_t tcdMemoryPoolPtr[TCD_QUEUE_SIZE + 1], sizeof(edma_tcd_t));

/*******************************************************************************
 * **EDMA** Code
 ******************************************************************************/

/* User callback function for EDMA transfer. */
void EDMA_Callback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
#if defined(LED_APP)
      static volatile m_test = 0;
	  FLEXIO_Type *p = DEMO_FLEXIO_BASEADDR;

      m_test = p->SHIFTBUFBIS[1];
      m_test++;
	  /* Disable interrupts on TCD 1.  If there's a "disable interrupt" function in DMASetting,
	   * we should use that, but I don't see one */
//	  TDWS2811::dmaSetting[1].TCD->CSR &= ~DMA_TCD_CSR_INTMAJOR;

	  /* Swap the buffer pointer in TCD 0 */
	  if (m_activeBuffer==0) {
		  m_activeBuffer=1;
//		  TDWS2811::dmaSetting[0].sourceBuffer(frameBuffer[1],24*LEDCOUNT*4);
	  }
	  else {
		  m_activeBuffer=0;
//		  TDWS2811::dmaSetting[0].sourceBuffer(frameBuffer[0],24*LEDCOUNT*4);
	  }

	  /* Clear the interrupt so we don't get triggered again */
//	  TDWS2811::dmaChannel.clearInterrupt();

	  /* Spin for a few cycles.  If we don't do this, the interrupt doesn't clear and we get
	   * triggered a second time */
	  // Some race condition between clearInterrupt() and the return of the ISR.
	  // If we don't delay here, the ISR will fire again.
	  for (uint8_t i=0;i<10;i++) __asm__ __volatile__ ("nop\n\t");

	  if (transferDone)
	  {
		  g_Transfer_Done = true;
	  }
#else
	if (transferDone)
    {
        g_Transfer_Done = true;
    }
#endif
}

/*
void TDWS2811::configureDma() {
  // Enable DMA trigger on Shifter 1 - This was moved to the function:
  p->SHIFTSDEN |= 0X00000002;

  // TCD 0 is responsible for the bulk of the data transfer.  It shuffles data from the frame buffer to Shifter 1 //
  dmaSetting[0].sourceBuffer(frameBuffer[0],24*LEDCOUNT*4);
  dmaSetting[0].destination(p->SHIFTBUFBIS[1]);
  dmaSetting[0].replaceSettingsOnCompletion(dmaSetting[1]);

  // TCD 1 is responsible for setting Shifter 0 (Phase 1) to zero.  I tried "source(zeros[0])", but it doesn't work.  Maybe that's a byte-wide operation?
  Note that the DMA channel is triggered by FlexIO shifter 1.  The only way to acknowledge the trigger is to write to one of Shifter 1's data registers.
  TCD 1 only touches Shifter 0's register, so the DMA channel is immediately triggered again upon TCD 1's completion, moving us directly into TCD 2. //
  dmaSetting[1].sourceBuffer(zeros,4);
  dmaSetting[1].destination(p->SHIFTBUF[0]);
  dmaSetting[1].replaceSettingsOnCompletion(dmaSetting[2]);

  // TCD 2 is responsible for the blanking delay.  That delay is, in turn, dictated by the size of the "zeros" array.
  As this TCD executes once per (1.25us) bit period, an array of 40 zeros gets us a 50us delay.//
  dmaSetting[2].sourceBuffer(zeros,sizeof(zeros));
  dmaSetting[2].destination(p->SHIFTBUF[1]);
  dmaSetting[2].replaceSettingsOnCompletion(dmaSetting[3]);

  // TCD 3 is responsible for setting Shifter 0 (Phase 1) to ones.  I tried "source(&ones)", but it doesn't work.  Maybe that's a byte-wide operation?
  Note that the DMA channel is triggered by FlexIO shifter 1.  The only way to acknowledge the trigger is to write to one of Shifter 1's data registers.
  TCD 3 only touches Shifter 0's register, so the DMA channel is immediately triggered again upon TCD 3's completion, moving us directly into TCD 0. //
  dmaSetting[3].sourceBuffer(&ones,4);
  dmaSetting[3].destination(p->SHIFTBUF[0]);
  dmaSetting[3].replaceSettingsOnCompletion(dmaSetting[0]);

  // Now set up the DMA channel and initialize it with TCD 0 //
  dmaChannel.disable();
  dmaChannel=dmaSetting[0];
  dmaChannel.triggerAtHardwareEvent(hw->shifters_dma_channel[1]);
  dmaChannel.attachInterrupt(&_dmaIsr);

  // Needed for the ISR, for reasons relating to using C++ in an embedded environment, which is most foolish //
  pTD=this;
  dmaChannel.enable();
}
*/

/*
void replaceSettingsOnCompletion(const DMABaseClass &settings) {
	TCD->DLASTSGA = (int32_t)(settings.TCD);
	TCD->CSR &= ~DMA_TCD_CSR_DONE;
	TCD->CSR |= DMA_TCD_CSR_ESG;
}
*/

void configureEDMA(void) // From the "evkmimxrt1010_edma_scatter_gather" project
{
    uint32_t i = 0;
    edma_transfer_config_t transferConfig;
    edma_config_t userConfig;

    //LED App specific code
    FLEXIO_Type *p = DEMO_FLEXIO_BASEADDR;
    /* Enable DMA trigger on Shifter 1 */
    p->SHIFTSDEN |= 0X00000002;
    //LED App specific code

    /* Configure DMAMUX */
	DMAMUX_Init(EXAMPLE_DMAMUX);
	#if defined(FSL_FEATURE_DMAMUX_HAS_A_ON) && FSL_FEATURE_DMAMUX_HAS_A_ON
	DMAMUX_EnableAlwaysOn(EXAMPLE_DMAMUX, 0, true);
	#else
	DMAMUX_SetSource(EXAMPLE_DMAMUX, LED_APP_DMA_CHANNEL_0, 63);
	#endif /* FSL_FEATURE_DMAMUX_HAS_A_ON */
	DMAMUX_EnableChannel(EXAMPLE_DMAMUX, LED_APP_DMA_CHANNEL_0);
	/* Configure EDMA one shot transfer */
	/*
	 * userConfig.enableRoundRobinArbitration = false;
	 * userConfig.enableHaltOnError = true;
	 * userConfig.enableContinuousLinkMode = false;  TODO: Should this be true?
	 * userConfig.enableDebugMode = false;
	 */
	EDMA_GetDefaultConfig(&userConfig);
	EDMA_Init(EXAMPLE_DMA, &userConfig);
	EDMA_CreateHandle(&g_EDMA_Handle, EXAMPLE_DMA, LED_APP_DMA_CHANNEL_0);
	EDMA_SetCallback(&g_EDMA_Handle, EDMA_Callback, NULL);
	EDMA_ResetChannel(g_EDMA_Handle.base, g_EDMA_Handle.channel);
	EDMA_InstallTCDMemory(&g_EDMA_Handle, tcdMemoryPoolPtr, TCD_QUEUE_SIZE);

	// TCD 0 is responsible for the bulk of the data transfer.  It shuffles data from the frame buffer to Shifter 1 //
/*	dmaSetting[0].sourceBuffer(frameBuffer[0],24*LEDCOUNT*4);
	dmaSetting[0].destination(p->SHIFTBUFBIS[1]);
	dmaSetting[0].replaceSettingsOnCompletion(dmaSetting[1]); */
	EDMA_PrepareTransfer(&transferConfig,
						  (void *)frameBuffer[0],   // void *srcAddr
						  sizeof(frameBuffer[0][0]),   // uint32_t srcWidth
						  (void *)&p->SHIFTBUFBIS[1],// void *destAddr      - The "SHIFTBUFBIS" register is Bit Swapped!
						  sizeof(p->SHIFTBUFBIS[1]),// uint32_t destWidth
						  sizeof(frameBuffer[0]),   // uint32_t bytesEachRequest
						  (24 * LEDCOUNT * 4),      // uint32_t transferBytes  - TODO: What is the "24" & "4"?
						  kEDMA_MemoryToPeripheral);// edma_transfer_type_t transferType
	EDMA_SubmitTransfer(&g_EDMA_Handle, &transferConfig);

	// TCD 1 is responsible for setting Shifter 0 (Phase 1) to zero.  I tried "source(zeros[0])", but it doesn't work.  Maybe that's a byte-wide operation?
	// Note that the DMA channel is triggered by FlexIO shifter 1.  The only way to acknowledge the trigger is to write to one of Shifter 1's data registers.
	// TCD 1 only touches Shifter 0's register, so the DMA channel is immediately triggered again upon TCD 1's completion, moving us directly into TCD 2. //
/*	dmaSetting[1].sourceBuffer(zeros,4);
	dmaSetting[1].destination(p->SHIFTBUF[0]);
	dmaSetting[1].replaceSettingsOnCompletion(dmaSetting[2]);*/
	EDMA_PrepareTransfer(&transferConfig,
						  (void *)zeros,              // void *srcAddr
						  sizeof(zeros[0]),           // uint32_t srcWidth
						  (void *)&p->SHIFTBUF[0],     // void *destAddr
						  sizeof(p->SHIFTBUF[0]),     // uint32_t destWidth
						  sizeof(zeros[0]),           // uint32_t bytesEachRequest - Just send 4 bytes
						  sizeof(zeros[0]),           // uint32_t transferBytes    - Just send 4 bytes
						  kEDMA_MemoryToPeripheral);  // edma_transfer_type_t transferType
	EDMA_SubmitTransfer(&g_EDMA_Handle, &transferConfig);

	// TCD 2 is responsible for the blanking delay.  That delay is, in turn, dictated by the size of the "zeros" array.
	// As this TCD executes once per (1.25us) bit period, an array of 40 zeros gets us a 50us delay.//
/*	dmaSetting[2].sourceBuffer(zeros,sizeof(zeros));
	dmaSetting[2].destination(p->SHIFTBUF[1]);
	dmaSetting[2].replaceSettingsOnCompletion(dmaSetting[3]); */
	EDMA_PrepareTransfer(&transferConfig,
						  (void *)zeros,             // void *srcAddr
						  sizeof(zeros[0]),          // uint32_t srcWidth
						  (void *)&p->SHIFTBUF[1],    // void *destAddr
						  sizeof(p->SHIFTBUF[1]),    // uint32_t destWidth
						  sizeof(zeros),             // uint32_t bytesEachRequest
						  sizeof(zeros),             // uint32_t transferBytes
						  kEDMA_MemoryToPeripheral); // edma_transfer_type_t transferType
	EDMA_SubmitTransfer(&g_EDMA_Handle, &transferConfig);

	// TCD 3 is responsible for setting Shifter 0 (Phase 1) to ones.  I tried "source(&ones)", but it doesn't work.  Maybe that's a byte-wide operation?
	// Note that the DMA channel is triggered by FlexIO shifter 1.  The only way to acknowledge the trigger is to write to one of Shifter 1's data registers.
	// TCD 3 only touches Shifter 0's register, so the DMA channel is immediately triggered again upon TCD 3's completion, moving us directly into TCD 0. //
/*	dmaSetting[3].sourceBuffer(&ones,4);
	dmaSetting[3].destination(p->SHIFTBUF[0]);
	dmaSetting[3].replaceSettingsOnCompletion(dmaSetting[0]);*/
	EDMA_PrepareTransfer(&transferConfig,
						  (void *)&ones,             // void *srcAddr
						  sizeof(&ones),             // uint32_t srcWidth
						  (void *)&p->SHIFTBUF[0],    // void *destAddr
						  sizeof(p->SHIFTBUF[0]),    // uint32_t destWidth
						  sizeof(ones),              // uint32_t bytesEachRequest - Just send 4 bytes
						  sizeof(ones),              // uint32_t transferBytes    - Just send 4 bytes
						  kEDMA_MemoryToPeripheral); // edma_transfer_type_t transferType
	EDMA_SubmitTransfer(&g_EDMA_Handle, &transferConfig);

	// Let the show begin!!!
	EDMA_StartTransfer(&g_EDMA_Handle);


/*
	// Wait for EDMA transfer finish
	while (g_Transfer_Done != true)
	{
	}
	// Print destination buffer
	PRINTF("\r\n\r\nEDMA scatter gather transfer example finish.\r\n\r\n");
	PRINTF("Destination Buffer:\r\n");
	for (i = 0; i < BUFFER_LENGTH; i++)
	{
		PRINTF("%d\t", destAddr[i]);
	}
	// Free the memory space allocated
	while (1)
	{
	}
*/
}

//------------------------------------------------------------------------------

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

// Note: The FLEX IO has a "PARAM" register, that contains the following:
// TRIGGER[31:24] = Number of external triggers implemented.
// PIN[23:16]     = Number of Pins implemented.
// TIMER[15:8]    = Number of Timers implemented.
// SHIFTER[7:0]   = Number of Shifters implemented.

void configureFlexIO(void)
{
// From Ward's TDWS2811 Constructor Code:
	  /* Pointer to the port structure in the FlexIO channel */
	  /* p = &pFlex->port(); */
	  /* Pointer to the port structure in the FlexIO channel */
	  /* p = &pFlex->port(); */

	  /* Pointer to the hardware structure in the FlexIO channel */
	  /* hw = &pFlex->hardware(); */

  FLEXIO_Type *p = DEMO_FLEXIO_BASEADDR;

  /* Map the FlexIO pins */
/* TODO
 * pFlex->mapIOPinToFlexPin(2); // This is from the "FlexIO_t4" Github project
 * pFlex->mapIOPinToFlexPin(3);
 * pFlex->mapIOPinToFlexPin(4);
 *
 * // And set up the pin mux
 * pFlex->setIOPinToFlexMode(2);
 * pFlex->setIOPinToFlexMode(3);
 * pFlex->setIOPinToFlexMode(4);
 *
 * // Enable the clock
 * hw->clock_gate_register |= hw->clock_gate_mask;
 *
 * // Enable the FlexIO
 * p->CTRL = FLEXIO_CTRL_FLEXEN;
 *
 */

  /* Shifter configuration, see reference manual for a description of each register */
  /* Note that shifter 4, although unused, must be configured and tied into the chain.  If it's not, there's a data glitch that trips us up.
  /* See the Github README for more details. */

  // TIMSEL[26:24] - (Timer Select)         0 = Tmr 0
  // TIMPOL[23]    - (Timer Polarity)       1 = Shift on Neg. Edge
  // PINCFG[17:16] - (Shifter Pin Config.)  3 = Shifter pin Output
  // PINSEL[12:8]  - (Shifter Pin Select)   6 = FXIO_D6
  // PINPOL[7]     - (Shifter Pin Polarity) 0 = Active High
  // SMOD[2:0]     - (Shifter Mode)         2 = Transmit Mode
  p->SHIFTCTL[0] = 0x00830602;

  // TIMSEL[26:24] - (Timer Select)         0 = Tmr 0
  // TIMPOL[23]    - (Timer Polarity)       1 = Shift on Neg. Edge
  // PINCFG[17:16] - (Shifter Pin Config.)  0 = Shifter pin output disabled
  // PINSEL[12:8]  - (Shifter Pin Select)   0 = -NONE-
  // PINPOL[7]     - (Shifter Pin Polarity) 0 = Active High
  // SMOD[2:0]     - (Shifter Mode)         2 = Transmit Mode
  p->SHIFTCTL[1] = 0x00800002;
  p->SHIFTCTL[2] = 0x00800002;
  p->SHIFTCTL[3] = 0x00800002;
  p->SHIFTCTL[4] = 0x00800002;

  // PWIDTH[20:16] (Parallel Width)   0 = 1-bit shift
  // INSRC[8]      (Input Source)     1 = Shifter N+1 Output
  // SSTOP[5:4]    (Shifter Stop bit) 0 = Stop Bit disabled
  // SSTART[1:0] = Start bit disabled
  p->SHIFTCFG[0] = 0x00000100;
  p->SHIFTCFG[1] = 0x00000100;
  p->SHIFTCFG[2] = 0x00000100;
  p->SHIFTCFG[3] = 0x00000100;
  p->SHIFTCFG[4] = 0x00000100;

  /* Timer configuration, see reference manual for a description of each register */
  // TIMOUT[25:24] - 0 = Timer output is logic one when enabled and is not affected by timer reset

  // TIMDEC[21:20] - 0 = Decrement counter on FlexIO clock, Shift clock equals Timer output.
  // TIMRST[18:16] - 0 = Timer never reset

  // TIMDIS[14:12] - 0 = Timer never disabled
  // TIMENA[10:8]  - 2 = Timer enabled on Trigger high

  // TSTART[5:4] - 0 = Start bit disabled
  // TSTOP[1]    - 0 = Stop bit disabled
  p->TIMCFG[0] =   0x00000200;
  // TIMENA[10:8]  - 1 = Timer enabled on Timer N-1 enable
  p->TIMCFG[1] =   0x00000100;

  // TRGSEL[29-24] - (Trigger Select)     1 = Shifter 0 Flag - MAYBE - SEE RM! IT'S COMPLICATED!
  // TRGPOL[23]    - (Trigger Polarity)   1 = Trigger active low
  // TRGSRC[22]    - (Trigger Source)     1 = Internal trigger selected
  // PINCFG[17:16] - (Timer Pin Config)   3 = Timer pin is output
  // PINSEL[12:8]  - (Timer Pin Select)   4 = Timer pin output #FXIO_D4
  // PINPOL[7]     - (Timer Pin Polarity) 0 = Pin is active high
  // TIMOD [1:0]   - (Timer Mode)         1 = Dual 8-bit counters baud mode
  p->TIMCTL[0] =   0x01C30401;

  // TRGSEL[29-24] - (Trigger Select)     0 = Pin 0 - MAYBE - SEE RM! IT'S COMPLICATED!
  // TRGPOL[23]    - (Trigger Polarity)   0 = Trigger active HIGH
  // TRGSRC[22]    - (Trigger Source)     0 = External trigger selected
  // PINCFG[17:16] - (Timer Pin Config)   3 = Timer pin is output
  // PINSEL[12:8]  - (Timer Pin Select)   5 = Timer pin output #FXIO_D5
  // PINPOL[7]     - (Timer Pin Polarity) 0 = Pin is active high
  // TIMOD [1:0]   - (Timer Mode)         3 = Single 16-bit counter mode
  p->TIMCTL[1] =   0x00030503;

  p->TIMCMP[0] =   0x0000BF00; // 16 bit compare value = 48,896
  p->TIMCMP[1] =   0x0000001F; // 31

  /* Finally, set up the values to be loaded into the shift registers at the beginning of each bit */
  p->SHIFTBUF[0] = 0xA00000F;
  p->SHIFTBUFBIS[1] = 0x5a5aa5a5;  //Identifiable pattern should DMA fail to write SHIFTBUFBIS[1]
  p->SHIFTBUF[2] = 0xFFFFFFFF;
  p->SHIFTBUF[3] = 0x00000000;
  p->SHIFTBUF[3] = 0x00000000;

}

/*!
 * @brief Configures the timer as a 8-bits PWM mode to generate the PWM waveform
 *
 * @param freq_Hz PWM frequency in hertz, range is [FLEXIO_MIN_FREQUENCY, FLEXIO_MAX_FREQUENCY]
 * @param duty Specified duty in unit of %, with a range of [0, 100]
 */
static status_t flexio_pwm_init(uint32_t freq_Hz, uint32_t duty);

/*!
 * @brief Set PWM output in idle status (high or low).
 *
 * @param base               FlexIO peripheral base address
 * @param timerChannel       FlexIO timer channel
 * @param idleStatus         True: PWM output is high in idle status; false: PWM output is low in idle status
 */
static void FLEXIO_SetPwmOutputToIdle(FLEXIO_Type *base, uint8_t timerChannel, bool idleStatus);

#if defined(FSL_FEATURE_FLEXIO_HAS_PIN_STATUS) && FSL_FEATURE_FLEXIO_HAS_PIN_STATUS
/*!
 * @brief Get the pwm dutycycle value.
 *
 * @param base        FlexIO peripheral base address
 * @param timerChannel  FlexIO timer channel
 * @param channel     FlexIO as pwm output channel number
 *
 * @return Current channel dutycycle value.
 */
static uint8_t PWM_GetPwmOutputState(FLEXIO_Type *base, uint8_t timerChannel, uint8_t channel);
#endif

/*!
 * @brief Get pwm duty cycle value.
 */
static uint8_t s_flexioGetPwmDutyCycle[FLEXIO_TIMER_CHANNELS] = {0};

/*******************************************************************************
 * Variables
 *******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
static status_t flexio_pwm_init(uint32_t freq_Hz, uint32_t duty)
{
	static bool code_test = false;
	bool test = false;

    assert((freq_Hz < FLEXIO_MAX_FREQUENCY) && (freq_Hz > FLEXIO_MIN_FREQUENCY));

    uint32_t lowerValue = 0; /* Number of clock cycles in high logic state in one period */
    uint32_t upperValue = 0; /* Number of clock cycles in low logic state in one period */
    uint32_t sum        = 0; /* Number of clock cycles in one period */
    flexio_timer_config_t fxioTimerConfig;

    /* Configure the timer DEMO_FLEXIO_TIMER_CH for generating PWM */
    fxioTimerConfig.triggerSelect   = FLEXIO_TIMER_TRIGGER_SEL_SHIFTnSTAT(0U);
    fxioTimerConfig.triggerSource   = kFLEXIO_TimerTriggerSourceInternal;
    fxioTimerConfig.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveLow;

    fxioTimerConfig.pinConfig       = kFLEXIO_PinConfigOutput;
    fxioTimerConfig.pinPolarity     = kFLEXIO_PinActiveHigh;
    fxioTimerConfig.pinSelect       = DEMO_FLEXIO_OUTPUTPIN; /* Set pwm output */

    fxioTimerConfig.timerMode       = kFLEXIO_TimerModeDisabled;
    fxioTimerConfig.timerOutput     = kFLEXIO_TimerOutputOneNotAffectedByReset;
    fxioTimerConfig.timerDecrement  = kFLEXIO_TimerDecSrcOnFlexIOClockShiftTimerOutput;
    fxioTimerConfig.timerDisable    = kFLEXIO_TimerDisableNever;
    fxioTimerConfig.timerEnable     = kFLEXIO_TimerEnabledAlways;
    fxioTimerConfig.timerReset      = kFLEXIO_TimerResetNever;
    fxioTimerConfig.timerStart      = kFLEXIO_TimerStartBitDisabled;
    fxioTimerConfig.timerStop       = kFLEXIO_TimerStopBitDisabled;

    /* Calculate timer lower and upper values of TIMCMP */
    /* Calculate the nearest integer value for sum, using formula round(x) = (2 * floor(x) + 1) / 2 */
    /* sum = DEMO_FLEXIO_CLOCK_FREQUENCY / freq_H */
    if(code_test)
    {
        sum = DEMO_FLEXIO_CLOCK_FREQUENCY; // What is the Timer input clock frequency?
        sum *= 2;
        sum /= (freq_Hz + 1);
    	sum /= 2;
    }
    else
    {
        sum = (DEMO_FLEXIO_CLOCK_FREQUENCY * 2 / freq_Hz + 1) / 2; // original code
    }

    /* Calculate the nearest integer value for lowerValue, the high period of the pwm output */
    lowerValue = (sum * duty) / 100;
    /* Calculate upper value, the low period of the pwm output */
    upperValue = sum - lowerValue - 2;

    fxioTimerConfig.timerCompare = ((upperValue << 8U) | (lowerValue));

    if ((duty > 0) && (duty < 100))
    {
        /* Set Timer mode to kFLEXIO_TimerModeDual8BitPWM to start timer */
        fxioTimerConfig.timerMode = kFLEXIO_TimerModeDual8BitPWM;
    }
    else if (duty == 100)
    {
        fxioTimerConfig.pinPolarity = kFLEXIO_PinActiveLow;
    }
    else if (duty == 0)
    {
        /* Set high level as active level */
        fxioTimerConfig.pinPolarity = kFLEXIO_PinActiveHigh;
    }
    else
    {
        return kStatus_Fail;
    }

    FLEXIO_SetTimerConfig(DEMO_FLEXIO_BASEADDR, DEMO_FLEXIO_TIMER_CH, &fxioTimerConfig);

    s_flexioGetPwmDutyCycle[DEMO_FLEXIO_TIMER_CH] = duty;

    return kStatus_Success;
}

/*!
 * brief Set PWM output in idle status (high or low).
 *
 * param base               FlexIO peripheral base address
 * param timerChannel       FlexIO timer channel
 * param idleStatus         True: PWM output is high in idle status; false: PWM output is low in idle status
 */
static void FLEXIO_SetPwmOutputToIdle(FLEXIO_Type *base, uint8_t timerChannel, bool idleStatus)
{
    flexio_timer_config_t fxioTimerConfig;

    /* Configure the timer DEMO_FLEXIO_TIMER_CH for generating PWM */
    fxioTimerConfig.triggerSelect   = FLEXIO_TIMER_TRIGGER_SEL_SHIFTnSTAT(0U);
    fxioTimerConfig.triggerSource   = kFLEXIO_TimerTriggerSourceInternal;
    fxioTimerConfig.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveLow;
    fxioTimerConfig.pinConfig       = kFLEXIO_PinConfigOutput;
    fxioTimerConfig.pinSelect       = DEMO_FLEXIO_OUTPUTPIN; /* Set pwm output */
    fxioTimerConfig.timerMode       = kFLEXIO_TimerModeDisabled;
    fxioTimerConfig.timerOutput     = kFLEXIO_TimerOutputOneNotAffectedByReset;
    fxioTimerConfig.timerDecrement  = kFLEXIO_TimerDecSrcOnFlexIOClockShiftTimerOutput;
    fxioTimerConfig.timerDisable    = kFLEXIO_TimerDisableNever;
    fxioTimerConfig.timerEnable     = kFLEXIO_TimerEnabledAlways;
    fxioTimerConfig.timerReset      = kFLEXIO_TimerResetNever;
    fxioTimerConfig.timerStart      = kFLEXIO_TimerStartBitDisabled;
    fxioTimerConfig.timerStop       = kFLEXIO_TimerStopBitDisabled;
    fxioTimerConfig.timerCompare    = 0U;

    /* Clear TIMCMP register */
    base->TIMCMP[timerChannel] = 0;

    if (idleStatus)
    {
        /* Set low level as active level */
        fxioTimerConfig.pinPolarity = kFLEXIO_PinActiveLow;
    }
    else
    {
        /* Set high level as active level */
        fxioTimerConfig.pinPolarity = kFLEXIO_PinActiveHigh;
    }

    FLEXIO_SetTimerConfig(DEMO_FLEXIO_BASEADDR, timerChannel, &fxioTimerConfig);

    s_flexioGetPwmDutyCycle[timerChannel] = 0;
}

#if defined(FSL_FEATURE_FLEXIO_HAS_PIN_STATUS) && FSL_FEATURE_FLEXIO_HAS_PIN_STATUS
/*!
 * brief Get the pwm dutycycle value.
 *
 * param base          FlexIO peripheral base address
 * param timerChannel  FlexIO timer channel
 * param channel       FlexIO as pwm output channel number
 *
 * return Current channel dutycycle value.
 */
static uint8_t PWM_GetPwmOutputState(FLEXIO_Type *base, uint8_t timerChannel, uint8_t channel)
{
    if ((base->PIN & (1U << channel)) ^ (base->TIMCTL[timerChannel] & FLEXIO_TIMCTL_PINPOL_MASK))
    {
        return kFLEXIO_PwmHigh;
    }
    else
    {
        return kFLEXIO_PwmLow;
    }
}
#endif

/*!
 * @brief Main function
 */
int main(void)
{
    uint8_t duty            = 0;
    uint8_t idleState       = 0;
    uint32_t dutyCycleValue = 0;
    uint32_t idleStateValue = 0;
    flexio_config_t fxioUserConfig;

    /* Init board hardware */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    /* Clock setting for Flexio */
    CLOCK_SetMux(kCLOCK_Flexio1Mux, FLEXIO_CLOCK_SELECT);
    CLOCK_SetDiv(kCLOCK_Flexio1PreDiv, FLEXIO_CLOCK_PRE_DIVIDER);
    CLOCK_SetDiv(kCLOCK_Flexio1Div, FLEXIO_CLOCK_DIVIDER);

    /* Init flexio, use default configure
     * Disable doze and fast access mode
     * Enable in debug mode
     */
    FLEXIO_GetDefaultConfig(&fxioUserConfig);
    FLEXIO_Init(DEMO_FLEXIO_BASEADDR, &fxioUserConfig);

    PRINTF("\r\nFLEXIO_PWM demo start.\r\n");

    //////////////////////////////
    // Fill the frame buffer with an incrementing value
    for(uint8_t i = 0; i < LEDCOUNT; i++)
    {
        frameBuffer[0][i] = (i * 4);
        frameBuffer[1][i] = (i * 10);
    }

    //  "4U" = 100 (binary) ALT4 — Select mux mode: ALT4 mux port: FLEXIO1_IO06 of instance: FLEXIO1
    // Reference Manual pg. 299, GPIO_SD_00 Mux Setting.
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_00_FLEXIO1_IO06, 4U);

    //  "F8U" = 100 (binary) ALT4 — Select mux mode: ALT4 mux port: FLEXIO1_IO06 of instance: FLEXIO1
    // Reference Manual pg. 377, GPIO_SD_00 pin Config.
    //             Spd| Drv  |    | slew
    //                | Str  |    | rate
    //  0  | 0   |    F   |     8      |
    // 0000 0000 - 11 | 11  1| xx |  0
    //              3      7    --     0
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_00_FLEXIO1_IO06, 0x00F8U);

    configureFlexIO();

    // **EDMA**
    //////////////////////////////
    configureEDMA();
    //////////////////////////////
    // **EDMA**

    // Get pointer to FLEX IO HW Block
    FLEXIO_Type *p = DEMO_FLEXIO_BASEADDR;

    bool testPWM = false;
    uint32_t inc = 0;


    while(!testPWM)
    {

    	if(g_Transfer_Done)
    	{
    		EDMA_StartTransfer(&g_EDMA_Handle);
    		g_Transfer_Done = false;
    	}

    	if(p->SHIFTBUFBIS[1] != 0)
    	{
    		inc = p->SHIFTBUFBIS[1];
    		inc++;
    	}
//    	p->SHIFTBUFBIS[1] = 0x1234567/* + inc++*/;  //Identifiable pattern should DMA fail to write SHIFTBUFBIS[1]
    }
    //////////////////////////////

    while (1)
    {
        duty           = 50;
        dutyCycleValue = 50;
        idleState      = 0;
        idleStateValue = 0;

#if 0
        PRINTF("\r\nPlease input a value (0 - 100) to set duty cycle: ");
        while (duty != 0x0D)
        {
            duty = GETCHAR();
            if ((duty >= '0') && (duty <= '9'))
            {
                PUTCHAR(duty);
                dutyCycleValue = dutyCycleValue * 10 + (duty - 0x30U);
            }
        }
        PRINTF("\r\nInput value is %d\r\n", dutyCycleValue);
#endif
        if (dutyCycleValue > 0x64U)
        {
            PRINTF("Your value is output of range.\r\n");
            PRINTF("Set pwm output to IDLE.\r\n");

            PRINTF("\r\nPlease input pwm idle status (0 or 1): ");
            while (idleState != 0x0D)
            {
                idleState = GETCHAR();
                if ((idleState >= '0') && (idleState <= '9'))
                {
                    PUTCHAR(idleState);
                    idleStateValue = idleStateValue * 10 + (idleState - 0x30U);
                }
            }

            PRINTF("\r\nInput IDLE state value is %d\r\n", idleStateValue);

            if (idleStateValue > 0x1U)
            {
                PRINTF("\r\nYour value is output of range.\r\n");

                continue;
            }

            FLEXIO_SetPwmOutputToIdle(DEMO_FLEXIO_BASEADDR, DEMO_FLEXIO_TIMER_CH, idleStateValue);
#if defined(FSL_FEATURE_FLEXIO_HAS_PIN_STATUS) && FSL_FEATURE_FLEXIO_HAS_PIN_STATUS
            PRINTF("\r\nPWM leave is: %d \r\n",
                   PWM_GetPwmOutputState(DEMO_FLEXIO_BASEADDR, DEMO_FLEXIO_TIMER_CH, DEMO_FLEXIO_OUTPUTPIN));
#endif
        }
        else
        {
            if (flexio_pwm_init(DEMO_FLEXIO_FREQUENCY, dutyCycleValue) == kStatus_Fail)
            {
                PRINTF("FLEXIO PWM initialization failed\n");
                return -1;
            }

            PRINTF("\r\nPWM duty cycle is: %d\r\n", s_flexioGetPwmDutyCycle[DEMO_FLEXIO_TIMER_CH]);
#if defined(FSL_FEATURE_FLEXIO_HAS_PIN_STATUS) && FSL_FEATURE_FLEXIO_HAS_PIN_STATUS
            PRINTF("\r\nPWM leave is: %d \r\n",
                   PWM_GetPwmOutputState(DEMO_FLEXIO_BASEADDR, DEMO_FLEXIO_TIMER_CH, DEMO_FLEXIO_OUTPUTPIN));
#endif
        }
    }
}
