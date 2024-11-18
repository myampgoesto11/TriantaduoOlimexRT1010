/*
 * TDWS2811.c
 *
 *  Created on: Nov 17, 2024
 *      Author: Randy.Lathrop
 */
#include <tdws2811.h>
#include <stdint.h>

#include "fsl_common.h"
#include "fsl_device_registers.h"
#include "fsl_dmamux.h"
#include "fsl_edma.h"
#include "fsl_flexio.h"
#include "fsl_iomuxc.h"


/*******************************************************************************
 *  Definitions
 ******************************************************************************/
// --Triantaduo--
#undef HASWHITE

#ifndef LEDCOUNT
#define LEDCOUNT 10
#endif

// **EDMA**
#define EXAMPLE_DMA    DMA0
#define EXAMPLE_DMAMUX DMAMUX
#define BUFFER_LENGTH      8U
#define HALF_BUFFER_LENGTH (BUFFER_LENGTH / 2U)
#define TCD_QUEUE_SIZE     2U

/*******************************************************************************
 * Variables
 ******************************************************************************/
// **EDMA**
edma_handle_t g_EDMA_Handle;
volatile bool g_Transfer_Done = false;

AT_NONCACHEABLE_SECTION_INIT(uint32_t srcAddr[BUFFER_LENGTH])  = {0x01U, 0x02U, 0x03U, 0x04U,
                                                                 0x05U, 0x06U, 0x07U, 0x08U};
AT_NONCACHEABLE_SECTION_INIT(uint32_t destAddr[BUFFER_LENGTH]) = {0x00U, 0x00U, 0x00U, 0x00U,
                                                                  0x00U, 0x00U, 0x00U, 0x00U};
/* Allocate TCD memory poll */
AT_QUICKACCESS_SECTION_DATA_ALIGN(edma_tcd_t tcdMemoryPoolPtr[TCD_QUEUE_SIZE + 1], sizeof(edma_tcd_t));

// --Triantaduo--
#ifdef HASWHITE
    /*volatile*/ uint32_t frameBuffer[2][32*LEDCOUNT]={{0},{0}};  /* If the collection of channels has white, size the frame buffer for 32 bits per pixel */
#else
    /*volatile*/ uint32_t frameBuffer[2][24*LEDCOUNT]={{0},{0}};  /* If no white, size the frame buffer for 24 bits per pixel */
#endif

uint8_t m_activeBuffer = 0;
const uint32_t zeros[40]={0};
const uint32_t ones=0xFFFFFFFF;

/*******************************************************************************
 *  ++ LED App ++ Variables
 ******************************************************************************/
/* Allocate TCD memory poll */
AT_QUICKACCESS_SECTION_DATA_ALIGN(edma_tcd_t ledAppTcdMemoryPoolPtr[LED_APP_TCD_QUEUE_SIZE + 1], sizeof(edma_tcd_t));


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void configurePins(void);
void configureEDMA(void);
void configureFlexIO(void);
void EDMA_Callback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds);


/*******************************************************************************
 * Public
 ******************************************************************************/
void tdws2811Init(void)
{
    configurePins();
    configureFlexIO();
    configureEDMA();

	// Fill the frame buffer with an incrementing value
	for(uint8_t i = 0; i < LEDCOUNT; i++)
	{
		frameBuffer[0][i] = (i * 4);
		frameBuffer[1][i] = (i * 10);
	}
}

void configurePins(void)
{
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
}

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

void tdws2811Run(void)
{
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
}


/*******************************************************************************
 * Callbacks
 ******************************************************************************/

/* User callback function for EDMA transfer. */
void EDMA_Callback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
#if defined(LED_APP)
      static volatile int m_test = 0;
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
