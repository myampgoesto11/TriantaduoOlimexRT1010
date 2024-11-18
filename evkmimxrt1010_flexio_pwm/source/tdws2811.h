/*
 * TDWS2811.h
 *
 *  Created on: Nov 17, 2024
 *      Author: Randy.Lathrop
 */

#ifndef TDWS2811_H_
#define TDWS2811_H_

#include "board.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_TIME_DELAY_FOR_DUTY_CYCLE_UPDATE (2000000U)
#define DEMO_FLEXIO_BASEADDR                  FLEXIO1
// The description appears wrong, FLEXIO_IO26 is muxed to GPIO_AD_14 in the RM p. 190
#define DEMO_FLEXIO_OUTPUTPIN                 (26U) /* Select FLEXIO1_FLEXIO05 as PWM output */
#define DEMO_FLEXIO_TIMER_CH                  (0U)  /* Flexio timer0 used */

/* Select USB1 PLL (480 MHz) as flexio clock source */
#define FLEXIO_CLOCK_SELECT 		(3U)
/* Clock pre divider for flexio clock source */
#define FLEXIO_CLOCK_PRE_DIVIDER 	(4U)
/* Clock divider for flexio clock source */
#define FLEXIO_CLOCK_DIVIDER 		(7U)
#define DEMO_FLEXIO_CLOCK_FREQUENCY \
    (CLOCK_GetFreq(kCLOCK_Usb1PllClk) / (FLEXIO_CLOCK_PRE_DIVIDER + 1U) / (FLEXIO_CLOCK_DIVIDER + 1U))
/* FLEXIO output PWM frequency */
#define DEMO_FLEXIO_FREQUENCY 		(48000U)
#define FLEXIO_MAX_FREQUENCY 		(DEMO_FLEXIO_CLOCK_FREQUENCY / 2U)
#define FLEXIO_MIN_FREQUENCY 		(DEMO_FLEXIO_CLOCK_FREQUENCY / 512U)

/* flexio timer number */
#define FLEXIO_TIMER_CHANNELS 		(8)

//--------------------------------
//  ++ LED App ++
#define LED_APP
#define LED_APP_DMA_CHANNEL_0	(0)
#define LED_APP_TCD_QUEUE_SIZE  (4U)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void tdws2811Init(void);
void tdws2811Run(void);

#endif /* TDWS2811_H_ */
