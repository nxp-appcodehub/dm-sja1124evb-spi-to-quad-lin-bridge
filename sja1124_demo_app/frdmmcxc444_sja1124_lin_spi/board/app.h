/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _APP_H_
#define _APP_H_
#include "fsl_clock.h"
#include "fsl_spi_cmsis.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
#define DRIVER_MASTER_SPI Driver_SPI1
#define SJA1124_INT_PIN        BOARD_INT_PIN
#define SPI_S_DEVICE_INDEX SPI1_INDEX
#define SPI_S_SIGNAL_EVENT SPI1_SignalEvent_t

#define BOARD_TPM_BASEADDR TPM0
#define BOARD_TPM_CHANNEL  2U
#define TPM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_McgIrc48MClk)

#ifndef DEMO_PWM_FREQUENCY
#define DEMO_PWM_FREQUENCY (1000000U)
#endif

#ifndef TPM_LED_ON_LEVEL
#define TPM_LED_ON_LEVEL kTPM_HighTrue
#endif

/*${macro:end}*/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*${prototype:start}*/
void BOARD_InitHardware(void);
/*${prototype:end}*/

#endif /* _APP_H_ */
