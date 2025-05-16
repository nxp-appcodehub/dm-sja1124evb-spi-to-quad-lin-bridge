/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef HARDWARE_INIT_H_
#define HARDWARE_INIT_H_

#include "fsl_lpuart_cmsis.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define USART_LPUART             Driver_USART0

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_InitHardware(void);
void BOARD_UARTInitDebugConsole(void);

#endif /* HARDWARE_INIT_H_ */
