/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*${header:start}*/
#include <hardware_init.h>
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console_cmsis.h"
/*${header:end}*/

/*${function:start}*/
void BOARD_UARTInitDebugConsole(void)
{
	DebugConsole_Init();
	DebugConsole_PowerControl();
	DebugConsole_Control();
}

void BOARD_InitHardware(void)
{
    BOARD_InitPins();
    BOARD_BootClockRUN();
    //BOARD_InitDebugConsole();
    CLOCK_SetLpuart0Clock(0x1U);
    BOARD_UARTInitDebugConsole();
    /* Select the clock source for the TPM counter as kCLOCK_McgIrc48MClk */
    CLOCK_SetTpmClock(1U);

}

uint32_t SPI0_GetFreq(void)
{
    return CLOCK_GetBusClkFreq();
}

uint32_t SPI1_GetFreq(void)
{
    return CLOCK_GetBusClkFreq();
}

uint32_t LPUART0_GetFreq(void)
{
    return CLOCK_GetFreq(SYS_CLK);
}
/*${function:end}*/
