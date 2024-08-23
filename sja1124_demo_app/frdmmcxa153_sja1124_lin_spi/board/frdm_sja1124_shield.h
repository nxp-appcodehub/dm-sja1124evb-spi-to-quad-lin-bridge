/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: frdm_sja1124_shield.h
 * @brief The frdm_sja1124_shield.h file declares arduino pin mapping for frdm_sja1124_shield expansion board.
 */

#ifndef _FRDM_SJA1124_SHIELD_H_
#define _FRDM_SJA1124_SHIELD_H_

//-----------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------

#define BOARD_PWM_BASEADDR 			(FLEXPWM0)
#define PWM_SRC_CLK_FREQ 			(CLOCK_GetFreq(kCLOCK_BusClk))
#define DEMO_PWM_CLOCK_DEVIDER			(kPWM_Prescale_Divide_4)
#define DEMO_PWM_FAULT_LEVEL   			true

/* The shield name */
#define SHIELD_NAME "FRDM-SJA1124EVB"

#define SJA1124_CS        D7
#define SJA1124_MOSI      D11
#define SJA1124_MISO      D12
#define SJA1124_SCLK      D13

#endif /* _FRDM_SJA1124_SHIELD_H_ */
