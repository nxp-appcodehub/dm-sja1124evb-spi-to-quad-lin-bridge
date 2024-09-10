/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file frdmmcxn947.c
 * @brief The frdmmcxn947.c file defines GPIO pins and I2C CMSIS utilities for frdmmcxn947 board.
 */

#include <frdmmcxa153.h>

// I2C0 Pin Handles
gpioHandleKSDK_t D8 = {.base = GPIO3, .pinNumber = 15, .mask = 1 << (15), .clockName = kCLOCK_GateGPIO3, .portNumber = 3};
gpioHandleKSDK_t D9 = {.base = GPIO3, .pinNumber = 14, .mask = 1 << (14), .clockName = kCLOCK_GateGPIO3, .portNumber = 3};

// SPI1 Handle
/* [PIO0_20] (coords: 74) configured as SPI master-out/slave-in (MOSI) data */
gpioHandleKSDK_t D11 = {.base = GPIO2, .pinNumber = 13, .mask = 1 << (13), .clockName = kCLOCK_GateGPIO2, .portNumber = 2};
/* [PIO0_19] (coords: 90) is configured as SPI master-in/slave-out (MISO) data */
gpioHandleKSDK_t D12 = {.base = GPIO2, .pinNumber = 16, .mask = 1 << (16), .clockName = kCLOCK_GateGPIO2, .portNumber = 2};
/* [PIO0_21] (coords: 76) configured as SPI SCK */
gpioHandleKSDK_t D13 = {.base = GPIO2, .pinNumber = 12, .mask = 1 << (12), .clockName = kCLOCK_GateGPIO2, .portNumber = 2};
/* [PIO0_21] (coords: 76) configured as SPI SCK */
gpioHandleKSDK_t D10 = {.base = GPIO2, .pinNumber = 6, .mask = 1 << (6), .clockName = kCLOCK_GateGPIO2, .portNumber = 2};

gpioHandleKSDK_t D2 = {.base = GPIO2, .pinNumber = 4, .mask = 1 << (4), .clockName = kCLOCK_GateGPIO2, .portNumber = 2};

gpioHandleKSDK_t D7 = {.base = GPIO1, .pinNumber = 13, .mask = 1 << (13), .clockName = kCLOCK_GateGPIO1, .portNumber = 1};

gpioHandleKSDK_t PWM_SJA1124 = {.base = GPIO3, .pinNumber = 14, .mask = 1 << (14), .clockName = kCLOCK_GateGPIO3, .portNumber = 3};


//GPIO Pin Handles
gpioHandleKSDK_t INTA_PIN = {.base = GPIO3, .pinNumber = 15, .mask = 1 << (15), .irq = GPIO3_IRQn , .clockName = kCLOCK_GateGPIO3, .portNumber = 3};

//GPIO Pin Handles
gpioHandleKSDK_t SJA1124_INT = {.base = GPIO2, .pinNumber = 4, .mask = 1 << (4), .irq = GPIO2_IRQn, .clockName = kCLOCK_GateGPIO2, .portNumber = 2};


// LPCXpresso55s69 Internal Peripheral Pin Definitions
gpioHandleKSDK_t RED_LED = {
    .base = GPIO3, .pinNumber = 12, .mask = 1 << (12), .clockName = kCLOCK_GateGPIO3, .portNumber = 3};
gpioHandleKSDK_t GREEN_LED = {
    .base = GPIO3, .pinNumber = 13, .mask = 1 << (13), .clockName = kCLOCK_GateGPIO3, .portNumber = 3};
gpioHandleKSDK_t BLUE_LED = {
    .base = GPIO3, .pinNumber = 0, .mask = 1 << (0), .clockName = kCLOCK_GateGPIO3, .portNumber = 3};

/*!
 * @brief Configures the system to WAIT power mode.
 *        API name used from Kinetis family to maintain compatibility.
 *
 * @param Power peripheral base address (dummy).
 * @return Configuration error code.
 */
status_t SMC_SetPowerModeWait(void *arg)
{
    // POWER_EnterSleep();

    return kStatus_Success;
}

/*!
 * @brief Configures the system to VLPR power mode.
 *        API name used from Kinetis family to maintain compatibility.
 *
 * @param Power peripheral base address (dummy).
 * @return Configuration error code.
 */
status_t SMC_SetPowerModeVlpr(void *arg)
{
    // POWER_EnterSleep();

    return kStatus_Success;
}

/*! @brief       Determines the Clock Frequency feature.
 *  @details     The Clock Frequecny computation API required by fsl_uart_cmsis.c.
 *  @param[in]   void
 *  @Constraints None
 *  @Reentrant   Yes
 *  @return      uint32_t Returns the clock frequency .
 */
uint32_t LPUART0_GetFreq(void)
{
    return CLOCK_GetLpuartClkFreq(0u);
}

/*! @brief       Determines the Clock Frequency feature.
 *  @details     The Clock Frequecny computation API required by fsl_i2c_cmsis.c.
 *  @param[in]   void
 *  @Constraints None
 *  @Reentrant   Yes
 *  @return      uint32_t Returns the clock frequency .
 */
uint32_t LPI2C0_GetFreq(void)
{
    /* Attach peripheral clock */
    CLOCK_SetClockDiv(kCLOCK_DivLPI2C0, 1u);
    CLOCK_AttachClk(kFRO12M_to_LPI2C0);

    return CLOCK_GetLpi2cClkFreq();
}

/*! @brief       Determines the Clock Frequency feature.
 *  @details     The Clock Frequecny computation API required by fsl_spi_cmsis.c.
 *  @param[in]   void
 *  @Constraints None
 *  @Reentrant   Yes
 *  @return      uint32_t Returns the clock frequency .
 */
uint32_t LPSPI1_GetFreq(void)
{
    /* Attach peripheral clock */
    CLOCK_SetClockDiv(kCLOCK_DivLPSPI1, 1u);
    CLOCK_AttachClk(kFRO12M_to_LPSPI1);

    return CLOCK_GetLpspiClkFreq(1);
}
