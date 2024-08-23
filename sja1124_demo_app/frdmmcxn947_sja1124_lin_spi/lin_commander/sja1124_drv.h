/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SJA1124_DRV_H_
#define SJA1124_DRV_H_

/* Standard C Includes */
#include <stdint.h>

/* ISSDK Includes */
#include "sensor_io_spi.h"
#include "register_io_spi.h"
#include "../lin_commander/sja1124.h"

//-----------------------------------------------------------------------
// CMSIS Includes
//-----------------------------------------------------------------------
#include "Driver_GPIO.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/**
 * @brief Maximum number of channels per device.
 */

#define BOARD_PWM_BASEADDR 				(FLEXPWM0)
#define PWM_SRC_CLK_FREQ 				(CLOCK_GetFreq(kCLOCK_BusClk))
#define DEMO_PWM_CLOCK_DEVIDER			(kPWM_Prescale_Divide_4)
#define DEMO_PWM_FAULT_LEVEL   			true

#define MAX_LIN_CHANNELS		    (4U)
#define WAKEUP_CHARACTER		    (0xAAU)
#define SJA1124_LIN1C_WAKEUPREQ_MASK	    (0x10u)
#define SJA1124_MAX_DEVICE_COUNT	    (4)

/*! @def    SJA1124_SPI_MAX_MSG_SIZE
 *  @brief  The MAX size of SPI message. */
#define SJA1124_SPI_MAX_MSG_SIZE (64)

/*! @def    SJA1124_SPI_CMD_LEN
 *  @brief  The size of the Transceiver specific SPI Header. */
#define SJA1124_SPI_CMD_LEN (2)

/*! @def    SJA1124_SS_ACTIVE_VALUE
 *  @brief  Is the Slave Select Pin Active Low or High. */
#define SJA1124_SS_ACTIVE_VALUE SPI_SS_ACTIVE_LOW

/**
 * @brief Sets to 1 an interrupt given by Interrupt parameter.
 *
 * @param[in] InterruptsList     Should be one of Lin_sja1124_TopLevelInterruptsType or
 *                               Lin_sja1124_SecondLevelInterruptsType.
 * @param[in] Interrupt          Bit shift defined either in Lin_sja1124_TopLevelIntType or
 *                               Lin_sja1124_SecondLevelIntType enums.
 */
#define SET_INTERRUPT(InterruptsList, Interrupt) ((InterruptsList) | (0x1U << ((uint32_t)(Interrupt))))

/**
 * @brief Returns false if interrupt is not set and true if interrupt is set. This macro can
 *        be used for both top and second level itnerrupts.
 *
 * @param[in] InterruptsList     Should be one of Lin_sja1124_TopLevelInterruptsType or
 *                               Lin_sja1124_SecondLevelInterruptsType.
 * @param[in] Interrupt          Bit shift defined either in Lin_sja1124_TopLevelIntType or
 *                               Lin_sja1124_SecondLevelIntType enums.
 */
#define IS_INTERRUPT_SET(InterruptsList, Interrupt)     ((int32_t)(((InterruptsList) >> ((uint32_t)(Interrupt))) & 0x1U))

/**
 * @brief Calculates address offset from base register address of given LIN channel.
 *
 * @param[in] BaseAddress        Address of register which serves as a base for offset calculation.
 * @param[in] Channel            Offset from base register.
 */
#define SJA1124_CHAN_ADDR_OFFSET(BaseAddress, Channel)          ((BaseAddress) + (0x30U * (uint8_t)(Channel)))

#define SJA1124_HS_BAUDRATE 	(20000)

/*!
 * @brief This defines the Transceiver specific information for SPI.
 */
typedef struct
{
	registerDeviceInfo_t deviceInfo;      /*!< SPI device context. */
	ARM_DRIVER_SPI *pCommDrv;             /*!< Pointer to the spi driver. */
	bool isInitialized;                   /*!< Whether Transceiver is initialized or not.*/
	spiSlaveSpecificParams_t slaveParams; /*!< Slave Specific Params.*/
} Lin_sja1124_spi_transceiverhandle_t;

/*==================================================================================================
 *                                              ENUMS
==================================================================================================*/

/**
 * @brief            Enumeration of possible wake up reasons detected by the LIN transceiver.
 * @details          Underlying HW transceiver can detected these wake up reasons: LINTRCV_WU_BY_BUS,
 *                   LINTRCV_WU_INTERNALLY, otherwise reports LINTRCV_WU_ERROR.
 * @implements       LinTrcv_TrcvWakeupReasonType_enum
 */
typedef enum
{
	LINTRCV_WU_ERROR,           /**< @brief Due to an error wake up reason was not detected. */
	LINTRCV_WU_NOT_SUPPORTED,   /**< @brief The transceiver does not support any information
                                            for the wake up reason. */
	LINTRCV_WU_BY_BUS,          /**< @brief The transceiver has detected, that the network has
                                            caused the wake up of the ECU. */
	LINTRCV_WU_BY_PIN,          /**< @brief The transceiver has detected a wake-up event at one
                                            of the transceiver's pins (not at the LIN bus). */
	LINTRCV_WU_INTERNALLY,      /**< @brief The transceiver has detected, that the network has
                                            been woken up by the ECU via a request to NORMAL mode. */
	LINTRCV_WU_RESET,           /**< @brief The transceiver has detected, that the wake up is due
                                            to an ECU reset. */
	LINTRCV_WU_POWER_ON         /**< @brief The transceiver has detected, that the wake up is due
                                            to an ECU reset after power on. */
} LinTrcv_TrcvWakeupReasonType;

/**
 * @brief            Wake up operating modes of the LIN Transceiver Driver.
 * @details
 * @implements       LinTrcv_TrcvWakeupModeType_enum
 */
typedef enum
{
	LINTRCV_WUMODE_ENABLE,      /**< @brief The notification for wakeup events is enabled
                                            on the addressed network. */
	LINTRCV_WUMODE_DISABLE,     /**< @brief The notification for wakeup events is disabled
                                            on the addressed network. */
	LINTRCV_WUMODE_CLEAR        /**< @brief A stored wakeup event is cleared on the addressed network. */
} LinTrcv_TrcvWakeupModeType;


/**
 * @brief Enumeration of possible results of command execution.
 */
typedef enum
{
	/** @brief No error. */
	SJA1124_SUCCESS         = 0,
	/** @brief General error, command failed to execute task successfully. */
	SJA1124_FAIL,
	/** @brief SPI communication error. */
	SJA1124_ERR_SPI,
	/** @brief Device has not been initialized (used at device configuration). */
	SJA1124_ERR_INIT,
	/** @brief Wrong parameter. */
	SJA1124_ERR_PARAM
} Lin_sja1124_StatusType;

/**
 * @brief Enumeration of LIN channels.
 */
typedef enum
{
	/** @brief LIN CHANNEL 1. */
	LIN_CHANNEL_1           = 0,
	/** @brief LIN CHANNEL 2. */
	LIN_CHANNEL_2,
	/** @brief LIN CHANNEL 3. */
	LIN_CHANNEL_3,
	/** @brief LIN CHANNEL 4. */
	LIN_CHANNEL_4
} Lin_sja1124_LinChannelType;

/**
 * @brief State of LIN channel.
 */
typedef enum
{
	/** @brief LIN channel in sleep mode. */
	STATE_SLEEP           = 0,
	/** @brief LIN channel in initialization mode. */
	STATE_INITIALIZATION,
	/** @brief LIN channel idle. */
	STATE_IDLE,
	/** @brief Sync break transmission ongoing. */
	STATE_BREAK_TRANS_ONGOING,
	/** @brief Sync break transmission completed and delimiter transmission is ongoing. */
	STATE_BREAK_COMPLETE_DELIMITER_ONGOING,
	/** @brief Sync field transmission ongoing. */
	STATE_FIELD_TRANS_ONGOING,
	/** @brief Identifier transmission ongoing. */
	STATE_ID_TRANS_ONGOING,
	/** @brief Header transmitted. */
	STATE_HEADER_TRANS_COMPLETE,
	/** @brief Response reception ongoing in receiver mode; response transmission ongoing in transmitter mode. */
	STATE_RESPONSE_TRANS_ONGOING,
	/** @brief Data transmission/reception completed, checksum transmission/reception ongoing. */
	STATE_DATA_COMPLETE_CHECKSUM_ONGOING
} Lin_sja1124_LinChannelStateType;

/**
 * @brief Receiver state.
 */
typedef enum
{
	/** @brief Receiver is idle. */
	RECEIVER_IDLE           = 0,
	/** @brief LIN header has been received, reception ongoing. */
	RECEIVER_BUSY,
} Lin_sja1124_ReceiverStateType;

/**
 * @brief LIN master break length.
 */
typedef enum
{
	/** @brief Master break length 10 bits. */
	MASTER_BREAK_LENGTH_10_BITS     = 0,
	/** @brief Master break length 11 bits. */
	MASTER_BREAK_LENGTH_11_BITS,
	/** @brief Master break length 12 bits. */
	MASTER_BREAK_LENGTH_12_BITS,
	/** @brief Master break length 13 bits. */
	MASTER_BREAK_LENGTH_13_BITS,
	/** @brief Master break length 14 bits. */
	MASTER_BREAK_LENGTH_14_BITS,
	/** @brief Master break length 15 bits. */
	MASTER_BREAK_LENGTH_15_BITS,
	/** @brief Master break length 16 bits. */
	MASTER_BREAK_LENGTH_16_BITS,
	/** @brief Master break length 17 bits. */
	MASTER_BREAK_LENGTH_17_BITS,
	/** @brief Master break length 18 bits. */
	MASTER_BREAK_LENGTH_18_BITS,
	/** @brief Master break length 19 bits. */
	MASTER_BREAK_LENGTH_19_BITS,
	/** @brief Master break length 20 bits. */
	MASTER_BREAK_LENGTH_20_BITS,
	/** @brief Master break length 21 bits. */
	MASTER_BREAK_LENGTH_21_BITS,
	/** @brief Master break length 22 bits. */
	MASTER_BREAK_LENGTH_22_BITS,
	/** @brief Master break length 23 bits. */
	MASTER_BREAK_LENGTH_23_BITS,
	/** @brief Master break length 36 bits. */
	MASTER_BREAK_LENGTH_36_BITS,
	/** @brief Master break length 50 bits. */
	MASTER_BREAK_LENGTH_50_BITS
} Lin_sja1124_MasterBreakLenType;

/**
 * @brief Delimiter type: 2 bit, 1 bit.
 */
typedef enum
{
	/** @brief 1 bit delimiter. */
	DELIMITER_1_BIT         = 0,
	/** @brief 2 bit delimiter. */
	DELIMITER_2_BIT
} Lin_sja1124_DelimiterType;

/**
 * @brief PLL multiplication factor. Each range of input frequency has its
 *        own value of multiplication factor to provide required output
 *        frequency. Select a value corresponding to the oscillator used
 *        in the design.
 */
typedef enum
{
	/** @brief M = 78, Fin = 0.4-0.5 MHz. */
	MULT_FACTOR_78         = 0,
	/** @brief M = 65, Fin = 0.5-0.7 MHz. */
	MULT_FACTOR_65,
	/** @brief M = 39, Fin = 0.7-1.0 MHz. */
	MULT_FACTOR_39,
	/** @brief M = 28, Fin = 1.0-1.4 MHz. */
	MULT_FACTOR_28,
	/** @brief M = 20, Fin = 1.4-1.9 MHz. */
	MULT_FACTOR_20,
	/** @brief M = 15, Fin = 1.9-2.6 MHz. */
	MULT_FACTOR_15,
	/** @brief M = 11, Fin = 2.6-3.5 MHz. */
	MULT_FACTOR_11,
	/** @brief M = 8.5, Fin = 3.5-4.5 MHz. */
	MULT_FACTOR_8_5,
	/** @brief M = 6.4, Fin = 4.5-6.0 MHz. */
	MULT_FACTOR_6_4,
	/** @brief M = 4.8, Fin = 6.0-8.0 MHz. */
	MULT_FACTOR_4_8,
	/** @brief M = 3.9, Fin = 8.0-10.0 MHz. */
	MULT_FACTOR_3_9
} Lin_sja1124_MultiplicationFactorType;

/**
 * @brief Some events can reset state machine. This enumerates
 *        what to do when such event occurs.
 */
typedef enum
{
	/** @brief Don't reset state machine on bit error. */
	DO_NOT_RESET_STATE_MACHINE    = 0,
	/** @brief Reset state machine on bit error. */
	RESET_STATE_MACHINE
} Lin_sja1124_OnEventActionType;

/**
 * @brief LIN channel mode: sleep, normal.
 */
typedef enum
{
	/** @brief Normal mode. */
	MODE_NORMAL          = 0,
	/** @brief Sleep mode. */
	MODE_SLEEP
} Lin_sja1124_LinChannelModeType;

/**
 * @brief Stop bit configuration: 1 stop bit, 2 stop bits.
 */
typedef enum
{
	/** @brief 1 stop bit. */
	STOP_BIT_1          = 0,
	/** @brief 2 stop bits. */
	STOP_BIT_2
} Lin_sja1124_StopBitConfigType;

/**
 * @brief Response direction, node can send or receive data.
 */
typedef enum
{
	/** @brief Node receives response. */
	RESPONSE_RECEIVE         = 0,
	/** @brief Node transmits response. */
	RESPONSE_SEND
} Lin_sja1124_ResponseDirType;

/**
 * @brief Checksum type: classic, enhanced.
 */
typedef enum
{
	/** @brief Enhanced checksum. */
	CHECKSUM_ENHANCED         = 0,
	/** @brief Classic checksum. */
	CHECKSUM_CLASSIC
} Lin_sja1124_ChecksumType;

/**
 * @brief Checksum Calculation: HARDWARE, SOFTWARE.
 */
typedef enum
{
	/** @brief Enhanced checksum. */
	HARDWARE_CHECKSUM         = 0,
	/** @brief Classic checksum. */
	SOFTWARE_CHECKSUM
} Lin_sja1124_ChecksumCalc;

/**
 * @brief LIN channel termination configuration.
 */
typedef enum
{
	/** @brief Off in normal and low power modes. */
	OFF_NORMAL_AND_LOW_POWER    = 0,
	/** @brief Rcommander in normal mode and off in low power mode. */
	RCOM_NORMAL_OFF_LOW_POWER,
	/** @brief Rcommander in normal mode and Rcommanderlp in low power mode. */
	RCOM_NORMAL_RCOMLP_LOW_POWER,
	/** @brief Rcommander in normal and low power modes. */
	RCOM_NORMAL_AND_LOW_POWER
} Lin_sja1124_PullUpConfigType;

/**
 * @brief Non-volatile memory status.
 */
typedef enum
{
	/** @brief Device is configured and MTPNVM is protected against overwriting. */
	NVM_LOCKED    = 0,
	/** @brief Device in factory restore state and MTPNVM can be programmed. */
	NVM_UNLOCKED
} Lin_sja1124_NonvolatileMemStatusType;

/**
 * @brief List of top level interrupts.
 */
typedef enum
{
	/** @brief LIN 1 wake up interrupt. */
	INT_LIN1_WAKE_UP        = 0U,
	/** @brief LIN 2 wake up interrupt. */
	INT_LIN2_WAKE_UP,
	/** @brief LIN 3 wake up interrupt. */
	INT_LIN3_WAKE_UP,
	/** @brief LIN 4 wake up interrupt. */
	INT_LIN4_WAKE_UP,
	/** @brief Initialization status interrupt. */
	INT_INIT_STATUS         = 7U,
	/** @brief Low power request fail interrupt. */
	INT_LOW_POWER_REQ_FAIL,
	/** @brief SPI error interrupt. */
	INT_SPI_ERROR,
	/** @brief PLL input frequency fail interrupt. */
	INT_PLL_INPUT_FREQ_FAIL,
	/** @brief PLL in-lock interrupt. */
	INT_PLL_IN_LOCK,
	/** @brief PLL out-of-lock interrupt. */
	INT_PLL_OUT_OF_LOCK,
	/** @brief Over temperature warning interrupt. */
	INT_OVERTEMP_WARN,
	/** @brief LIN 1 controller status interrupt. */
	INT_LIN1_CONTROLLER_STATUS  = 16U,
	/** @brief LIN 2 controller status interrupt. */
	INT_LIN2_CONTROLLER_STATUS,
	/** @brief LIN 3 controller status interrupt. */
	INT_LIN3_CONTROLLER_STATUS,
	/** @brief LIN 4 controller status interrupt. */
	INT_LIN4_CONTROLLER_STATUS,
	/** @brief LIN 1 controller error interrupt. */
	INT_LIN1_CONTROLLER_ERROR,
	/** @brief LIN 2 controller error interrupt. */
	INT_LIN2_CONTROLLER_ERROR,
	/** @brief LIN 3 controller error interrupt. */
	INT_LIN3_CONTROLLER_ERROR,
	/** @brief LIN 4 controller error interrupt. */
	INT_LIN4_CONTROLLER_ERROR,
	/** @brief No interrupt detected. */
	NO_TOP_LEVEL_INT
} Lin_sja1124_TopLevelIntType;

typedef enum
{
	LIN_NOT_OK = 0,     /**< @brief Development or production error
                                    occurred.*/
	LIN_TX_OK,          /**< @brief Successful transmission.*/
	LIN_TX_BUSY,        /**< @brief Ongoing transmission (Header or
                                    Response).*/
	LIN_TX_HEADER_ERROR,/**< @brief Erroneous header transmission such
                                    as:
                                     - Mismatch between sent and read
                                       back data
                                     - Identifier parity error
                                     - Physical bus error.*/
	LIN_TX_ERROR,       /**< @brief Erroneous transmission such as:
                                     - Mismatch between sent and read
                                       back data
                                     - Physical bus error.*/
	LIN_RX_OK,          /**< @brief Reception of correct response.*/
	LIN_RX_BUSY,        /**< @brief Ongoing reception:
                                    at least one response byte has
                                    been received, but the checksum
                                    byte has not been received.*/
	LIN_RX_ERROR,       /**< @brief Erroneous reception such as:
                                     - Framing error
                                     - Overrun error
                                     - Checksum error
                                     - Short response.*/
	LIN_RX_NO_RESPONSE, /**< @brief No response byte has been received
                                    so far.*/
	/**< @brief This is a mess !!
                                   Frame status is mixed with channel
                                   status but i kept it here only
                                   because of LIN168.*/
	LIN_OPERATIONAL,    /**< @brief Normal operation;
                                     - The related LIN channel is ready
                                       to transmit next header
                                     - No data from previous frame
                                       available (e.g. after
                                       initialization).*/
	LIN_CH_SLEEP        /**< @brief Sleep mode operation;
                                     - In this mode wake-up detection
                                       from slave nodes is enabled.*/
} Lin_StatusType;

/**
 * @brief List of seco	nd level interrupts.
 */
typedef enum
{
	/** @brief Frame error interrupt. */
	INT_FRAME_ERROR         = 0U,
	/** @brief Checksum error interrupt. */
	INT_CHECKSUM_ERROR      = 4U,
	/** @brief Bit error interrupt. */
	INT_BIT_ERROR,
	/** @brief Timeout error interrupt. */
	INT_TIMEOUT_ERROR,
	/** @brief Stuck at zero interrupt. */
	INT_STUCK_AT_ZERO,
	/** @brief Data transmission complete interrupt. */
	INT_DATA_TRANSMISSION_COMPLETE      = 9U,
	/** @brief Data reception complete interrupt. */
	INT_DATA_RECEPTION_COMPLETE,
	/** @brief Data reception buffer not empty flag. This
	 *         bit does not generate and interrupt. */
	INT_DATA_RECEPTION_BUF_NOT_EMPTY    = 14U,
	/** @brief No interrupt detected. */
	NO_SECOND_LEVEL_INT
} Lin_sja1124_SecondLevelIntType;

/*==================================================================================================
 *                                  STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/
/**
 * @brief Groups top level interrupts (SPI, PLL, overtemperature warning, global
 *        interrupts of LIN channels: wake up, error and status interrupts). This
 *        type is used for reading and clearing top level interrupts. Each interrupt
 *        has its position (bit shift) defined by Lin_sja1124_TopLevelIntType enum.
 *        Use macros IS_INTERRUPT_SET and SET_INTERRUPT to get/set the value of particular
 *        interrupt.
 */
typedef uint32_t Lin_sja1124_TopLevelInterruptsType;

/**
 * @brief Groups second level interrupts (timeout error, bit error, stuck at zero,
 *        checksum error, transmission/reception complete). This type is used for
 *        reading and clearing second level interrupts. Each interrupt has its
 *        position (bit shift) defined by Lin_sja1124_SecondLevelIntType enum.
 *        Use macros IS_INTERRUPT_SET and SET_INTERRUPT to get/set the value
 *        of particular interrupt.
 */
typedef uint32_t Lin_sja1124_SecondLevelInterruptsType;

/**
 * @brief SPI configuration.
 */
typedef struct
{
	/** @brief SPI instance (non-AUTOSAR) / SPI channel (AUTOSAR). */
	uint32_t SpiChannel;
	/** @brief SPI External Device (non-AUTOSAR) / SPI sequence (AUTOSAR). */
	const void* SpiExtension;
	/** @brief SPI status pin configuration. */
	const void* StatPin;
} Lin_sja1124_SpiConfigType;


/**
 * @brief Configuration of global interrupts of LIN controller.
 */
typedef struct
{
	/** @brief LIN channel wake up interrupt enable. */
	int WakeupIntEn;
	/** @brief LIN controller error interrupt enable. */
	int ControllerErrIntEn;
	/** @brief LIN controller status interrupt enable. */
	int ControllerStatusIntEn;
} Lin_sja1124_LinGlobalIntConfigType;

/**
 * @brief Top level interrupts configuration (valid for whole device and LIN controller).
 */
typedef struct
{
	/** @brief Overtemperature interrupt enable. */
	int OvertempIntEn;
	/** @brief PLL out-of-lock interrupt enable. */
	int PllOutOfLockIntEn;
	/** @brief PLL in-lock interrupt enable. */
	int PllInLockIntEn;
	/** @brief PLL input frequency fail interrupt enable. */
	int PllInFreqFailIntEn;
	/** @brief SPI error interrupt enable. */
	int SpiErrIntEn;
	/** @brief Configuration of global interrupts of LIN controller. */
	Lin_sja1124_LinGlobalIntConfigType LinGlobalIntConfig[MAX_LIN_CHANNELS];
} Lin_sja1124_TopLevelIntConfigType;

/**
 * @brief Second level interrupts. These interrupts are related to a LIN controller
 *        of a particular channel.
 */
typedef struct
{
	/** @brief Stuck at 0 interrupt enable. */
	int StuckAtZeroIntEn;
	/** @brief Timeout interrupt enable. */
	int TimeoutIntEn;
	/** @brief Bit error interrupt enable. */
	int BitErrIntEn;
	/** @brief Checksum error interrupt enable. */
	int ChecksumErrIntEn;
	/** @brief Data reception complete interrupt enable. */
	int DataRecepCompleteIntEn;
	/** @brief Data transmission complete interrupt enable. */
	int DataTransCompleteIntEn;
	/** @brief Frame error interrupt enable. */
	int FrameErrIntEn;
} Lin_sja1124_SecondLevelIntConfigType;

/**
 * @brief LIN channel configuration.
 */
typedef struct
{
	/** @brief Enable/disable checksum calculation. */
	int ChecksumCalcEn;
	/** @brief LIN master break length. */
	Lin_sja1124_MasterBreakLenType LinMasterBreakLength;
	/** @brief LIN channel mode select: sleep, normal. */
	Lin_sja1124_LinChannelModeType Mode;
	/** @brief Delimiter length: 2-bit, 1-bit. */
	Lin_sja1124_DelimiterType Delimiter;
	/** @brief On bit error behavior: reset/don't reset state machine. */
	Lin_sja1124_OnEventActionType IdleOnBitErrorEn;
	/** @brief Idle on timeout. */
	Lin_sja1124_OnEventActionType IdleOnTimeoutEn;
	/** @brief Stop bit configuration: 2 stop bits, 1 stop bit. */
	Lin_sja1124_StopBitConfigType StopBitConfig;
	/** @brief Response timeout. */
	uint8_t ResponseTimeout;
	/** @brief Fractional baudrate. */
	uint8_t FractionalBaudrate;
	/** @brief Integer baudrate. */
	uint16_t IntegerBaudrate;
	/** @brief LIN controller interrupts configuration (second level interrupts). */
	Lin_sja1124_SecondLevelIntConfigType SecondLevelIntConfig;
} Lin_sja1124_LinChannelConfigType;

/**
 * @brief This structure holds state of LIN channel and LIN receiver.
 */
typedef struct
{
	/** @brief LIN channel state. */
	Lin_sja1124_LinChannelStateType LinChannelState;
	/** @brief LIN receiver state. */
	Lin_sja1124_ReceiverStateType LinReceiverState;
} Lin_sja1124_LinStateType;

/**
 * @brief LIN frame structure.
 */
typedef struct
{
	/** @brief LIN message ID. */
	uint8_t Pid;
	/** @brief Number of bytes to send (1..8 bytes). */
	uint8_t DataFieldLength;
	/** @brief Direction of LIN frame response. */
	Lin_sja1124_ResponseDirType ResponseDir;
	/** @brief Checksum type: enhanced, classic. */
	Lin_sja1124_ChecksumType ChecksumType;
	/** @brief Calculated checksum, if SW checksum calculation is enabled. */
	uint8_t Checksum;
	/** @brief Data bytes. */
	uint8_t* Data;
} Lin_sja1124_LinFrameType;

/**
 * @brief LIN received frame structure.
 */
typedef struct
{
	/** @brief Checksum received from the LIN bus. */
	uint8_t Checksum;
	/** @brief Data bytes received from the LIN bus. */
	uint8_t Data[8U];
} Lin_sja1124_ReceivedFrameType;

typedef struct
{
	/** @brief Channel in sleep mode. */
	int32_t IsChannelInSleep;
	/** @brief Stores type of transmission, needed for status reporting. */
	Lin_sja1124_ResponseDirType ResponseDir;
} GetStatusType;


/**
 * @brief Non-volatile memory status.
 */
typedef struct
{
	/** @brief Non-volatile memory status: locked/unlocked. */
	Lin_sja1124_NonvolatileMemStatusType NonvolatileMemoryStatus;
	/** @brief Write cycles counter. */
	uint8_t WriteCyclesCounter;
} Lin_sja1124_NvmStatusType;

/**
 * @brief LIN channel controller configuration.
 */
typedef struct
{
	/** @brief LIN channel high speed mode enable. */
	int HighSpeedModeEn;
	/** @brief LIN channel synchronous frame transmission enable. */
	int SyncFrameTransmissionEn;
} Lin_sja1124_LinChanControllerConfigType;

/**
 * @brief Items of this structure, if set to true, trigger header transmission.
 */
typedef struct
{
	/** @brief Triggers header transmission on LIN channel 1. */
	int Chan1HeaderTransRequest;
	/** @brief Triggers header transmission on LIN channel 2. */
	int Chan2HeaderTransRequest;
	/** @brief Triggers header transmission on LIN channel 3. */
	int Chan3HeaderTransRequest;
	/** @brief Triggers header transmission on LIN channel 4. */
	int Chan4HeaderTransRequest;
} Lin_sja1124_HeaderTransRequestType;

/**
 * @brief Maps LIN channel configuration to physical channel.
 */
typedef struct
{
	/** @brief Specifies HW LIN channel which the configuration is mapped to. */
	Lin_sja1124_LinChannelType LinChannelMapping;
	/** @brief LIN channel configuration. */
	Lin_sja1124_LinChannelConfigType LinChannelConfig;
} Lin_sja1124_LinChannelMappingType;

/**
 * @brief List of configured LIN channels.
 */
typedef struct
{
	/** @brief Number of configured LIN channels. */
	uint8_t ConfiguredChannelsNumber;
	/** @brief List of configurations mapped to physical channels. */
	Lin_sja1124_LinChannelMappingType LinChannels[MAX_LIN_CHANNELS];
} Lin_sja1124_LinChannelListType;

/**
 * @brief Device configuration structure.
 */
typedef struct
{
	/** @brief Configuration of top level interrupts
	 * (valid for whole device and LIN controller). */
	Lin_sja1124_TopLevelIntConfigType TopLevelIntConfig;
	/** @brief LIN commander configuration. Settings of high speed mode,
	 *  and synchronized frame transmission. */
	Lin_sja1124_LinChanControllerConfigType ChanControllerConfig[MAX_LIN_CHANNELS];
	/** @brief Multiplication factor configuration. */
	Lin_sja1124_MultiplicationFactorType MultiplicationFactor;
	/** @brief List of configured LIN channels. */
	Lin_sja1124_LinChannelListType LinChannelList;
	/** @brief SPI configuration. */
	Lin_sja1124_spi_transceiverhandle_t* spiHandle;
	/** @brief Lin Commander Freq */
	uint32_t Freq;
} Lin_sja1124_DeviceConfigType;

/**
 * @brief Stores information about device's/driver's status and configuration.
 */
typedef struct
{
	/** @brief Pointer to an array of device configuration structure. */
	const Lin_sja1124_DeviceConfigType* DeviceConfig;
} Lin_sja1124_DriverDataType;

/*==================================================================================================
 *                                  GLOBAL VARIABLE DECLARATIONS
==================================================================================================*/
#ifdef LIN_SJA1124_IP_CONFIG_EXT
/** @brief Device/driver configuration. */
LIN_SJA1124_IP_CONFIG_EXT
#endif

/*==================================================================================================
 *                                       FUNCTION PROTOTYPES
==================================================================================================*/

/**
 * @brief        Initializes the device. For a successful device initialization, the device must be
 *               woken up before calling this function. Function SJA1124_DRV_DriverInit trigerss SPI
 *               wake up and min 2.5 ms must pass for the device to enter initialization mode (to be
 *               ready for configuration). Also no more than (typically) 3 s can pass, otherwise the
 *               device will return to low power mode.
 *
 * @details      Checks bit INITI. If the bit is set, the device is ready for configuration and the
 *               function proceeds with setting PLL, global interrupts, LIN commander and LIN channels.
 *               After writing registers successfully, clears INITI bit to switch to fully operational
 *               normal mode.
 *
 * @param[in]    Device                  Which device to initialize.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
/** @implements   SJA1124_DRV_DeviceInit_Activity */
Lin_sja1124_StatusType SJA1124_DRV_DeviceInit(uint8_t Device, uint32_t Freq, Lin_sja1124_DeviceConfigType* DeviceConfig, uint8_t PowerCycle);

/**
 * @brief        Writes default configurations in SJA1124EVB register defined by its address.
 *
 * @param        configType             different device configurations.
 * @param        handle                 handle to spi transceiver.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_get_default_config(Lin_sja1124_DeviceConfigType* configType, Lin_sja1124_spi_transceiverhandle_t* handle);

/**
 * @brief        Enable Hardware/Software checksum.
 *
 * @param        Device                  Which device to write.
 * @param        LinChannel              Selects LIN channel to enable checksum.
 * @param        ChecksumCalc            Checksum type
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_EnableHwSwChecksumCalc(uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel, Lin_sja1124_ChecksumCalc ChecksumCalc);

/**
 * @brief        Set commander break length.
 *
 * @param        Device                  Which device to write.
 * @param        LinChannel              Selects LIN channel to set commander break length.
 * @param        MBL                     Commander break length type
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_SetMBL(uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel, Lin_sja1124_MasterBreakLenType MBL);

/**
 * @brief        Set Delimiter.
 *
 * @param        Device                  Which device to write.
 * @param        LinChannel              Selects LIN channel to set delimiter.
 * @param        TBDE                    delimiter type
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_SetTBDE(uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel, Lin_sja1124_DelimiterType TBDE);

/**
 * @brief        Set stop bit.
 *
 * @param        Device                  Which device to write.
 * @param        LinChannel              Selects LIN channel to set stop bits.
 * @param        StopBits                stop bit type
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_SetStopBits(uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel, Lin_sja1124_StopBitConfigType StopBits);

/**
 * @brief        Initialize the LIN channel with desired Baud rate.
 *
 * @param        Device                  Which device to write.
 * @param        LinChannel              Selects LIN channel to initialize.
 * @param        DesiredBaudInB          Desired Baud rate
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_ChannelInit(uint8_t Device, uint32_t DesiredBaudInB,  Lin_sja1124_LinChannelType LinChannel);

/**
 * @brief        Writes data to a SJA1124 register defined by its address.
 *
 * @param        Device                  Which device to write.
 * @param        RegAddress              Register address.
 * @param        Data                    Data to write.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_WriteRegister(uint8_t Device, uint8_t RegAddress, uint8_t Data);

/**
 * @brief        Reads one SJA1124 register.
 *
 * @param        Device                  Which device to read from.
 * @param        RegAddress              Register address.
 * @param        Data                    Pointer to memory location where retrieved data is stored.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_ReadRegister(uint8_t Device, uint8_t RegAddress, uint8_t* Data);

/**
 * @brief        Configures top level interrupts.
 *
 * @details      Top level interrupts are: SPI, PLL, overtemperature warning, global interrupts
 *               of LIN channels (channel wake up, error and status interrupts) which are associated
 *               with second level interrupts (timeout error, bit error, stuck at zero, checksum error,
 *               transmission/reception complete) which are configured for each channel separately through
 *               API SJA1124_DRV_ConfigureLinControllerInterrupts or SJA1124_DRV_ConfigureLinChannel.
 *               If a second level interrupt occurres and its corresponding global interrupt is enabled
 *               then this global interrupt bit is set.
 *
 * @param        Device                  Which device to configure.
 * @param        TopLevelIntConfig       Top level interrupts configuration structure.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_ConfigureTopLevelInterrupts(uint8_t Device, Lin_sja1124_TopLevelIntConfigType* TopLevelIntConfig);

/**
 * @brief        Enables/disables high speed mode of a LIN channel.
 *
 * @details      High speed mode supports baud rates higher than 20 kBd.
 *
 * @param        Device                  Device ID.
 * @param        LinChannel              LIN channel ID.
 * @param        EnableHighSpeed         Enables/disables high speed mode.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_EnableHighSpeedMode(uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel, uint8_t EnableHighSpeed);

/**
 * @brief        Enables/disables synchronous transfer for a LIN channel.
 *
 * @details      Configures synchronous transfer feature of given LIN channel, this feature facilitates synchronous
 *               data transfer accross multiple devices.
 *
 * @param        Device                  Device ID.
 * @param        LinChannel              LIN channel ID.
 * @param        EnableSyncTransfer      Enables/disables synchronous transfer.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_EnableSynchronousTransfer(uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel, uint8_t EnableSyncTransfer);

/**
 * @brief        Triggers software reset of the device.
 *
 * @details      All settings (registers) are reset to their default values.
 *
 * @param        Device                  Which device to reset.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_ResetDevice(uint8_t Device);

/**
 * @brief        Puts the device in low power mode.
 *
 * @details      Following conditions must be met for the device to enter low power
 *               mode: all LIN channels must be in LIN Sleep mode, there is no ongoing
 *               LIN frame transmission and no interrupts are pending.
 *
 * @param        Device                  Device ID.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_EnterLowPowerMode(uint8_t Device);

/**
 * @brief        Exit the device from low power mode.
 *
 * @details      Following conditions must be met for the device to exit low power
 *               mode: wake-up request either via SPI or wake-up request via LINx.
 *
 * @param        Device                  Device ID.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_ExitLowPowerMode(uint8_t Device);

/**
 * @brief        Triggers soft reset of the LIN master controller.
 *
 * @details      Resets FSMs, timers, status and error registers (LS and LES) without modifying
 *               the configuration registers.
 *
 *               Note: Resetting LIN controller requires the channel to enter initialization
 *                     mode, which aborts all ongoing communication on the channel. Therefore,
 *                     it is important to check any activity on the wire (using API function
 *                     SJA1124_DRV_GetLinState) before resetting the controller.
 *
 * @param        Device                  Device ID.
 * @param        LinChannel              Which LIN controller to reset.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_ResetLinMasterController(uint8_t Device, Lin_sja1124_LinChannelType LinChannel);

/**
 * @brief        Sets mode of given LIN channel.
 *
 * @details      The channel does not have to be in initialization mode in order
 *               to enter a new mode.
 *
 * @param        Device                  Device ID.
 * @param        LinChannel              Which LIN channel to set.
 * @param        LinChannelMode          New mode of LIN channel.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_SetLinChannelMode( Lin_sja1124_spi_transceiverhandle_t *Handle,
		Lin_sja1124_LinChannelType LinChannel, Lin_sja1124_LinChannelModeType LinChannelMode);

/**
 * @brief        Configures LIN channel.
 *
 * @details      LIN channel must be put in initialization mode before configuration. This function
 *               configures checksum, LIN transmission settings (break length, delimiter type,
 *               baudrate, stop bit), behavior on error events (bit error, timeout) and second
 *               level interrupts.
 *
 * @param        Device                  Device ID.
 * @param        LinChannel              Which LIN channel to configure.
 * @param        LinChannelConfig        Channel configuration structure.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_ConfigureLinChannel(uint8_t Device, uint32_t Freq, uint32_t DesiredBaudInB, uint8_t Idx);

/**
 * @brief        Configures second level interrupts. These interrupts are related to LIN commander
 *               controller of particular channel.
 *
 * @details      Second level interrupts are: timeout error, bit error, stuck at zero, checksum error,
 *               transmission/reception complete. Configuration of these interrupts is in separate function,
 *               unlike other settings, to provide the possibility to quickly turn on/off all interrupt sources.
 *
 * @param        Device                      Device ID.
 * @param        LinChannel                  LIN channel to configure.
 * @param        SecondLevelIntConfig        Second level interrupts configuration structure.
 *
 * @return       Lin_sja1124_StatusType      Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_ConfigureSecondLevelInterrupts(
		uint8_t Device,	Lin_sja1124_LinChannelType LinChannel,	Lin_sja1124_SecondLevelIntConfigType* SecondLevelIntConfig);

/**
 * @brief        Sends LIN frame.
 *
 * @details      Configures LIN channel send frame registers and triggers LIN header transmission
 *               by setting request bit.
 *
 * @param        Device                  Device ID.
 * @param        LinChannel              Selects LIN channel to send data to.
 * @param        LinFrame                LIN frame structure with data to send.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_SendFrame(
		uint8_t Device,	Lin_sja1124_LinChannelType LinChannel, Lin_sja1124_LinFrameType* LinFrame);

/**
 * @brief        Sends LIN Header.
 *
 * @details      Configures LIN channel send Header registers and triggers LIN header transmission
 *               by setting request bit.
 *
 * @param        Device                  Device ID.
 * @param        LinChannel              Selects LIN channel to send data to.
 * @param        LinFrame                LIN frame structure with data to send.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_SendHeader(uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel, Lin_sja1124_LinFrameType* LinFrame);

/**
 * @brief        Read the LIN channel status.
 *
 * @details      Get the LIN channel status either in idle mode, sleep mode, initialization mode and sleep mode.
 *
 * @param        Device                  Device ID.
 * @param        LinChannel              Selects LIN channel to send data to.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_StatusType SJA1124_DRV_GetStatus(uint8_t Device, Lin_sja1124_LinChannelType Channel);

/**
 * @brief        Prepares LIN channel for transfer but does not trigger the transfer.
 *
 * @details      Configures LIN channel send frame registers. This function is meant
 *               to be used in cooperation with SJA1124_DRV_TriggerSyncTransmission.
 *               Each channel of each device which participates in synchronous transfer
 *               must be initialized before triggering synchronous transfer.
 *
 * @param        Device                  Device ID.
 * @param        LinChannel              Selects LIN channel to send data to.
 * @param        LinFrame                LIN frame structure with data to send.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_PrepareChannelTx(uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel, const Lin_sja1124_LinFrameType* LinFrame);

/**
 * @brief        Prepares LIN channel for LIN response reception.
 *
 * @details      Configures expected response length and checksum type.
 *
 * @param[in]    Device                  Device ID.
 * @param[in]    LinChannel              Selects LIN channel to send data to.
 * @param[in]    LinFrame                LIN frame structure with data to send.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_PrepareChannelRx(uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel, const Lin_sja1124_LinFrameType* LinFrame);

/**
 * @brief        Sends wake up request on given LIN channel.
 *
 * @details      Writes the wake up character to LIN buffer data register 1
 *               and sets bit WURQ.
 *
 * @param        Device                  Device ID.
 * @param        LinChannel              Selects LIN channel to send wake up request to.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_SendWakeupRequest(uint8_t Device, Lin_sja1124_LinChannelType LinChannel);

/**
 * @brief        Aborts current message or wake up transmission.
 *
 * @param        Device                  Device ID.
 * @param        LinChannel              Selects LIN channel to send abort request to.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_SendAbortRequest(uint8_t Device, Lin_sja1124_LinChannelType LinChannel);

/**
 * @brief        Triggers LIN header transmission request for all LIN channels which are enabled
 *               in HeaderTransRequest parameter.
 *
 * @param[in]    Device                  Device ID.
 * @param[in]    HeaderTransRequest      Configuration of requests, if header transmission request
 *                                       is enabled on particular channel, it will be sent.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_SendHeaderTransmissionRequest(uint8_t Device,
		const Lin_sja1124_HeaderTransRequestType* HeaderTransRequest);

/**
 * @brief        Reads data and checksum received from the LIN bus and stores them
 *               in the parameter ReceivedFrame.
 *
 * @param        Device              Device ID.
 * @param        LinChannel          Selects LIN channel to read data from.
 * @param        ReceivedFrame       Stores received data and checksum.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_GetReceivedData(uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel, Lin_sja1124_ReceivedFrameType* ReceivedFrame);

/**
 * @brief        Reads PID of received header. This function shall be used in slave mode.
 *
 * @param[in]    Device                  Device ID.
 * @param[in]    LinChannel              Selects LIN channel to read data from.
 * @param[out]   HeaderPid               PID of received header.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_GetHeaderPid(uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel, uint8_t* HeaderPid);

/**
 * @brief        Reads the data received frome responder.
 *
 * @param[in]    Device                  Device ID.
 * @param[in]    LinChannel              Selects LIN channel to read data from.
 * @param[in]    LinFrame                LIN Frame type
 * @param[in]    SecondLevelInt_type     read the second level interrupt type
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_GetData(uint8_t Device, Lin_sja1124_LinChannelType LinChannel,
		Lin_sja1124_LinFrameType* LinFrame,	Lin_sja1124_SecondLevelIntType *SecondLevelInt_type);

/**
 * @brief        Reads top level interrupts (global system and LIN interrupts).
 *
 * @details      Reads registers INT1, INT2 and INT3 which collect global system and LIN interrupts.
 *               Compresses content of these registers in TopLevelInterrupts variable which is of type
 *               Lin_sja1124_TopLevelInterruptsType which groups top level interrupts. Use macro
 *               IS_INTERRUPT_SET to extract value of particular top level interrupt. Top level
 *               interrupts are enumerated in Lin_sja1124_TopLevelIntType enum.
 *
 *               Note: Does not read STATUS register which is a subset of INT2 register.
 *
 * @param        Device                      Device ID.
 * @param        TopLevelInterrupts          Stores top level interrupts.
 *
 * @return       Lin_sja1124_StatusType      Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_GetTopLevelInterrupts(uint8_t Device, Lin_sja1124_TopLevelInterruptsType* TopLevelInterrupts);

/**
 * @brief        Clears top level interrupts which are set in TopLevelInterrupts parameter.
 *
 *               Important: Only interrupts from registers INT1 and INT2 can be cleared.
 *                          Register INT3 is read only. Interrupts from INT3 are cleared
 *                          by clearing associated interrupts in LES and LS registers.
 *                          Following interrupts from INT3 cannot be cleared directly:
 *                          INT_LINx_CONTROLLER_STATUS, INT_LINx_CONTROLLER_ERROR.
 *
 * @details      TopLevelInterrupts parameter groups all top level interrupts, each interrupt
 *               which is set to 1 will be cleared. The value of TopLevelInterrupts can be the
 *               value previously returned by SJA1124_DRV_GetTopLevelInterrupts function or it
 *               can be created using macro SET_INTERRUPT for each interrupt to be cleared.
 *
 *               Note: INITI must be cleared before the INITI idle timeout time expires (typ. 3s).
 *
 * @param        Device                      Device ID.
 * @param        TopLevelInterrupts          Top level interrupts to clear.
 *
 * @return       Lin_sja1124_StatusType      Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_ClearTopLevelInterrupts(uint8_t Device, Lin_sja1124_TopLevelInterruptsType TopLevelInterrupts);

/**
 * @brief        Reads second level interrupts of a LIN controller.
 *
 * @details      Reads LES and LS registers of given LIN controller. Compresses the content
 *               of these two registers in SecondLevelInterrupts variable which is of type
 *               Lin_sja1124_SecondLevelInterruptsType which groups second level interrupts.
 *               Use macro IS_INTERRUPT_SET to extract value of particular second level interrupt.
 *               Second level interrupts are enumerated in Lin_sja1124_SecondLevelIntType enum.
 *
 * @param        Device                      Device ID.
 * @param        LinChannel                  ID of LIN channel to read interrupts from.
 * @param        SecondLevelInterrupts       Stores second level interrupts.
 *
 * @return       Lin_sja1124_StatusType      Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_GetSecondLevelInterrupts(uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel, Lin_sja1124_SecondLevelInterruptsType* SecondLevelInterrupts);

/**
 * @brief        Clears all second level interrupts of given channel which are set in
 *               SecondLevelInterrupts parameter.
 *
 * @details      SecondLevelInterrupts parameter groups all second level interrupts, each
 *               interrupt which is set to 1 will be cleared. The value of SecondLevelInterrupts
 *               can be the value previously returned by SJA1124_DRV_GetSecondLevelInterrupts
 *               function or it can be created using macro SET_INTERRUPT for each interrupt to
 *               be cleared.
 *
 * @param        Device                      Device ID.
 * @param        LinChannel                  LIN channel ID.
 * @param        SecondLevelInterrupts       Second level interrupts to clear.
 *
 * @return       Lin_sja1124_StatusType      Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_ClearSecondLevelInterrupts(uint8_t Device, Lin_sja1124_LinChannelType LinChannel,
		Lin_sja1124_SecondLevelInterruptsType SecondLevelInterrupts);

/**
 * @brief        Globally enables/disables interrupts.
 *
 * @details      If EnableInterrupts parameter is true, enables all interrupts which are enabled
 *               in Lin_sja1124_TopLevelIntConfigType and Lin_sja1124_SecondLevelIntConfigType
 *               configurations. If EnableInterrupts parameter is false, disables all interrupts.
 *
 * @param        Device                  Device ID.
 * @param        EnableInterrupts        Enable/disable interrupts.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_EnableInterrupts(uint8_t Device, int EnableInterrupts);

/**
 * @brief        Returns state of LIN channel and LIN receiver of given channel.
 *
 * @details      Possible states of LIN channel are defined in Lin_sja1124_LinChannelStateType
 *               enum and possible states of LIN receiver are defined in Lin_sja1124_ReceiverStateType.
 *
 * @param        Device                  Device ID.
 * @param        LinChannel              Selects LIN channel to read state from.
 * @param        LinState                Stores LIN state.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_GetLinState(uint8_t Device, Lin_sja1124_LinChannelType LinChannel, Lin_sja1124_LinStateType* LinState);

/**
 * @brief        Read the LIN channel state.
 *
 * @param        Device                  Device ID.
 * @param        LinChannel              Selects LIN channel to read state from.
 *
 * @return       uint8_t                 Return code.
 *
 * @api
 */
uint8_t SJA1124_DRV_GetChannelState(uint8_t Device, Lin_sja1124_LinChannelType LinChannel);

/**
 * @brief        Programs user configuration of channel termination to non-volatile memory.
 *
 * @details      User configuration is loaded from non-volatile memory after every system boot
 *               and reset.
 *               Termination configuration of all 4 channels must be provided through PullUpConfig
 *               parameter.
 *
 * @param        Device                  Device ID.
 * @param        FactoryRestoreEnable    Enable/disable factory restore feature.
 * @param        PullUpConfig            Pointer to an array with configurations of channel
 *                                       termination. Configuration of all 4 channels must
 *                                       be provided.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_ProgramNonvolatileMemory(uint8_t Device,
		int FactoryRestoreEnable, const Lin_sja1124_PullUpConfigType PullUpConfig[]);

/**
 * @brief        Reads status of non-volatile memory and write counter.
 *
 * @details      Non-volatile memory can be in locked or unlocked state.
 *
 * @param        Device                  Device ID.
 * @param        NvmStatus               Stores status of non-volatile memory and write counter.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_GetNonvolatileMemStatus(uint8_t Device, Lin_sja1124_NvmStatusType* NvmStatus);

/**
 * @brief        Restores factory preset values in non-volatile memory.
 *
 * @details      Enables factory restore feature and restores factory preset values. Typically,
 *               1 s delay between writing MCFG register and CRC value is needed to restore
 *               factory settings.
 *               The SJA1124 performs a system reset after the factory preset values have been
 *               restored.
 *
 * @param        Device                  Device ID.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_RestoreFactoryConfig(uint8_t Device);

/**
 * @brief        Calculates checksum of LIN frame.
 *
 * @details      Calculates classic checksum from data provided by Data pointer if classic checksum
 *               type is selected. If enhanced type is needed, calculates checksum from data and Pid.
 *
 * @param[in]    ChecksumType    Type of checksum (enhanced or classic).
 * @param[in]    Pid             Protected identifier (used for calculation only for enhanced type).
 * @param[in]    Data            Pointer to data to calculate checksum from.
 * @param[in]    Length          Number of bytes which Data pointer points to.
 *
 * @return       uint8_t           Checksum value.
 *
 * @api
 */
uint8_t SJA1124_DRV_CalculateChecksum(Lin_sja1124_ChecksumType ChecksumType, uint8_t Pid, uint8_t* Data, uint8_t Length);

/**
 * @brief        Time delay function used for proper non-volatile memory programming.
 *
 * @param        DelayMs         Delay duration in milliseconds.
 *
 * @return       void
 */
void SJA1124_DRV_Delay(const uint32_t DelayMs);

/**
 * @brief        Reads ID register.
 *
 * @param        Device                  Device ID.
 * @param        Id                      Stores ID of the device.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_GetId(uint8_t Device, uint8_t* Id);

/**
 * @brief       Checks device availability. Returns SJA1124_SUCCESS if device is
 *              ready, SJA1124_ERR_SPI if device is not available and SJA1124_FAIL
 *              if fails to returns device availability.
 * @details     Reads value on STAT pin, logical 1 means that device is ready.
 *              Possible conditions under which the device is not available are:
 *              VIO under-voltage, over-temperature and low power modes.
 *
 * @param[in]   Device                     Device ID.
 *
 * @return      Lin_sja1124_StatusType     Return code.
 */
Lin_sja1124_StatusType SJA1124_DRV_CheckSpiStatus(uint8_t Device);

/**
 * @brief        Deinitializes the driver and the device.
 *
 * @param        Device                  Which device to deintialize.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_Deinit(uint8_t Device);

/*! @brief       The interface function to initialize the Transceiver.
 *  @details     This function initializes the sensor and sensor handle.
 *  @param[in]   pSensorHandle handle to the Transceiver.
 *  @param[in]   pBus          pointer to the CMSIS API compatible SPI bus object.
 *  @param[in]   index         the I2C device number.
 *  @param[in]   pSlaveSelect  slave select handle of the device on the bus.
 *  @param[in]   whoami        WHO_AM_I value of the device.
 *  @constraints This should be the first API to be called.
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::SJA1124_SPI_Initialize() returns the status .
 */
int32_t SJA1124_SPI_Initialize(Lin_sja1124_spi_transceiverhandle_t *pSensorHandle,
		ARM_DRIVER_SPI *pBus, uint8_t index, void *pSlaveSelect);

/*! @brief      :  The interface function to set the SPI Idle Task.
 *  @param[in]  :  sja1124_spi_transceiverhandle_t *pSensorHandle, handle to the sensor handle.
 *  @param[in]  :  registeridlefunction_t idleTask, function pointer to the function to execute on SPI Idle Time.
 *  @param[in]  :  void *userParam, the pointer to the user idle ftask parameters.
 *  @return        void.
 *  @constraints   This can be called any number of times only after SJA1124_SPI_Initialize().
 *                 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant    No
 */
void SJA1124_SPI_SetIdleTask(Lin_sja1124_spi_transceiverhandle_t *pSensorHandle,
		registeridlefunction_t idleTask, void *userParam);

/**
 * @brief        Process second level Interrupt.
 *
 * @param        Device                  Device ID.
 * @param        Channel                 LIN Channel to process second level Interrupt
 * @param        SecondLevelInt          Second Level Interrupt type
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType Lin_sja1124_ProcessSecondLevelInterrupt(uint8_t Device,
		Lin_sja1124_LinChannelType Channel, Lin_sja1124_SecondLevelIntType* SecondLevelInt);

/*! @brief       The interface function to configure he sensor.
 *  @details     This function configure the sensor with requested ODR, Range and registers in the register pair array.
 *  @param[in]   pSensorHandle handle to the sensor.
 *  @param[in]   pRegWriteList pointer to the register list.
 *  @constraints This can be called any number of times only after FXLS8974_SPI_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::SJA1124_SPI_Configure() returns the status .
 */
int32_t SJA1124_SPI_Configure(Lin_sja1124_spi_transceiverhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList);

/*! @brief       The interface function to read the sensor data.
 *  @details     This function read the sensor data out from the device and returns raw data in a byte stream.
 *  @param[in]   pSensorHandle handle to the sensor.
 *  @param[in]   pReadList     pointer to the list of device registers and values to read.
 *  @param[out]  pBuffer       buffer which holds raw sensor data.This buffer may be back to back data buffer based
 *               command read in the list.
 *  @constraints This can be called any number of times only after SJA1124_SPI_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::SJA1124_SPI_ReadData() returns the status .
 */
Lin_sja1124_StatusType SJA1124_SPI_ReadData(Lin_sja1124_spi_transceiverhandle_t *pSensorHandle,
		const registerreadlist_t *pReadList, uint8_t *pBuffer);

/*! @brief       The interface function to De Initialize sensor..
 *  @details     This function made Transceiver in a power safe state and de initialize its handle.
 *  @param[in]   pSensorHandle      handle to the sensor.
 *  @constraints This can be called only after after SJA1124_SPI_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::SJA1124_SPI_Deinit() returns the status .
 */
int32_t SJA1124_SPI_Deinit(Lin_sja1124_spi_transceiverhandle_t *pSensorHandle);

/*! @brief       The SPI Read Pre-Process function to generate Transceiver specific SPI Message Header.
 *  @details     This function prepares the SPI Read Command Header with register address and
 *               R/W bit encoded as the Transceiver.
 *  @param[out]  pCmdOut  handle to the output buffer.
 *  @param[in]   offset   the address of the register to start reading from.
 *  @param[in]   size     number of bytes to read.
 *  @constraints None
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      :: None.
 */
void SJA1124_SPI_ReadPreprocess(void *pCmdOut, uint32_t offset, uint32_t size);

/*! @brief       The SPI Write Pre-Process function to generate Transceiver specific SPI Message Header.
 *  @details     This function prepares the SPI Write Command Header with register address and
 *               R/W bit encoded as the Transceiver.
 *  @param[out]  pCmdOut  handle to the output buffer.
 *  @param[in]   offset   the address of the register to start writing from.
 *  @param[in]   size     number of bytes to write.
 *  @constraints None
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      :: None.
 */
void SJA1124_SPI_WritePreprocess(void *pCmdOut, uint32_t offset, uint32_t size, void *pWritebuffer);

/**
 * @brief        Register the sja1124evb interrupt ISR.
 *
 * @param        sja1124_int             pointer of type sja1124
 * @param        sja1124_int_isr         handle to sja1124 interrupt ISR
 *
 * @return       :: None
 *
 * @api
 */
void init_sja1124_register_int(void *sja1124_int, void  (*sja1124_int_isr) (void));

/**
 * @brief        Calculate the baud rate of w.r.t. frequency provided in SJA1124EVB.
 *
 * @param[in]    Device                  Device ID.
 * @param[in]    Freq                    Sja1124 frequency to operate on.
 * @param[in]    DesiredBaudInB          Sja1124 Baud rate.
 * @param[out]   Data                    Buffer to store the calculated baud rate.
 *
 * @return       void                    Return code.
 *
 * @api
 */
void SJA1124_Cal_Baud_Rate_Reg(uint8_t Device, uint32_t Freq, uint32_t DesiredBaudInB, uint8_t* Data);

/**
 * @brief        LIN channel enter into initialization mode.
 *
 * @param[in]    Device                  Device ID.
 * @param[in]    LinChannel              LIN channel to enter into initialization mode.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_EnterLinChannelInitMode(uint8_t Device, Lin_sja1124_LinChannelType LinChannel);

/**
 * @brief        LIN channel leave the initialization mode.
 *
 * @param[in]    Device                  Device ID.
 * @param[in]    LinChannel              LIN channel to leave the initialization mode.
 *
 * @return       Lin_sja1124_StatusType  Return code.
 *
 * @api
 */
Lin_sja1124_StatusType SJA1124_DRV_LeaveLinChannelInitMode(uint8_t Device, Lin_sja1124_LinChannelType LinChannel);

#endif /* SJA1124_DRV_H_ */
