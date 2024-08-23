/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file sja1124_lin_commander_test.c
 *  @brief The sja1124_lin_commander_demo.c file implements the ISSDK SJA1124EVB SPI to Quad LIN Bridge driver
 *         example demonstration for SPI with Interrupt and EDMA mode.
 */

//-----------------------------------------------------------------------
// SDK Includes
//-----------------------------------------------------------------------
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_ctimer.h"

//-----------------------------------------------------------------------
// ISSDK Includes
//-----------------------------------------------------------------------
#include "issdk_hal.h"
#include "gpio_driver.h"
#include "sja1124_drv.h"
#include "systick_utils.h"

//-----------------------------------------------------------------------
// CMSIS Includes
//-----------------------------------------------------------------------
#include "Driver_GPIO.h"

//-----------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------
#define EXAMPLE_LPI2C_DMA_BASEADDR DMA0
#define EXAMPLE_LPI2C_DMA_CLOCK    kCLOCK_GateDMA

#define SJA1124_INT_ISR      GPIO2_IRQHandler

#define SJA1124_DEVICE	2
#define SJA_FRQ_KHZ		1000
#define SJA_FRQ_HZ		(SJA_FRQ_KHZ * 1000)
#define SJA_CHANNEL1_BAUD_RATE	(20000)

#define PID          0x03U
#define SIZE_ARRAY   8U
#define RECEIVE_DATA_SIZE 8U

Lin_sja1124_LinStateType sja1124_linstate_type;
Lin_sja1124_LinFrameType sja1124_linframe_type;
Lin_sja1124_spi_transceiverhandle_t	spiHandle;
Lin_sja1124_DeviceConfigType	deviceConfig;
Lin_sja1124_LinChannelType LinChannel;
Lin_sja1124_TopLevelInterruptsType TLI;
uint8_t LinChannelInit[MAX_LIN_CHANNELS] = {0};

uint8_t swcsm;
uint32_t srcClock_Hz;
uint32_t timerClock;
int32_t i;
ctimer_config_t config;
ctimer_match_config_t matchConfig;
ARM_DRIVER_SPI *pSPIdriver = &SPI_S_DRIVER;
volatile Lin_StatusType LinStatus = LIN_NOT_OK;
uint32_t Desried_Baud_rate;

uint8_t DataToSend[SIZE_ARRAY] = {};
Lin_sja1124_LinFrameType Lin_Master_Response_Frame =
{(uint8_t)PID, (uint8_t)SIZE_ARRAY, RESPONSE_SEND, CHECKSUM_ENHANCED, 0x00, DataToSend};

uint8_t DataToReceive[RECEIVE_DATA_SIZE] = {};
Lin_sja1124_LinFrameType Lin_Slave_Response_Frame =
{(uint8_t)PID, (uint8_t)RECEIVE_DATA_SIZE, RESPONSE_RECEIVE, CHECKSUM_ENHANCED, 0x00, DataToReceive};

volatile uint32_t g_pwmPeriod   = 0U;
volatile uint32_t g_pulsePeriod = 0U;
volatile bool sja1124WakeupIntFlag = false;

//-----------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------
void SJA1124_INT_ISR(void)
{
	/* Clear external interrupt flag. */
	GPIO_GpioClearInterruptFlags(SJA1124_INT.base, 1U << SJA1124_INT.pinNumber);
	sja1124WakeupIntFlag = true;
	SDK_ISR_EXIT_BARRIER;
}

/*!@brief        LIN wake-up request.
 *  @details     check LIN wake-up request from responder end.
 *  @param[in]   None.
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No
 */
static void checkLinWakeupRequest()
{
	PRINTF("\r\nwaiting for LIN wake-up request via responder to exit from low power mode........\r\n");
		/* Check LIN wake-up interrupt flag */
		while(!sja1124WakeupIntFlag);

		SJA1124_DRV_GetTopLevelInterrupts(SJA1124_DEVICE, &TLI);
		SJA1124_DRV_ClearTopLevelInterrupts(SJA1124_DEVICE, TLI);

		if (TLI & (SJA1124_INT1_L4WAKEUPINT_MASK | SJA1124_INT1_L3WAKEUPINT_MASK
				| SJA1124_INT1_L2WAKEUPINT_MASK | SJA1124_INT1_L1WAKEUPINT_MASK))
		{

			SJA1124_DRV_DeviceInit(SJA1124_DEVICE, SJA_FRQ_KHZ, &deviceConfig, 1);

			for(int l = 0; l < MAX_LIN_CHANNELS; l++)
			{
				LinChannelInit[l] = 1;
				SJA1124_DRV_ChannelInit(SJA1124_DEVICE, SJA1124_HS_BAUDRATE, l);
				SJA1124_DRV_SendWakeupRequest(SJA1124_DEVICE, l);
			}
			PRINTF("\r\n\033[32m Successfully exit from Low Power Mode via LIN Wakeup.\033[37m\r\n");
		}
		sja1124WakeupIntFlag = false;
}

status_t CTIMER_GetPwmPeriodValue(uint32_t pwmFreqHz, uint8_t dutyCyclePercent, uint32_t timerClock_Hz)
{
	/* Calculate PWM period match value */
	g_pwmPeriod = (timerClock_Hz / pwmFreqHz) - 1U;

	/* Calculate pulse width match value */
	g_pulsePeriod = (g_pwmPeriod + 1U) * (100 - dutyCyclePercent) / 100;

	return kStatus_Success;
}

/*!@brief        Send LIN Frame.
 *  @details     Send LIN Frame from commander to responder.
 *  @param[in]   LinChannel LIN Channel no to send Frame.
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No
 */
static void SendLinFrame(Lin_sja1124_LinChannelType LinChannel)
{
	uint8_t temp;
	int32_t status;
	Lin_sja1124_SecondLevelInterruptsType SecondLevelInt;
	int retries = MAX_RETRIES;
	LinStatus = LIN_NOT_OK;

	if(! LinChannelInit[LinChannel])
	{
		LinChannelInit[LinChannel] = 1;
		SJA1124_DRV_ChannelInit(SJA1124_DEVICE, Desried_Baud_rate, LinChannel);
		SJA1124_DRV_SendWakeupRequest(SJA1124_DEVICE, LinChannel);
	}

	PRINTF("\r\n********************************\r\n");

    do
	{
		PRINTF("\r\nEnter no of bytes to send (1 to 8): ");
		temp = GETCHAR();
		temp -= 48;
		PRINTF("%d\r\n",temp);
		GETCHAR();
		if(temp < 1 | temp > 8)
			PRINTF("\r\nInvalid Value, Please enter correct value\r\n");
	}while(temp < 1 | temp > 8);

	Lin_Master_Response_Frame.DataFieldLength = temp;
    PRINTF("\r\nEnter data bytes to send one by one:- \r\n");

	for(int p=0; p<temp; p++)
	{
		SCANF("%x", &DataToSend[p]);
		PRINTF("\r\nData[%d] = 0x%x\r\n", p, DataToSend[p]);
	}

	PRINTF("\r\n********************************\r\n");
	SJA1124_DRV_GetLinState(SJA1124_DEVICE, LinChannel, &sja1124_linstate_type);
	if (sja1124_linstate_type.LinChannelState == STATE_IDLE)
	{
		SJA1124_DRV_SendFrame(SJA1124_DEVICE, LinChannel, &Lin_Master_Response_Frame);
		while (LIN_TX_OK != LinStatus && retries--) {
			LinStatus = SJA1124_DRV_GetStatus(SJA1124_DEVICE, LinChannel);
		}
		if(LinStatus == LIN_TX_OK)
		{
			PRINTF("\r\n\033[32m Data is sent Successfully from Commander to Responder.\033[37m\r\n");
		}
		else
		{
			PRINTF("\r\n\033[31m Responder is not ready to receive LIN Frame. Please check on responder end. \033[37m\r\n");
		}
	}
	PRINTF("\r\n********************************\r\n");
}

/*!@brief        Receive LIN Frame.
 *  @details     Receive LIN Frame from responder to commander.
 *  @param[in]   LinChannel LIN Channel no to receive Frame.
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No
 */
static void ReceiveLinFrame(Lin_sja1124_LinChannelType LinChannel)
{
	uint8_t temp;
	int32_t status;
	uint8_t data_receive = 1;
	Lin_sja1124_StatusType Status;
	Lin_sja1124_SecondLevelIntType SecondLevelInt_type;

	if(! LinChannelInit[LinChannel])
	{
		LinChannelInit[LinChannel] = 1;
		SJA1124_DRV_ChannelInit(SJA1124_DEVICE, Desried_Baud_rate, LinChannel);
		SJA1124_DRV_SendWakeupRequest(SJA1124_DEVICE, LinChannel);
	}

	PRINTF("\r\n********************************\r\n");
    do
	{
		PRINTF("\r\nEnter no of bytes to receive (1 to 8): ");
		temp = GETCHAR();
		temp -= 48;
		PRINTF("%d\r\n",temp);
		GETCHAR();
		if(temp < 1 | temp > 8)
			PRINTF("\r\nInvalid Value, Please enter correct value\r\n");
	}while(temp < 1 | temp > 8);
	
	Lin_Slave_Response_Frame.DataFieldLength = temp;

	SJA1124_DRV_GetLinState(SJA1124_DEVICE, LinChannel, &sja1124_linstate_type);

	if (sja1124_linstate_type.LinChannelState == STATE_IDLE)
	{
		SJA1124_DRV_SendHeader(SJA1124_DEVICE, LinChannel, &Lin_Slave_Response_Frame);
		Status = SJA1124_DRV_GetData(SJA1124_DEVICE, LinChannel, &Lin_Slave_Response_Frame, &SecondLevelInt_type);

		if(Status == SJA1124_SUCCESS && SecondLevelInt_type == INT_DATA_RECEPTION_COMPLETE)
		{
			PRINTF("\r\n\033[32m Data received from the responder: \033[37m\r\n");
			for (i = 0; i < Lin_Slave_Response_Frame.DataFieldLength; i++)
				PRINTF("\r\nData[%d] =  0x%x\r\n", i, Lin_Slave_Response_Frame.Data[i]);
			PRINTF("\r\nChecksum = 0x%x\r\n", Lin_Slave_Response_Frame.Checksum);
		}
		else
		{
			switch(SecondLevelInt_type)
			{
			case INT_FRAME_ERROR:
				PRINTF("\r\n\033[31m Framing error (invalid stop bit) detected. \033[37m\r\n");
				break;
			case INT_CHECKSUM_ERROR:
				PRINTF("\r\n\033[31m Checksum error detected. \033[37m\r\n");
				break;
			case INT_BIT_ERROR:
				PRINTF("\r\n\033[31m Bit error detected. \033[37m\r\n");
				break;
			case INT_TIMEOUT_ERROR:
				PRINTF("\r\n\033[31m Timeout error detected. \033[37m\r\n");
				break;
			case INT_STUCK_AT_ZERO:
				PRINTF("\r\n\033[31m Stuck-at-zero timeout error detected. \033[37m\r\n");
				break;
			default:
				PRINTF("\r\n\033[31m Responder has not sent the LIN Frame. Please check on responder end. \033[37m\r\n");
				break;
			}
			PRINTF("\r\n\033[93m Please check the configurations settings on both Commander and Responder end and try again. \033[37m\r\n");
		}
	}
	PRINTF("\r\n********************************\r\n");
}

/*!@brief        Exit Low Power Mode.
 *  @details     sja1124 device exit from low power mode via SPI wakeup or LIN wakeup.
 *  @param[in]   None.
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No
 */
static void ExitLowPowerMode()
{
	SJA1124_DRV_ExitLowPowerMode(SJA1124_DEVICE);

		SJA1124_DRV_DeviceInit(SJA1124_DEVICE, SJA_FRQ_KHZ, &deviceConfig, 1);

		for(int l = 0; l < MAX_LIN_CHANNELS; l++)
		{
			LinChannelInit[l] = 1;
			SJA1124_DRV_ChannelInit(SJA1124_DEVICE, SJA1124_HS_BAUDRATE, l);
			SJA1124_DRV_SendWakeupRequest(SJA1124_DEVICE, l);
		}

		PRINTF("\r\n\033[32m Successfully exit from Low Power Mode via SPI Wake-up.\033[37m\r\n");
}

/*!@brief        Enter Low Power Mode.
 *  @details     sja1124 device will enter into low power mode.
 *  @param[in]   None.
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No
 */
static void EnterLowPowerMode()
{
	uint8_t character;
	sja1124WakeupIntFlag = false;

	SJA1124_DRV_EnterLowPowerMode(SJA1124_DEVICE);
	PRINTF("\r\n\033[31m Successfully entered into Low Power Mode\033[37m\r\n");
	PRINTF("\r\n********************************\r\n");
	PRINTF("\r\nChoose any one option to exit\r\n");
	PRINTF("\r\n1. Wake-up via SPI \r\n");
	PRINTF("\r\n2. Wake-up via LIN Channel\r\n");

	do
	{
		PRINTF("\r\nEnter your choice :- ");
		character = GETCHAR();
		character -= 48;
		PRINTF("%d\r\n",character);
		GETCHAR();
		if(character < 1 | character > 2)
			PRINTF("\r\nInvalid Value, Please enter correct value\r\n");

	}while(character < 1 | character > 2);

	PRINTF("\r\n********************************\r\n");

	switch (character)
	{
	case 1:
		ExitLowPowerMode();
		break;
	case 2:
		checkLinWakeupRequest();
		break;
	default:
		ExitLowPowerMode();
		break;
	}
	PRINTF("\r\n********************************\r\n");
}

/*!@brief        Set Delimiter.
 *  @details     Set Delimiter for corresponding LIN Channel.
 *  @param[in]   None.
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No
 */
static void SetDelimiter()
{
	uint8_t character;
	Lin_sja1124_DelimiterType delimiter;
	uint8_t Data;

	SJA1124_DRV_ReadRegister(SJA1124_DEVICE, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LCFG2, LinChannel), &Data);
	Data = (Data & SJA1124_LIN1CFG2_TWOBITDELIM_MASK) >> SJA1124_LIN1CFG2_TWOBITDELIM_SHIFT;

	switch(Data)
	{
	case DELIMITER_1_BIT:
		PRINTF("\r\nCurrent Delimiter = 1 bit\r\n");
		break;
	case DELIMITER_2_BIT:
		PRINTF("\r\nCurrent Delimiter = 2 bit\r\n");
		break;
	default:
		PRINTF("\r\nError in Getting Delimiter\r\n");
	}

	PRINTF("\r\n1. 1-bit Delimiter\r\n");
	PRINTF("\r\n2. 2-bit Delimiter\r\n");

	do
	{
		PRINTF("\r\nEnter your choice :- ");
		character = GETCHAR();
		character -= 48;
		PRINTF("%d\r\n",character);
		GETCHAR();
		if(character < 1 | character > 2)
			PRINTF("\r\nInvalid Value, Please enter correct value\r\n");
	}while(character < 1 | character > 2);

	switch (character)
	{
	case 1:
		delimiter = DELIMITER_1_BIT;
		break;
	case 2:
		delimiter = DELIMITER_2_BIT;
		break;
	default:
		delimiter = DELIMITER_1_BIT;
		break;
	}

	SJA1124_DRV_SetTBDE(SJA1124_DEVICE, LinChannel, delimiter);

	SJA1124_DRV_ReadRegister(SJA1124_DEVICE, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LCFG2, LinChannel), &Data);
	Data = (Data & SJA1124_LIN1CFG2_TWOBITDELIM_MASK) >> SJA1124_LIN1CFG2_TWOBITDELIM_SHIFT;

	PRINTF("\r\n********************************\r\n");
	switch(Data)
	{
	case DELIMITER_1_BIT:
		PRINTF("\r\nDelimiter is set to 1 bit.\r\n");
		break;
	case DELIMITER_2_BIT:
		PRINTF("\r\nDelimiter is set to 2 bit.\r\n");
		break;
	default:
		PRINTF("\r\nError in Getting Delimiter\r\n");
	}
	PRINTF("\r\n********************************\r\n");
}

/*!@brief        Set Stop Bit.
 *  @details     Set Stop Bit for corresponding LIN Channel.
 *  @param[in]   None.
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No
 */
static void SetStopBit()
{
	uint8_t character;
	Lin_sja1124_StopBitConfigType stopBit;
	uint8_t Data;

	SJA1124_DRV_ReadRegister(SJA1124_DEVICE, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LGC, LinChannel), &Data);

	Data = (Data & SJA1124_LIN1GC_STOPBITCONF_MASK) >> SJA1124_LIN1GC_STOPBITCONF_SHIFT;

	switch(Data)
	{
	case STOP_BIT_1:
		PRINTF("\r\nCurrently Stop Bit = 1\r\n");
		break;
	case DELIMITER_2_BIT:
		PRINTF("\r\nCurrently Stop Bit = 2\r\n");
		break;
	default:
		PRINTF("\r\nError in Getting Stop Bit\r\n");
	}

	PRINTF("\r\n1. One stop bit\r\n");
	PRINTF("\r\n2. Two stop bit\r\n");

	do
	{
		PRINTF("\r\nEnter your choice :- ");
		character = GETCHAR();
		character -= 48;
		PRINTF("%d\r\n",character);
		GETCHAR();
		if(character < 1 | character > 2)
			PRINTF("\r\nInvalid Value, Please enter correct value\r\n");
	}while(character < 1 | character > 2);

	switch (character)
	{
	case 1:
		stopBit = STOP_BIT_1;
		break;
	case 2:
		stopBit = STOP_BIT_2;
		break;
	default:
		stopBit = STOP_BIT_1;
		break;
	}

	SJA1124_DRV_SetStopBits(SJA1124_DEVICE, LinChannel, stopBit);
	SJA1124_DRV_ReadRegister(SJA1124_DEVICE, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LGC, LinChannel), &Data);

	Data = (Data & SJA1124_LIN1GC_STOPBITCONF_MASK) >> SJA1124_LIN1GC_STOPBITCONF_SHIFT;

	PRINTF("\r\n********************************\r\n");
	switch(Data)
	{
	case STOP_BIT_1:
		PRINTF("\r\nStop Bit is set to 1.\r\n");
		break;
	case DELIMITER_2_BIT:
		PRINTF("\r\nStop Bit is set to 2.\r\n");
		break;
	default:
		PRINTF("\r\nError in Getting Stop Bit\r\n");
	}
	PRINTF("\r\n********************************\r\n");
}

/*!@brief        Set Commander Break Length.
 *  @details     Set commander break length for corresponding LIN Channel.
 *  @param[in]   None.
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No
 */
static void SetCommanderBreakLength()
{
	uint8_t character;
	Lin_sja1124_MasterBreakLenType MBL;
	uint8_t Data;

	SJA1124_DRV_ReadRegister(SJA1124_DEVICE, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LCFG1, LinChannel), &Data);
	Data = (Data & SJA1124_LIN1CFG1_MASBREAKLEN_MASK) >> SJA1124_LIN1CFG1_MASBREAKLEN_SHIFT;

	switch(Data)
	{
	case MASTER_BREAK_LENGTH_10_BITS:
		PRINTF("\r\nCurrently Commander Break Length = 10\r\n");
		break;
	case MASTER_BREAK_LENGTH_11_BITS:
		PRINTF("\r\nCurrently Commander Break Length = 11\r\n");
		break;
	case MASTER_BREAK_LENGTH_12_BITS:
		PRINTF("\r\nCurrently Commander Break Length = 12\r\n");
		break;
	case MASTER_BREAK_LENGTH_13_BITS:
		PRINTF("\r\nCurrently Commander Break Length = 13\r\n");
		break;
	case MASTER_BREAK_LENGTH_14_BITS:
		PRINTF("\r\nCurrently Commander Break Length = 14\r\n");
		break;
	case MASTER_BREAK_LENGTH_15_BITS:
		PRINTF("\r\nCurrently Commander Break Length = 15\r\n");
		break;
	case MASTER_BREAK_LENGTH_16_BITS:
		PRINTF("\r\nCurrently Commander Break Length = 16\r\n");
		break;
	case MASTER_BREAK_LENGTH_17_BITS:
		PRINTF("\r\nCurrently Commander Break Length = 17\r\n");
		break;
	case MASTER_BREAK_LENGTH_18_BITS:
		PRINTF("\r\nCurrently Commander Break Length = 18\r\n");
		break;
	case MASTER_BREAK_LENGTH_19_BITS:
		PRINTF("\r\nCurrently Commander Break Length = 19\r\n");
		break;
	case MASTER_BREAK_LENGTH_20_BITS:
		PRINTF("\r\nCurrently Commander Break Length = 20\r\n");
		break;
	case MASTER_BREAK_LENGTH_21_BITS:
		PRINTF("\r\nCurrently Commander Break Length = 21\r\n");
		break;
	case MASTER_BREAK_LENGTH_22_BITS:
		PRINTF("\r\nCurrently Commander Break Length = 22\r\n");
		break;
	case MASTER_BREAK_LENGTH_23_BITS:
		PRINTF("\r\nCurrently Commander Break Length = 23\r\n");
		break;
	case MASTER_BREAK_LENGTH_36_BITS:
		PRINTF("\r\nCurrently Commander Break Length = 36\r\n");
		break;
	case MASTER_BREAK_LENGTH_50_BITS:
		PRINTF("\r\nCurrently Commander Break Length = 50\r\n");
		break;
	default:
		PRINTF("\r\nError in getting Commander Break Length.\r\n");
		break;
	}

	PRINTF("\r\nSelect any one Commander Break Length\r\n");
	PRINTF("\r\n1. 10 bit\r\n");
	PRINTF("\r\n2. 11 bit\r\n");
	PRINTF("\r\n3. 12 bit\r\n");
	PRINTF("\r\n4. 13 bit\r\n");
	PRINTF("\r\n5. 14 bit\r\n");
	PRINTF("\r\n6. 15 bit\r\n");
	PRINTF("\r\n7. 16 bit\r\n");
	PRINTF("\r\n8. 17 bit\r\n");
	PRINTF("\r\n9. 18 bit\r\n");
	PRINTF("\r\n10. 19 bit\r\n");
	PRINTF("\r\n11. 20 bit\r\n");
	PRINTF("\r\n12. 21 bit\r\n");
	PRINTF("\r\n13. 22 bit\r\n");
	PRINTF("\r\n14. 23 bit\r\n");
	PRINTF("\r\n15. 36 bit\r\n");
	PRINTF("\r\n16. 50 bit\r\n");

	do
	{
		PRINTF("\r\nEnter your choice :- ");
		SCANF("%d",&character);
		PRINTF("%d\r\n",character);
		if(character < 1 | character > 16)
			PRINTF("\r\nInvalid Value, Please enter correct value\r\n");

	}while(character < 1 | character > 16);

	switch (character)
	{
	case 1:
		MBL = MASTER_BREAK_LENGTH_10_BITS;
		break;
	case 2:
		MBL = MASTER_BREAK_LENGTH_11_BITS;
		break;
	case 3:
		MBL = MASTER_BREAK_LENGTH_12_BITS;
		break;
	case 4:
		MBL = MASTER_BREAK_LENGTH_13_BITS;
		break;
	case 5:
		MBL = MASTER_BREAK_LENGTH_14_BITS;
		break;
	case 6:
		MBL = MASTER_BREAK_LENGTH_15_BITS;
		break;
	case 7:
		MBL = MASTER_BREAK_LENGTH_16_BITS;
		break;
	case 8:
		MBL = MASTER_BREAK_LENGTH_17_BITS;
		break;
	case 9:
		MBL = MASTER_BREAK_LENGTH_18_BITS;
		break;
	case 10:
		MBL = MASTER_BREAK_LENGTH_19_BITS;
		break;
	case 11:
		MBL = MASTER_BREAK_LENGTH_20_BITS;
		break;
	case 12:
		MBL = MASTER_BREAK_LENGTH_21_BITS;
		break;
	case 13:
		MBL = MASTER_BREAK_LENGTH_22_BITS;
		break;
	case 14:
		MBL = MASTER_BREAK_LENGTH_23_BITS;
		break;
	case 15:
		MBL = MASTER_BREAK_LENGTH_36_BITS;
		break;
	case 16:
		MBL = MASTER_BREAK_LENGTH_50_BITS;
		break;
	default:
		MBL = MASTER_BREAK_LENGTH_10_BITS;
		break;
	}

	SJA1124_DRV_SetMBL( SJA1124_DEVICE, LinChannel, MBL);
	SJA1124_DRV_ReadRegister(SJA1124_DEVICE, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LCFG1, LinChannel), &Data);

	Data = (Data & SJA1124_LIN1CFG1_MASBREAKLEN_MASK) >> SJA1124_LIN1CFG1_MASBREAKLEN_SHIFT;

	PRINTF("\r\n********************************\r\n");
	switch(Data)
	{
	case MASTER_BREAK_LENGTH_10_BITS:
		PRINTF("\r\nCommander Break Length is set to 10 Bit\r\n");
		break;
	case MASTER_BREAK_LENGTH_11_BITS:
		PRINTF("\r\nCommander Break Length is set to 11 Bit\r\n");
		break;
	case MASTER_BREAK_LENGTH_12_BITS:
		PRINTF("\r\nCommander Break Length is set to 12 Bit\r\n");
		break;
	case MASTER_BREAK_LENGTH_13_BITS:
		PRINTF("\r\nCommander Break Length is set to 13 Bit\r\n");
		break;
	case MASTER_BREAK_LENGTH_14_BITS:
		PRINTF("\r\nCommander Break Length is set to 14 Bit\r\n");
		break;
	case MASTER_BREAK_LENGTH_15_BITS:
		PRINTF("\r\nCommander Break Length is set to 15 Bit\r\n");
		break;
	case MASTER_BREAK_LENGTH_16_BITS:
		PRINTF("\r\nCommander Break Length is set to 16 Bit\r\n");
		break;
	case MASTER_BREAK_LENGTH_17_BITS:
		PRINTF("\r\nCommander Break Length is set to 17 Bit\r\n");
		break;
	case MASTER_BREAK_LENGTH_18_BITS:
		PRINTF("\r\nCommander Break Length is set to 18 Bit\r\n");
		break;
	case MASTER_BREAK_LENGTH_19_BITS:
		PRINTF("\r\nCommander Break Length is set to 19 Bit\r\n");
		break;
	case MASTER_BREAK_LENGTH_20_BITS:
		PRINTF("\r\nCommander Break Length is set to 20 Bit\r\n");
		break;
	case MASTER_BREAK_LENGTH_21_BITS:
		PRINTF("\r\nCommander Break Length is set to 21 Bit\r\n");
		break;
	case MASTER_BREAK_LENGTH_22_BITS:
		PRINTF("\r\nCommander Break Length is set to 22 Bit\r\n");
		break;
	case MASTER_BREAK_LENGTH_23_BITS:
		PRINTF("\r\nCommander Break Length is set to 23 Bit\r\n");
		break;
	case MASTER_BREAK_LENGTH_36_BITS:
		PRINTF("\r\nCommander Break Length is set to 36 Bit\r\n");
		break;
	case MASTER_BREAK_LENGTH_50_BITS:
		PRINTF("\r\nCommander Break Length is set to 50 Bit\r\n");
		break;
	default:
		PRINTF("\r\nError in getting Commander Break Length.\r\n");
		break;
	}
	PRINTF("\r\n********************************\r\n");
}

/*!@brief        Enable Check Sum.
 *  @details     Enable hardware/software check sum for corresponding LIN Channel.
 *  @param[in]   None.
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No
 */
static void Enable_Check_sum()
{
	uint8_t character;
	Lin_sja1124_ChecksumCalc checksum;
	uint8_t Data;

	SJA1124_DRV_ReadRegister(SJA1124_DEVICE, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LCFG1, LinChannel), &Data);

	Data = (Data & SJA1124_LIN1CFG1_CSCALSW_MASK) >> SJA1124_LIN1CFG1_CSCALCDIS_SHIFT;

	switch (Data)
	{
	case HARDWARE_CHECKSUM:
		PRINTF("\r\nCurrently Hardware Checksum is enabled.\r\n");
		break;
	case SOFTWARE_CHECKSUM:
		PRINTF("\r\nCurrently Software Checksum is enabled.\r\n");
		break;
	default:
		PRINTF("\r\nError in getting the Checksum configuration\r\n");
		break;
	}

	PRINTF("\r\n1. Hardware Checksum\r\n");
	PRINTF("\r\n2. Software Checksum\r\n");

	do
	{
		PRINTF("\r\nEnter your choice :- ");
		character = GETCHAR();
		character -= 48;
		PRINTF("%d\r\n",character);
		GETCHAR();
		if(character < 1 | character > 2)
			PRINTF("\r\nInvalid Value, Please enter correct value\r\n");

	}while(character < 1 | character > 2);

	switch (character)
	{
	case 1:
		checksum = HARDWARE_CHECKSUM;
		break;
	case 2:
		checksum = SOFTWARE_CHECKSUM;
		break;
	default:
		checksum = HARDWARE_CHECKSUM;
		break;
	}

	SJA1124_DRV_EnableHwSwChecksumCalc(SJA1124_DEVICE, LinChannel, checksum);

	if(checksum == SOFTWARE_CHECKSUM)
	{
		swcsm = SJA1124_DRV_CalculateChecksum( CHECKSUM_ENHANCED , PID, DataToSend, Lin_Master_Response_Frame.DataFieldLength);
		Lin_Master_Response_Frame.Checksum = swcsm;
	}

	SJA1124_DRV_ReadRegister(SJA1124_DEVICE, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LCFG1, LinChannel), &Data);
	Data = (Data & SJA1124_LIN1CFG1_CSCALSW_MASK) >> SJA1124_LIN1CFG1_CSCALCDIS_SHIFT;

	PRINTF("\r\n********************************\r\n");
	switch (Data)
	{
	case HARDWARE_CHECKSUM:
		PRINTF("\r\nEnabled Hardware Checksum.\r\n");
		break;
	case SOFTWARE_CHECKSUM:
		PRINTF("\r\nEnabled Software Checksum.\r\n");
		break;
	default:
		PRINTF("\r\nError in getting the Checksum configuration\r\n");
		break;
	}
	PRINTF("\r\n********************************\r\n");
}

/*!@brief        Get Device Status.
 *  @details     Get status of PLL input frequency status, over-temperature warning and PLL lock status.
 *  @param[in]   None.
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No
 */
static void get_device_status()
{
	uint8_t character, data;

	PRINTF("\r\n1. PLL lock status\r\n");
	PRINTF("\r\n2. Over-temperature warning\r\n");
	PRINTF("\r\n3. PLL input frequency status\r\n");
	PRINTF("\r\n********************************\r\n");

	do
	{
		PRINTF("\r\nEnter your choice :- ");
		character = GETCHAR();
		character -= 48;
		PRINTF("%d\r\n",character);
		GETCHAR();
		if(character < 1 | character > 3)
			PRINTF("\r\nInvalid Value, Please enter correct value\r\n");

	}while(character < 1 | character > 3);

	SJA1124_DRV_ReadRegister(SJA1124_DEVICE, SJA1124_STATUS, &data);

	PRINTF("\r\n********************************\r\n");
	switch (character)
	{
	case 1:
		if(data & SJA1124_INT2_PLLINLOCKINT_MASK)
			PRINTF("\r\nPLL in lock\r\n");
		else
			PRINTF("\r\nPLL out of lock\r\n");
		break;
	case 2:
		if(data & SJA1124_INT2_OVERTMPWARNINT_MASK)
			PRINTF("\r\nChip over-temperature warming thershold exceeded\r\n");
		else
			PRINTF("\r\nChip temperature ok\r\n");
		break;
	case 3:
		if(data & SJA1124_INT2_PLLFRQFAILINT_MASK)
			PRINTF("\r\nPLL input frequency failure (outside defined range)\r\n");
		else
			PRINTF("\r\nPLL input frequency ok (within defined range)\r\n");
		break;
	default:
		PRINTF("\r\nInvalid option selected\r\n");
		break;
	}
	PRINTF("\r\n********************************\r\n");

}

/*!@brief        Get Device ID.
 *  @details     Read the SJA1124EVB device ID.
 *  @param[in]   None.
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No
 */
static void get_device_id()
{
	uint8_t character, data;
	int32_t status;

	PRINTF("\r\n********************************\r\n");
	status = SJA1124_DRV_ReadRegister(SJA1124_DEVICE, SJA1124_OR_ID, &data);
	if (SJA1124_SUCCESS != status)
	{
		PRINTF("\r\nError in getting SJA1124EVB Device ID\r\n");
	}
	else
	{
		PRINTF("\r\nDevice ID = %x\r\n", data);
	}
	PRINTF("\r\n********************************\r\n");
}

/*!@brief        Get Address.
 *  @details     read the register address in hex.
 *  @param[in]   None.
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No
 */
uint8_t get_address()
{
	uint8_t address;
	PRINTF("\r\nEnter register address in HEX :- ");
	SCANF("%x",&address);
	PRINTF("0x%x\r\n",address);

	return address;
}

/*!@brief        Read Register.
 *  @details     read the register data.
 *  @param[in]   addr Hex value of register address
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No
 */
static void ReadRegister(uint8_t addr)
{
	uint8_t data = 0;
	int32_t status;

	status = SJA1124_DRV_ReadRegister(SJA1124_DEVICE, addr, &data);
	if (SJA1124_SUCCESS != status)
	{
		PRINTF("\r\nError in reading the SJA1124EVB device register.\r\n");
	}
	else
	{
		PRINTF("\r\nData = 0x%x\r\n",data);
	}

	PRINTF("\r\n********************************\r\n");
}

/*!@brief        Reset SJA1124EVB.
 *  @details     reset sja1124evb device.
 *  @param[in]   None.
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No
 */
static void Reset_sja1124evb()
{
	SJA1124_DRV_Deinit(SJA1124_DEVICE);

	SJA1124_DRV_DeviceInit(SJA1124_DEVICE, SJA_FRQ_KHZ, &deviceConfig, 1);
	PRINTF("\r\n********************************\r\n");
	PRINTF("\r\n\033[93m Device reset is done successfully. \033[37m \r\n\r\n");

	PRINTF("\r\n\033[32m All LIN Channels initialized with default configurations. \033[37m \r\n");
	PRINTF("\r\n\033[32m Baud Rate-20000, 1 Stop Bit, 1-bit Delimiter, 10 bits Commander Break Length and Hardware Checksum Enabled.\033[37m \r\n");

	for(int l=0; l<MAX_LIN_CHANNELS; l++)
	{
		LinChannelInit[l] = 1;
		SJA1124_DRV_ChannelInit(SJA1124_DEVICE, SJA1124_HS_BAUDRATE, l);
		SJA1124_DRV_SendWakeupRequest(SJA1124_DEVICE, l);
	}
	PRINTF("\r\n********************************\r\n");
}

/*!@brief        Change Baud Rate.
 *  @details     change baud rate for corresponding LIN Channel.
 *  @param[in]   None.
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No
 */
static void ChangeBaudRate()
{
	uint8_t Data[3];
	Lin_sja1124_StatusType Status;

	PRINTF("\r\nEnter Channel Baud Rate in Hz (2000Hz - 20000Hz):- ");
	SCANF("%d",&Desried_Baud_rate);
	PRINTF("%d\r\n",Desried_Baud_rate);

	SJA1124_Cal_Baud_Rate_Reg(SJA1124_DEVICE, SJA_FRQ_KHZ, Desried_Baud_rate, Data);
	Status = SJA1124_DRV_EnterLinChannelInitMode(SJA1124_DEVICE, LinChannel);

	if (SJA1124_SUCCESS == Status)
	{
		SJA1124_DRV_WriteRegister(SJA1124_DEVICE, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LFR, LinChannel), Data[0U]);
		SJA1124_DRV_WriteRegister(SJA1124_DEVICE, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LBRM, LinChannel), Data[1U]);
		SJA1124_DRV_WriteRegister(SJA1124_DEVICE, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LBRL, LinChannel), Data[2U]);
	}

	/* Leave LIN initialization mode. */
	SJA1124_DRV_LeaveLinChannelInitMode(SJA1124_DEVICE, LinChannel);
	PRINTF("\r\nBaud Rate Changed Successfully.\r\n");
}

/*!@brief        Select LIN Channel.
 *  @details     select the particular LIN Channel from 1 to 4.
 *  @param[in]   None.
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No
 */
static void SelectLINChannel()
{
	uint32_t temp;

	PRINTF("\r\n********************************\r\n");
	PRINTF("\r\nSelect LIN Channel\r\n");
	PRINTF("\r\n1. LIN Channel 1\r\n");
	PRINTF("\r\n2. LIN Channel 2\r\n");
	PRINTF("\r\n3. LIN Channel 3\r\n");
	PRINTF("\r\n4. LIN Channel 4\r\n");

	do{
		PRINTF("\r\nEnter your choice :- ");
		temp = GETCHAR();
		temp -= 48;
		PRINTF("%d\r\n",temp);
		GETCHAR();
		if(temp < 1 || temp > 4)
			PRINTF("\r\n Invalid Value, Please enter correct value\r\n");
	} while(temp < 1 || temp > 4);

	PRINTF("\r\nLIN Channel %d Selected\r\n",temp);
	PRINTF("\r\n********************************\r\n");

	switch(temp)
	{
	case 1:
		LinChannel = LIN_CHANNEL_1;
		break;
	case 2:
		LinChannel = LIN_CHANNEL_2;
		break;
	case 3:
		LinChannel = LIN_CHANNEL_3;
		break;
	case 4:
		LinChannel = LIN_CHANNEL_4;
		break;
	default:
		LinChannel = LIN_CHANNEL_1;
		break;
	}
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the The main function implementation.
 *  @details     This function invokes board initializes routines, then then brings up the SPI to Quad LIN bridge and
 *               finally enters an endless loop to continuously read available samples.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints None
 *  @reentrant  No
 *  -----------------------------------------------------------------------*/
int main(void)
{
	int32_t status;
	uint32_t temp;
	uint8_t character;
	char dummy;
	uint8_t address;

#if RTE_SPI1_DMA_EN
	/*  Enable DMA clock. */
	CLOCK_EnableClock(EXAMPLE_LPI2C_DMA_CLOCK);
	edma_config_t edmaConfig = {0};
	EDMA_GetDefaultConfig(&edmaConfig);
	EDMA_Init(EXAMPLE_LPI2C_DMA_BASEADDR, &edmaConfig);
#endif

	/*! Initialize the MCU hardware. */
	BOARD_InitPins();
	BOARD_InitBootClocks();
	BOARD_SystickEnable();
	BOARD_InitDebugConsole();
	RESET_PeripheralReset(kLPSPI1_RST_SHIFT_RSTn);

	PRINTF("\r\n ISSDK SJA1124 SPI to Quad LIN Bridge driver example demonstration.\r\n");

	/*! Initialize the SPI driver. */
	status = pSPIdriver->Initialize(SPI_S_SIGNAL_EVENT);
	if (ARM_DRIVER_OK != status)
	{
		PRINTF("\r\n SPI Initialization Failed\r\n");
		return -1;
	}

	/*! Set the SPI Power mode. */
	status = pSPIdriver->PowerControl(ARM_POWER_FULL);
	if (ARM_DRIVER_OK != status)
	{
		PRINTF("\r\n SPI Power Mode setting Failed\r\n");
		return -1;
	}

	/*! Set the SPI Slave speed. */
	status = pSPIdriver->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA1, SPI_S_BAUDRATE);
	if (ARM_DRIVER_OK != status)
	{
		PRINTF("\r\n SPI Control Mode setting Failed\r\n");
		return -1;
	}

	/*! Initialize the SJA1124  Quad LIN Commander Transceiver driver. */
	status = SJA1124_SPI_Initialize(&spiHandle, &SPI_S_DRIVER, SPI_S_DEVICE_INDEX, &SJA1124_CS);
	if (SENSOR_ERROR_NONE != status)
	{
		PRINTF("\r\nSJA1124 Quad LIN Commander Transceiver Initialization Failed\r\n");
		return -1;
	}
	init_sja1124_register_int(&SJA1124_INT, &SJA1124_INT_ISR);

	/* default configurations of SJA1124 */
	SJA1124_get_default_config(&deviceConfig, &spiHandle);

	/* PWM0_X2 module to generate PLL input frequency clock for SJA1124EVB */
	SJA1124_Start_Pwm(SJA_FRQ_HZ,50);

	SJA1124_DRV_DeviceInit(SJA1124_DEVICE, SJA_FRQ_KHZ, &deviceConfig, 1);
	PRINTF("\r\n\033[32m All LIN Channels initialized with default configurations. \033[37m \r\n");
	PRINTF("\r\n\033[32m Baud Rate-20000, 1 Stop Bit, 1-bit Delimiter, 10 bits Commander Break Length and Hardware Checksum Enabled.\033[37m \r\n");

	for(int l=0; l<MAX_LIN_CHANNELS; l++)
	{
		LinChannelInit[l] = 1;
		SJA1124_DRV_ChannelInit(SJA1124_DEVICE, SJA1124_HS_BAUDRATE, l);
		SJA1124_DRV_SendWakeupRequest(SJA1124_DEVICE, l);
	}

	do
	{
		PRINTF("\r\n");
		PRINTF("\r\n*********** Main Menu ***************\r\n");
		PRINTF("\r\n1. Send/Receive LIN Frame via LIN Channel 1\r\n");
		PRINTF("\r\n2. Send/Receive LIN Frame via LIN Channel 2\r\n");
		PRINTF("\r\n3. Send/Receive LIN Frame via LIN Channel 3\r\n");
		PRINTF("\r\n4. Send/Receive LIN Frame via LIN Channel 4\r\n");
		PRINTF("\r\n5. Change Baud Rate\r\n");
		PRINTF("\r\n6. Enable Checksum\r\n");
		PRINTF("\r\n7. Set Stop Bit\r\n");
		PRINTF("\r\n8. Set Delimiter\r\n");
		PRINTF("\r\n9. Set Commander Break Length\r\n");
		PRINTF("\r\n10. Enter Low Power Mode\r\n");
		PRINTF("\r\n11. Reset SJA1124EVB\r\n");
		PRINTF("\r\n12. SJA1124EVB Device Status\r\n");
		PRINTF("\r\n13. SJA1124EVB Device ID\r\n");
		PRINTF("\r\n14. Read any register\r\n");
		PRINTF("\r\n*************************************\r\n");

		PRINTF("\r\nEnter your choice :- ");
		SCANF("%d",&character);
		PRINTF("%d\r\n",character);

		switch (character)
		{
		case 1:
			PRINTF("\r\n1. Send LIN Frame\r\n");
			PRINTF("\r\n2. Receive LIN Frame\r\n");

			do
			{
				PRINTF("\r\nEnter your choice :- ");
				character = GETCHAR();
				character -= 48;
				PRINTF("%d\r\n",character);
				GETCHAR();
				if(character < 1 | character > 2)
					PRINTF("\r\nInvalid Value, Please enter correct value\r\n");
			}while(character < 1 | character > 2);

			switch(character)
			{
			case 1:
				SendLinFrame(LIN_CHANNEL_1);
				break;
			case 2:
				ReceiveLinFrame(LIN_CHANNEL_1);
				break;
			}
			break;
		case 2:
			PRINTF("\r\n1. Send LIN Frame\r\n");
			PRINTF("\r\n2. Receive LIN Frame\r\n");

			do
			{
				PRINTF("\r\nEnter your choice :- ");
				character = GETCHAR();
				character -= 48;
				PRINTF("%d\r\n",character);
				GETCHAR();
				if(character < 1 | character > 2)
					PRINTF("\r\nInvalid Value, Please enter correct value\r\n");
			}while(character < 1 | character > 2);


			switch(character)
			{
			case 1:
				SendLinFrame(LIN_CHANNEL_2);
				break;
			case 2:
				ReceiveLinFrame(LIN_CHANNEL_2);
				break;
			}
			break;
		case 3:
			PRINTF("\r\n1. Send LIN Frame\r\n");
			PRINTF("\r\n2. Receive LIN Frame\r\n");

			do
			{
				PRINTF("\r\nEnter your choice :- ");
				character = GETCHAR();
				character -= 48;
				PRINTF("%d\r\n",character);
				GETCHAR();
				if(character < 1 | character > 2)
					PRINTF("\r\nInvalid Value, Please enter correct value\r\n");
			}while(character < 1 | character > 2);


			switch(character)
			{
			case 1:
				SendLinFrame(LIN_CHANNEL_3);
				break;
			case 2:
				ReceiveLinFrame(LIN_CHANNEL_3);
				break;
			}
			break;
		case 4:
			PRINTF("\r\n1. Send LIN Frame\r\n");
			PRINTF("\r\n2. Receive LIN Frame\r\n");

			do
			{
				PRINTF("\r\nEnter your choice :- ");
				character = GETCHAR();
				character -= 48;
				PRINTF("%d\r\n",character);
				GETCHAR();
				if(character < 1 | character > 2)
					PRINTF("\r\nInvalid Value, Please enter correct value\r\n");
			}while(character < 1 | character > 2);


			switch(character)
			{
			case 1:
				SendLinFrame(LIN_CHANNEL_4);
				break;
			case 2:
				ReceiveLinFrame(LIN_CHANNEL_4);
				break;
			}
			break;
		case 5:
			SelectLINChannel();
			ChangeBaudRate();
			break;
		case 6:
			SelectLINChannel();
			Enable_Check_sum();
			break;
		case 7:
			SelectLINChannel();
			SetStopBit();
			break;
		case 8:
			SelectLINChannel();
			SetDelimiter();
			break;
		case 9:
			SelectLINChannel();
			SetCommanderBreakLength();
			break;
		case 10:
			EnterLowPowerMode();
			break;
		case 11:
			Reset_sja1124evb();
			break;
		case 12:
			get_device_status();
			break;
		case 13:
			get_device_id();
			break;
		case 14:
			address = get_address();
			ReadRegister(address);
			break;
		default:
			PRINTF("\r\nInvalid option...choose correct one from Main Menu\r\n");
			break;
		}
		PRINTF("\r\nPress Enter to goto Main Menu\r\n");
		do
		{
			dummy = GETCHAR();
		} while(dummy != 13);
	}while(1);

	return 0;
}




