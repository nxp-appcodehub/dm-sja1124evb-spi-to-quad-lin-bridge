/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file sja1124_drv.c
 * @brief The sja1124_drv.c file implements the SJA1124 SPI to Quad LIN driver interfaces.
 */

//-----------------------------------------------------------------------
// ISSDK Includes
//-----------------------------------------------------------------------
#include "../lin_commander/sja1124_drv.h"
#include "frdmmcxn947.h"
#include "gpio_driver.h"
#include "systick_utils.h"
#include "fsl_debug_console.h"

//-----------------------------------------------------------------------
// CMSIS Includes
//-----------------------------------------------------------------------
#include "Driver_GPIO.h"

//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------
#define SJA1124_DELAY	(1000000)  //Approx 100ms Delay on FRDM-MCXN947

uint8_t sja1124_spiRead_CmdBuffer[SJA1124_SPI_MAX_MSG_SIZE] = {0};
uint8_t sja1124_spiRead_DataBuffer[SJA1124_SPI_MAX_MSG_SIZE] = {0};
uint8_t sja1124_spiWrite_CmdDataBuffer[SJA1124_SPI_MAX_MSG_SIZE] = {0};
Lin_sja1124_DeviceConfigType* Sja1124_DeviceList[SJA1124_MAX_DEVICE_COUNT];

GENERIC_DRIVER_GPIO *pGpioDriver = &Driver_GPIO_KSDK;

//-----------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------

static void SJA1124_Delay(uint64_t s_delay)
{
	for(volatile int i = 0; i < s_delay; i++);
}

static void SJA1124_DRV_Copy(uint8_t* Dst, const uint8_t* Src, uint32_t Size)
{
	uint32_t Index;

	for (Index = 0; Index < Size; Index++)
	{
		Dst[Index] = Src[Index];
	}
}

static void SJA1124_DRV_SerializeLinFrame(Lin_sja1124_LinFrameType* LinFrame, uint8_t* Data, uint8_t* BytesCount)
{
	Data[0U] = LinFrame->Pid;
	Data[1U] = (uint8_t) (((uint8_t) LinFrame->ChecksumType << SJA1124_LIN1BC_CLASSICCS_SHIFT) |
			((uint8_t) LinFrame->ResponseDir << SJA1124_LIN1BC_DIRECTION_SHIFT) |
			((uint8_t) (LinFrame->DataFieldLength - 1U) << SJA1124_LIN1BC_DTFIELDLEN_SHIFT));

	/* If CCD = 0 (checksum HW calculation) register LCF is RO and writing to it has not effect. */
	Data[2U] = LinFrame->Checksum;
	*BytesCount = 3U;

	if (RESPONSE_SEND == LinFrame->ResponseDir)
	{
		SJA1124_DRV_Copy(Data + 3U, LinFrame->Data, LinFrame->DataFieldLength);
		*BytesCount += LinFrame->DataFieldLength;
	}
}

static void SJA1124_set_Mult(uint8_t Device, uint32_t Freq, uint8_t* val)
{
	if(Freq >= 400 && Freq < 500)
		*val = MULT_FACTOR_78;
	else if(Freq >= 500 && Freq < 700)
		*val = MULT_FACTOR_65;
	else if(Freq >= 700 && Freq < 1000)
		*val = MULT_FACTOR_39;
	else if(Freq >= 1000 && Freq < 1400)
		*val = MULT_FACTOR_28;
	else if(Freq >= 1400 && Freq < 1900)
		*val = MULT_FACTOR_20;
	else if(Freq >= 1900 && Freq < 2600)
		*val = MULT_FACTOR_15;
	else if(Freq >= 2600 && Freq < 3500)
		*val = MULT_FACTOR_11;
	else if(Freq >= 3500 && Freq < 4500)
		*val = MULT_FACTOR_8_5;
	else if(Freq >= 4500 && Freq < 6000)
		*val = MULT_FACTOR_6_4;
	else if(Freq >= 6000 && Freq < 8000)
		*val = MULT_FACTOR_4_8;
	else
		*val = MULT_FACTOR_3_9;

	Sja1124_DeviceList[Device]->MultiplicationFactor = *val;
	Sja1124_DeviceList[Device]->Freq = Freq;
}

static Lin_sja1124_MultiplicationFactorType SJA1124_get_Mult(uint8_t Device)
{
	return Sja1124_DeviceList[Device]->MultiplicationFactor;
}

void SJA1124_Cal_Baud_Rate_Reg(uint8_t Device, uint32_t Freq, uint32_t DesiredBaudInB, uint8_t* Data)
{
	double temp_val;
	uint16_t ibr;
	Lin_sja1124_MultiplicationFactorType  mult = SJA1124_get_Mult(Device);
	double pll_mult = 0;
	switch(mult)
	{
	case MULT_FACTOR_78:
		pll_mult = 78;
		break;
	case MULT_FACTOR_65:
		pll_mult = 65;
		break;
	case MULT_FACTOR_39:
		pll_mult = 39;
		break;
	case MULT_FACTOR_28:
		pll_mult = 28;
		break;
	case MULT_FACTOR_20:
		pll_mult = 20;
		break;
	case MULT_FACTOR_15:
		pll_mult = 15;
		break;
	case MULT_FACTOR_11:
		pll_mult = 11;
		break;
	case MULT_FACTOR_8_5:
		pll_mult = 8.5;
		break;
	case MULT_FACTOR_6_4:
		pll_mult = 6.4;
		break;
	case MULT_FACTOR_3_9:
		pll_mult = 3.9;
		break;
	default:
		pll_mult=3.9;
	}

	temp_val = (double)Freq * pll_mult;
	Data[0] = (uint8_t) ((uint64_t)(temp_val * 1000 / DesiredBaudInB) % 16);

	ibr = (temp_val*1000 / (16 * DesiredBaudInB));
	Data[1] = (uint8_t) (ibr >> 8U);
	Data[2] = (uint8_t) (ibr & 0xFFU);
}

Lin_sja1124_StatusType SJA1124_DRV_EnterLinChannelInitMode(uint8_t Device, Lin_sja1124_LinChannelType LinChannel)
{
	Lin_sja1124_StatusType Status;
	Lin_sja1124_LinStateType LinState;
	uint8_t Data;
	Lin_sja1124_spi_transceiverhandle_t *Handle = Sja1124_DeviceList[Device]->spiHandle;

	SJA1124_DRV_SetLinChannelMode(Handle, LinChannel, MODE_NORMAL);

	Register_SPI_Read(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams,
			SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LCFG1, LinChannel), SJA1124_REG_SIZE_BYTE, &Data);
	Data = Data & (uint8_t)(~SJA1124_LIN1CFG1_SLEEP_MASK) & (uint8_t)(~SJA1124_LIN1CFG1_INIT_MASK);
	Data = Data | (uint8_t)(SJA1124_LIN1CFG1_INIT_MASK);
	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LCFG1, LinChannel), Data, 0);

	Status = SJA1124_DRV_GetLinState(Device, LinChannel, &LinState);
	if ((STATE_INITIALIZATION != LinState.LinChannelState))
	{
		Status = SJA1124_FAIL;
	}

	return Status;
}

static void SJA1124_DRV_SerializeSecondLevelInterrupts(Lin_sja1124_SecondLevelIntConfigType* SecondLevelInt, uint8_t* Data)
{
	*Data = (uint8_t) (((uint8_t) SecondLevelInt->StuckAtZeroIntEn << SJA1124_LIN1IE_STUCKZEROIEN_SHIFT) |
			((uint8_t) SecondLevelInt->TimeoutIntEn << SJA1124_LIN1IE_TIMEOUTIEN_SHIFT) |
			((uint8_t) SecondLevelInt->BitErrIntEn << SJA1124_LIN1IE_BITERRIEN_SHIFT) |
			((uint8_t) SecondLevelInt->ChecksumErrIntEn << SJA1124_LIN1IE_CSERRIEN_SHIFT) |
			((uint8_t) SecondLevelInt->DataRecepCompleteIntEn << SJA1124_LIN1IE_DTRCVIEN_SHIFT) |
			((uint8_t) SecondLevelInt->DataTransCompleteIntEn << SJA1124_LIN1IE_DTTRANSIEN_SHIFT) |
			((uint8_t) SecondLevelInt->FrameErrIntEn << SJA1124_LIN1IE_FRAMEERRIEN_SHIFT));
}

Lin_sja1124_StatusType SJA1124_DRV_LeaveLinChannelInitMode(uint8_t Device, Lin_sja1124_LinChannelType LinChannel)
{
	uint8_t Data = 0U;
	int32_t status;
	Lin_sja1124_spi_transceiverhandle_t *Handle = Sja1124_DeviceList[Device]->spiHandle;

	status = Register_SPI_Read(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams,
			SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LCFG1, LinChannel), SJA1124_REG_SIZE_BYTE, &Data);

	if (ARM_DRIVER_OK != status)
	{
		return SJA1124_ERR_SPI;
	}

	Data = Data & ~SJA1124_LIN1CFG1_INIT_MASK;
	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LCFG1, LinChannel), Data, 0);

	return SJA1124_SUCCESS;
}

static void SJA1124_DRV_SerializeTopLevelInterrupts(Lin_sja1124_TopLevelIntConfigType* TopLevelInt, uint8_t* Data)
{
	uint8_t Idx;
	Lin_sja1124_LinGlobalIntConfigType* LinGlobalInt;

	LinGlobalInt = TopLevelInt->LinGlobalIntConfig;

	for (Idx = 0U; Idx < MAX_LIN_CHANNELS; Idx++)
	{
		Data[0U] |= (uint8_t) LinGlobalInt[Idx].WakeupIntEn << Idx;
		Data[2U] |= (uint8_t) (((uint8_t) LinGlobalInt[Idx].ControllerStatusIntEn << Idx) |
				((uint8_t) LinGlobalInt[Idx].ControllerErrIntEn << (Idx + 4U)));
	}

	Data[1U] = (uint8_t)(((uint8_t) TopLevelInt->OvertempIntEn << SJA1124_INT2EN_OVERTMPWARNIEN_SHIFT) |
			((uint8_t) TopLevelInt->PllOutOfLockIntEn << SJA1124_INT2EN_PLLNOLOCKIEN_SHIFT) |
			((uint8_t) TopLevelInt->PllInLockIntEn << SJA1124_INT2EN_PLLINLOCKIEN_SHIFT) |
			((uint8_t) TopLevelInt->PllInFreqFailIntEn << SJA1124_INT2EN_PLLFRQFAILIEN_SHIFT) |
			((uint8_t) TopLevelInt->SpiErrIntEn << SJA1124_INT2EN_SPIERRIEN_SHIFT));
}

static void SJA1124_DRV_SerializeLinCommander(uint8_t Device, uint8_t *Data, uint32_t DesiredBaudInB, Lin_sja1124_LinChannelType LinChannel)
{
	uint8_t Sja1124Data;
	Lin_sja1124_spi_transceiverhandle_t *Handle = Sja1124_DeviceList[Device]->spiHandle;
	Lin_sja1124_LinChanControllerConfigType* ChanControllerConfig = &(Sja1124_DeviceList[Device]->ChanControllerConfig[LinChannel]);

	Register_SPI_Read(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_LCOM1, SJA1124_REG_SIZE_BYTE, &Sja1124Data);

	*Data = Sja1124Data;

	*Data = (*Data & ~(1 << LinChannel)) | ((uint8_t) ChanControllerConfig->SyncFrameTransmissionEn << LinChannel);

	// TODO: High Speed Mode Enable feature is not working correctly. As of now, just enable this high speed mode enable bit. Will fix later.
#if 0
	if (DesiredBaudInB != 0 && DesiredBaudInB < SJA1124_HS_BAUDRATE)
		ChanControllerConfig->HighSpeedModeEn = 0;
#endif

	*Data = (*Data & ~(1 << (LinChannel + SJA1124_LCOM1_L1HSMODE_SHIFT))) | ((uint8_t) ChanControllerConfig->HighSpeedModeEn << (LinChannel + SJA1124_LCOM1_L1HSMODE_SHIFT));
}

static Lin_sja1124_StatusType Lin_sja1124_GetStatus(
		uint8_t Device,
		Lin_sja1124_LinChannelType Channel,
		const GetStatusType* GetStatus,
		Lin_StatusType* TransmissionStatus)
{
	Lin_sja1124_StatusType Status;
	Lin_sja1124_LinStateType LinState;
	Lin_sja1124_SecondLevelIntType SecondLevelInt = NO_SECOND_LEVEL_INT;

	Status = Lin_sja1124_ProcessSecondLevelInterrupt(Device, Channel, &SecondLevelInt);

	if (SJA1124_SUCCESS == Status)
	{
		Status = SJA1124_DRV_GetLinState(Device, Channel, &LinState);
	}

	if (SJA1124_SUCCESS == Status)
	{
		switch (SecondLevelInt)
		{
		case INT_DATA_TRANSMISSION_COMPLETE:
			*TransmissionStatus = LIN_TX_OK;
			break;
		case INT_DATA_RECEPTION_COMPLETE:
			*TransmissionStatus = LIN_RX_OK;
			break;
		case INT_FRAME_ERROR:
			/* Frame error is set when a dominant state is sampled on a stop
			 * bit of the currently received character. */
		case INT_CHECKSUM_ERROR:
			/* Checksum error is set when received checksum doesn't match
			 * calculated checksum. */
			*TransmissionStatus = LIN_RX_ERROR;
			break;
		case INT_STUCK_AT_ZERO:
			*TransmissionStatus = \
			(GetStatus->ResponseDir == RESPONSE_RECEIVE) ? LIN_RX_ERROR : LIN_TX_ERROR;
			break;
		case INT_TIMEOUT_ERROR:
			/* Timeout error is set if response timeout expires. */
			*TransmissionStatus = LIN_RX_NO_RESPONSE;
			break;
		case INT_BIT_ERROR:
			/* If bit error flag is set, status of LIN controller is captured. */
			switch (LinState.LinChannelState)
			{
			case STATE_BREAK_TRANS_ONGOING:
			case STATE_BREAK_COMPLETE_DELIMITER_ONGOING:
			case STATE_FIELD_TRANS_ONGOING:
			case STATE_ID_TRANS_ONGOING:
				*TransmissionStatus = LIN_TX_HEADER_ERROR;
				break;
			default:
				*TransmissionStatus = LIN_TX_ERROR;
				break;
			}
			break;
			case NO_SECOND_LEVEL_INT:
			case INT_DATA_RECEPTION_BUF_NOT_EMPTY:
			default:
				/* No error and no transfer complete flags are set,
				 * check LIN channel status. */
				switch (LinState.LinChannelState)
				{
				case STATE_BREAK_TRANS_ONGOING:
				case STATE_BREAK_COMPLETE_DELIMITER_ONGOING:
				case STATE_FIELD_TRANS_ONGOING:
				case STATE_ID_TRANS_ONGOING:
				case STATE_HEADER_TRANS_COMPLETE:
					*TransmissionStatus = LIN_TX_BUSY;
					break;
				case STATE_RESPONSE_TRANS_ONGOING:
				case STATE_DATA_COMPLETE_CHECKSUM_ONGOING:
					*TransmissionStatus = \
					(RESPONSE_RECEIVE == GetStatus->ResponseDir) ? LIN_RX_BUSY : LIN_TX_BUSY;
					break;
				case STATE_SLEEP:
					*TransmissionStatus = LIN_CH_SLEEP;
					break;
				case STATE_IDLE:
					*TransmissionStatus = GetStatus->IsChannelInSleep ? LIN_CH_SLEEP : LIN_OPERATIONAL;
					break;
				case STATE_INITIALIZATION:
				default:
					*TransmissionStatus = LIN_OPERATIONAL;
					break;
				}
				break;
		}
	}
	return Status;
}


/*! -----------------------------------------------------------------------
 *  @brief       Initialize SJA1124 ALert Interrupt Pin and Enable IRQ
 *  @details     This function initializes SJA1124 interrupt pin
 *  @return      void  There is no return value.
 *  -----------------------------------------------------------------------*/
void init_sja1124_register_int(void *sja1124_int, void  (*sja1124_int_isr) (void))
{
	pGpioDriver->pin_init(sja1124_int, GPIO_DIRECTION_IN, NULL, (gpio_isr_handler_t) sja1124_int_isr, NULL);
}

Lin_sja1124_StatusType SJA1124_get_default_config(Lin_sja1124_DeviceConfigType* configType, Lin_sja1124_spi_transceiverhandle_t* handle)
{
	int i = 0;

	configType->spiHandle = handle;
	// default Mult factor
	configType->MultiplicationFactor = MULT_FACTOR_3_9;

	for (i = 0; i < MAX_LIN_CHANNELS; i++) {
		(configType->ChanControllerConfig[i]).HighSpeedModeEn = 1;
		(configType->ChanControllerConfig[i]).SyncFrameTransmissionEn = 1;
	}

	(configType->TopLevelIntConfig).OvertempIntEn = 0;
	(configType->TopLevelIntConfig).PllOutOfLockIntEn = 0;
	(configType->TopLevelIntConfig).PllInLockIntEn = 0;
	(configType->TopLevelIntConfig).PllInFreqFailIntEn = 0;
	(configType->TopLevelIntConfig).SpiErrIntEn = 0;
	for (i = 0; i < MAX_LIN_CHANNELS; i++) {
		((configType->TopLevelIntConfig).LinGlobalIntConfig[i]).WakeupIntEn= 1;
		((configType->TopLevelIntConfig).LinGlobalIntConfig[i]).ControllerErrIntEn= 0;
		((configType->TopLevelIntConfig).LinGlobalIntConfig[i]).ControllerStatusIntEn= 0;
	}

	(configType->LinChannelList).ConfiguredChannelsNumber = MAX_LIN_CHANNELS;
	(configType->LinChannelList).LinChannels[0].LinChannelMapping = LIN_CHANNEL_1;
	(configType->LinChannelList).LinChannels[1].LinChannelMapping = LIN_CHANNEL_2;
	(configType->LinChannelList).LinChannels[2].LinChannelMapping = LIN_CHANNEL_3;
	(configType->LinChannelList).LinChannels[3].LinChannelMapping = LIN_CHANNEL_4;
	for (i = 0; i < MAX_LIN_CHANNELS; i++) {
		(configType->LinChannelList).LinChannels[i].LinChannelConfig.ChecksumCalcEn = 0;
		(configType->LinChannelList).LinChannels[i].LinChannelConfig.LinMasterBreakLength = MASTER_BREAK_LENGTH_13_BITS;
		(configType->LinChannelList).LinChannels[i].LinChannelConfig.Mode = MODE_NORMAL;
		(configType->LinChannelList).LinChannels[i].LinChannelConfig.Delimiter = DELIMITER_2_BIT;
		(configType->LinChannelList).LinChannels[i].LinChannelConfig.IdleOnBitErrorEn = RESET_STATE_MACHINE;
		(configType->LinChannelList).LinChannels[i].LinChannelConfig.IdleOnTimeoutEn = RESET_STATE_MACHINE;
		(configType->LinChannelList).LinChannels[i].LinChannelConfig.StopBitConfig = STOP_BIT_1;
		(configType->LinChannelList).LinChannels[i].LinChannelConfig.ResponseTimeout = 0xe;
		(configType->LinChannelList).LinChannels[i].LinChannelConfig.FractionalBaudrate = 0;
		(configType->LinChannelList).LinChannels[i].LinChannelConfig.IntegerBaudrate = 0;

		(configType->LinChannelList).LinChannels[i].LinChannelConfig.SecondLevelIntConfig.StuckAtZeroIntEn = 0;
		(configType->LinChannelList).LinChannels[i].LinChannelConfig.SecondLevelIntConfig.TimeoutIntEn = 0;
		(configType->LinChannelList).LinChannels[i].LinChannelConfig.SecondLevelIntConfig.BitErrIntEn = 0;
		(configType->LinChannelList).LinChannels[i].LinChannelConfig.SecondLevelIntConfig.ChecksumErrIntEn = 0;
		(configType->LinChannelList).LinChannels[i].LinChannelConfig.SecondLevelIntConfig.DataRecepCompleteIntEn = 0;
		(configType->LinChannelList).LinChannels[i].LinChannelConfig.SecondLevelIntConfig.DataTransCompleteIntEn = 0;
		(configType->LinChannelList).LinChannels[i].LinChannelConfig.SecondLevelIntConfig.FrameErrIntEn = 0;
		//	(configType->LinChannelList).LinChannels[i].LinChannelConfig.SecondLevelIntConfig.StuckAtZeroIntEn = 1;
	}

	return SJA1124_SUCCESS;
}

Lin_sja1124_StatusType SJA1124_DRV_SetLinChannelMode( Lin_sja1124_spi_transceiverhandle_t *Handle,
		Lin_sja1124_LinChannelType LinChannel,
		Lin_sja1124_LinChannelModeType LinChannelMode)
{
	uint8_t Data;

	Register_SPI_Read(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams,
			SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LCFG1, LinChannel), SJA1124_REG_SIZE_BYTE, &Data);
	Data = Data & (uint8_t)(~SJA1124_LIN1CFG1_SLEEP_MASK) & (uint8_t)(~SJA1124_LIN1CFG1_INIT_MASK);
	Data = Data | (uint8_t)(LinChannelMode << SJA1124_LIN1CFG1_SLEEP_SHIFT);
	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LCFG1, LinChannel), Data, 0);

	return SJA1124_SUCCESS;
}

Lin_sja1124_StatusType SJA1124_DRV_GetLinState(uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel,
		Lin_sja1124_LinStateType* LinState)
{
	uint8_t RegisterData = 0U;
	Lin_sja1124_StatusType Status= SJA1124_SUCCESS;
	int32_t status;
	Lin_sja1124_spi_transceiverhandle_t *Handle = Sja1124_DeviceList[Device]->spiHandle;

	status = Register_SPI_Read(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams,
			SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_GS_LSTATE, LinChannel), SJA1124_REG_SIZE_BYTE, &RegisterData);

	if (ARM_DRIVER_OK != status)
	{
		return SJA1124_ERR_SPI;
	}

	LinState->LinChannelState = (Lin_sja1124_LinChannelStateType) (RegisterData & SJA1124_LIN1STATE_LINSTATE_MASK);
	LinState->LinReceiverState = (Lin_sja1124_ReceiverStateType) (RegisterData >> SJA1124_LIN1STATE_RCVBUSY_SHIFT);

	return Status;
}

uint8_t SJA1124_DRV_GetChannelState(uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel)
{
	uint8_t RegisterData = 0U;
	int32_t status;
	Lin_sja1124_spi_transceiverhandle_t *Handle = Sja1124_DeviceList[Device]->spiHandle;

	status = Register_SPI_Read(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams,
			SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_GS_LS, LinChannel), SJA1124_REG_SIZE_BYTE, &RegisterData);
	if (ARM_DRIVER_OK != status)
	{
		return SJA1124_ERR_SPI;
	}

	return RegisterData;
}

Lin_sja1124_StatusType SJA1124_DRV_EnableHighSpeedMode(uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel,
		uint8_t EnableHighSpeed)
{
	uint8_t Data;
	Lin_sja1124_spi_transceiverhandle_t *Handle = Sja1124_DeviceList[Device]->spiHandle;

	Sja1124_DeviceList[Device]->ChanControllerConfig[LinChannel].HighSpeedModeEn = EnableHighSpeed;
	SJA1124_DRV_SerializeLinCommander(Device, &Data, 0, LinChannel);
	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_LCOM1, Data, 0);

	return SJA1124_SUCCESS;
}

Lin_sja1124_StatusType SJA1124_DRV_EnableSynchronousTransfer(uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel,
		uint8_t EnableSyncTransfer)
{
	uint8_t Data;
	Lin_sja1124_spi_transceiverhandle_t *Handle = Sja1124_DeviceList[Device]->spiHandle;

	Sja1124_DeviceList[Device]->ChanControllerConfig[LinChannel].SyncFrameTransmissionEn = EnableSyncTransfer;
	SJA1124_DRV_SerializeLinCommander(Device, &Data, 0, LinChannel);
	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_LCOM1, Data, 0);

	return SJA1124_SUCCESS;
}

Lin_sja1124_StatusType SJA1124_DRV_EnableHwSwChecksumCalc(uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel,
		Lin_sja1124_ChecksumCalc ChecksumCalc)
{
	uint8_t Data;
	Lin_sja1124_spi_transceiverhandle_t *Handle = Sja1124_DeviceList[Device]->spiHandle;

	Sja1124_DeviceList[Device]->LinChannelList.LinChannels[LinChannel].LinChannelConfig.ChecksumCalcEn = ChecksumCalc;

	SJA1124_DRV_EnterLinChannelInitMode(Device, LinChannel);
	Register_SPI_Read(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams,
			SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LCFG1, LinChannel), SJA1124_REG_SIZE_BYTE, &Data);
	Data = Data & (uint8_t)(~SJA1124_LIN1CFG1_CSCALSW_MASK) | (uint8_t)(SJA1124_LIN1CFG1_INIT_MASK);
	Data = Data | (uint8_t)(ChecksumCalc << SJA1124_LIN1CFG1_CSCALCDIS_SHIFT);

	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LCFG1, LinChannel), Data, 0);
	SJA1124_DRV_LeaveLinChannelInitMode(Device, LinChannel);

	return SJA1124_SUCCESS;
}

Lin_sja1124_StatusType SJA1124_DRV_SetMBL(uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel,
		Lin_sja1124_MasterBreakLenType MBL)
{
	uint8_t Data;
	Lin_sja1124_spi_transceiverhandle_t *Handle = Sja1124_DeviceList[Device]->spiHandle;

	Sja1124_DeviceList[Device]->LinChannelList.LinChannels[LinChannel].LinChannelConfig.LinMasterBreakLength = MBL;

	SJA1124_DRV_EnterLinChannelInitMode(Device, LinChannel);
	Register_SPI_Read(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams,
			SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LCFG1, LinChannel), SJA1124_REG_SIZE_BYTE, &Data);
	Data = Data & (uint8_t)(~SJA1124_LIN1CFG1_MASBREAKLEN_MASK) | (uint8_t)(SJA1124_LIN1CFG1_INIT_MASK);
	Data = Data | (uint8_t)(MBL << SJA1124_LIN1CFG1_MASBREAKLEN_SHIFT);

	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LCFG1, LinChannel), Data, 0);
	SJA1124_DRV_LeaveLinChannelInitMode(Device, LinChannel);

	return SJA1124_SUCCESS;
}

Lin_sja1124_StatusType SJA1124_DRV_SetTBDE(uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel,
		Lin_sja1124_DelimiterType TBDE)
{
	uint8_t Data;
	Lin_sja1124_spi_transceiverhandle_t *Handle = Sja1124_DeviceList[Device]->spiHandle;
	uint8_t shift, mask;

	Sja1124_DeviceList[Device]->LinChannelList.LinChannels[LinChannel].LinChannelConfig.Delimiter= TBDE;

	SJA1124_DRV_EnterLinChannelInitMode(Device, LinChannel);
	Register_SPI_Read(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams,
			SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LCFG2, LinChannel), SJA1124_REG_SIZE_BYTE, &Data);

	Data = Data & (uint8_t)(~SJA1124_LIN1CFG2_TWOBITDELIM_MASK);
	Data = Data | (uint8_t)(TBDE << SJA1124_LIN1CFG2_TWOBITDELIM_SHIFT);

	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LCFG2, LinChannel), Data, 0);
	SJA1124_DRV_LeaveLinChannelInitMode(Device, LinChannel);

	return SJA1124_SUCCESS;
}

Lin_sja1124_StatusType SJA1124_DRV_SetStopBits(uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel,
		Lin_sja1124_StopBitConfigType StopBits)
{
	uint8_t Data;
	Lin_sja1124_spi_transceiverhandle_t *Handle = Sja1124_DeviceList[Device]->spiHandle;

	Sja1124_DeviceList[Device]->LinChannelList.LinChannels[LinChannel].LinChannelConfig.StopBitConfig = StopBits;

	SJA1124_DRV_EnterLinChannelInitMode(Device, LinChannel);
	Register_SPI_Read(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams,
			SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LGC, LinChannel), SJA1124_REG_SIZE_BYTE, &Data);
	Data = Data & (uint8_t)(~SJA1124_LIN1GC_STOPBITCONF_MASK);
	Data = Data | (uint8_t)(StopBits << SJA1124_LIN1GC_STOPBITCONF_SHIFT);

	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LGC, LinChannel), Data, 0);
	SJA1124_DRV_LeaveLinChannelInitMode(Device, LinChannel);

	return SJA1124_SUCCESS;
}

Lin_sja1124_StatusType SJA1124_DRV_ConfigureLinChannel(uint8_t Device, uint32_t Freq, uint32_t DesiredBaudInB, uint8_t Idx)
{
	uint8_t Data[9U];
	Lin_sja1124_StatusType Status;
	Lin_sja1124_spi_transceiverhandle_t *Handle = Sja1124_DeviceList[Device]->spiHandle;
	Lin_sja1124_LinChannelType LinChannel = Sja1124_DeviceList[Device]->LinChannelList.LinChannels[Idx].LinChannelMapping;
	Lin_sja1124_LinChannelConfigType* LinChannelConfig = &Sja1124_DeviceList[Device]->LinChannelList.LinChannels[Idx].LinChannelConfig;

	Data[0U] = (uint8_t) (((uint8_t) LinChannelConfig->ChecksumCalcEn << SJA1124_LIN1CFG1_CSCALCDIS_SHIFT) |
			((uint8_t) LinChannelConfig->LinMasterBreakLength << SJA1124_LIN1CFG1_MASBREAKLEN_SHIFT) |
			(uint8_t) SJA1124_LIN1CFG1_INIT_MASK); /* INIT bit must remain set during channel configuration. */
	Data[1U] = (uint8_t) (((uint8_t) LinChannelConfig->Delimiter << SJA1124_LIN1CFG2_TWOBITDELIM_SHIFT) |
			((uint8_t) LinChannelConfig->IdleOnBitErrorEn << SJA1124_LIN1CFG2_IDLEONBITERR_SHIFT));
	Data[2U] = (uint8_t) LinChannelConfig->IdleOnTimeoutEn    << SJA1124_LIN1ITC_IDLEONTIMEOUT_SHIFT;
	Data[3U] = (uint8_t) LinChannelConfig->StopBitConfig      << SJA1124_LIN1GC_STOPBITCONF_SHIFT;
	Data[4U] = (uint8_t) LinChannelConfig->ResponseTimeout;

	SJA1124_Cal_Baud_Rate_Reg(Device, Freq, DesiredBaudInB, &Data[5]);
	SJA1124_DRV_SerializeSecondLevelInterrupts(&LinChannelConfig->SecondLevelIntConfig, &Data[8U]);

	/* Enter LIN initialization mode in order to initialize LIN channel. */
	Status = SJA1124_DRV_EnterLinChannelInitMode(Device, LinChannel);

	if (SJA1124_SUCCESS == Status)
	{
		Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LCFG1, LinChannel), Data[0U], 0);
		Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LCFG2, LinChannel), Data[1U], 0);
		Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LITC, LinChannel), Data[2U], 0);
		Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LGC, LinChannel), Data[3U], 0);
		Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LRTC, LinChannel), Data[4U], 0);
		Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LFR, LinChannel), Data[5U], 0);
		Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LBRM, LinChannel), Data[6U], 0);
		Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LBRL, LinChannel), Data[7U], 0);
		Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LIE, LinChannel), Data[8U], 0);
	}

	/* Leave LIN initialization mode. */
	return (SJA1124_SUCCESS == Status) ? SJA1124_DRV_LeaveLinChannelInitMode(Device, LinChannel) : Status;
}

Lin_sja1124_StatusType SJA1124_DRV_DeviceInit(uint8_t Device, uint32_t Freq, Lin_sja1124_DeviceConfigType* DeviceConfig, uint8_t PowerCycle)
{
	int32_t status;
	Lin_sja1124_StatusType Status = SJA1124_SUCCESS;
	uint8_t sja1124Data[4];
	Lin_sja1124_spi_transceiverhandle_t* Handle;

	Sja1124_DeviceList[Device] = DeviceConfig;
	Handle= Sja1124_DeviceList[Device]->spiHandle;

	if (PowerCycle) {
		SJA1124_DRV_Deinit(Device);
	}
	SJA1124_Delay(SJA1124_DELAY);

	status = Register_SPI_Read(DeviceConfig->spiHandle->pCommDrv, &DeviceConfig->spiHandle->deviceInfo, &DeviceConfig->spiHandle->slaveParams,
			SJA1124_STATUS, SJA1124_REG_SIZE_BYTE, &sja1124Data[0]);
	if (ARM_DRIVER_OK != status)
	{
		return SJA1124_ERR_SPI;
	}
	SJA1124_Delay(SJA1124_DELAY);

	status = Register_SPI_Read(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams,
			SJA1124_INT1, SJA1124_REG_SIZE_BYTE, &sja1124Data[0]);
	if (ARM_DRIVER_OK != status)
	{
		return SJA1124_ERR_SPI;
	}

	if((sja1124Data[0] & SJA1124_INT1_INITSTATINT_MASK) == 0)
	{
		return SJA1124_ERR_INIT;
	}

	SJA1124_set_Mult(Device, Freq, &sja1124Data[0]);
	SJA1124_DRV_SerializeTopLevelInterrupts(&Sja1124_DeviceList[Device]->TopLevelIntConfig, &sja1124Data[1]);
	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_PLLCFG, sja1124Data[0], 0);
	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_INT1EN, sja1124Data[1], 0);
	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_INT2EN, sja1124Data[2], 0);
	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_INT3EN, sja1124Data[3], 0);
	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_INT1, (uint8_t)(1U << SJA1124_INT1_INITSTATINT_SHIFT), 0);

	return Status;
}

Lin_sja1124_StatusType SJA1124_DRV_ChannelInit(uint8_t Device, uint32_t DesiredBaudInB, Lin_sja1124_LinChannelType LinChannel)
{
	uint8_t Idx;
	int32_t status;
	Lin_sja1124_StatusType Status;
	uint8_t sja1124Data;
	Lin_sja1124_spi_transceiverhandle_t* Handle= Sja1124_DeviceList[Device]->spiHandle;

	SJA1124_DRV_SerializeLinCommander(Device, &sja1124Data, DesiredBaudInB, LinChannel);
	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_LCOM1, sja1124Data, 0);

	Status = SJA1124_DRV_ConfigureLinChannel(Device, Sja1124_DeviceList[Device]->Freq, DesiredBaudInB, LinChannel);

	return Status;
}

Lin_sja1124_StatusType SJA1124_DRV_ConfigureTopLevelInterrupts(uint8_t Device, Lin_sja1124_TopLevelIntConfigType* TopLevelIntConfig)
{
	uint8_t Data[3U] = { 0U };
	Lin_sja1124_spi_transceiverhandle_t* Handle= Sja1124_DeviceList[Device]->spiHandle;

	SJA1124_DRV_SerializeTopLevelInterrupts(TopLevelIntConfig, Data);
	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_INT1EN, Data[0], 0);
	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_INT2EN, Data[1], 0);
	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_INT3EN, Data[2], 0);

	return SJA1124_SUCCESS;
}

Lin_sja1124_StatusType SJA1124_DRV_EnterLowPowerMode(uint8_t Device)
{
	Lin_sja1124_spi_transceiverhandle_t* Handle= Sja1124_DeviceList[Device]->spiHandle;
	uint8_t ChannelsNo = Sja1124_DeviceList[Device]->LinChannelList.ConfiguredChannelsNumber;
	Lin_sja1124_TopLevelInterruptsType TopLevelInt;
	Lin_sja1124_SecondLevelInterruptsType SecondLevelInt;
	int i;

	for (i = 0; i < ChannelsNo; i++)
		SJA1124_DRV_SendAbortRequest(Device, i);

	SJA1124_DRV_GetTopLevelInterrupts(Device, &TopLevelInt);
	SJA1124_DRV_ClearTopLevelInterrupts(Device, TopLevelInt);

	for (i = 0; i < ChannelsNo; i++) {
		SJA1124_DRV_GetSecondLevelInterrupts(Device, i, &SecondLevelInt);
		SJA1124_DRV_ClearSecondLevelInterrupts(Device, i, SecondLevelInt);
		SJA1124_DRV_SetLinChannelMode(Handle, i, MODE_SLEEP);
	}

	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_MODE, SJA1124_MODE_LOWPWRMODE_MASK, 0);

	return SJA1124_SUCCESS;
}

Lin_sja1124_StatusType SJA1124_DRV_ExitLowPowerMode(uint8_t Device)
{
	Lin_sja1124_spi_transceiverhandle_t* Handle= Sja1124_DeviceList[Device]->spiHandle;
	int32_t status;
	uint8_t sja1124Data;

	status = Register_SPI_Read(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams,
			SJA1124_STATUS, SJA1124_REG_SIZE_BYTE, &sja1124Data);
	if (ARM_DRIVER_OK != status)
	{
		return SJA1124_ERR_SPI;
	}
	SJA1124_Delay(SJA1124_DELAY);
	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_INT1, (uint8_t)(1U << SJA1124_INT1_INITSTATINT_SHIFT), 0);

	return SJA1124_SUCCESS;
}

Lin_sja1124_StatusType SJA1124_DRV_ResetLinMasterController(uint8_t Device, Lin_sja1124_LinChannelType LinChannel)
{
	Lin_sja1124_StatusType Status;
	Lin_sja1124_spi_transceiverhandle_t* Handle= Sja1124_DeviceList[Device]->spiHandle;

	/* Enter LIN initialization mode because soft reset can be trigger only in this mode. */
	Status = SJA1124_DRV_EnterLinChannelInitMode(Device, LinChannel);

	if (SJA1124_SUCCESS == Status)
		Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LGC, LinChannel), SJA1124_LIN1GC_SOFTRESET_MASK, 0);

	if (SJA1124_SUCCESS == Status)
		Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LGC, LinChannel), 0, 0);

	/* Leave LIN channel initialization mode. */
	return (SJA1124_SUCCESS == Status) ? SJA1124_DRV_LeaveLinChannelInitMode(Device, LinChannel) : Status;
}

Lin_sja1124_StatusType SJA1124_DRV_ConfigureSecondLevelInterrupts(
		uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel,
		Lin_sja1124_SecondLevelIntConfigType* SecondLevelIntConfig)
{
	uint8_t Data;
	Lin_sja1124_spi_transceiverhandle_t* Handle= Sja1124_DeviceList[Device]->spiHandle;

	SJA1124_DRV_SerializeSecondLevelInterrupts(SecondLevelIntConfig, &Data);
	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_CI_LIE, LinChannel), Data, 0);

	return SJA1124_SUCCESS;
}

Lin_sja1124_StatusType SJA1124_DRV_SendWakeupRequest(uint8_t Device, Lin_sja1124_LinChannelType LinChannel)
{
	Lin_sja1124_StatusType Status;
	Lin_sja1124_spi_transceiverhandle_t* Handle= Sja1124_DeviceList[Device]->spiHandle;

	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_SF_LBD1, LinChannel), WAKEUP_CHARACTER, 0);
	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_SF_LC, LinChannel), SJA1124_LIN1C_WAKEUPREQ_MASK, 0);

	return SJA1124_SUCCESS;
}

Lin_sja1124_StatusType SJA1124_DRV_SendAbortRequest(uint8_t Device, Lin_sja1124_LinChannelType LinChannel)
{
	int32_t status;
	Lin_sja1124_spi_transceiverhandle_t* Handle= Sja1124_DeviceList[Device]->spiHandle;

	status = Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_SF_LC, LinChannel), SJA1124_LIN1C_ABORTREQ_MASK, 0);
	if (ARM_DRIVER_OK != status)
	{
		return SJA1124_ERR_SPI;
	}

	return SJA1124_SUCCESS;
}

Lin_sja1124_StatusType SJA1124_DRV_GetSecondLevelInterrupts(
		uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel,
		Lin_sja1124_SecondLevelInterruptsType* SecondLevelInterrupts)
{
	uint8_t Data[2U];
	int32_t status;
	Lin_sja1124_StatusType Status = SJA1124_SUCCESS;
	Lin_sja1124_spi_transceiverhandle_t *Handle = Sja1124_DeviceList[Device]->spiHandle;

	status = Register_SPI_Read(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams,
			SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_GS_LES, LinChannel), SJA1124_REG_SIZE_BYTE, &Data[0]);

	if (ARM_DRIVER_OK != status)
	{
		return SJA1124_ERR_SPI;
	}
	status = Register_SPI_Read(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams,
			SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_GS_LS, LinChannel), SJA1124_REG_SIZE_BYTE, &Data[1]);
	if (ARM_DRIVER_OK != status)
	{
		return SJA1124_ERR_SPI;
	}
	*SecondLevelInterrupts = (uint32_t) Data[0U] | ((uint32_t) Data[1U] << 8U);

	return Status;
}

Lin_sja1124_StatusType SJA1124_DRV_ClearSecondLevelInterrupts(
		uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel,
		Lin_sja1124_SecondLevelInterruptsType SecondLevelInterrupts)
{
	uint8_t Data[2U];
	Lin_sja1124_spi_transceiverhandle_t *Handle = Sja1124_DeviceList[Device]->spiHandle;

	Data[0U] = (uint8_t) (SecondLevelInterrupts & 0xFFU);
	Data[1U] = (uint8_t) (SecondLevelInterrupts >> 8U);

	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_GS_LES, LinChannel), Data[0], 0);
	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_GS_LS, LinChannel), Data[1], 0);

	return SJA1124_SUCCESS;
}

Lin_sja1124_StatusType Lin_sja1124_ProcessSecondLevelInterrupt(
		uint8_t Device,
		Lin_sja1124_LinChannelType Channel,
		Lin_sja1124_SecondLevelIntType* SecondLevelInt)
{
	Lin_sja1124_StatusType Status;
	Lin_sja1124_SecondLevelInterruptsType ClearInterrupt = 0U;
	Lin_sja1124_SecondLevelInterruptsType SecondLevelInterrupts = 0U;

	Status = SJA1124_DRV_GetSecondLevelInterrupts(Device, Channel, &SecondLevelInterrupts);

	if (SJA1124_SUCCESS == Status)
	{
		/* Check if Tx has finished successfully. */
		if (IS_INTERRUPT_SET(SecondLevelInterrupts, INT_DATA_TRANSMISSION_COMPLETE))
		{
			*SecondLevelInt = INT_DATA_TRANSMISSION_COMPLETE;
			ClearInterrupt = SET_INTERRUPT(0U, INT_DATA_TRANSMISSION_COMPLETE);
		}
		/* Check if Rx has finished successfully. */
		else if (IS_INTERRUPT_SET(SecondLevelInterrupts, INT_DATA_RECEPTION_COMPLETE))
		{
			*SecondLevelInt = INT_DATA_RECEPTION_COMPLETE;
			ClearInterrupt = SET_INTERRUPT(0U, INT_DATA_RECEPTION_COMPLETE);
		}
		/* Check bit error flag. */
		else if (IS_INTERRUPT_SET(SecondLevelInterrupts, INT_BIT_ERROR))
		{
			*SecondLevelInt = INT_BIT_ERROR;
			ClearInterrupt = SET_INTERRUPT(0U, INT_BIT_ERROR);
		}
		/* Timeout error is set if response timeout expires. */
		else if (IS_INTERRUPT_SET(SecondLevelInterrupts, INT_TIMEOUT_ERROR))
		{
			*SecondLevelInt = INT_TIMEOUT_ERROR;
			ClearInterrupt = SET_INTERRUPT(0U, INT_TIMEOUT_ERROR);
		}
		/* Frame error is set when a dominant state is sampled on a stop
		 * bit of the currently received character. */
		else if (IS_INTERRUPT_SET(SecondLevelInterrupts, INT_FRAME_ERROR))
		{
			*SecondLevelInt = INT_FRAME_ERROR;
			ClearInterrupt = SET_INTERRUPT(0U, INT_FRAME_ERROR);
		}
		/* Checksum error is set when received checksum doesn't match
		 * calculated checksum. */
		else if (IS_INTERRUPT_SET(SecondLevelInterrupts, INT_CHECKSUM_ERROR))
		{
			*SecondLevelInt = INT_CHECKSUM_ERROR;
			ClearInterrupt = SET_INTERRUPT(0U, INT_CHECKSUM_ERROR);
		}
		/* The last case: stuck at zero. */
		else if (IS_INTERRUPT_SET(SecondLevelInterrupts, INT_STUCK_AT_ZERO))
		{
			*SecondLevelInt = INT_STUCK_AT_ZERO;
			ClearInterrupt = SET_INTERRUPT(0U, INT_STUCK_AT_ZERO);
		}
		else if (IS_INTERRUPT_SET(SecondLevelInterrupts, INT_DATA_RECEPTION_BUF_NOT_EMPTY))
		{
			*SecondLevelInt = INT_DATA_RECEPTION_BUF_NOT_EMPTY;
		}
		else
		{
			*SecondLevelInt = NO_SECOND_LEVEL_INT;
		}
	}

	if ((NO_SECOND_LEVEL_INT != *SecondLevelInt) && (SJA1124_SUCCESS == Status))
	{
		/* Clear interrupt flag along with DRBNE bit which is set when a byte is sent or received. */
		Status = SJA1124_DRV_ClearSecondLevelInterrupts(
				Device,
				Channel,
				ClearInterrupt | SET_INTERRUPT(0U, INT_DATA_RECEPTION_BUF_NOT_EMPTY));
	}


	Status = SJA1124_DRV_GetSecondLevelInterrupts(Device, Channel, &SecondLevelInterrupts);
	return Status;
}

Lin_sja1124_StatusType SJA1124_DRV_GetTopLevelInterrupts(uint8_t Device, Lin_sja1124_TopLevelInterruptsType* TopLevelInterrupts)
{
	uint8_t Data[3U];
	Lin_sja1124_StatusType Status = SJA1124_SUCCESS;
	int32_t status;
	Lin_sja1124_spi_transceiverhandle_t *Handle = Sja1124_DeviceList[Device]->spiHandle;

	// dummy read
	status = Register_SPI_Read(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_INT1, SJA1124_REG_SIZE_BYTE, &Data[0]);
	status = Register_SPI_Read(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_INT1, SJA1124_REG_SIZE_BYTE, &Data[0]);
	if (ARM_DRIVER_OK != status)
	{
		return SJA1124_ERR_SPI;
	}
	status = Register_SPI_Read(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_INT2, SJA1124_REG_SIZE_BYTE, &Data[1]);
	if (ARM_DRIVER_OK != status)
	{
		return SJA1124_ERR_SPI;
	}
	status = Register_SPI_Read(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_INT3, SJA1124_REG_SIZE_BYTE, &Data[2]);
	if (ARM_DRIVER_OK != status)
	{
		return SJA1124_ERR_SPI;
	}

	*TopLevelInterrupts = (uint32_t) Data[0U] | ((uint32_t) Data[1U] << 8U) | ((uint32_t) Data[2U] << 16U);

	return Status;
}

Lin_sja1124_StatusType SJA1124_DRV_ClearTopLevelInterrupts(uint8_t Device, Lin_sja1124_TopLevelInterruptsType TopLevelInterrupts)
{
	uint8_t Data[2U];
	Lin_sja1124_spi_transceiverhandle_t *Handle = Sja1124_DeviceList[Device]->spiHandle;

	Data[0U] = (uint8_t) (TopLevelInterrupts & 0xFFU);
	Data[1U] = (uint8_t) (TopLevelInterrupts >> 8U);

	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_INT1, Data[0], 0);
	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_INT2, Data[1], 0);

	return SJA1124_SUCCESS;
}

Lin_StatusType SJA1124_DRV_GetStatus(
		uint8_t Device,
		Lin_sja1124_LinChannelType Channel)
{
	GetStatusType GetStatus;
	volatile Lin_sja1124_StatusType Status = SJA1124_FAIL;
	Lin_StatusType TransmissionStatus = LIN_NOT_OK;

	Status = Lin_sja1124_GetStatus(Device, Channel, &GetStatus, &TransmissionStatus);

	return (Lin_StatusType)((SJA1124_SUCCESS == Status) ? TransmissionStatus : LIN_NOT_OK);
}

Lin_sja1124_StatusType SJA1124_DRV_SendFrame(
		uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel,
		Lin_sja1124_LinFrameType* LinFrame)
{
	uint8_t BytesCount = 0U;
	uint8_t Data[12U] = { 0U };
	uint8_t offset;
	int32_t i;
	Lin_sja1124_spi_transceiverhandle_t* Handle= Sja1124_DeviceList[Device]->spiHandle;

	Data[0U] = SJA1124_LIN1C_HEADERTRANSREQ_MASK;

	/* Serialize LIN frame, number of serialized bytes is returned in BytesCount. */
	SJA1124_DRV_SerializeLinFrame(LinFrame, Data + 1U, &BytesCount);
	offset = SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_SF_LBI, LinChannel);
	for (i = 0; i < BytesCount; i++)
	{
		Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, offset, Data[i + 1], 0);
		offset = offset + 1;
	}
	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_SF_LC, LinChannel), Data[0], 0);

	return SJA1124_SUCCESS;
}

Lin_sja1124_StatusType SJA1124_DRV_GetData(
		uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel,
		Lin_sja1124_LinFrameType* LinFrame,
		Lin_sja1124_SecondLevelIntType *SecondLevelInt_type)
{
	uint8_t GetState;
	uint8_t Data[9U] = { 0U };
	uint8_t offset;
	int32_t i;
	Lin_sja1124_spi_transceiverhandle_t* Handle= Sja1124_DeviceList[Device]->spiHandle;
	Lin_sja1124_StatusType Status = SJA1124_SUCCESS;
	Lin_sja1124_SecondLevelInterruptsType SecondLevelInt;
	int retries = MAX_RETRIES;

	GetState = SJA1124_DRV_GetChannelState(Device, LinChannel);

	while (retries-- && !(GetState & SJA1124_LIN1S_RCVCOMPFLG_MASK))
	{
		GetState = SJA1124_DRV_GetChannelState(Device, LinChannel);
	}

	offset = SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_GS_LBD1, LinChannel);

	for (i = 0; i < LinFrame->DataFieldLength; i++) {
		Register_SPI_Read(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, offset, SJA1124_REG_SIZE_BYTE, &Data[i]);
		offset = offset + 1;
		LinFrame->Data[i] = Data[i];
	}
	offset = SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_GS_LCF, LinChannel);
	Register_SPI_Read(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, offset, SJA1124_REG_SIZE_BYTE, &Data[i]);
	LinFrame->Checksum = Data[i];

	SJA1124_DRV_GetSecondLevelInterrupts(Device, LinChannel, &SecondLevelInt);

	if(SecondLevelInt & SJA1124_LIN1IE_STUCKZEROIEN_MASK)
	{
		*SecondLevelInt_type = INT_STUCK_AT_ZERO;
		 Status = SJA1124_FAIL;
	}
	else if(SecondLevelInt & SJA1124_LIN1IE_TIMEOUTIEN_MASK)
	{
		*SecondLevelInt_type = INT_TIMEOUT_ERROR;
		 Status = SJA1124_FAIL;
	}
	else if(SecondLevelInt & SJA1124_LIN1IE_BITERRIEN_MASK)
	{
		*SecondLevelInt_type = INT_BIT_ERROR;
		Status = SJA1124_FAIL;
	}
	else if(SecondLevelInt & SJA1124_LIN1IE_CSERRIEN_MASK)
	{
		*SecondLevelInt_type = INT_STUCK_AT_ZERO;
		Status = SJA1124_FAIL;
	}
	else if(SecondLevelInt & SJA1124_LIN1IE_FRAMEERRIEN_MASK)
	{
		*SecondLevelInt_type = INT_FRAME_ERROR;
		Status = SJA1124_FAIL;
	}
	else if ( (SecondLevelInt >> 8) & SJA1124_LIN1IE_DTRCVIEN_MASK )
	{
		*SecondLevelInt_type = INT_DATA_RECEPTION_COMPLETE;
	}

	SJA1124_DRV_ClearSecondLevelInterrupts(Device, LinChannel, SecondLevelInt);

	return Status;
}

Lin_sja1124_StatusType SJA1124_DRV_SendHeaderTransmissionRequest(
		uint8_t Device,
		const Lin_sja1124_HeaderTransRequestType* HeaderTransRequest)
{
	uint8_t Data;
	Lin_sja1124_spi_transceiverhandle_t* Handle= Sja1124_DeviceList[Device]->spiHandle;

	Data = (uint8_t)((((uint8_t) HeaderTransRequest->Chan1HeaderTransRequest) << LIN_CHANNEL_1) |
			(((uint8_t) HeaderTransRequest->Chan2HeaderTransRequest) << LIN_CHANNEL_2) |
			(((uint8_t) HeaderTransRequest->Chan3HeaderTransRequest) << LIN_CHANNEL_3) |
			(((uint8_t) HeaderTransRequest->Chan4HeaderTransRequest) << LIN_CHANNEL_4));

	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_LCOM2, Data, 0);

	return SJA1124_SUCCESS;
}

Lin_sja1124_StatusType SJA1124_DRV_SendHeader(
		uint8_t Device,
		Lin_sja1124_LinChannelType LinChannel,
		Lin_sja1124_LinFrameType* LinFrame)
{
	uint8_t Data[3];
	Lin_sja1124_spi_transceiverhandle_t* Handle= Sja1124_DeviceList[Device]->spiHandle;

	Data[0U] = LinFrame->Pid;
	Data[1U] = (uint8_t) (((uint8_t) LinFrame->ChecksumType << SJA1124_LIN1BC_CLASSICCS_SHIFT) |
			((uint8_t) LinFrame->ResponseDir << SJA1124_LIN1BC_DIRECTION_SHIFT) |
			((uint8_t) (LinFrame->DataFieldLength - 1U) << SJA1124_LIN1BC_DTFIELDLEN_SHIFT));
	Data[2U] = SJA1124_LIN1C_HEADERTRANSREQ_MASK;

	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_SF_LBI, LinChannel), Data[0], 0);
	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_SF_LBC, LinChannel), Data[1], 0);
	Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_CHAN_ADDR_OFFSET(SJA1124_LIN1_SF_LC, LinChannel), Data[2], 0);

	return SJA1124_SUCCESS;
}

void SJA1124_SPI_ReadPreprocess(void *pCmdOut, uint32_t offset, uint32_t size)
{
	spiCmdParams_t *pSlaveCmd = pCmdOut;

	uint8_t *pWBuff = sja1124_spiRead_CmdBuffer;
	uint8_t *pRBuff = sja1124_spiRead_DataBuffer;

	/* Formatting for Read command of SJA1124 Quad LIN Commander Transceiver. */
	*(pWBuff) = offset; /* offset is the internal register address of the sensor at which write is performed. */
	*(pWBuff + 1) = SJA1124_SPI_READ_CMD | (size-1);

	/* Create the slave read command. */
	pSlaveCmd->size = size + SJA1124_SPI_CMD_LEN;
	pSlaveCmd->pWriteBuffer = pWBuff;
	pSlaveCmd->pReadBuffer = pRBuff;
}

void SJA1124_SPI_WritePreprocess(void *pCmdOut, uint32_t offset, uint32_t size, void *pWritebuffer)
{
	spiCmdParams_t *pSlaveCmd = pCmdOut;

	uint8_t *pWBuff = sja1124_spiWrite_CmdDataBuffer;
	uint8_t *pRBuff = sja1124_spiWrite_CmdDataBuffer + size + SJA1124_SPI_CMD_LEN;

	/* Formatting for Write command of SJA1124 Quad LIN Commander Transceiver. */
	*(pWBuff) = offset; /* offset is the internal register address of the sensor at which write is performed. */
	*(pWBuff + 1) = (size-1);

	/* Copy the slave write command */
	memcpy(pWBuff + SJA1124_SPI_CMD_LEN, pWritebuffer, size);

	/* Create the slave command. */
	pSlaveCmd->size = size + SJA1124_SPI_CMD_LEN;
	pSlaveCmd->pWriteBuffer = pWBuff;
	pSlaveCmd->pReadBuffer = pRBuff;
}

int32_t SJA1124_SPI_Initialize(
		Lin_sja1124_spi_transceiverhandle_t *pSensorHandle, ARM_DRIVER_SPI *pBus, uint8_t index, void *pSlaveSelect)
{
	int32_t status;
	uint8_t reg;
	GENERIC_DRIVER_GPIO *pGPIODriver = &Driver_GPIO_KSDK;

	/*! Check the input parameters. */
	if ((pSensorHandle == NULL) || (pBus == NULL) || (pSlaveSelect == NULL))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Initialize the sensor handle. */
	pSensorHandle->pCommDrv = pBus;
	pSensorHandle->slaveParams.pReadPreprocessFN = SJA1124_SPI_ReadPreprocess;
	pSensorHandle->slaveParams.pWritePreprocessFN = SJA1124_SPI_WritePreprocess;
	pSensorHandle->slaveParams.pTargetSlavePinID = pSlaveSelect;
	pSensorHandle->slaveParams.spiCmdLen = SJA1124_SPI_CMD_LEN;
	pSensorHandle->slaveParams.ssActiveValue = SJA1124_SS_ACTIVE_VALUE;

	pSensorHandle->deviceInfo.deviceInstance = index;
	pSensorHandle->deviceInfo.functionParam = NULL;
	pSensorHandle->deviceInfo.idleFunction = NULL;

	//	/* Initialize the Slave Select Pin. */
	pGPIODriver->pin_init(pSlaveSelect, GPIO_DIRECTION_OUT, NULL, NULL, NULL);
	if (pSensorHandle->slaveParams.ssActiveValue == SPI_SS_ACTIVE_LOW)
	{
		pGPIODriver->set_pin(pSlaveSelect);
	}
	else
	{
		pGPIODriver->clr_pin(pSlaveSelect);
	}

	pSensorHandle->isInitialized = true;
	return SENSOR_ERROR_NONE;
}

void SJA1124_SPI_SetIdleTask(Lin_sja1124_spi_transceiverhandle_t *pSensorHandle,
		registeridlefunction_t idleTask,
		void *userParam)
{
	pSensorHandle->deviceInfo.functionParam = userParam;
	pSensorHandle->deviceInfo.idleFunction = idleTask;
}

int32_t SJA1124_SPI_Configure(Lin_sja1124_spi_transceiverhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList)
{
	int32_t status;

	/*! Validate for the correct handle and register write list.*/
	if ((pSensorHandle == NULL) || (pRegWriteList == NULL))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before applying configuration.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	status = Sensor_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			pRegWriteList);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

Lin_sja1124_StatusType SJA1124_SPI_ReadData(Lin_sja1124_spi_transceiverhandle_t *pSensorHandle,
		const registerreadlist_t *pReadList,
		uint8_t *pBuffer)
{
	int32_t status;

	/*! Validate for the correct handle and register read list.*/
	if ((pSensorHandle == NULL) || (pReadList == NULL) || (pBuffer == NULL))
	{
		return SJA1124_ERR_SPI;
	}

	/*! Check whether sensor handle is initialized before reading sensor data.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SJA1124_ERR_INIT;
	}

	/*! Parse through the read list and read the data one by one. */
	status = Sensor_SPI_Read(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			pReadList, pBuffer);
	if (ARM_DRIVER_OK != status)
	{
		return SJA1124_ERR_SPI;
	}

	return SJA1124_SUCCESS;
}

uint8_t SJA1124_DRV_CalculateChecksum(Lin_sja1124_ChecksumType ChecksumType, uint8_t Pid, uint8_t* Data, uint8_t Length)
{
	uint8_t Idx;
	uint16_t CheckSum = 0U;

	if (CHECKSUM_ENHANCED == ChecksumType)
	{
		CheckSum = Pid;
	}

	for (Idx = 0U; Idx < Length; Idx++)
	{
		CheckSum += Data[Idx];

		if (CheckSum > 0xFFU)
		{
			CheckSum -= 0xFFU;
		}
	}

	return (~CheckSum) & 0xFFU;
}

Lin_sja1124_StatusType SJA1124_DRV_WriteRegister(uint8_t Device, uint8_t RegAddress, uint8_t Data)
{
	int32_t status;
	Lin_sja1124_spi_transceiverhandle_t* Handle= Sja1124_DeviceList[Device]->spiHandle;

	status = Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, RegAddress, Data, 0);
	if (ARM_DRIVER_OK != status)
	{
		return SJA1124_FAIL;
	}

	return SJA1124_SUCCESS;
}

Lin_sja1124_StatusType SJA1124_DRV_ReadRegister(uint8_t Device, uint8_t RegAddress, uint8_t* Data)
{
	int32_t status;
	Lin_sja1124_spi_transceiverhandle_t* Handle= Sja1124_DeviceList[Device]->spiHandle;

	status = Register_SPI_Read(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, RegAddress, SJA1124_REG_SIZE_BYTE, Data);
	if (ARM_DRIVER_OK != status)
	{
		return SJA1124_FAIL;
	}

	return SJA1124_SUCCESS;
}

Lin_sja1124_StatusType SJA1124_DRV_Deinit(uint8_t Device)
{
	int32_t status;
	Lin_sja1124_spi_transceiverhandle_t* Handle= Sja1124_DeviceList[Device]->spiHandle;

	status = Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_MODE, SJA1124_MODE_RESET_MASK, 0);
	if (ARM_DRIVER_OK != status)
	{
		return SJA1124_FAIL;
	}

	return SJA1124_SUCCESS;
}

Lin_sja1124_StatusType SJA1124_Device_status(uint8_t Device)
{
	int32_t status;
	Lin_sja1124_spi_transceiverhandle_t* Handle= Sja1124_DeviceList[Device]->spiHandle;

	status = Register_SPI_Write(Handle->pCommDrv, &Handle->deviceInfo, &Handle->slaveParams, SJA1124_MODE, SJA1124_MODE_RESET_MASK, 0);
	if (ARM_DRIVER_OK != status)
	{
		return SJA1124_FAIL;
	}

	return SJA1124_SUCCESS;
}
