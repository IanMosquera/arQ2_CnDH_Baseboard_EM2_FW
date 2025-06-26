/*
 * arQ.c
 *
 *  Created on: Jun 26, 2025
 *      Author: IanCMosquera
 */

#include "arQ_CnDH_BaseBoard.h"
#include "string.h"
#include "stdarg.h"
#include "usbd_cdc_if.h"
#include "stm32wbxx_hal_uart.h"


extern UART_HandleTypeDef huart1;
extern arQ_t arQ;
extern uint8_t rxBuffer[100];


/**
 	* @brief Extract DateTime value from FUNCRETVAL to %s%s%s/%s%s%s format
 	* @param None
 	* @retval None
 	* @author ICM
  */
/*void DateTimeStatus(void)
{
	//uint32_t x = 0;
	//uint32_t y = 0;
	//uint32_t z = 0;
	//uint32_t start = 0;
	//uint32_t end   = 0;

  Get_ARQ_DateTime();

  //x = arQ.DTm.Hour;
  //y = arQ.DTm.Min;
  //z = arQ.DTm.Sec;

  if (zerosec == 1)
  {
  	sprintf(arQ.Buf.FUNCRETURNVAL, "%2d%2d%2d/%2d%2d00",
  			arQ.DTm.Year-2000,
				arQ.DTm.Month,
				arQ.DTm.Days,
				arQ.DTm.Hour,
				arQ.DTm.Min);
  	zerosec = 0;
  }

  else if (arQ.Flg.PROC_START == true)
  {
  	sprintf(arQ.Buf.FUNCRETURNVAL, "start: %2d%2d%2d/%2d%2d%2d",
  			arQ.DTm.Year-2000,
				arQ.DTm.Month,
				arQ.DTm.Days,
				arQ.DTm.Hour,
				arQ.DTm.Min,
				arQ.DTm.Sec);
  	//start = ((x * 3600) + (y * 60) + z);
  	arQ.Flg.PROC_START = false;
  }

  else if (arQ.Flg.PROC_END == true)
  {
  	sprintf(arQ.Buf.FUNCRETURNVAL, "end: %2d%2d%2d/%2d%2d%2d",
  			arQ.DTm.Year-2000,
				arQ.DTm.Month,
				arQ.DTm.Days,
				arQ.DTm.Hour,
				arQ.DTm.Min,
				arQ.DTm.Sec);
  	//end   = ((x * 3600) + (y * 60) + z);
  	arQ.Flg.PROC_END = false;
  }

  else
  {
  	sprintf(arQ.Buf.FUNCRETURNVAL, "%2d%2d%2d/%2d%2d%2d",
  			arQ.DTm.Year-2000,
				arQ.DTm.Month,
				arQ.DTm.Days,
				arQ.DTm.Hour,
				arQ.DTm.Min,
				arQ.DTm.Sec);
  }
}






// New SensorPoll
void Sensor_Poll(void)
{
	// uint32_t startTime = 0;

	arQ.Flg.PROC_START = 1;
	DateTimeStatus();
	// startTime = start;

	if (arQ.Flg.SENSOR_DEBUG == true)
	{
		DateTimeStatus();
		strcpy(arQ.Buf.ZEROED, arQ.Buf.FUNCRETURNVAL);
		arQ.Flg.SENSOR_DEBUG = false;
	}
	else
	{
		zerosec = 1;
		DateTimeStatus();
		strcpy(arQ.Buf.ZEROED, arQ.Buf.FUNCRETURNVAL);
	}

	// COMMENT: dateTimeStatus is formatted in "YYMMDD/hhmmss"
	// COMMENT: ZEROED was not found on any related codes
	Get_ARQ_DateTime();
	// PCDM_Boost_Config(1);
	Execute_PCDM_Boost_Config(ON_DEMAND_ON);
	HAL_Delay(300);

	if (arQ.Cfg.SensorConfig[0] == '\0')
		strcpy(arQ.Buf.REALDATA, "No Sensor Configuration\r\n");

	Get_arQ_Data();
}




void Get_arQ_Data(void)
{
	memset(arQ.Buf.REALDATA, '\0', 255);
	// [] Read arQ temperature and Pressure

	xprintf(PC, "Sensor Polling...");
	xprintf(PC, "\r\n----------DATA SUMMARY----------\r\n");
	HAL_Delay(10);

	// 1) Serial Number
	xprintf(PC, "1) arQ serial number: %s\r\n", arQ.Cfg.SerialNumber);
	Append_To_RealData(arQ.Cfg.SerialNumber);
	HAL_Delay(10);

	// 2) SIM Number
	xprintf(PC, "2) SIM number: %s\r\n", arQ.Cfg.SimNumber);
	HAL_Delay(10);

	// 3) Sensor Config
	xprintf(PC, "3) Station Type: %s\r\n", arQ.Cfg.SensorConfig);
	HAL_Delay(10);

	// 4) Sending Time
	xprintf(PC, "4) Sending time: %d\r\n", arQ.Cfg.SendingTime);
	HAL_Delay(10);

	// 5) Sensors
	if (strcmp(arQ.Cfg.SensorConfig,"WR") == 0 ||
			strcmp(arQ.Cfg.SensorConfig,"WL") == 0)
	{
		// [] Change function to Get_WL_Data()
		xprintf(PC, "5) Water Level: %s\r\n", "9.2");
		Append_To_RealData(arQ.Buf.WATER_LVL);
	}
	HAL_Delay(10);

	if (strcmp(arQ.Cfg.SensorConfig,"WR") == 0 ||
			strcmp(arQ.Cfg.SensorConfig,"RG") == 0)
	{
		// [] Change function to Get_Rain_Data()
		GetRainCountCumTipping();
		xprintf(PC, "5) Rain Count Tips: %s\r\n", "15");
		Append_To_RealData(arQ.Rin.RainCount);
		xprintf(PC, "Commulative Rain Count: %s\r\n", "115");
		Append_To_RealData(arQ.Rin.RainCummulative);
	}
	HAL_Delay(10);

	// [] Simplify the if Statement !Not Appended
	if (strcmp(arQ.Cfg.SensorConfig,"MBH") == 0 ||
			strcmp(arQ.Cfg.SensorConfig,"MBA") == 0)
	{
		// [] Get_AirMar_Data();
		// strcpy(arQ.Buf.RXD_DATA, arQ.Buf.FUNCRETURNVAL);
		//Dummy Data
		sprintf(arQ.Buf.RXD_DATA, "NoData!");
		xprintf(PC, "5) AirMar Data: %s\r\n", arQ.Buf.RXD_DATA);

		// Send Value to LTE
		// xprintf(GSM, "airmar%s",arQ.Buf.RXD_DATA);

		// Write to Flash
		// [] DTFWriteToFlash(RXD_DATA, strlen(RXD_DATA), 1);

		HAL_Delay(100);

		// [] Restart Watchdog Timer

		// [/] Get_WQL_Data();
		// strcpy(arQ.Buf.SMS_TOBE_SENT, arQ.Buf.FUNCRETURNVAL);
		sprintf(arQ.Buf.SMS_TOBE_SENT, "NoData!");
		xprintf(PC, "5) Water quality data: %s\r\n", arQ.Buf.SMS_TOBE_SENT);

		// Send Value to LTE
		// xprintf(GSM, "wql%s", arQ.Buf.SMS_TOBE_SENT);
		HAL_Delay(100);
	}
	HAL_Delay(10);

	// 6) arQ Temperature
	// Get STH40 Temperature
	xprintf(PC, "6) arQ Temp: %s\r\n", arQ.Tmp.TempSTR);
	Append_To_RealData(arQ.Tmp.TempSTR);
	HAL_Delay(10);

	// [] Get Pressure
	xprintf(PC, "7) arQ Pressure: %s\r\n", arQ.Prs.PressSTRR);
	Append_To_RealData(arQ.Prs.PressSTRR);
	HAL_Delay(10);

	// 7) Date and Time
	Print_Current_DateTime();
	Append_To_RealData(arQ.DTm.DateTime);
	HAL_Delay(200);

	// 8) Server Number !Not Appended
	xprintf(PC, "8) Server Number: %s\r\n", arQ.Cfg.ServerNumber);
	HAL_Delay(10);

	// 9) Battery Voltage
	xprintf(PC, "9) PCDM VBAT: %s\r\n", arQ.Pwr.PCDMVoltSTR);
	Append_To_RealData(arQ.Pwr.PCDMVoltSTR);
	HAL_Delay(10);

	// 10) Board Current
	// [] Get_PCDM_Board_Current
	xprintf(PC, "10) PCDM Current: %s\r\n", arQ.Pwr.PCDMCurSTR);
	Append_To_RealData(arQ.Pwr.PCDMCurSTR);
	HAL_Delay(10);

	// 11) PCDM Boost Voltage
	xprintf(PC, "11) PCDM Boost1 Voltage: %6.2f\r\n", "0.00");
	xprintf(PC, "    PCDM Boost2 Voltage: %6.2f\r\n", "0.00");

	Append_To_RealData(arQ.Pwr.VBoost1STR);
	Append_To_RealData(arQ.Pwr.VBoost2STR);
	HAL_Delay(10);

	// 12) Power Board Configuration
	xprintf(PC, "12) PCDM Board Configuration: %s\r\n", arQ.Cfg.PowerBoardConfig);
	HAL_Delay(10);

	// Get LTC Data !No Need to get this

	// 13) Hybrid Mode Status
	if (arQ.Cfg.SleepTime > 0)
	{
		xprintf(PC, "13) Hybrid power saving: ON\r\n");
		HAL_Delay(10);
		xprintf(PC, "    LTE modem sleep duration: %i\r\n", arQ.Cfg.SleepTime);
		HAL_Delay(10);
	}
	else
		xprintf(PC, "13) Hybrid power saving: OFF\r\n");
	HAL_Delay(10);

	// 14) Get Lead-Acid Battery !LTC.VIN
	xprintf(PC, "14) Lead-Acid Battery Voltage: %s\r\n", arQ.Pwr.EXTVBATSTR);
	HAL_Delay(10);

	// 15) Charge/Fault Check !LTC4162 Error
	// [] Charge_Fault_Check();

	// 16) Get Registered Numbers !Not Appended
	xprintf(PC, "16) Registered Numbers: %s, %s, %s\r\n",
			arQ.Cfg.RegisteredNumber1,
			arQ.Cfg.RegisteredNumber2,
			arQ.Cfg.RegisteredNumber3);
	HAL_Delay(10);

	// Get Password
	xprintf(PC, "17) Password: %s\r\n", arQ.Cfg.Password);
	HAL_Delay(10);

	// Turn off PCDM Boost Voltage
	xprintf(PC, "18) Data to Flash: %s\r\n", arQ.Buf.REALDATA);
	// [] Write REALDATA to FLASH
	HAL_Delay(10);

	arQ.Flg.DATA_TAKEN = true;
}





void Update_ARQ_Time(void)
{
	uint8_t stat = 0;
	stat = Sync_Time_From_NTP();

	if (stat == 0)
	{
		xprintf(PC, "Time sync FAILED!\r\n NTP server connection FAILED!\r\n");
		RTCAcquireSync();
	}
	else
		xprintf(PC, "Time sync SUCCESSFUL!\r\n");
}




void Append_To_RealData(char pData[])
{
	char delimiter = '+';
	strcat(arQ.Buf.REALDATA, pData);
	strncat(arQ.Buf.REALDATA, &delimiter, 1);
}





void SMSCall_Interrupt_Check(void)
{
	if (arQ.Flg.EXT_FLAG == true)
	{
		xprintf(PC, "INCOMING CALL/SMS...\r\n");
		strcpy(arQ.Buf.RXD_DATA, arQ.Buf.SMS_RCV);
		ProcessReceived();
		Clear_Serial_Buffers();
		arQ.Flg.EXT_FLAG = false;
	}
}





void USBSerial_Interrupt_Check(void)
{
	bool escape = false;

	if (arQ.Flg.USBSERIAL_FLAG == 1)
	{
		SMSCall_Interrupt_Check();
		HAL_Delay(200);

		arQ.Buf.RXD2_DATA[arQ.Ctr.WRITE2_CNTR-1] = '\0';

		if (strcmpi((char *)rxBuffer, "DEBUG\r") == 0)
		{
			Clear_USB_Buffers();
			Print_Setting_Menu();

			while(escape == false)
			{
				if (debugDetails(rxBuffer)) escape = true;
			}
		}
		else if (!stricmp(arQ.Buf.RXD2_DATA, "RESET"))
		{
			xprintf(PC, "Resetting arQ. . . \r\n");
			HAL_Delay(200);
			NVIC_SystemReset();
		}
		Clear_USB_Buffers();
	}
}





void ARQ_Reset_Check(void)
{
	if (arQ.Ctr.INTERRUPTCHECKER++ > 150)
	{
		// [] AddDataEEprom("INTPROB",EEPROM_RESET_NUMBER);
		if (!ShutdownLTE(VIA_IGT_PIN))
			if (!ShutdownLTE(VIA_RST_PIN))
				ShutdownLTE(VIA_AT_CMD);

		NVIC_SystemReset();
		HAL_Delay(3000);
	}
}





void Cummulative_Rain_Reset_Check(void)
{
	if (arQ.Flg.CUM_RAIN_RESET_FLAG == true)
	{
		ClearRainCumVars();
		arQ.Flg.CUM_RAIN_RESET_FLAG = false;
	}
}





void LTE_MidNight_Reset_Check(void)
{
	if (arQ.Flg.MIDNIGHT_RESET_LTE_FLAG == true)
	{
		xprintf(PC, "Daily LTE module reset..\r\n");

		GPS_Power_ON();
		Sync_Time_From_NTP();
		Sensor_Poll();
		Send_Data_To_NRF();
		LTE_GNSS();
		GPS_Power_OFF();
		AttemptToSendLTE();
		ShutdownLTE(VIA_IGT_PIN);
		HAL_Delay(3000);
		LTE_Init();
		RainResetTipping();
		arQ.Flg.MIDNIGHT_RESET_LTE_FLAG = false;

		xprintf(PC, "\r\nLTE reset successful\r\n");
	}
}





void Enable_Normal_Power_Mode(void)
{
  if (Normal_Power_Mode())
  {
  	LTE_Init();
  	Update_ARQ_Time();
  	Clear_Serial_Buffers();
  	nRF_Delete_SMS();
  	Sensor_Poll();
  	Boot_Status();
  }
  else
  	ShutdownLTE(VIA_IGT_PIN);
}





void Trigger_Lora_Transmit_Pin(void)
{
	if (!stricmp(arQ.Cfg.SensorConfig, "MBH") ||
			!stricmp(arQ.Cfg.SensorConfig, "MBA"))
	{
		//HAL_GPIO_WritePin(LORA_GPIO_Port, LORA_Pin, GPIO_PIN_SET);
		HAL_Delay(1000);
		//HAL_GPIO_WritePin(LORA_GPIO_Port, LORA_Pin, GPIO_PIN_RESET);
		HAL_Delay(500);
		//HAL_GPIO_WritePin(LORA_GPIO_Port, LORA_Pin, GPIO_PIN_SET);
		HAL_Delay(1000);
		//HAL_GPIO_WritePin(LORA_GPIO_Port, LORA_Pin, GPIO_PIN_RESET);
		xprintf(PC, "Data sending via LoRa...\r\n");
	}
}





void Print_Header_Text(void)
{
	xprintf(PC, "\r\nAdvanced Science and Technology Institute\r\n");
	xprintf(PC, "             Embedded Systems Group\r\n");
}





void Sensor_Init(void)
{
	Execute_PCDM_Boost_Config(ON_DEMAND_OFF);
}





void Get_Config_Init_Values(void)
{
	//char strx[20] = {};
	char charReply[2];

	// [] Get Sending Time From EEPROM
	// readCharEE(EEPROM_SENDINGTIME_CHECK, charReply[0]);
	sprintf(charReply, "$"); // DUMMY DATA, Remove if updated
	if (charReply[0] != '$')
	{
    // writeCharEE('$',EEPROM_SENDINGTIME_CHECK);
    // AddDataEEprom("10",EEPROM_SENDING_TIME);
		arQ.Cfg.SendingTime = 10;
	}

	// Display Sending Time
	xprintf(PC, "Sending Time: %i\r\n", arQ.Cfg.SendingTime);

	// Get Server Number from the EEPROM
	// readDataEEprom(EEPROM_SERVER_NUMBER);
	xprintf(PC, "Server number: %s\r\n", arQ.Cfg.ServerNumber);

	// Display SIM Number
	// readDataEEprom(EEPPROM_SIMNUMBER);
	xprintf(PC, "SIM Card number: %s\r\n", arQ.Cfg.SimNumber);

	// Display arQ Serial Number
	// readDataEEprom(EEPROM_ARQ_SERIAL_NUMBER);
	xprintf(PC, "arQ serial number: %s\r\n", arQ.Cfg.SerialNumber);

	// Display Sensor Type
	// readDataEEprom(EEPROM_SENSOR_TYPE);
	if(!stricmp(arQ.Buf.FROMSERIALPC, "MBH"))
		water_qualilty_sensor_type = 0;
	else if (!stricmp(arQ.Buf.FROMSERIALPC, "MBH"))
		water_qualilty_sensor_type = 1;
	xprintf(PC, "Sensor Config: %s\r\n", arQ.Cfg.SensorConfig);

	// Get power Board Config
	// readCharEE(EEPROM_PBOARDCONFIG_CHECK, charReply[0]);
	if (charReply[0] != '$')
	{
    // writeCharEE('$',EEPROM_PBOARDCONFIG_CHECK); // write to password check
    // AddDataEEprom("22",EEPROM_PBOARD_CONFIG);
		sprintf(arQ.Cfg.PowerBoardConfig, "22");
	}
	xprintf(PC, "Power Board Config: %s\r\n", arQ.Cfg.PowerBoardConfig);

	// Get Password
	// readCharEE(EEPROM_PASSWORD_CHECK, charReply[0]);
	//if (charReply[0] != '$')
	//{
    // writeCharEE('$',EEPROM_PASSWORD_CHECK); // write to password check
    // AddDataEEprom("EMBEDDED", EEPROM_PASSWORD);
	//}
	xprintf(PC, "Password: %s\r\n", arQ.Cfg.Password);

	// Display Hybrid Mode status
	// readDataEEprom(EEPROM_SLEEP_TIME);
	xprintf(PC, "LTE modem sleep duration: %d\n", arQ.Cfg.SleepTime);

	// Display Cummulative Rain Fall
	// readDataEEprom(EEPROM_RAIN_CUMULATIVE);
	// xprintf(PC, "Cummulative Rain Fall: 0000\n");
}





void Boot_Status(void)
{
	switch (Restart_Cause())
	{
		case WDT_TIMEOUT:
			xprintf(GSM, "boot0");
			break;
		case NORMAL_POWER_UP:
			xprintf(GSM, "boot1");
			break;
		case RESET_INSTRUCTION:
			xprintf(GSM, "boot2");
			break;
		default:
			xprintf(GSM, "boot3");
			break;
	}
}





uint8_t Restart_Cause(void)
{
	return 0;
}





bool Normal_Power_Mode(void)
{
	if (arQ.Pwr.PowerSaving == INACTIVE)
		return true;
	else
		return false;
}

bool Power_Saving_Mode(void)
{
	if (arQ.Pwr.PowerSaving == ACTIVE)
		return true;
	else
		return false;
}




void Reset_WD(void)
{
	// HAL_IWDG_Refresh(&hiwdg);
}





void getAirMar(void)
{
	uint8_t whilex = 1;

	AssignNMEA(&arQ.Amr, WIMDA);
	arQ.Amr.NMEANameFilled = 0;

	HAL_UART_Receive_IT(&huart1, &arQ.Amr.rxChar, 1); // Start UART character receiving



	while(whilex == 1)
	{
		// Get your Test Here
		if (arQ.Amr.NMEANameFilled == 1)
		whilex = 0;
	}

	arQ.Amr.NMEANameFilled = 0;


	// HAL_UART_AbortReceive_IT(&huart1);
}





void BootloadFlag(void)
{
	// writeCharEE(FIRMWARECHAR,EEPROM_BOOTLOAD_FLAG);
}

bool Data_Not_Yet_Taken(void)
{
	if (arQ.Flg.DATA_TAKEN == false)
		return true;
	else
		return false;
}*/




void xprintf(uint8_t stream, char *FormatString, ...)
{
	va_list args;
	char *sval;
	int  ival;
	float fval;
	char tempSTR[100];
	char cdcSTR[100];
	char format[10];
	int8_t i, j, x;
	uint8_t len;

	len = strlen(FormatString);
	va_start(args, FormatString);

	for (i = 0, j = 0; j < len; i++, j++)
	{
		tempSTR[i] = FormatString[j];

		if (FormatString[j] == '%')
		{
			tempSTR[i] = '\0';
			j++;

			if (stream == PC) CDC_Transmit_FS((uint8_t *)tempSTR, strlen(tempSTR));
			else if (stream == MCU) HAL_UART_Transmit(&huart1, (uint8_t *)tempSTR, strlen(tempSTR), HAL_MAX_DELAY);

			HAL_Delay(10);
			x = 0;
			format[x++] = '%';

			if (FormatString[j] != 's')
			{
				do format[x++] = FormatString[j++];
				while (FormatString[j] != 'd' && FormatString[j] != 'f');
			}

			if (FormatString[j] == 's')
			{
				sval = va_arg(args, char *);

				if (stream == PC) CDC_Transmit_FS((uint8_t *)sval, strlen(sval));
				else if (stream == MCU) HAL_UART_Transmit(&huart1, (uint8_t *)sval, strlen(sval), HAL_MAX_DELAY);
			}
			else if (FormatString[j] == 'd')
			{
				format[x] = 'd';
				format[x+1] = '\0';
				ival = va_arg(args, int);
				sprintf(cdcSTR, format, ival);

				if (stream == PC) CDC_Transmit_FS((uint8_t *)cdcSTR, strlen(cdcSTR));
				else if (stream == MCU) HAL_UART_Transmit(&huart1, (uint8_t *)cdcSTR, strlen(cdcSTR), HAL_MAX_DELAY);
			}
			else if (FormatString[j] == 'f')
			{
				format[x] = 'f';
				format[x+1] = '\0';
				fval = va_arg(args, double);
				sprintf(cdcSTR, format, fval);
				if (stream == PC) CDC_Transmit_FS((uint8_t *)cdcSTR, strlen(cdcSTR));
				else if (stream == MCU) HAL_UART_Transmit(&huart1, (uint8_t *)cdcSTR, strlen(cdcSTR), HAL_MAX_DELAY);
			}
			HAL_Delay(10);
			i = -1;
		}
	}
	tempSTR[i] = '\0';
	if (stream == PC) CDC_Transmit_FS((uint8_t *)tempSTR, strlen(tempSTR));
	else if (stream == MCU) HAL_UART_Transmit(&huart1, (uint8_t *)tempSTR, strlen(tempSTR), HAL_MAX_DELAY);
	va_end(args);
}


/*void PrintPC(char *szFormat, ...){
  uint8_t i;
  uint8_t *uintval;
  float *fval;
  int8_t *sval;
  uint8_t fmtDetect;
  char  *pt;
  va_list ptArg;

  if (BusyFlag == FREE_FLAG){
		BusyFlag = BUSY_FLAG;
		fmtDetect = 0;
		va_start(ptArg, szFormat);
		i = 0;

		for (pt = szFormat; *pt != 0; ++pt){
			str[i++] = *pt;
			if (fmtDetect == 0){
				if (*pt == '%') fmtDetect = 1;
			}
			else{
				switch (*pt){
					case 'c':
						break;

					case 'd':
						fmtDetect = 0;
						str[i++] = 0;
						uintval = va_arg(ptArg, uint8_t *);
						sprintf((char *) strDisplay, (char *) str, uintval);
						i = 0;
						CDC_Transmit_FS((uint8_t *)strDisplay, strlen((char *)strDisplay));
						break;

					case 'f':
						fmtDetect = 0;
						str[i++] = 0;
						fval = va_arg(ptArg, float *);
						sprintf((char *) strDisplay, (char *) str, fval);
						i = 0;
						CDC_Transmit_FS((uint8_t *)strDisplay, strlen((char *)strDisplay));
						break;

					case 's':
						fmtDetect = 0;
						str[i++] = 0;
						sval = va_arg(ptArg, int8_t *);
						sprintf((char *) strDisplay, (char *) str, sval);
						i = 0;
						CDC_Transmit_FS((uint8_t *)strDisplay, strlen(strDisplay));
						break;

					default:
						break;
				}
			}
		}
		str[i++] = 0;
		CDC_Transmit_FS((uint8_t *)str, strlen(str));
		va_end(ptArg);
		BusyFlag = FREE_FLAG;
  }
}*/
