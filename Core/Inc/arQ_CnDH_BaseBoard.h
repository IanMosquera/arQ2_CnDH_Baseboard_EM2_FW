/*
 * arQ.h
 *
 *  Created on: Jun 26, 2025
 *      Author: IanCMosquera
 */

#ifndef INC_ARQ_CNDH_BASEBOARD_H_
#define INC_ARQ_CNDH_BASEBOARD_H_

#include "main.h"

#include "stdbool.h"


#define ACTIVE 		1
#define INACTIVE 	0

#define WDT_TIMEOUT 0
#define NORMAL_POWER_UP		1
#define RESET_INSTRUCTION	2

// Shutdown Condition
#define VIA_AT_CMD		1
#define VIA_RST_PIN		2
#define VIA_IGT_PIN		3

// xprintf stream
#define PC			0
#define MCU			1

typedef struct
{
	char Firmware[8];
	char FirmwareDesc[30];
	char FirmwareYear[5];
	char SimNumber[12];
	char SerialNumber[12];
	char SensorType;
	char SensorConfig[6];
	char ServerNumber[12];
	char RegisteredNumber1[12];
	char RegisteredNumber2[12];
	char RegisteredNumber3[12];
	char CumulativeRain[5];
	char PowerBoardConfig[3];
	char Password[10];

	uint8_t SendingTime;
	uint8_t SleepTime;
}systemConfig_t;

typedef struct{
	uint8_t ctr;
	uint8_t Sec;
	uint8_t Min;
	uint8_t Hour;
	uint8_t Hour_Old;
	uint8_t Days;
	uint8_t Month;

	uint16_t Year;
	uint16_t Year_Prev;

	char DateTime[30];
	char RTCDateTime[50];

	bool ResetCPU;
}dateTime_t;

typedef struct
{
	char ICCID[32];
	char IMEI[32];
	char Network[8];
	char BackDoorNumber[16];
	char SystemInfo[32];
	char RSSI[8];
	char RSSIdbm[8];
	char LTEBand[8];
	char RSRQ[8];
	char RSRP[8];
	char SINR[8];
	char ConnectionMode[8];
	char ConnectionStatus[8];

	uint8_t SMSIndex;
	uint8_t BLEMode;
	uint8_t RIMode;
	uint8_t CallFlag;
}LTE_t;

typedef struct
{
	char SMS_RCV[255];
	char SMS_TOBE_SENT[300];
	char UART_DATA[255];
	char RXD2_DATA[255];
	char USB_BUFFER[255];
	char DESIRED_RESPONSE[17];
	char GSM_RESPONSE[255];
	char PC_MSG[300];
	char FUNCRETURNVAL[255];
	char FUNCSMALLSTR[15];
	char REALDATA[255];
	char ZEROED[20];
	char WATER_LVL[7];
	char LTC_BUFFER[63];
	char TOKEN[10];
	char FROMSERIALPC[25];
	char STATCAT;
} buffers_t;


typedef struct
{
	uint8_t SendSMSResult;
	uint8_t SendSMSResult_ASTI;
	uint8_t UnsentCount;

	bool	TwoServers;

	float Current;
} Variables_t;


typedef struct
{
	uint8_t  PowerSaving;

	char 	VbatSTR[10];
	char	BoardCurSTR[10];
	char	PCDMVoltSTR[10];
	char	PCDMCurSTR[10];
	char 	EXTVBATSTR[10];
	char	VBoost1STR[10];
	char	VBoost2STR[10];

	float VBAT;
	float BoardCurrent;
	float	PCDMVoltage;
	float PCDMCurrent;
	float ExtVBAT;
	float VBoost1;
	float VBoost2;
	float Primary_Batt_Voltage;
	float Secondary_Batt_Voltage;
}power_t;

typedef struct
{
	bool EXT_FLAG;
	bool USB_SERIAL_FLAG;
	bool UART_SERIAL_FLAG;
	bool PMCU_STAT_FLAG_OK;
	bool CUM_RAIN_RESET_FLAG;
	bool MIDNIGHT_RESET_LTE_FLAG;
	bool PROC_START;
	bool PROC_END;
	bool SENSOR_DEBUG;
	bool SENDING_FLAG;
	bool SYNC_FLAG;
	bool TEST_FLAG;
	bool REPLYTOSENDER;
	bool THESAMEMINUTE_FLAG;
	bool DATA_TAKEN;
	bool RETURN_FLAG;
}flag_t;


typedef struct
{
	char RainCount[5];
	char RainCummulative[5];
}Raint_t;

typedef struct
{
	float Temperature;
	char	TempSTR[5];
}Temp_t;

typedef struct
{
	float Pressure;
	char	PressSTRR[10];
}Pressure_t;

typedef struct
{
	uint8_t INTERRUPTCHECKER;
	uint8_t WRITE_CNTR;
	uint8_t WRITE2_CNTR;
	uint8_t READ_CNTR;
}Counter_t;

typedef struct
{
	systemConfig_t 	Cfg;
	dateTime_t			DTm;
	buffers_t				Buf;
	power_t					Pwr;
	flag_t					Flg;
	Raint_t					Rin;
	Temp_t					Tmp;
	Pressure_t			Prs;
	Counter_t				Ctr;
	Variables_t			Var;
	LTE_t						LTE;
}arQ_t;






// Exported Functions
//void DateTimeStatus(void);
//void Print_Header_Text(void);
//void Sensor_Init(void);
//void Get_Config_Init_Values(void);
//void Boot_Status(void);
//void getAirMar(void);
//void BootloadFlag(void);
//uint8_t Restart_Cause(void);

// Revise Functions from Ported Codes
//bool Normal_Power_Mode(void);
//bool Power_Saving_Mode(void);
//void Reset_WD(void);
//void Sensor_Poll(void);
//void Get_arQ_Data(void);
//void Update_ARQ_Time(void);
//void Append_To_RealData(char *pData);
//void SMSCall_Interrupt_Check(void);
//void USBSerial_Interrupt_Check(void);
//void ARQ_Reset_Check(void);
//void Cummulative_Rain_Reset_Check(void);
//void LTE_MidNight_Reset_Check(void);
//void Enable_Normal_Power_Mode(void);
//void Trigger_Lora_Transmit_Pin(void);
//bool Data_Not_Yet_Taken(void);
void xprintf(uint8_t stream, char *FormatString, ...);

#endif /* INC_ARQ_CNDH_BASEBOARD_H_ */
