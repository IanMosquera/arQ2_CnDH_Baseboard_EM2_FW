/*
 * Global_Variables.c
 *
 *  Created on: Oct 13, 2025
 *      Author: IanMo
 */


#include "stdbool.h"
#include "stdint.h"
#include "Global_Variables.h"

bool			f_DataTaken;
bool			f_LTCError;
bool			f_LTEBoardAttached;
bool			f_SMS;
bool 			f_UART;
bool			RESET_MCU;
bool 			USB_FLAG;

char 			g_DateTime[18];
char			g_Buff_RetVal[300];
char 			g_DateTimeZeroed[18];
char			g_firmwareDesc[30];
char			g_firmwareVer[7];
char			g_hDataBuf[300];
char 			g_healthData[300];
char			g_Password[10];
char			g_PCDMBoostCFG[3];
char			g_RealData[255];
char			g_RegNum1[12];
char			g_RegNum2[12];
char			g_RegNum3[12];
char 			g_RTCDateTime[25];
char 			g_SensorConfig[6];
char			g_SensorType;
char 			g_SerialNum[10];
char 			g_ServerNum[12];
char 			g_SIMNum[12];
char 			g_SMSMessage[255];
char			g_Token[10];
char 			RESP_Buffer[100];
char 			TEMP_Buffer[100];
char 			UART_Buffer[100];
char 			UART_Char;
char			USB_BUFFER[255];


float			g_BoardPres;
float			g_BoardTemp;
uint16_t	FIRMWARE_YRS;
uint16_t 	Process_Ctr;
uint8_t 	PREV_YRS;
uint8_t 	YRS;
uint8_t 	ctr;
uint8_t 	DAY;
uint8_t 	HRS_OLD;
uint8_t 	HRS;
uint8_t 	Mili_Sec_Ctr;
uint8_t 	MIN;
uint8_t 	MON;
uint8_t 	SEC;
uint8_t 	UART_Index;
uint8_t		g_SendingTime;
uint8_t		g_ErrorCount;
uint8_t 	SMS_UNSENT_COUNT;

const float LOWBATLEVEL = 11.80f;
const float CELBATLEVEL = 3.20f;
