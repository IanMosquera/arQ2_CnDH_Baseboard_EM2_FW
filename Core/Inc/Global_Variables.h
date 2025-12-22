/*
 * Global_Variables.h
 *
 *  Created on: Oct 13, 2025
 *      Author: IanMo
 */

#ifndef INC_GLOBAL_VARIABLES_H_
#define INC_GLOBAL_VARIABLES_H_

#include "main.h"

extern bool			f_DataTaken;
extern bool			f_LTCError;
extern bool			f_LTEBoardAttached;
extern bool			f_SMS;
extern bool 		f_UART;
extern bool			RESET_MCU;
extern bool 		USB_FLAG;

extern char			g_Buff_RetVal[];
extern char 		g_DateTime[];      /*03/04/25,13:50:00*/
extern char 		g_DateTimeZeroed[];
extern char			g_firmwareDesc[];
extern char			g_firmwareVer[];
extern char			g_hDataBuf[];
extern char 		g_healthData[];
extern char			g_Password[];
extern char			g_PCDMBoostCFG[];
extern char			g_RealData[];
extern char			g_RegNum1[12];
extern char			g_RegNum2[12];
extern char			g_RegNum3[12];
extern char 		g_RTCDateTime[25];
extern char 		g_SensorConfig[6];
extern char			g_SensorType;
extern char 		g_SerialNum[10];
extern char 		g_ServerNum[12];
extern char			g_SIMNum[12];
extern char 		g_SMSMessage[255];
extern char			g_Token[10];
extern char 		RESP_Buffer[100];
extern char 		TEMP_Buffer[100];

extern char 		UART_Buffer[100];
extern char 		UART_Char;
extern char			USB_BUFFER[255];


extern float		g_BoardPres;
extern float		g_BoardTemp;
extern uint16_t	FIRMWARE_YRS;
extern uint16_t Process_Ctr;
extern uint8_t 	ctr;
extern uint8_t 	DAY;
extern uint8_t	g_ErrorCount;
extern uint8_t	g_SendingTime;
extern uint8_t 	HRS_OLD;
extern uint8_t 	HRS;
extern uint8_t 	Mili_Sec_Ctr;
extern uint8_t 	MIN;
extern uint8_t 	MON;
extern uint8_t 	PREV_YRS;
extern uint8_t 	SEC;
extern uint8_t 	SMS_UNSENT_COUNT;
extern uint8_t 	UART_Index;
extern uint8_t 	YRS;

extern const float LOWBATLEVEL;
extern const float CELBATLEVEL;
#endif /* INC_GLOBAL_VARIABLES_H_ */
