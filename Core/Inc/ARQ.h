/*
 * ARQ.h
 *
 *  Created on: Feb 24, 2026
 *      Author: IanMo
 */

#ifndef INC_ARQ_H_
#define INC_ARQ_H_

#include "main.h"
#include "tim.h"

typedef enum{
	PC,
	GSM,
	PMCU
}Stream_t;


typedef enum{
	fail,
	success
}Function_Return_Status;


#define BLE_ENABLED 1
#define arQTimer &htim17



typedef enum{
	Server_Num				= 'A',
	SIM_Num						= 'B',
	Sending_Time			=	'C',
	Password					= 'D',
	Date_Time					= 'E',
	Sensor_Config			= 'F',
	Reg_Num1					= 'G',
	Reg_Num2					= 'H',
	Reg_Num3					= 'I',
	List_Reg_Num			= 'J',
	Delete_Reg_Num		= 'K',
	ARQ_Serial_Num		= 'L',
	Get_Sensor_Data		= 'M',
	Send_STR_Via_SMS	= 'N',
	Reset_PMCUx				= 'X',
	Display_Menu			= 'Y'
}DEBUG_MENU_options;



extern bool f_Fault_Incremented;
extern bool f_InitState;
extern bool f_PMCU_CheckTime;
extern bool f_PMCU_CMD;
extern bool f_PMCU_MSG;
extern bool f_PMCU_QRY;
extern bool f_PMCU_Responds;
extern bool f_Printed;
extern bool f_USB;

extern char BLE_BUFFER[];
extern char g_DateTime[];
extern char g_firmwareVer[];
extern char g_Password[];
extern char g_Reg1[];
extern char g_Reg2[];
extern char g_Reg3[];
extern char g_SensorConfig[];
extern char g_SerialNum[];
extern char g_ServerNum[];
extern char g_SIMNum[];
extern char PMCU_Buffer[];
extern char	TEMP_Buffer[];
extern char UART_Buffer[];
extern char USB_BUFFER[];


extern uint16_t Process_Ctr;
extern uint8_t UART_Index;
extern uint8_t g_Fault_Ctr;
extern uint8_t g_RGAccuTipsData;
extern uint8_t g_RGTipsData;
extern uint8_t Mili_Sec_Ctr;
extern uint8_t UART_CHAR;



bool Get_DateTime_From_PMCU(void);
bool Retry(bool (*func)(void), uint8_t maxRetry);


uint8_t Examine_BLE_Buffer(void);
uint8_t Send_DEBUG_To_PMCU(void);
uint8_t Set_Variable(char *variable, char *value);


void Clear_Buffer(char *pBuffer, uint16_t len);
void Clear_UART_Buffer(void);
void Clear_USB_Buffers(void);
void Extract_PMCUCommand(void);
void Extract_Value(char *dest, char *source);
void Extract_Variable(char *dest, char *source);
void Get_Config_From_PMCU(void);
void PMCU_Check(void);
void Print_BLE_BUFFER_to_USB(void);
void Print_PMCU_Message_To_USB(void);
void Reset_PMCU(void);
void Respond_Attached_To_PMCU(void);
void xprintf(uint8_t stream, char *FormatString, ...);

#endif /* INC_ARQ_H_ */
