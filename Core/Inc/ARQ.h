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


#define BLE_ENABLED 1

#define arQTimer &htim17


extern bool f_CheckPMCU;
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
extern char g_SIMNum[];
extern char g_firmwareVer[];
extern char	TEMP_Buffer[];
extern char UART_Buffer[];
extern char USB_BUFFER[];


extern uint16_t Process_Ctr;
extern uint8_t CHAR_CTR;
extern uint8_t g_Fault_Ctr;
extern uint8_t g_RGAccuTipsData;
extern uint8_t g_RGTipsData;
extern uint8_t Mili_Sec_Ctr;
extern uint8_t UART_CHAR;



bool Get_DateTime_From_PMCU(void);
bool Retry(bool (*func)(void), uint8_t maxRetry);

char GetChar(uint8_t timeout);

uint8_t CurrentState_Base_On_BLE_String(char *pBuf);
uint8_t Set_Variable(char *variable, char *value);

void BLE_Print_to_PMCU(void);
void BLE_Print_to_USB(void);
void Clear_Buffer(char *pBuffer, uint16_t len);
void Clear_USB_Buffers(void);
void Extract_PMCUCommand(void);
void Extract_Value(char *dest, char *source);
void Extract_Variable(char *dest, char *source);
void Get_Config(void);
void Interrupt_PMCU(void);
void PMCU_Check(void);
void Print_UARTBuffer(void);
void Reset_PMCU(void);
void RTC_Assign_Date(RTC_DateTypeDef *pDate);
void RTC_Assign_Time(RTC_TimeTypeDef *pTime);
void RTC_Init(void);
void RTC_ShowDateTime(void);
void Uninterrupt_PMCU(void);
void xprintf(uint8_t stream, char *FormatString, ...);

#endif /* INC_ARQ_H_ */
