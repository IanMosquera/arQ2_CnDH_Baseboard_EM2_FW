/*
 * ARQ.c
 *
 *  Created on: Feb 24, 2026
 *      Author: IanMo
 */


#include "ARQ.h"
#include "rtc.h"
#include "Timer.h"
#include "DateTime.h"
#include "UtilityFunctions.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <usart.h>
#include <usbd_cdc_if.h>


bool f_CheckPMCU = false;
bool f_Fault_Incremented = false;
bool f_InitState = true;
bool f_PMCU_Responds = false;
bool f_PMCU_CMD = false;
bool f_PMCU_MSG = false;
bool f_PMCU_QRY = false;
bool f_Printed = false;
bool f_USB = false;

char g_DateTime[18];
char g_SIMNum[12];
char g_firmwareVer[4];
char TEMP_Buffer[100];
char UART_Buffer[100];
char USB_BUFFER[255];

uint16_t Process_Ctr = 0;

uint8_t CHAR_CTR;
uint8_t g_Fault_Ctr = 0;
uint8_t g_RGAccuTipsData;
uint8_t g_RGTipsData;
uint8_t g_SendingTime;
uint8_t Mili_Sec_Ctr = 0;
uint8_t UART_CHAR;



/*************************** Functions ****************************************/





/******************************************************************************
  * @brief	Clear buffer
  * @param	pBuffer	pointer to a buffer
  * @return None
  * @FVer		7.0
  * ***************************************************************************
*/
void	Clear_Buffer(char *pBuffer, uint16_t len){
	memset(pBuffer, '\0', len);
}






void Clear_USB_Buffers(void)
{
	memset(USB_BUFFER, '\0', 255);
	f_USB = false;
}


void ExtractValue(char *dest, char *source){
	uint8_t i = 6;
	do{
		dest[i-6] = source[i];
		i++;
	}while(source[i] != '\0');
}


void ExtractVariable(char *dest, char *source){
	strncpy(dest, source+2, 3);
}






void Get_Config(void){
	xprintf(PC, "\r\n#Getting arQ Configuration\r\n");
	HAL_Delay(100);


	xprintf(PMCU, "G_SDT\r\n");
	if (Get_Desired_Response("SDT:", 10)){
		xprintf(PC, "Sending Time: %i\r\n", atoi(RESP_Buffer));
	}
	HAL_Delay(100);


	xprintf(PMCU, "G_SVR\r\n");
	if (Get_Desired_Response("SVR:", 10)){
		xprintf(PC, "Server number: %s\r\n", RESP_Buffer);
	}
	HAL_Delay(100);


	xprintf(PMCU, "G_SIM\r\n");
	if (Get_Desired_Response("SIM:", 10)){
		xprintf(PC, "SIM Card number: %s\r\n", RESP_Buffer);
	}
	HAL_Delay(100);


	xprintf(PMCU, "G_SRL\r\n");
	if (Get_Desired_Response("SRL:", 10)){
		xprintf(PC, "arQ Serial Number: %s\r\n", RESP_Buffer);
	}
	HAL_Delay(100);


	xprintf(PMCU, "G_SCF\r\n");
	if (Get_Desired_Response("SCF:", 10)){
		xprintf(PC, "Sensor Config: %s\r\n", RESP_Buffer);
	}
	HAL_Delay(100);


	xprintf(PMCU, "G_PSW\r\n");
	if (Get_Desired_Response("PSW:", 10)){
		xprintf(PC, "Password: %s\r\n", RESP_Buffer);
	}
	HAL_Delay(100);
}








bool Get_DateTime_From_PMCU(void){
	//HAL_GPIO_WritePin(PMC, GPIO_Pin, PinState);
	xprintf(PMCU, "G_DTM\r\n");
	if (Get_Desired_Response("DTM:", 10))
		return true;
	return false;
}






/******************************************************************************
  * @brief	Get character from USB
  * @param	None
  * @return None
  * @FVer		7.0
  * ***************************************************************************
*/
char GetChar(uint8_t timeout){
	bool i = true;

	Clear_Buffer(USB_BUFFER, 255);
	Task_TimeOut_Start();

	while(i){
		if (USB_BUFFER[1] == '\r' && USB_BUFFER[2] == '\n'){
			return USB_BUFFER[0];
		}

		if (Task_TimeOut(timeout)){
			return '\0';
		}
	}
	return '\0';
}







/******************************************************************************
  * @brief	Retries a function x times
  * @param	func pointer to a function
  * @param	maxRetry maximum number of retries
  * @return None
  * @FVer		7.0
  * ***************************************************************************
*/
bool Retry(bool (*func)(void), uint8_t maxRetry){
	uint8_t i;

	if (func == NULL || maxRetry < 1) // test for invalid parameters
		return false;


	for (i = 0; i < maxRetry; i++){ // retry function for x times
		if (func() == true) // return tru if function succeed
			return true;

		if (i == (maxRetry-1)) // return false after x times of retries
			return false;

		HAL_Delay(500);
	}

	return false;
}






void Interrupt_PMCU(void){

}





void PMCU_Check(void){
	if ((SEC > 25) && (SEC < 31))
		f_CheckPMCU = true;
	else
		f_CheckPMCU = false;




/*	if ((SEC > 25) && (SEC < 31)){
		f_CheckPMCU = true;
		HAL_GPIO_WritePin(STAT_GPIO_Port, STAT_Pin, GPIO_PIN_SET);
		Interrupt_PMCU();

		if ((SEC == 24) && (!f_PMCU_Responds)){
			//Reset_PMCU();
		}

		if (f_PMCU_Responds){

			//HAL_GPIO_WritePin(STAT_GPIO_Port, STAT_Pin, GPIO_PIN_RESET);
		}

	}
	else{
		f_CheckPMCU = false;
		HAL_GPIO_WritePin(STAT_GPIO_Port, STAT_Pin, GPIO_PIN_RESET);
		Uninterrupt_PMCU();
		f_PMCU_Responds =  false;
	}*/
}





void Uninterrupt_PMCU(void){

}






void RTC_Init(void)
{
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};

	RTC_Assign_Date(&sDate);
	RTC_Assign_Time(&sTime);
}

void RTC_Assign_Date(RTC_DateTypeDef *pDate)
{
	pDate->WeekDay		= RTC_WEEKDAY_THURSDAY;
	pDate->Month			= RTC_MONTH_MARCH;
	pDate->Date				= 0x03;
	pDate->Year				= 0x25;
	if (HAL_RTC_SetDate(&hrtc, pDate, RTC_FORMAT_BCD) != HAL_OK) Error_Handler();
}

void RTC_Assign_Time(RTC_TimeTypeDef *pTime)
{
	pTime->Hours 			= 0x09;
	pTime->Minutes		=	0x00;
	pTime->Seconds		= 0x00;
	pTime->SubSeconds	= 0x00;
	pTime->DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	pTime->StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, pTime, RTC_FORMAT_BCD) != HAL_OK) Error_Handler();
}

void RTC_ShowDateTime(void)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;

  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
  HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);

  xprintf(PC, "Date and Time: %02d/%02d/%02d,%02d:%02d:%02d\r\n",
  		sdatestructureget.Year, sdatestructureget.Month, sdatestructureget.Date,
  		stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
}





uint8_t SetVariable(char *variable, char *value){
	if (UTL_CompareEqual(variable, "DTM")){
		DTM_DateTime_Set(value);
		DTM_DateTime_Get();
		xprintf(PC, "Date and Time Synched: %s\r\n", g_DateTime);
	}

	else if (UTL_CompareEqual(variable, "SIM")){
		strcpy(g_SIMNum, value);
		xprintf(PC, "Sim Number Synched: %s\r\n", g_SIMNum);
	}

	HAL_Delay(200);
	return 0;
}




void xprintf(uint8_t stream, char *FormatString, ...){
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
			else if (stream == PMCU) HAL_UART_Transmit(&huart1, (uint8_t *)tempSTR, strlen(tempSTR), HAL_MAX_DELAY);

			HAL_Delay(50);
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
				else if (stream == PMCU) HAL_UART_Transmit(&huart1, (uint8_t *)sval, strlen(sval), HAL_MAX_DELAY);
			}
			else if (FormatString[j] == 'd')
			{
				format[x] = 'd';
				format[x+1] = '\0';
				ival = va_arg(args, int);
				sprintf(cdcSTR, format, ival);

				if (stream == PC) CDC_Transmit_FS((uint8_t *)cdcSTR, strlen(cdcSTR));
				else if (stream == PMCU) HAL_UART_Transmit(&huart1, (uint8_t *)cdcSTR, strlen(cdcSTR), HAL_MAX_DELAY);
			}
			else if (FormatString[j] == 'f')
			{
				format[x] = 'f';
				format[x+1] = '\0';
				fval = va_arg(args, double);
				sprintf(cdcSTR, format, fval);
				if (stream == PC) CDC_Transmit_FS((uint8_t *)cdcSTR, strlen(cdcSTR));
				else if (stream == PMCU) HAL_UART_Transmit(&huart1, (uint8_t *)cdcSTR, strlen(cdcSTR), HAL_MAX_DELAY);
			}
			HAL_Delay(50);
			i = -1;
		}
	}
	tempSTR[i] = '\0';
	if (stream == PC) CDC_Transmit_FS((uint8_t *)tempSTR, strlen(tempSTR));
	else if (stream == PMCU) HAL_UART_Transmit(&huart1, (uint8_t *)tempSTR, strlen(tempSTR), HAL_MAX_DELAY);
	va_end(args);
}




