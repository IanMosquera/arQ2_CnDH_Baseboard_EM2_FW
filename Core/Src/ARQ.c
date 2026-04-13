/*
 * ARQ.c
 *
 *  Created on: Feb 24, 2026
 *      Author: IanMo
 */


#include "ARQ.h"
#include "DateTime.h"
#include "Debug.h"
#include "rtc.h"
#include "StateMachine.h"
#include "Timer.h"
#include "UtilityFunctions.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <usart.h>
#include <usbd_cdc_if.h>


#if (BLE_ENABLED)
#include "ble_types.h"
#include "custom_stm.h"
#include "stm32_seq.h"
#endif




bool f_Fault_Incremented = false;
bool f_InitState = true;
bool f_PMCU_Responds = false;
bool f_PMCU_CMD = false;
bool f_PMCU_MSG = false;
bool f_PMCU_QRY = false;
bool f_Printed = false;
bool f_USB = false;

char BLE_BUFFER[255];
char g_DateTime[18];
char g_firmwareVer[4];
char g_Password[9] = "EMBEDDED";
char g_Reg1[12] = "09091234567";
char g_Reg2[12] = "09091230000";
char g_Reg3[12] = "09090004567";
char g_SensorConfig[4];
char g_SerialNum[13];
char g_ServerNum[12];
char g_SIMNum[12];
char PMCU_Buffer[100];
char TEMP_Buffer[100];
char UART_Buffer[100];
char USB_BUFFER[255];

uint16_t Process_Ctr = 0;

uint8_t UART_Index;
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




void Clear_UART_Buffer(void){
	memset(UART_Buffer, '\0', 100);
	UART_Index = 0;
}





void Clear_USB_Buffers(void){
	memset(USB_BUFFER, '\0', 255);
	f_USB = false;
}








void Examine_BLE_Buffer(void){
	char try_again[16] = "Try Again\r\n";
	char invalid_char[32] = "Invalid character input!\r\n";

	if (g_currentState == s_IDLE){
		if (Correct_DEBUG_Detected()){
			if (!Send_DEBUG_To_PMCU()){
				SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)try_again);
			}
			UTIL_SEQ_SetTask(1<<CFG_TASK_SETTINGSMENU, CFG_SCH_PRIO_0);
		}
	}

	if (g_currentState == s_DBUG){
		if (DEBUG_Exit_Detected()){
			if (!Exit_Debug()){
				SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)try_again);
			}
		}

		if (CHAR_is_Not_Within_In_AtoZ()){
			HAL_Delay(100);
			SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)invalid_char);
		}

		char val[100];
		Extract_Value_From_BLE_DebugMessage(val);
		Send_DEBUG_Value_to_PMCU(val);
	}
}




void Exit_BLE_DEBUG(void){
	char ble_msg[32] = "Timeout! Exiting BLE Debug\r\n";
	uint8_t len = strlen(ble_msg);

	g_currentState =  s_IDLE;
	SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)ble_msg);
	CDC_Transmit_FS((uint8_t *)ble_msg, len);

	Clear_UART_Buffer();
	HAL_UART_Receive_IT(&huart1, &UART_CHAR, 1);
}


// Sample S_SVR:09191234567
void Extract_PMCUCommand(void){
	char var[4];
	char val[30];

	Extract_Variable(var, UART_Buffer);

	if (UART_Buffer[0] == 'S'){
		Extract_Value(val, UART_Buffer);
		Set_Variable(var, val);
	}
	else if (UART_Buffer[0] == 'G'){

	}
}



// Sample S_SVR:09191234567
void Extract_Value(char *dest, char *source){
	uint8_t i = 6;
	do{
		dest[i-6] = source[i];
		i++;
	}while(source[i] != '\0');
}








void Extract_Variable(char *dest, char *source){
	strncpy(dest, source+2, 3);
}





void Get_Config_From_PMCU(void){
	char str[300] = "DEBUG Recognized\r\n";

	xprintf(PMCU, "DEBUG##");
	if (!Get_Desired_Response("ACK", 2)){
		strcpy(str, "ERROR sync\r\n");
		SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)str);
		return;
	}


	xprintf(PMCU, "G_FVR##");
	if (Get_Desired_Response("FVR:", 10)){
		strcpy(g_firmwareVer, RESP_Buffer);
		HAL_Delay(50);
	}

	xprintf(PMCU, "G_SVR##");
	if (Get_Desired_Response("SVR:", 10)){
		strcpy(g_ServerNum, RESP_Buffer);
		HAL_Delay(50);
	}

	xprintf(PMCU, "DONE##");

	sprintf(str, "%s, %s\r\n",
			g_firmwareVer,
			g_ServerNum);

	SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)str);
}






bool Get_DateTime_From_PMCU(void){
	//HAL_GPIO_WritePin(PMC, GPIO_Pin, PinState);
	xprintf(PMCU, "G_DTM\r\n");
	if (Get_Desired_Response("DTM:", 10))
		return true;
	return false;
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






void Print_BLE_BUFFER_to_USB(void){
	uint8_t size = 0;

	while(BLE_BUFFER[size] != '\0') size++;

	CDC_Transmit_FS((uint8_t *)BLE_BUFFER, size);
}





void Print_PMCU_Message_To_USB(void){
	uint8_t len = strlen(UART_Buffer);
	CDC_Transmit_FS((uint8_t *)UART_Buffer, len);

	SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)UART_Buffer);
	Clear_UART_Buffer();
	HAL_UART_Receive_IT(&huart1, &UART_CHAR, 1);
}







void Reset_PMCU(void){
	HAL_GPIO_WritePin(NRST_PMCU_GPIO_Port, NRST_PMCU_Pin, GPIO_PIN_RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(NRST_PMCU_GPIO_Port, NRST_PMCU_Pin, GPIO_PIN_SET);
}






void Respond_Attached_To_PMCU(void){
	if (Strings_Are_Equal(UART_Buffer, "Status")){
		xprintf(PMCU, "Attached\r\n");
	}
}







uint8_t Set_Variable(char *variable, char *value){

	if (Strings_Are_Equal(variable, "DTM")){
		DTM_DateTime_Set(value);
		DTM_DateTime_Get();
		xprintf(PC, "Date and Time Synched: %s\r\n", g_DateTime);
		return 0;
	}

	if (Strings_Are_Equal(variable, "SIM")){
		strcpy(g_SIMNum, value);
		xprintf(PC, "Sim Number Synched: %s\r\n", g_SIMNum);
		return 0;
	}

	if (Strings_Are_Equal(variable, "SDT")){
		g_SendingTime = atoi(value);
		xprintf(PC, "Sim Number Synched: %s\r\n", g_SendingTime);
		return 0;
	}


	if (Strings_Are_Equal(variable, "SVR")){
		strcpy(g_ServerNum, value);
		xprintf(PC, "Sim Number Synched: %s\r\n", g_ServerNum);
		return 0;
	}

	HAL_Delay(200);
	return 0;
}





uint8_t Send_DEBUG_Value_to_PMCU(char *value){
	char deleted[16] = "Deleted\r\n";
	char invalid_val[32] = "Invalid value\r\n";
	char not_retrieved[32] = "Not retrieved\r\n";
	char reset_pmcu[32] = "Resetting PMCU\r\n";
	char returned_value[100];
	char saved[16] = "SAVED!\r\n";
	char try_again[16] = "Try again\r\n";
	char pmcu_msg[16];

	/*GUARD CLAUSE*/

	if (!Strings_Are_Equal(value, "NULL")){
		if (!Valid_Value_Format(value)){
			SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)invalid_val);
			return fail;
		}
	}



	switch (BLE_BUFFER[0])
	{
		case Server_Num:{
			sprintf(pmcu_msg, "S_SVR:%s\r\n", value);
			HAL_UART_Transmit(&huart1, (uint8_t *)pmcu_msg, strlen(pmcu_msg), 100);

			if (Get_Desired_Response("ACK", 3)){

				if (Strings_Are_Equal(value, "NULL")){
					strcpy(g_ServerNum, RESP_Buffer);
					sprintf(returned_value, "Server Number: %s\r\n", g_ServerNum);
					SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)returned_value);
					return success;
				}
				else{
					SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)saved);
					return success;
				}
			}

			SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)try_again);
			return fail;
		}

		case SIM_Num:{

			xprintf(PMCU, "S_SIM:%s", value);
			if (Get_Desired_Response("ACK", 1)){
				SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)saved);
				return success;
			}
			return fail;
		}

		case Sending_Time:{
			xprintf(PMCU, "S_SDT:%s", value);
			if (Get_Desired_Response("ACK", 1)){
				SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)saved);
				return success;
			}
			return fail;
		}

		case Password:{
			xprintf(PMCU, "S_PWD:%s", value);
			if (Get_Desired_Response("ACK", 1)){
				SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)saved);
				return success;
			}
			return fail;
		}

		case Date_Time:{
			sprintf(pmcu_msg, "S_DTM:%s\r\n", value);
			HAL_UART_Transmit(&huart1, (uint8_t *)pmcu_msg, strlen(pmcu_msg), 100);

			if (Get_Desired_Response("ACK", 3)){

				if (Strings_Are_Equal(value, "NULL")){
					DTM_DateTime_Set(RESP_Buffer);
					sprintf(returned_value, "DTM: %s\r\n", RESP_Buffer);
					SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)returned_value);
					return success;
				}
				else{
					SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)saved);
					DTM_DateTime_Set(value);
					return success;
				}
			}

			SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)try_again);
			return fail;
		}

		case Sensor_Config:{
			if (Strings_Are_Equal(value, "NULL")){
				xprintf(PMCU, "S_CFG:%s\r\n", "NULL");
				if (Get_Desired_Response("ACK", 3)){
					char a[4] = "";
					strncpy(a, RESP_Buffer, 3);
					sprintf(returned_value,"Config: %s\r\n", a);
					SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)returned_value);
					return success;
				}

				SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)not_retrieved);
				return fail;
			}

			xprintf(PMCU, "S_CFG:%s\r\n", value);
			if (Get_Desired_Response("ACK", 3)){
				SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)saved);
				strcpy(g_SensorConfig, value);
				return success;
			}
			return fail;
		}

		case Reg_Num1:
		case Reg_Num2:
		case Reg_Num3:{
			xprintf(PMCU, "S_RN%c:%s", (BLE_BUFFER[0]-70) ,value);
			if (Get_Desired_Response("ACK", 1)){
				SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)saved);
				if (BLE_BUFFER[0] == Reg_Num1) strcpy(g_Reg1, value);
				if (BLE_BUFFER[0] == Reg_Num2) strcpy(g_Reg2, value);
				if (BLE_BUFFER[0] == Reg_Num3) strcpy(g_Reg3, value);
				return success;
			}
			return fail;
		}

		case List_Reg_Num:{
			sprintf(returned_value, "%s, %s, %s\r\n", g_Reg1, g_Reg2, g_Reg3);
			return success;
		}

		case Delete_Reg_Num:{
			if (atoi(value) == 1){
				xprintf(PMCU, "S_DN1:%s", value);
				if (Get_Desired_Response("ACK", 1)){
					SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)deleted);
					strcpy(g_Reg1, "");
					return success;
				}
				SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)try_again);
				return fail;
			}

			if (atoi(value) == 2){
				xprintf(PMCU, "S_DN2:%s", value);
				if (Get_Desired_Response("ACK", 1)){
					SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)deleted);
					strcpy(g_Reg2, "");
					return success;
				}
				SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)try_again);
				return fail;
			}

			if (atoi(value) == 3){
				xprintf(PMCU, "S_DN3:%s", value);
				if (Get_Desired_Response("ACK", 1)){
					SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)deleted);
					strcpy(g_Reg3, "");
					return success;
				}
				SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)try_again);
				return fail;
			}
			return fail;
		}

		case ARQ_Serial_Num:{
			xprintf(PMCU, "S_SRL:%s\r\n", value);
			if (Get_Desired_Response("ACK", 3)){
				SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)saved);
				strcpy(g_SerialNum, value);
				return success;
			}
			SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)try_again);
			return fail;
		}

		case Get_Sensor_Data:{
			if (Strings_Are_Equal(value, "NULL")){
				xprintf(PMCU, "S_AAA:%s\r\n", "NULL");
				if (Get_Desired_Response("ACK", 3)){
					char TOKEN[2] = ",";
					char *pData = NULL;

					pData = strtok(RESP_Buffer, TOKEN);
					g_RGTipsData = atoi(pData);
					pData = strtok(0, TOKEN);
					g_RGAccuTipsData =  atoi(pData);

					sprintf(returned_value, "Tips: %d, Accu:%d\r\n", g_RGTipsData, g_RGAccuTipsData);
					SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)returned_value);
					return success;
				}

				SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)not_retrieved);
				return false;
			}
		}

		case Reset_PMCUx:{
			SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)reset_pmcu);
			Reset_PMCU();
			return success;
		}

		case Display_Menu:{
			Print_Setting_Menu();
			return success;
		}


		default:{
			break;
		}
	}

	return success;
}







void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == PMCU_INT_Pin)
		f_PMCU_Responds = true;
}




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart == &huart1){

		if (UART_Index == 0)
			memset(TEMP_Buffer, '\0', 100);

		TEMP_Buffer[UART_Index] = UART_CHAR;

		if (UART_Index > 100)
			UART_Index = 0;
		else
			UART_Index++;


		if ((TEMP_Buffer[UART_Index-1] == '^') && (TEMP_Buffer[UART_Index-2] == '^')){
			UART_Index = 0;
			snprintf(UART_Buffer, strlen(TEMP_Buffer)-1, "%s", TEMP_Buffer);

			if (g_currentState != s_DBUG){
				UTIL_SEQ_SetTask(1<<CFG_TASK_PrintPMCUMessageToUSB, CFG_SCH_PRIO_0);
			}

			if (Strings_Are_Equal(UART_Buffer, "EXIT_DEBUG")){
				UTIL_SEQ_SetTask(1<<CFG_TASK_ExitBLEDEBUG, CFG_SCH_PRIO_0);
			}
		}

		if ((TEMP_Buffer[UART_Index-1] == '$') && (TEMP_Buffer[UART_Index-2] == '$')){
			TEMP_Buffer[UART_Index-2] = '\0'; // Remove trailing \r
			TEMP_Buffer[UART_Index-1] = '\0';	// Remove trailing \n
			UART_Index = 0;
			strcpy(UART_Buffer, TEMP_Buffer);
			UTIL_SEQ_SetTask(1<<CFG_TASK_EXTRACTPMCUCMD, CFG_SCH_PRIO_0);
		}

		if ((TEMP_Buffer[UART_Index-1] == '?') && (TEMP_Buffer[UART_Index-2] == '?')){
			UART_Index = 0;
			snprintf(UART_Buffer, strlen(TEMP_Buffer)-1, "%s", TEMP_Buffer);
			f_PMCU_QRY = true;
			UTIL_SEQ_SetTask(1<<CFG_TASK_PRINTTOPMCU, CFG_SCH_PRIO_0);
		}

		HAL_UART_Receive_IT(&huart1, &UART_CHAR, 1);
	}
}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == arQTimer){


		if (Mili_Sec_Ctr == 20){
			Mili_Sec_Ctr = 0;
			TMR_SEC_Count();
			HAL_GPIO_TogglePin(STAT_GPIO_Port, STAT_Pin);
			if (SEC == 30){
				if (!f_PMCU_Responds)
					g_Fault_Ctr++;
				else
					g_Fault_Ctr = 0;

				if (g_Fault_Ctr == 3){
					g_Fault_Ctr = 0;
					UTIL_SEQ_SetTask(1<<CFG_TASK_RESETPMCU, CFG_SCH_PRIO_0);
				}
			}
		}
		else
			Mili_Sec_Ctr++;


		if (Process_Ctr >= 65000)	Process_Ctr = 0;
		else	Process_Ctr++;


		if (SEC == 32) f_PMCU_Responds = false;


		if (g_currentState == s_IDLE){
			if ((SEC > 25) && (SEC < 31))
				HAL_GPIO_WritePin(INT_PMCU_GPIO_Port, INT_PMCU_Pin, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(INT_PMCU_GPIO_Port, INT_PMCU_Pin, GPIO_PIN_RESET);
		}


	}
}




void xprintf(uint8_t stream, char *FormatString, ...){
	char *sval;
	char cdcSTR[100];
	char format[10];
	char tempSTR[100];

	float fval;

	int8_t i, j, x;

	int  ival;

	uint8_t len;
	va_list args;

	len = strlen(FormatString);
	va_start(args, FormatString);

	for (i = 0, j = 0; j < len; i++, j++)
	{
		tempSTR[i] = FormatString[j];

		if (FormatString[j] == '%'){
			tempSTR[i] = '\0';
			j++;

			if (stream == PC) CDC_Transmit_FS((uint8_t *)tempSTR, strlen(tempSTR));
			else if (stream == PMCU) HAL_UART_Transmit(&huart1, (uint8_t *)tempSTR, strlen(tempSTR), HAL_MAX_DELAY);

			HAL_Delay(50);
			x = 0;
			format[x++] = '%';

			if (FormatString[j] != 's'){
				do {
				 if (FormatString[j] == 'd')
						 format[x] = FormatString[j];
				 else
						 format[x++] = FormatString[j++];
				}
				while (FormatString[j] != 'd' && FormatString[j] != 'f');
			}

			if (FormatString[j] == 's'){
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




