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


bool f_CheckPMCU = false;
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
char g_Reg1[12] = "09091234567";
char g_Reg2[12] = "09091230000";
char g_Reg3[12] = "09090004567";
char g_ServerNum[12];
char g_SIMNum[12];
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


void BLE_Debug_Mode(void){

}





uint8_t BLE_Examine_String(char *pString){
	char val[100];
	switch (g_CurrentState){
		case s_IDLE:{	// Entering Debug Mode
			if (UTL_CompareEqual(pString, "DEBUG\r\n")){
				g_CurrentState = s_DBUG;
				//UTIL_SEQ_SetTask(1<<CFG_TASK_GETCFGFROMPMCU, CFG_SCH_PRIO_0);
				UTIL_SEQ_SetTask(1<<CFG_TASK_SETTINGSMENU, CFG_SCH_PRIO_0);
			}
			break;
		}

		case s_DBUG:{
			if (UTL_CompareEqual(pString, "EXIT\r\n")){
				g_CurrentState =  s_IDLE;
				strcpy(val, "IDLE\r\n");
				SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)val);
			}
			else{
				// [x] Create a function for this
				if ((pString[0] < 'A') || (pString[0] > 'Z')){
					strcpy(val, "Invalid character input!\r\n");
					SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)val);
				}
				else{
					Clear_Buffer(val, 100);
					BLE_Extract_Value(val, pString);
					BLE_Set_Settings(pString[0], val);
				}
			}
			break;
		}
		default:
			break;
	}
	return 0;
}



void BLE_Print_to_PMCU(void){
	if (UTL_CompareEqual(UART_Buffer, "Status")){
		xprintf(PMCU, "Attached\r\n");
	}
}




void BLE_Print_to_USB(void){
	uint8_t size = 0;

	while(BLE_BUFFER[size] != '\0') size++;

	CDC_Transmit_FS((uint8_t *)BLE_BUFFER, size);
}





// from S_SVR:09191234567
// to   A:09191234567
uint8_t BLE_Extract_Value(char *dest, char *source){
	uint8_t i = 2;

	// Guard clause
	if (source[1] != ':')
		strcpy(dest, "NULL");

	if ((source[1] == '\r') && (source[2] == '\n')){
		strcpy(dest, "NULL");
		return 0;
	}

	do{
		dest[i-2] = source[i];
		i++;
	}
	while(source[i] != '\0');

	return 0;
}




uint8_t BLE_Set_Settings(char variable, char *value){
	char x[300] = "Invalid value\r\n";
	uint8_t len;

	switch (variable){
		// Server Number
		case 'A':{
			if (BLE_Valid_Value('A', value)){
				xprintf(PMCU, "S_SVR:%s", value);
				if (Get_Desired_Response("ACK", 1))
					strcpy(x,"SAVED!\r\n");
				else
					strcpy(x,"Not Save, try again!\r\n");
			}
			break;
		}


		// SIM Number
		case 'B':{
			if (BLE_Valid_Value('B', value)){
				xprintf(PMCU, "S_SIM:%s", value);
				if (Get_Desired_Response("ACK", 1))
					strcpy(x,"SAVED!\r\n");
				else
					strcpy(x,"Not Save, try again!\r\n");
			}
			break;
		}


		// Sending Time
		case 'C':{
			if (BLE_Valid_Value('C', value)){
				xprintf(PMCU, "S_SDT:%s", value);
				if (Get_Desired_Response("ACK", 1))
					strcpy(x,"SAVED!\r\n");
				else
					strcpy(x,"Not Save, try again!\r\n");
			}
			break;
		}


		// Password
		case 'D':{
			if (BLE_Valid_Value('D', value)){
				xprintf(PMCU, "S_PWD:%s", value);
				if (Get_Desired_Response("ACK", 1))
					strcpy(x,"SAVED!\r\n");
				else
					strcpy(x,"Not Save, try again!\r\n");
			}
			break;
		}

		/*Date and Time*/
		case 'E':{
			if (BLE_Valid_Value('E', value)){
				xprintf(PMCU, "S_DTM:%s", value);
				if (Get_Desired_Response("ACK", 1)){
					strcpy(x,"SAVED!\r\n");
					DTM_DateTime_Set(value);
				}
				else
					strcpy(x,"Not Save, try again!\r\n");
			}
			else if (UTL_CompareEqual(value, "NULL")){
				DTM_DateTime_Get();
				sprintf(x, "DTM: %s\r\n", g_DateTime);
			}
			break;
		}

		case 'F':{
			len = sprintf(x, "S_CFG:%s\r\n", value);
			break;
		}

		case 'G':{
			if (BLE_Valid_Value('G', value)){
				xprintf(PMCU, "S_RN1:%s", value);
				if (Get_Desired_Response("ACK", 1))
					strcpy(x,"SAVED!\r\n");
				else
					strcpy(x,"Not Save, try again!\r\n");
			}
			break;
		}

		case 'H':{
			if (BLE_Valid_Value('H', value)){
				xprintf(PMCU, "S_RN2:%s", value);
				if (Get_Desired_Response("ACK", 1))
					strcpy(x,"SAVED!\r\n");
				else
					strcpy(x,"Not Save, try again!\r\n");
			}
			break;
		}

		case 'I':{
			if (BLE_Valid_Value('I', value)){
				xprintf(PMCU, "S_RN3:%s", value);
				if (Get_Desired_Response("ACK", 1))
					strcpy(x,"SAVED!\r\n");
				else
					strcpy(x,"Not Save, try again!\r\n");
			}
			break;
		}


		case 'J':{
			len = sprintf(x, "%s, %s, %s\r\n", g_Reg1, g_Reg2, g_Reg3);
			break;
		}


		case 'K':{
			if (atoi(value) == 1){
				xprintf(PMCU, "D_RN1:%s", value);
				if (Get_Desired_Response("ACK", 1)){
					strcpy(x,"Deleted!\r\n");
					strcpy(g_Reg1, "");
				}
				else
					strcpy(x,"Not Deleted, try again!\r\n");
			}
			else if (atoi(value) == 2){
				xprintf(PMCU, "D_RN2:%s", value);
				if (Get_Desired_Response("ACK", 1)){
					strcpy(x,"Deleted!\r\n");
					strcpy(g_Reg2, "");
				}
				else
					strcpy(x,"Not Deleted, try again!\r\n");
			}
			else if (atoi(value) == 3){
				xprintf(PMCU, "D_RN3:%s", value);
				if (Get_Desired_Response("ACK", 1)){
					strcpy(x,"Deleted!\r\n");
					strcpy(g_Reg3, "");
				}
				else
					strcpy(x,"Not Deleted, try again!\r\n");
			}
			break;
		}

		case 'X':{
			strcpy(x, "Resetting PMCU\r\n");
			break;
			Reset_PMCU();
		}

		case 'Y':{
			Print_Setting_Menu();
			return 0;
		}


		default:{
			len = 0;
			break;
		}
	}

	SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)x);
	return len;
}






bool BLE_Valid_Value(char ch, char *pVal){
	bool valid;

	bool inV_YY = false;
	bool inV_MM = false;
	bool inV_DD = false;
	bool inV_hh = false;
	bool inV_mm = false;
	bool inV_ss = false;
	bool inV_SP = false;

	uint8_t YY,MM,DD,hh,mm,ss;

	switch (ch){
		case 'A':{
			if ((pVal[0] < '0') || (pVal[0] > '9') ||
					(pVal[1] < '0') || (pVal[1] > '9') ||
					(pVal[2] < '0') || (pVal[2] > '9') ||
					(pVal[3] < '0') || (pVal[3] > '9') ||
					(pVal[4] < '0') || (pVal[4] > '9') ||
					(pVal[5] < '0') || (pVal[5] > '9') ||
					(pVal[6] < '0') || (pVal[6] > '9') ||
					(pVal[7] < '0') || (pVal[7] > '9') ||
					(pVal[8] < '0') || (pVal[8] > '9') ||
					(pVal[9] < '0') || (pVal[9] > '9') ||
					(pVal[10]< '0') || (pVal[10]> '9') ||
					(pVal[0]=='\0'))
				valid =  false;
			else
				valid =  true;
			break;
		}

		case 'B':{
			if ((pVal[0] < '0') || (pVal[0] > '9') ||
					(pVal[1] < '0') || (pVal[1] > '9') ||
					(pVal[2] < '0') || (pVal[2] > '9') ||
					(pVal[3] < '0') || (pVal[3] > '9') ||
					(pVal[4] < '0') || (pVal[4] > '9') ||
					(pVal[5] < '0') || (pVal[5] > '9') ||
					(pVal[6] < '0') || (pVal[6] > '9') ||
					(pVal[7] < '0') || (pVal[7] > '9') ||
					(pVal[8] < '0') || (pVal[8] > '9') ||
					(pVal[9] < '0') || (pVal[9] > '9') ||
					(pVal[10]< '0') || (pVal[10]> '9') ||
					(pVal[0]=='\0'))
				valid =  false;
			else
				valid =  true;
			break;
		}

		case 'C':{
			if ((atoi(pVal) < 1) || (atoi(pVal) > 60))
				valid =  false;
			else
				valid =  true;
			break;
		}


		case 'D':{
		if ((pVal[8] != '\r') ||
				(pVal[9] != '\n') ||
				(pVal[10] != '\0'))
			valid = false;
		else
			valid = true;
		break;
		}


		case 'E':{
			if (pVal[2]	!= '/' ||
					pVal[5]	!= '/' ||
					pVal[8]	!= ',' ||
					pVal[11]!= ':' ||
					pVal[14]!= ':'){	//Invalid separator
				inV_SP = true;
			}
			else{
				inV_SP = false;
			}

			YY = ((pVal[0]-48)*10) + (pVal[1]-48);
			if (YY < 25)
				inV_YY = true;

			MM = ((pVal[3]-48)*10) + (pVal[4]-48);
			if (MM < 1 || MM > 12)
				inV_MM = true;

			DD = ((pVal[6]-48)*10) + (pVal[7]-48);
			if (DD < 1 || DD > 31)
				inV_DD = true;

			hh = ((pVal[9]-48)*10) + (pVal[10]-48);
			if (hh < 0 || hh > 24)
				inV_hh = true;

			mm = ((pVal[12]-48)*10) + (pVal[13]-48);
			if (mm < 0 || mm > 60)
				inV_mm = true;

			ss = ((pVal[15]-48)*10) + (pVal[16]-48);
			if (ss < 0 || ss > 60)
				inV_ss = true;

			if (inV_SP ||
					inV_YY || inV_MM || inV_DD ||
					inV_hh || inV_mm || inV_ss)
				valid = false;
			else
				valid = true;
			break;
		}

		case 'F':{
			if (!UTL_CompareEqual(pVal, "MBH") ||
					!UTL_CompareEqual(pVal, "MBA") ||
					!UTL_CompareEqual(pVal, "ARG"))
				valid = false;
			else
				valid = true;
			break;
		}

		case 'G':
		case 'H':
		case 'I':{
			if ((pVal[0] < '0') || (pVal[0] > '9') ||
					(pVal[1] < '0') || (pVal[1] > '9') ||
					(pVal[2] < '0') || (pVal[2] > '9') ||
					(pVal[3] < '0') || (pVal[3] > '9') ||
					(pVal[4] < '0') || (pVal[4] > '9') ||
					(pVal[5] < '0') || (pVal[5] > '9') ||
					(pVal[6] < '0') || (pVal[6] > '9') ||
					(pVal[7] < '0') || (pVal[7] > '9') ||
					(pVal[8] < '0') || (pVal[8] > '9') ||
					(pVal[9] < '0') || (pVal[9] > '9') ||
					(pVal[10]< '0') || (pVal[10]> '9') ||
					(pVal[0]=='\0'))
				valid =  false;
			else
				valid =  true;
			break;
		}

		default:
			valid = false;
			break;
	}

	return valid;
}

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






void Clear_USB_Buffers(void){
	memset(USB_BUFFER, '\0', 255);
	f_USB = false;
}





uint8_t CurrentState_Base_On_BLE_String(char *pBuf){
	if (UTL_CompareEqual(pBuf, "DEBUG\r\n")){
		return s_DBUG;
	}
	else if (UTL_CompareEqual(pBuf, "EXIT\r\n")){
		return s_IDLE;
	}

	return g_CurrentState;
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




void Extract_Value(char *dest, char *source){
	// Sample S_SVR:09191234567
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




void Print_UARTBuffer(void){
	uint8_t len = strlen(UART_Buffer);
	CDC_Transmit_FS((uint8_t *)UART_Buffer, len);

#if (BLE_ENABLED)
	SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)UART_Buffer);
#endif

	memset(UART_Buffer, '\0', 100);
}





void Uninterrupt_PMCU(void){

}





void Reset_PMCU(void){
	HAL_GPIO_WritePin(NRST_PMCU_GPIO_Port, NRST_PMCU_Pin, GPIO_PIN_RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(NRST_PMCU_GPIO_Port, NRST_PMCU_Pin, GPIO_PIN_SET);
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





uint8_t Set_Variable(char *variable, char *value){
	if (UTL_CompareEqual(variable, "DTM")){
		DTM_DateTime_Set(value);
		DTM_DateTime_Get();
		xprintf(PC, "Date and Time Synched: %s\r\n", g_DateTime);
	}

	else if (UTL_CompareEqual(variable, "SIM")){
		strcpy(g_SIMNum, value);
		xprintf(PC, "Sim Number Synched: %s\r\n", g_SIMNum);
	}

	else if (UTL_CompareEqual(variable, "SDT")){
		g_SendingTime = atoi(value);
		xprintf(PC, "Sim Number Synched: %s\r\n", g_SendingTime);
	}

	HAL_Delay(200);
	return 0;
}









void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == PMCU_INT_Pin)
		f_PMCU_Responds = true;
}




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart == &huart1){
		//HAL_UART_Receive_IT(&huart1, (uint8_t *)&UART_CHAR, 1);


		// Clear Buffer id starting to zero
		if (CHAR_CTR == 0)
			memset(TEMP_Buffer, '\0', 100);

		// Copy character to buffer
		TEMP_Buffer[CHAR_CTR] = UART_CHAR;

		if (CHAR_CTR > 100)
			CHAR_CTR = 0;
		else
			CHAR_CTR++;

		// reset counter when carriage return encounters
		/*if ((UART_CHAR== '\n') ||
			 ((TEMP_Buffer[CHAR_CTR-2] == '\r') && (TEMP_Buffer[CHAR_CTR-1] == '\n')))
		{
			CHAR_CTR = 0;
			sprintf(UART_Buffer, TEMP_Buffer);
		}*/

		if ((TEMP_Buffer[CHAR_CTR-1] == '^') && (TEMP_Buffer[CHAR_CTR-2] == '^')){
			CHAR_CTR = 0;
			snprintf(UART_Buffer, strlen(TEMP_Buffer)-1, "%s", TEMP_Buffer);

			if (g_CurrentState != s_DBUG){
				UTIL_SEQ_SetTask(1<<CFG_TASK_PRINTUARTBUFFER, CFG_SCH_PRIO_0);
			}
		}

		if ((TEMP_Buffer[CHAR_CTR-1] == '$') && (TEMP_Buffer[CHAR_CTR-2] == '$')){
			CHAR_CTR = 0;
			snprintf(UART_Buffer, strlen(TEMP_Buffer)-1, "%s", TEMP_Buffer);
			//f_PMCU_CMD = true;
			UTIL_SEQ_SetTask(1<<CFG_TASK_EXTRACTPMCUCMD, CFG_SCH_PRIO_0);
		}

		if ((TEMP_Buffer[CHAR_CTR-1] == '?') && (TEMP_Buffer[CHAR_CTR-2] == '?')){
			CHAR_CTR = 0;
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

				if (g_Fault_Ctr == 5){
					g_Fault_Ctr = 0;
					UTIL_SEQ_SetTask(1<<CFG_TASK_RESETPMCU, CFG_SCH_PRIO_0);
				}
			}
		}
		else
			Mili_Sec_Ctr++;



		// Task Counter Timer
		if (Process_Ctr >= 65000)	Process_Ctr = 0;
		else	Process_Ctr++;

		// PMCU Responds
		if (SEC == 32) f_PMCU_Responds = false;


		if (g_CurrentState == s_IDLE){
			if ((SEC > 25) && (SEC < 31))
				HAL_GPIO_WritePin(INT_PMCU_GPIO_Port, INT_PMCU_Pin, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(INT_PMCU_GPIO_Port, INT_PMCU_Pin, GPIO_PIN_RESET);
		}

		// moved to UART callback instead
//		if (f_PMCU_QRY){
//			f_PMCU_QRY = false;
//			UTIL_SEQ_SetTask(1<<CFG_TASK_PRINTTOPMCU, CFG_SCH_PRIO_0);
//		}
	}
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




