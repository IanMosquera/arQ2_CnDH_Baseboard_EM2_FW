/*
 * Debug.c
 *
 *  Created on: Dec 18, 2025
 *      Author: IanCMosquera
 */

#include "ARQ.h"
#include "DateTime.h"
#include "Debug.h"
#include "StateMachine.h"
#include "stdlib.h"
#include "string.h"
#include "usbd_cdc_if.h"
#include "UtilityFunctions.h"
#include <usart.h>


#if (BLE_ENABLED)
#include "ble_types.h"
#include "custom_stm.h"
#include "stm32_seq.h"
#endif





uint8_t BLE_Set_Server_Number(char *value){
	char pmcu_msg[16];
	char returned_value[100];
	char saved[16] = "SAVED!\r\n";
	char try_again[16] = "Try again\r\n";
	uint8_t len;

	len = sprintf(pmcu_msg, "S_SVR:%s\r\n", value);
	HAL_UART_Transmit(&huart1, (uint8_t *)pmcu_msg, len, 100);

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




bool CHAR_is_Not_Within_In_AtoZ(void){
	if ((BLE_BUFFER[0] < 'A') || (BLE_BUFFER[0] > 'Z')){
		return true;
	}
	else{
		return false;
	}
}







bool Correct_DEBUG_Detected(void){
	char correct_debug[32];

	sprintf(correct_debug, "DEBUG %s\r\n", g_Password);

	if (Strings_Are_Equal(BLE_BUFFER, correct_debug)){
		return true;
	}

	return false;
}






bool DEBUG_Exit_Detected(void){
	if (Strings_Are_Equal(BLE_BUFFER, "EXIT\r\n")){
		return true;
	}

	return false;
}






uint8_t Exit_Debug(void){
	char ble_msg[50];

	xprintf(PMCU, "EXIT\r\n");
	if (Get_Desired_Response("ACK", 3)){
		g_currentState =  s_IDLE;

		strcpy(ble_msg, "Exiting BLE DEBUG\r\n");
		SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)ble_msg);
		CDC_Transmit_FS((uint8_t *)ble_msg, strlen(ble_msg));
		return success;
	}

	strcpy(ble_msg, "Try Again\r\n");
	SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)ble_msg);
	CDC_Transmit_FS((uint8_t *)ble_msg, strlen(ble_msg));
	return fail;
}






// extract value after ":"
// Example: A:09191234567 -> 09191234567
//https://www.onlinegdb.com/#:~:text=https%3A//onlinegdb.com/YShqOopv2

uint8_t Extract_Value_From_BLE_DebugMessage(char *value){
	uint8_t i = 2;

	/*GUARD CLAUSE*/
	if ((BLE_BUFFER[1] == '\r') && (BLE_BUFFER[2] == '\n')){
		strcpy(value, "NULL");
		return fail;
	}

	do{
		value[i-2] = BLE_BUFFER[i];
		i++;
	}
	while(BLE_BUFFER[i] != '\0');
	value[i-2] = '\0';

	return success;
}







void Print_Setting_Menu(void){
	char msg[50];

	g_currentState = s_DBUG;


	sprintf(msg, "\r\nCHANGE SETTINGS\r\n");
	SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)msg);

  for (uint8_t i = 'A'; i < ('A'+ 30); i++){
  	if (i == Server_Num) 			sprintf(msg, "%c) Server Number\r\n", i);
  	if (i == SIM_Num) 				sprintf(msg, "%c) SIM Number\r\n", i);
  	if (i == Sending_Time) 		sprintf(msg, "%c) Sending Time\r\n", i);
  	if (i == Password) 				sprintf(msg, "%c) Password\r\n", i);
  	if (i == Date_Time) 			sprintf(msg, "%c) Date Time\r\n", i);
  	if (i == Sensor_Config) 	sprintf(msg, "%c) Sensor Config\r\n", i);
  	if (i == Reg_Num1) 				sprintf(msg, "%c) Register Number #1\r\n", i);
  	if (i == Reg_Num2) 				sprintf(msg, "%c) Register Number #2\r\n", i);
  	if (i == Reg_Num3) 				sprintf(msg, "%c) Register Number #3\r\n", i);
  	if (i == List_Reg_Num) 		sprintf(msg, "%c) List Reg Numbers\r\n", i);
  	if (i == Delete_Reg_Num) 	sprintf(msg, "%c) Delete Reg Numbers\r\n", i);
  	if (i == ARQ_Serial_Num) 	sprintf(msg, "%c) ARQ Serial Number\r\n", i);
  	if (i == Get_Sensor_Data) sprintf(msg, "%c) Get Sensor Data\r\n", i);
  	if (i == Send_STR_Via_SMS)sprintf(msg, "%c) Send String via SMS\r\n", i);
  	if (i == 87)							strcpy(msg,  "   ----------\r\n");
  	if (i == Reset_PMCUx) 		sprintf(msg, "%c) Reset PMCU\r\n", i);
  	if (i == Display_Menu) 		sprintf(msg, "%c) Display Menu Again\r\n", i);

  	SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)msg);
  	Clear_Buffer(msg, 50);
  	HAL_Delay(10);
  }

  for (uint8_t j = 0; j < 6; j++){
  	if (j == 0) strcpy(msg, "\r\nTo exit, type \"EXIT\"\r\n\r\n");
  	if (j == 1) strcpy(msg, "1) Type the letter then colon\r\n");
  	if (j == 2) strcpy(msg, "2) and input the correct value format\r\n");
  	if (j == 3) strcpy(msg, "E.g., Changing a server number\r\n");
  	if (j == 4) strcpy(msg, "      A:09091234567\r\n");

  	SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)msg);
  	Clear_Buffer(msg, 50);
  	HAL_Delay(10);
  }
}






uint8_t Send_DEBUG_To_PMCU(void){
	xprintf(PMCU, "DEBUG\r\n");
	if (Get_Desired_Response("ACK", 3)){
		g_currentState = s_DBUG;
		UTIL_SEQ_SetTask(1<<CFG_TASK_SETTINGSMENU, CFG_SCH_PRIO_0);
		return success;
	}
	return fail;
}

























bool Valid_Value_Format(char *pVal){
	bool valid;

	bool inV_YY = false;
	bool inV_MM = false;
	bool inV_DD = false;
	bool inV_hh = false;
	bool inV_mm = false;
	bool inV_ss = false;
	bool inV_SP = false;

	uint8_t YY,MM,DD,hh,mm,ss;

	switch (BLE_BUFFER[0]){
		case Server_Num:
		case SIM_Num:
		case Reg_Num1:
		case Reg_Num2:
		case Reg_Num3:{
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


		case Sending_Time:{
			if ((atoi(pVal) < 1) || (atoi(pVal) > 60))
				valid =  false;
			else
				valid =  true;
			break;
		}


		case Password:{
		if ((pVal[8] != '\r') ||
				(pVal[9] != '\n') ||
				(pVal[10] != '\0'))
			valid = false;
		else
			valid = true;
		break;
		}


		case Date_Time:{
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

		case Sensor_Config:{
			if (Strings_Are_Equal(pVal, "MBH\r\n") ||
					Strings_Are_Equal(pVal, "MBA\r\n") ||
					Strings_Are_Equal(pVal, "ARG\r\n"))
				valid = true;
			else
				valid = false;
			break;
		}


		case ARQ_Serial_Num:{
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
					(pVal[11]< '0') || (pVal[11]> '9') ||
					(pVal[0]=='\0'))
				valid =  false;
			else
				valid =  true;
			break;
		}

		case Delete_Reg_Num:{
			if (pVal[0] > '0' || pVal[0] < '4')
				valid =  true;
			else
				valid = false;
		}

		default:{
			valid = false;
			break;
		}
	}

	return valid;
}
