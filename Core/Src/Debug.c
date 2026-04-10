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
	char ble_msg[16];

	xprintf(PMCU, "EXIT\r\n");
	if (Get_Desired_Response("ACK", 3)){
		g_CurrentState =  s_IDLE;
		strcpy(ble_msg, "IDLE\r\n");
		SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)ble_msg);
		return success;
	}

	strcpy(ble_msg, "Try Again\r\n");
	SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)ble_msg);
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
	sprintf(BLE_BUFFER, "CHANGE SETTINGS\r\n");
	SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)BLE_BUFFER);

  for (uint8_t i = 'A'; i < ('A'+ 20); i++){
  	if (i == Server_Num) 			sprintf(BLE_BUFFER, "%c) Server Number\r\n", i);
  	if (i == SIM_Num) 				sprintf(BLE_BUFFER, "%c) SIM Number\r\n", i);
  	if (i == Sending_Time) 		sprintf(BLE_BUFFER, "%c) Sending Time\r\n", i);
  	if (i == Password) 				sprintf(BLE_BUFFER, "%c) Password\r\n", i);
  	if (i == Date_Time) 			sprintf(BLE_BUFFER, "%c) Date Time\r\n", i);
  	if (i == Sensor_Config) 	sprintf(BLE_BUFFER, "%c) Sensor Config\r\n", i);
  	if (i == Reg_Num1) 				sprintf(BLE_BUFFER, "%c) Register Number #1\r\n", i);
  	if (i == Reg_Num2) 				sprintf(BLE_BUFFER, "%c) Register Number #2\r\n", i);
  	if (i == Reg_Num3) 				sprintf(BLE_BUFFER, "%c) Register Number #3\r\n", i);
  	if (i == List_Reg_Num) 		sprintf(BLE_BUFFER, "%c) List Reg Numbers\r\n", i);
  	if (i == Delete_Reg_Num) 	sprintf(BLE_BUFFER, "%c) Delete Reg Numbers\r\n", i);
  	if (i == ARQ_Serial_Num) 	sprintf(BLE_BUFFER, "%c) ARQ Serial Number\r\n", i);
  	if (i == Get_Sensor_Data) sprintf(BLE_BUFFER, "%c) Get Sensor Data\r\n", i);
  	if (i == Reset_PMCUx) 		sprintf(BLE_BUFFER, "%c) Reset PMCU\r\n", i);
  	if (i == Display_Menu) 		sprintf(BLE_BUFFER, "%c) Display Menu Again\r\n", i);

  	SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)BLE_BUFFER);
  	HAL_Delay(10);
  }

  strcpy(BLE_BUFFER,
  		"To exit, type \"EXIT\"\r\n\r\n"
      "To change a setting:\r\n"
      "1) Type the letter then colon\r\n"
  		"2) and input the correct value format\r\n"
      "E.g., Changing a server number\r\n"
      "      A:09091234567");
  SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)BLE_BUFFER);

  Clear_Buffer(BLE_BUFFER, 255);
}






uint8_t Send_DEBUG_To_PMCU(void){
	xprintf(PMCU, "DEBUG\r\n");
	if (Get_Desired_Response("ACK", 3)){
		g_CurrentState = s_DBUG;
		UTIL_SEQ_SetTask(1<<CFG_TASK_SETTINGSMENU, CFG_SCH_PRIO_0);
		return success;
	}
	return fail;
}








uint8_t Set_DEBUG_Value_to_PMCU(char *value){
	char deleted[16] = "Deleted\r\n";
	char invalid_val[32] = "Invalid value\r\n";
	char not_retrieved[32] = "Not retrieved\r\n";
	char reset_pmcu[32] = "Resetting PMCU\r\n";
	char returned_value[100];
	char saved[16] = "SAVED!\r\n";
	char try_again[16] = "Try again\r\n";


	/*GUARD CLAUSE*/
	if (!Valid_Value_Format(value)){
		SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)invalid_val);
		return fail;
	}


	switch (BLE_BUFFER[0])
	{
		case Server_Num:{
			xprintf(PMCU, "S_SVR:%s", value);
			if (Get_Desired_Response("ACK", 1)){
				SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)saved);
				return success;
			}
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
			if (Strings_Are_Equal(value, "NULL")){
				xprintf(PMCU, "S_DTM:%s\r\n", "NULL");
				if (Get_Desired_Response("ACK", 3)){
					DTM_DateTime_Set(RESP_Buffer);
					sprintf(returned_value, "DTM: %s\r\n", RESP_Buffer);
					SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)returned_value);
					return success;
				}
				SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)not_retrieved);
				return fail;
			}

			xprintf(PMCU, "S_DTM:%s", value);
			if (Get_Desired_Response("ACK", 1)){
				SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)saved);
				DTM_DateTime_Set(value);
				return success;
			}
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

		case 'F':{
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

		default:{
			valid = false;
			break;
		}
	}

	return valid;
}
