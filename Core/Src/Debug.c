/*
 * Debug.c
 *
 *  Created on: Dec 18, 2025
 *      Author: IanCMosquera
 */

#include "Debug.h"
#include "string.h"
#include "ARQ.h"
#include "stdlib.h"
#include "StateMachine.h"
#include "usbd_cdc_if.h"
#include "UtilityFunctions.h"
#include <usart.h>


#if (BLE_ENABLED)
#include "ble_types.h"
#include "custom_stm.h"
#include "stm32_seq.h"
#endif

static void Print_InvalidInput(char *pData);
static void ListRegisteredNumber(void);
//static void Print_InvalidChar(char pChar);

uint8_t Debug_Mode(void){
	bool invalid = false;
	char c;
	uint8_t r = 0;

	// Initial Code, performed once
	if (f_InitState){
		f_InitState = false;
		xprintf(PC, "Sending DEBUG command to PMCU: ");
		HAL_Delay(200);

		xprintf(PMCU, "DEBUG\r\n");
		if (!Get_Desired_Response("ACK", 10)){
			xprintf(PC, "FAIL\r\n");
			HAL_Delay(200);
			return e_DONE;
		}
		xprintf(PC, "SUCCESS!\r\n");
		HAL_Delay(200);

		xprintf(PC, "Getting FW version: ");
		HAL_Delay(500);

		xprintf(PMCU, "G_FVR\r\n");
		if (Get_Desired_Response("FVR:", 10)){
			xprintf(PC, "%s\r\n\r\n", RESP_Buffer);
			HAL_Delay(500);
		}
	}

  // Action while in state
	Print_Setting_Menu();

	do{
		xprintf(PC, "Enter choice: (A-Z)\r\n");
		HAL_Delay(100);

		Clear_Buffer(USB_BUFFER, 255);
		c = UTL_GetChar(60);

		if (c < 'A' || c > 'Z')
			invalid = true;
		else
			invalid = false;

		if (invalid){
			xprintf(PC, "Invalid character!\r\n");
			HAL_Delay(100);

			if (++r >= 3){
				xprintf(PC, "Max retries! Exiting DEBUG mode\r\n");
				HAL_Delay(500);
				r = 0;
				xprintf(PMCU, "EXIT\r\n");
				HAL_Delay(200);
				return e_DONE;
			}
		}
	}
	while(invalid);

	switch (c){
		case 'A':{
			//UTL_SensorData_Get();
			//UTL_Append_Data();
			break;
		}
		case 'B':{
			Get_Config();
			break;
		}
		case 'C':{
			return e_NONE;
			break;
		}
		case 'D':{
			DBG_Change_SendingTime();
			break;
		}
		case 'E':{	//No settings assigned
			break;
		}
		case 'F':{
			DBG_Change_ServerNumber();
			break;
		}
		case 'G':{
			DBG_List_Registered_Numbers();
			return e_NONE;
			break;
		}
		case 'H':{
			//if (LTE_NTP_Assign())
			//		LTE_Query_Clock();
			break;
		}
		case 'I':{
			DBG_Change_DateTime();
			break;
		}
		case 'J':{
			//DBG_PCDM_Config_Set();
			break;
		}
		case 'K':{	//Get GPS Data
			break;
		}
		case 'L':{	//Read Data from Flash memory
			break;
		}
		case 'M':{
			DBG_Change_Password();
			break;
		}
		case 'N':{
			break;
		}
		case 'O':{
			DBG_Change_SensorConfig();
			break;
		}
		case 'P':{
			//DBG_HTTP_Send();
			break;
		}
		case 'Q':{
			//DBG_LORA_Send();
			break;
		}
		case 'R':{
			DBG_Change_SIMNumber();
			break;
		}
		case 'S':{	// Turn off Watchdog
			break;
		}
		case 'T':{
			//DBG_ArQSerialNumber_Set();
			break;
		}
		case 'U':{	// Turn On BLE Debug Mode
			break;
		}
		case 'V':{	// Enable hybrid power saving
			break;
		}
		case 'X':{	// Delete registered number
			DBG_Delete_RegisteredNumber();
			break;
		}
		case 'Y':{
			break;
		}
		case 'Z':{
			f_USB = false;
			f_InitState = true;

			HAL_UART_Receive_IT(&huart1, &UART_CHAR, 1);
			Clear_PMCU_Flags();

			xprintf(PC, "Exiting Debug mode\r\n");
			HAL_Delay(200);

			xprintf(PMCU, "EXIT\r\n");
			HAL_Delay(200);

			return e_DONE;
			break;
		}

		default:{
			f_USB = false;
			f_InitState = true;

			HAL_UART_Receive_IT(&huart1, &UART_CHAR, 1);
			Clear_PMCU_Flags();
			xprintf(PMCU, "EXIT\r\n");
			HAL_Delay(200);

			return e_DONE;
			break;
		}
	}


	return e_NONE;
}





uint8_t DBG_Change_DateTime(void){
	bool inV_YY = false;
	bool inV_MM = false;
	bool inV_DD = false;
	bool inV_hh = false;
	bool inV_mm = false;
	bool inV_ss = false;
	bool inV_SP = false;
	bool invalid = false;
	uint8_t r = 0;
	uint8_t YY,MM,DD,hh,mm,ss;

	xprintf(PC, "Current Date and Time: ");
	HAL_Delay(200);

	xprintf(PMCU, "G_DTM\r\n");
	if (!Get_Desired_Response("DTM:", 20)){
		xprintf(PC, "Failed to retrieve\r\n");
		return e_NONE;
	}
	xprintf(PC, "%s\r\n", RESP_Buffer);
	HAL_Delay(100);

	Print_Modify_Cancel();
	HAL_Delay(100);
	if (ModifyCancelled(3))
		return e_NONE;

	do{
		xprintf(PC, "Enter DateTime in YY/MM/DD,hh:mm:ss format\r\n");
		Clear_USB_Buffers();
		UTL_GetString(60);

		if (USB_BUFFER[2]	!= '/' ||
				USB_BUFFER[5]	!= '/' ||
				USB_BUFFER[8]	!= ',' ||
				USB_BUFFER[11]!= ':' ||
				USB_BUFFER[14]!= ':'){	//Invalid separator
			inV_SP = true;
		}
		else{
			inV_SP = false;
		}

		YY = ((USB_BUFFER[0]-48)*10) + (USB_BUFFER[1]-48);
		if (YY < 25)
			inV_YY = true;

		MM = ((USB_BUFFER[3]-48)*10) + (USB_BUFFER[4]-48);
		if (MM < 1 || MM > 12)
			inV_MM = true;

		DD = ((USB_BUFFER[6]-48)*10) + (USB_BUFFER[7]-48);
		if (DD < 1 || DD > 31)
			inV_DD = true;

		hh = ((USB_BUFFER[9]-48)*10) + (USB_BUFFER[10]-48);
		if (hh < 0 || hh > 24)
			inV_hh = true;

		mm = ((USB_BUFFER[12]-48)*10) + (USB_BUFFER[13]-48);
		if (mm < 0 || mm > 60)
			inV_mm = true;

		ss = ((USB_BUFFER[15]-48)*10) + (USB_BUFFER[16]-48);
		if (ss < 0 || ss > 60)
			inV_ss = true;

		if (inV_SP ||
				inV_YY || inV_MM || inV_DD ||
				inV_hh || inV_mm || inV_ss)
			invalid = true;
		else
			invalid = false;

		if (invalid){
			Print_InvalidInput(USB_BUFFER);
			if (++r == 3){
				xprintf(PC, "Max retries! Exiting DEBUG mode\r\n");
				HAL_Delay(500);
				return e_NONE;
			}
		}
	}
	while(invalid);

	xprintf(PMCU, "S_DTM:%s\r\n", USB_BUFFER);
	if (Get_Desired_Response("ACK", 10)){
		HAL_Delay(1000);
		xprintf(PC,  "New Date and Time: %s\r\n\r\n", USB_BUFFER);
		Clear_USB_Buffers();
		return e_NONE;
	}

	return e_NONE;
}






uint8_t DBG_Change_Password(void){
	bool invalid = false;
	uint8_t r = 0;

	xprintf(PC, "Current password: ");
	HAL_Delay(200);

	xprintf(PMCU, "G_PWD\r\n");
	if (!Get_Desired_Response("PWD:", 10)){
		xprintf(PC, "Failed to retrieve\r\n");
		return e_NONE;
	}
	xprintf(PC, "%s\r\n", RESP_Buffer);
	HAL_Delay(100);


	Print_Modify_Cancel();
	HAL_Delay(100);
	if (ModifyCancelled(3))
		return e_NONE;

	do{
		xprintf(PC, "Enter new password [max 8 char length]\r\n");
		UTL_GetString(60);

		if ((USB_BUFFER[8] != '\r') ||
				(USB_BUFFER[9] != '\n') ||
				(USB_BUFFER[10] != '\0'))
			invalid = true;
		else
			invalid = false;

		if (invalid){
			Print_InvalidInput(USB_BUFFER);
			if (++r == 3){
				xprintf(PC, "Max retries! Exiting DEBUG mode\r\n");
				HAL_Delay(500);
				return e_NONE;
			}
		}
	} while(invalid);

	xprintf(PMCU, "S_PWD:%s\r\n", USB_BUFFER);
	if (Get_Desired_Response("ACK", 10)){
		HAL_Delay(1000);
		xprintf(PC,  "New password: %s\r\n\r\n", USB_BUFFER);
		Clear_USB_Buffers();
		return e_NONE;
	}

	return e_NONE;
}






uint8_t DBG_Change_SendingTime(void){
	bool invalid = false;
	uint8_t x, r = 0;

	xprintf(PC, "Current sending time: ");
	HAL_Delay(200);

	xprintf(PMCU, "G_SDT\r\n");
	if (!Get_Desired_Response("SDT:", 10)){
		xprintf(PC, "Failed to retrieve\r\n");
		return e_NONE;
	}
	xprintf(PC, "%i\r\n", atoi(RESP_Buffer));
	HAL_Delay(100);


	Print_Modify_Cancel();
	HAL_Delay(100);
	if (ModifyCancelled(3))
		return e_NONE;

	do{
		xprintf(PC, "Enter number (1-60) in minutes\r\n");
		UTL_GetString(60);

		x = atoi(USB_BUFFER);
		if (x < 1 || x > 60)
			invalid = true;
		else
			invalid = false;

		if (invalid){
			Print_InvalidInput(USB_BUFFER);
			if (++r == 3){
				xprintf(PC, "Max retries! Exiting DEBUG mode\r\n");
				HAL_Delay(500);
				return e_NONE;
			}
		}
	} while(invalid);

	xprintf(PMCU, "S_SDT:%d\r\n", x);
	if (Get_Desired_Response("ACK", 5)){
		HAL_Delay(1000);
		char temp[100];
		uint8_t len = sprintf(temp, "New sending time: [%d]\r\n\r\n", x);
		CDC_Transmit_FS((uint8_t *)temp, len);
		//xprintf(PC,  "New sending time: [%d]\r\n\r\n", x);
		Clear_USB_Buffers();
		return e_NONE;
	}

	return e_NONE;
}





uint8_t DBG_Change_SensorConfig(void){
	bool invalid = false;
	uint8_t r = 0;

	xprintf(PC, "Current Sensor Config: ");
	HAL_Delay(200);

	xprintf(PMCU, "G_SCF\r\n");
	if (!Get_Desired_Response("SCF:", 10)){
		xprintf(PC, "Failed to retrieve\r\n");
		return e_NONE;
	}
	xprintf(PC, "%s\r\n", RESP_Buffer);
	HAL_Delay(100);


	Print_Modify_Cancel();
	HAL_Delay(100);
	if (ModifyCancelled(3))
		return e_NONE;

	// Test Condition while loop
	do{
		xprintf(PC, "Enter valid sensor config:\r\n");
		UTL_GetString(60);

		// Test condition
		if (!UTL_CompareEqual(USB_BUFFER, "MBH") ||
				!UTL_CompareEqual(USB_BUFFER, "MBA") ||
				!UTL_CompareEqual(USB_BUFFER, "ARG"))
			invalid = true;
		else
			invalid = false;

		if (invalid){
			Print_InvalidInput(USB_BUFFER);
			if (++r == 3){
				xprintf(PC, "Max retries! Exiting DEBUG mode\r\n");
				HAL_Delay(500);
				return e_NONE;
			}
		}
	}
	while(invalid);

	xprintf(PMCU, "S_SCF:%s\r\n", USB_BUFFER);
	if (Get_Desired_Response("ACK", 10)){
		HAL_Delay(1000);
		xprintf(PC,  "New Sensor Config: %d\r\n\r\n", USB_BUFFER);
		Clear_USB_Buffers();
		return e_NONE;
	}

	return e_NONE;
}





/******************************************************************************
  * @brief	Change Server Number
  * @param	None
  * @return Events
  * @FVer		1.2.00
  * ***************************************************************************
*/
uint8_t DBG_Change_ServerNumber(void){
	bool invalid = false;
	uint8_t r = 0;

	xprintf(PC, "Current Server Number: ");
	HAL_Delay(200);

	xprintf(PMCU, "G_SVR\r\n");
	if (!Get_Desired_Response("SVR:", 10)){
		xprintf(PC, "Failed to retrieve\r\n");
		return e_NONE;
	}
	xprintf(PC, "%s\r\n", RESP_Buffer);
	HAL_Delay(100);


	Print_Modify_Cancel();
	HAL_Delay(100);
	if (ModifyCancelled(3))
		return e_NONE;

	do{
		xprintf(PC, "Enter phone number in 09XX format\r\n");
		UTL_GetString(60);

		if ((USB_BUFFER[0] < '0') || (USB_BUFFER[0] > '9') ||
				(USB_BUFFER[1] < '0') || (USB_BUFFER[1] > '9') ||
				(USB_BUFFER[2] < '0') || (USB_BUFFER[2] > '9') ||
				(USB_BUFFER[3] < '0') || (USB_BUFFER[3] > '9') ||
				(USB_BUFFER[4] < '0') || (USB_BUFFER[4] > '9') ||
				(USB_BUFFER[5] < '0') || (USB_BUFFER[5] > '9') ||
				(USB_BUFFER[6] < '0') || (USB_BUFFER[6] > '9') ||
				(USB_BUFFER[7] < '0') || (USB_BUFFER[7] > '9') ||
				(USB_BUFFER[8] < '0') || (USB_BUFFER[8] > '9') ||
				(USB_BUFFER[9] < '0') || (USB_BUFFER[9] > '9') ||
				(USB_BUFFER[10]< '0') || (USB_BUFFER[10]> '9') ||
				(USB_BUFFER[0]=='\0'))
			invalid = true;
		else
			invalid = false;

		if (invalid){
			Print_InvalidInput(USB_BUFFER);
			if (++r == 3){
				xprintf(PC, "Max retries! Exiting DEBUG mode\r\n");
				HAL_Delay(500);
				return e_NONE;
			}
		}
	} while(invalid);

	xprintf(PMCU, "S_SVR:%s\r\n", USB_BUFFER);
	if (Get_Desired_Response("ACK", 10)){
		HAL_Delay(1000);
		xprintf(PC,  "New Server Number: %s\r\n\r\n", USB_BUFFER);
		Clear_USB_Buffers();
		return e_NONE;
	}

	return e_NONE;
}





uint8_t DBG_Change_SIMNumber(void){
	bool invalid = false;
	uint8_t r = 0;

	xprintf(PC, "Current SIM Number: ");
	HAL_Delay(200);

	xprintf(PMCU, "G_SIM\r\n");
	if (!Get_Desired_Response("SIM:", 10)){
		xprintf(PC, "Failed to retrieve\r\n");
		return e_NONE;
	}
	xprintf(PC, "%s\r\n", RESP_Buffer);
	HAL_Delay(100);


	Print_Modify_Cancel();
	HAL_Delay(100);
	if (ModifyCancelled(3))
		return e_NONE;

	do{
		xprintf(PC, "Enter phone number in 09XX format\r\n");
		UTL_GetString(60);

		if ((USB_BUFFER[0] < '0') || (USB_BUFFER[0] > '9') ||
				(USB_BUFFER[1] < '0') || (USB_BUFFER[1] > '9') ||
				(USB_BUFFER[2] < '0') || (USB_BUFFER[2] > '9') ||
				(USB_BUFFER[3] < '0') || (USB_BUFFER[3] > '9') ||
				(USB_BUFFER[4] < '0') || (USB_BUFFER[4] > '9') ||
				(USB_BUFFER[5] < '0') || (USB_BUFFER[5] > '9') ||
				(USB_BUFFER[6] < '0') || (USB_BUFFER[6] > '9') ||
				(USB_BUFFER[7] < '0') || (USB_BUFFER[7] > '9') ||
				(USB_BUFFER[8] < '0') || (USB_BUFFER[8] > '9') ||
				(USB_BUFFER[9] < '0') || (USB_BUFFER[9] > '9') ||
				(USB_BUFFER[10]< '0') || (USB_BUFFER[10]> '9') ||
				(USB_BUFFER[0]=='\0'))
			invalid = true;
		else
			invalid = false;

		if (invalid){
			Print_InvalidInput(USB_BUFFER);
			if (++r == 3){
				xprintf(PC, "Max retries! Exiting DEBUG mode\r\n");
				HAL_Delay(500);
				return e_NONE;
			}
		}
	} while(invalid);

	xprintf(PMCU, "S_SIM:%s\r\n", USB_BUFFER);
	if (Get_Desired_Response("ACK", 10)){
		HAL_Delay(1000);
		xprintf(PC,  "New SIM Number: %s\r\n\r\n", USB_BUFFER);
		Clear_USB_Buffers();
		return e_NONE;
	}

	return e_NONE;
}





uint8_t DBG_Delete_RegisteredNumber(void){
	bool invalid = false;
	char c;
	uint8_t r = 0;

	ListRegisteredNumber();
	Clear_USB_Buffers();

	do{
		xprintf(PC, "Enter which registered number to delete 1, 2, 3 or"
				" (D)elete all\r\n");

		c = UTL_GetChar(60);

		if ((c == '1') ||
				(c == '2') ||
				(c == '3') ||
				(c == 'D'))
			invalid = false;
		else
			invalid = true;

		if (invalid){
			xprintf(PC, "Invalid character!\r\n");
			if (++r == 3){
				xprintf(PC, "Max retries! Exiting DEBUG mode\r\n");
				HAL_Delay(500);
				return e_NONE;
			}
		}
	}
	while(invalid);

	xprintf(PMCU, "S_DRN:%c\r\n", c);
	if (Get_Desired_Response("ACK", 10)){
		ListRegisteredNumber();
		Clear_USB_Buffers();
		return e_NONE;
	}

	ListRegisteredNumber();
	Clear_USB_Buffers();

	return e_NONE;
}





uint8_t DBG_List_Registered_Numbers(void){
	ListRegisteredNumber();
	return e_NONE;
}





static void ListRegisteredNumber(void){
	xprintf(PC, "Current Registered Numbers\r\n");
	HAL_Delay(200);

	xprintf(PMCU, "G_RN1\r\n");
	if (!Get_Desired_Response("RN1:", 10)){
		xprintf(PC, "Failed to retrieve\r\n");
	}
	xprintf(PC, "1) %s\r\n", RESP_Buffer);
	HAL_Delay(100);


	xprintf(PMCU, "G_RN2\r\n");
	if (!Get_Desired_Response("RN2:", 10)){
		xprintf(PC, "Failed to retrieve\r\n");
	}
	xprintf(PC, "2) %s\r\n", RESP_Buffer);
	HAL_Delay(100);


	xprintf(PMCU, "G_RN3\r\n");
	if (!Get_Desired_Response("RN3:", 10)){
		xprintf(PC, "Failed to retrieve\r\n");
	}
	xprintf(PC, "3) %s\r\n", RESP_Buffer);
	HAL_Delay(100);
}





/******************************************************************************
  * @brief	Print Option to modify or cancel command
  * @param	None
  * @return None
  * @FVer		1.2.00
  * ***************************************************************************
*/
bool ModifyCancelled(uint8_t maxRetry){
	bool invalid = true;
	char c;
	uint8_t x = 0;

	do{
		Clear_Buffer(USB_BUFFER, 255);
		c = UTL_GetChar(60);

		if (c == 'M' || c == 'C')
			invalid = false;
		else
			invalid = true;

		if (invalid){
			xprintf(PC, "Invalid character!\r\n");
			HAL_Delay(200);

			if (++x == maxRetry){
				xprintf(PC, "Max retries! Exiting DEBUG mode\r\n");
				HAL_Delay(100);
				return true;
			}
		}
	}while(invalid);

	if (c == 'C')
		return true;

	return false;
}





static void Print_InvalidInput(char *pData){
	xprintf(PC, "\"%s\" is an invalid input!\r\n");
	HAL_Delay(100);
}


/*
static void Print_InvalidChar(char pChar){
	xprintf(PC, "%c is an invalid character!\r\n", pChar);
	HAL_Delay(100);
}
*/




void Print_Modify_Cancel(void){
	xprintf(PC, "(M)odify or (C)ancel?\r\n");
	Clear_USB_Buffers();
}







void Print_Setting_Menu(void){
  for (uint8_t i = 0; i < 26; i++){

		#if (BLE_ENABLED)
  	sprintf(BLE_BUFFER, "%s\r\n", Settings_Menu[i]);
  	SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)BLE_BUFFER);
		#else
  	xprintf(PC, "%s\r\n", Settings_Menu[i]);
		#endif

  	HAL_Delay(5);
  }

  Clear_USB_Buffers();
}
