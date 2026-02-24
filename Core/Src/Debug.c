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



static void Print_InvalidInput(char *pData);


uint8_t Debug_Mode(void){
	bool invalid = false;
	char c;
	uint8_t r = 0;

	xprintf(PC, "\nFIRMWARE Version: %s\r\n", g_firmwareVer);

  for (uint8_t i = 0; i < 26; i++){
  	xprintf(PC, "%s\r\n", Settings_Menu[i]);
  	HAL_Delay(5);
  }

	do{
		xprintf(PC, "Enter choice: (A-Z)\r\n");
		HAL_Delay(100);

		c = UTL_GetChar(60);

		if (c < 'A' || c > 'Z')
			invalid = true;

		if (invalid){
			Print_InvalidInput(&c);
			if (++r == 3){
				xprintf(PC, "Max retries! Exiting DEBUG mode\r\n^^");
				HAL_Delay(500);
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
			Change_SendingTime();
			break;
		}
		case 'E':{	//No settings assigned
			break;
		}
		case 'F':{
			//DBG_ServerNumber_Change();
			break;
		}
		case 'G':{
			//DBG_RegisterNumbers_List();
			break;
		}
		case 'H':{
			//if (LTE_NTP_Assign())
			//		LTE_Query_Clock();
			break;
		}
		case 'I':{
			//DBG_DateTime_Set();
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
			//DBG_Password_Change();
			break;
		}
		case 'N':{
			break;
		}
		case 'O':{
			//DBG_SensorType_Change();
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
			//DBG_SIMNumber_Set();
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
			//DBG_RegisterNumber_Delete();
			break;
		}
		case 'Y':{
			break;
		}
		case 'Z':{
			//return e_DONE;
			break;
		}
	}
	return e_NONE;
}


uint8_t Change_SendingTime(void){
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


	xprintf(PC, "(M)odify or (C)ancel?\r\n");
	HAL_Delay(100);
	if (ModifyCancelled(3))
		return e_NONE;

	do{
		xprintf(PC, "Enter number (1-60) in minutes\r\n");
		UTL_GetString(60);

		x = atoi(USB_BUFFER);
		if (x < 1 || x > 60)
			invalid = true;

		if (invalid){
			Print_InvalidInput(USB_BUFFER);
			if (++r == 3){
				xprintf(PC, "Max retries! Exiting DEBUG mode\r\n");
				HAL_Delay(500);
				return e_NONE;
			}
		}
	} while(invalid);

	xprintf(PMCU, "S_SDT:%d$$", x);
	if (!Get_Desired_Response("ACK", 10)){
		xprintf(PC, "Failed to save\r\n");
		return e_NONE;
	}

	//g_SendingTime = x;
	xprintf(PC,  "New sending time: %d\r\n", x);
	Clear_USB_Buffers();

	return e_NONE;
}





/******************************************************************************
  * @brief	Print Option to modify or cancel command
  * @param	None
  * @return None
  * @FVer		1.2.00
  * ***************************************************************************
*/
bool ModifyCancelled(uint8_t maxRetry){
	bool invalid = false;
	char c;
	uint8_t x = 0;

	do{
		Clear_USB_Buffers();

		c = GetChar(60);
		if (c != 'M' || c != 'C')
			invalid = true;

		if (invalid){
			Print_InvalidInput(&c);

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






void Print_Modify_Cancel(void){
	xprintf(PC, "(M)odify or (C)ancel?\r\n");
	Clear_USB_Buffers();
}







void Print_Setting_Menu(void){
	xprintf(PC, "\nFIRMWARE Version: %s\r\n", g_firmwareVer);

  for (uint8_t i = 0; i < 26; i++){
  	xprintf(PC, "%s\r\n", Settings_Menu[i]);
  	HAL_Delay(5);
  }

  Clear_USB_Buffers();
}
