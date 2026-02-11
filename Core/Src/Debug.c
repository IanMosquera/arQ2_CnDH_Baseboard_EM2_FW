/*
 * Debug.c
 *
 *  Created on: Dec 18, 2025
 *      Author: IanCMosquera
 */

#include "Debug.h"
#include "string.h"
#include "stdlib.h"
#include "StateMachine.h"
#include "usbd_cdc_if.h"
#include "UtilityFunctions.h"

char SMenu[] ={
"SETTINGS MENU\r\n"
"A.     Get data from the Sensor\r\n"
"B.     Get current system configuration\r\n"
"C.     Display SETTINGS MENU again\r\n"
"D.     Change sending time\r\n"
"E.     -----\r\n"
"F.     Set Server Number\r\n"
"G.     List registered numbers\r\n"
"H.     Synct Time with PAGASA NTP Server\r\n"
"I.     Set Date and Time\r\n"
"J.     Set power board configuration\r\n"
"K.     Get GPS Data\r\n"
"L.     Read Data from Flash memory\r\n"
"M.     Change password\r\n"
"O.     Set sensor type\r\n"
"P.     Send via HTTP\r\n"
"Q.     Send via LoRa\r\n"
"R.     Set phone number\r\n"
"S.     Turn off Watchdog\r\n"
"T.     Set arQ Serial Number\r\n"
"U.     Turn On BLE Debug Mode\r\n"
"V.     Enable hybrid power saving\r\n"
"X.     Delete registered number\r\n"
"Z.     Exit Debug Mode\r\n\r\n"
};


uint8_t DEBUG_State(void){
	char x;

	Print_Setting_Menu();
	HAL_Delay(200);

	xprintf(PC, "Enter char:(Task timout 1 min)\r\n");
	x = UTL_GetChar(60);

	ret:

	switch(x){
		case 'A':{	//Get data from the Sensor\r\n"
			goto ret;
			break;
		}
		case 'B':{	//Get current system configuration

			break;
		}
		case 'C':{	//Display SETTINGS MENU again\r\n"
			return e_DBUG;
			break;
		}
		case 'D':{	//Change sending time
			ChangeSendingTime();
			break;
		}
		case 'Z':{
			return e_DONE;
			break;
		}
		case '\0':
			break;
		default:
			goto ret;
			break;
	}


	return e_DBUG;
}


void ChangeSendingTime(void){
	char c;
	uint8_t x;
	xprintf(PC, "Current Sending Time: 10\r\n");
	HAL_Delay(100);

	invalid_char:
	Print_Modify_Cancel();
	HAL_Delay(100);
	xprintf(PC, "(Timeout: 60 Sec)\r\n");

	c = UTL_GetChar(60);
	if (c == 'C'){
		return;
	}

	if (c != 'M'){
		xprintf(PC, "Invalid character!\r\n");
		goto invalid_char;
	}

	invalid_num:
	xprintf(PC, "Enter integer value in minutes (1-99)\r\n");
	UTL_GetString(60);

	if ((USB_BUFFER[0] < 48) || (USB_BUFFER[0] > 57) ||
		 (USB_BUFFER[1] < 48) || (USB_BUFFER[1] > 57)){
		xprintf(PC, "Invalid value!\r\n");
		Clear_USB_Buffers();
		goto invalid_num;
	}

	x = atoi(USB_BUFFER);
	if ((x < 0) || (x > 100)){
		xprintf(PC, "Invalid value!\r\n");
		Clear_USB_Buffers();
		goto invalid_num;
	}

	xprintf(PC, "New Sending Time: %d\r\n\r\n", x);
}


void Print_Modify_Cancel(void){
	xprintf(PC, "(M)odify or (C)ancel?\r\n");
	Clear_USB_Buffers();
}

void Print_Setting_Menu(void){
	CDC_Transmit_FS((uint8_t *)SMenu, strlen(SMenu));
	Clear_USB_Buffers();
}
