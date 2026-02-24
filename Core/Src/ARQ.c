/*
 * ARQ.c
 *
 *  Created on: Feb 24, 2026
 *      Author: IanMo
 */


#include "ARQ.h"
#include "UtilityFunctions.h"
#include "Timer.h"
#include <stdbool.h>
#include <stdlib.h>

char g_firmwareVer[4];






/*************************** Functions ****************************************/





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
/******************************************************************************
  * @brief	Get character from USB
  * @param	None
  * @return None
  * @FVer		7.0
  * ***************************************************************************
*/
char GetChar(uint8_t timeout){
	bool i = true;

	UTL_Buffer_Clear(USB_BUFFER, 255);
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
