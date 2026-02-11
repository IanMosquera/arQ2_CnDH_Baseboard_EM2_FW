/*
 * UtilityFunctions.c
 *
 *  Created on: Dec 18, 2025
 *      Author: IanCMosquera
 */
#include "UtilityFunctions.h"
#include "Timer.h"

/******************************************************************************
  * @brief	Print header text of DOST ASTI
  * @param	None
  * @return None
  * @FVer		1.2.00
  * ***************************************************************************
*/
char UTL_GetChar(uint8_t timeout){
	uint8_t i = 1;

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
  * @brief	Print header text of DOST ASTI
  * @param	None
  * @return None
  * @FVer		1.2.00
  * ***************************************************************************
*/
void UTL_GetString(uint8_t timeout){
	f_USB = false;

	Task_TimeOut_Start();

	while(!f_USB){
		if (Task_TimeOut(timeout)){
			f_USB = false;
			return;
		}
	}
	return;
}
