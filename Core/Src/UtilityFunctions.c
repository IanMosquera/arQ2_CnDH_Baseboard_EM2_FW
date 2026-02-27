/*
 * UtilityFunctions.c
 *
 *  Created on: Dec 18, 2025
 *      Author: IanCMosquera
 */
#include "Timer.h"
#include "UtilityFunctions.h"
#include <stdio.h>
#include <string.h>
#include <usart.h>


char RESP_Buffer[100];


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





void Reset_PMCU(void){
	// Reset PMCU
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





bool UTL_CompareEqual(char *pStr1, char *pStr2){
	uint8_t i;
	i = strcmp(pStr1, pStr2);

	if (i == 0)
		return true;
	else
		return false;
}






/******************************************************************************
  * @brief	Gets the desired response based on the previous serial command sent
  * @param	Response	Pointer to an expected desired response string
  * @param	timeout		Maximum timeout
  * @return Pointer to string after the Desired response
  * @Testcode		https://onlinegdb.com/96cBLokyp
  * @FVer		1.2.00
  * ***************************************************************************
*/
char *Get_Desired_Response(char *Response, uint8_t timeout){
	bool whilex = true;
	char *ret = NULL;
	uint8_t len;

	CHAR_CTR = 0;
	HAL_UART_Receive_IT(UART_MCU, (uint8_t *)&UART_CHAR, 1);
	//HAL_UART_Receive_IT(UART_LTE, (uint8_t *)&UART_Char, 1);
	//HAL_UART_Receive_IT(UART_AMR, (uint8_t *)&UART_Char, 1);
	//HAL_UART_Receive_IT(UART_SDI, (uint8_t *)&UART_Char, 1);

	len = strlen(Response);
	Task_TimeOut_Start();

	while (whilex){
		ret = strstr(UART_Buffer, Response);
		if (ret){
			sprintf(RESP_Buffer, "%s", ret + len);
			return  RESP_Buffer;
		}

		if (Task_TimeOut(timeout))
			whilex = false;
	}

	return ret;
}






/******************************************************************************
  * @brief	Detects Command from SMCU, returns the traiing value
  * @param	cmd	Pointer to an expected save command
  * @return Trailing value
  * @Testcode		https://onlinegdb.com/YHIJEAjvH
  * @FVer		7.0
  * ***************************************************************************
*/
char *SaveCommandDetected(char *cmd){
	char *ret = NULL;
	uint8_t len;

	len = strlen(cmd);
	ret = strstr(UART_Buffer, cmd);

	if (ret){
		sprintf(RESP_Buffer, "%s", ret + len);
		return  RESP_Buffer;
	}

	return ret;
}
