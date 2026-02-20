/*
 * StateMachine.c
 *
 *  Created on: Dec 18, 2025
 *      Author: IanCMosquera
 */

#include "Debug.h"
#include "StateMachine.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "Timer.h"
#include "UtilityFunctions.h"

e_Events g_CurrentEvent;
s_States currentState;
s_States nextState;

s_nextState nState[] = {
{s_STRT, e_NONE, s_INIT},

{s_INIT, e_NONE, s_IDLE},
{s_INIT, e_BBDT, s_PMCU},
{s_INIT, e_USBP, s_IDLE},
{s_INIT, e_BDTC, s_IDLE},
{s_INIT, e_LWBT, s_PWSV},

{s_IDLE, e_NONE, s_IDLE},
{s_IDLE, e_CHCK, s_CHCK},
{s_IDLE, e_DBUG, s_DBUG},
{s_IDLE, e_BDTC, s_IDLE},
{s_IDLE, e_LWBT, s_PWSV},
{s_IDLE, e_TTGD, s_DATA},
{s_IDLE, e_SMSC, s_SMSC},
{s_IDLE, e_PRST, s_PRST},

{s_CHCK, e_NONE, s_CHCK},
{s_CHCK, e_DONE, s_IDLE},

{s_DBUG, e_DBUG, s_DBUG},
{s_DBUG, e_DONE, s_IDLE},

{s_DATA, e_NONE, s_SEND},

{s_SEND, e_NONE, s_IDLE},

{s_PWSV, e_NONE, s_PWSV},
{s_PWSV, e_LWBT, s_PWSV},
{s_PWSV, e_KBHT, s_FLTS},
{s_PWSV, e_CHGD, s_IDLE}};


char g_variable[4];

uint8_t STM_ActionWhileInState(uint8_t state){
	switch (state){
		case s_IDLE:{
			return IDLE_State();
			break;
		}
		case s_DBUG:{
			return DEBUG_State();
			break;
		}
	}

	return e_NONE;
}





void STM_UponEntering(uint8_t nextState){
	switch (nextState){
		case s_INIT:{
			INIT_State();
			break;
		}

		case s_FLTS:{
			g_CurrentEvent = FilterUSB_State();
			break;
		}

		case s_CHCK:{
			g_CurrentEvent = CHECK_State();
		}

		default:
			break;
	}
}





void STM_UponExiting(uint8_t currentState){

}





void STM_StateManager(uint8_t event){
	nextState = STM_DetermineNextState(currentState, event);

	// Transition to next state
	if (nextState != currentState){
		STM_UponExiting(currentState);
		STM_UponEntering(nextState);
		currentState = nextState;
	}

	if (event != e_Undefined)
		g_CurrentEvent = STM_ActionWhileInState(currentState);
}




e_Events FilterUSB_State(void){
	uint8_t i;

	i = strcmp(USB_BUFFER, "DEBUG\r\n");
	if (i == 0){
		Clear_USB_Buffers();
		return e_DBUG;
	}

	return e_NONE;
}





e_Events IDLE_State(void){

	if (f_USB){
		f_USB = false;
		return FilterUSB_State();
	}

	if (f_PMCU_MSG){
		xprintf(PC, "%s", UART_Buffer);
		memset(UART_Buffer, '\0', 100);
		f_PMCU_MSG = false;
  }

	if (f_PMCU_QRY){
		if (strcmp(UART_Buffer, "Status") == 0){
			xprintf(PMCU, "Attached\r\n");
			f_PMCU_QRY = false;
		}
	}

	// "S_RG1:100$$"
	if (f_PMCU_CMD){
		if (UART_Buffer[0] == 'S'){ //Save Data
			strncpy(g_variable, UART_Buffer+2, 3);
			if (strcmp(g_variable, "RG1")==0){
				char val[20];
				uint8_t i = 6;
				do{
					val[i-6] = UART_Buffer[i];
					i++;
				}while(UART_Buffer[i] != '\0');
				g_RGAccuTipsData = atoi(val);
				xprintf(PMCU, "ACK RG1:%d\r\n", g_RGAccuTipsData);
			}
		}

		if (UART_Buffer[0] == 'G'){ //Get Data
			strncpy(g_variable, UART_Buffer+2, 3);
			if (strcmp(g_variable, "RG1")==0){
				xprintf(PMCU, "RG1:%d\r\n", g_RGAccuTipsData);
			}
		}

		f_PMCU_CMD = false;
	}

	/*if (SEC%1 == 0){
		xprintf(PC, "IDLE State: %02d:%02d:%02d\r\n", HRS,MIN,SEC);
		HAL_Delay(500);
	}*/


	if ((MIN == 27) && (SEC < 2)){
		//HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState);
		return e_CHCK;
	}

	return e_NONE;
}




uint8_t CHECK_State(void){
	if (f_PMCU_CMD){
		if (!UTL_CompareEqual(UART_Buffer, "ACK")){
			Reset_PMCU();
			f_PMCU_CMD = false;
			return e_DONE;
		}


		xprintf(PMCU, "G_DTM$$");
		if (!Get_Desired_Response("DTM:", 5)){
			Reset_PMCU();
			f_PMCU_CMD = false;
			return e_DONE;
		}
		// Sync Date and Time


		xprintf(PMCU, "G_RG1$$");
		if (!Get_Desired_Response("RG1:", 5)){
			Reset_PMCU();
			f_PMCU_CMD = false;
			return e_DONE;
		}
		g_RGTipsData =  atoi(RESP_Buffer);


		xprintf(PMCU, "S_EXT$$");
		if (!Get_Desired_Response("ACK", 5)){
			Reset_PMCU();
			f_PMCU_CMD = false;
			return e_DONE;
		}
		//HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PIN_RESET);

	}

	return e_DONE;
}





uint8_t INIT_State(void){
	HAL_TIM_Base_Start_IT(arQTimer);
	return e_NONE;
}





uint8_t STM_DetermineNextState(uint8_t state, uint8_t event){
	for (uint8_t i = 0; i < 50; i++){
		if (nState[i].cST == currentState){
			if(nState[i].cEvt == event)
				return nState[i].nST;
		}
	}

	// Default
	return s_IDLE;
}
