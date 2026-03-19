/*
 * StateMachine.c
 *
 *  Created on: Dec 18, 2025
 *      Author: IanCMosquera
 */

#include "ARQ.h"
#include "DateTime.h"
#include "Debug.h"
#include "StateMachine.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "Timer.h"
#include "UtilityFunctions.h"

#include <usart.h>

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
{s_DBUG, e_NONE, s_DBUG},
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


		case s_CHCK:{
			g_CurrentEvent = CHECK_State();
		}

		default:
			break;
	}
}





void STM_UponExiting(uint8_t currentState){

}







void Clear_PMCU_Flags(void){
	f_PMCU_MSG = false;
	f_PMCU_QRY = false;
	f_PMCU_CMD = false;
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




uint8_t Filter_USB_String(void){
	if (UTL_CompareEqual(USB_BUFFER, "DEBUG\r\n")){
		Clear_USB_Buffers();
		f_InitState = true;
		return e_DBUG;
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








/******************************************************************************
  * @brief	Debug State code
  * @param	None
  * @return events
  * @FVer		7.0
  * ***************************************************************************
*/
uint8_t DEBUG_State(void){
	return Debug_Mode();
}






uint8_t IDLE_State(void){
	// Initial State
	if (f_InitState){
	  CHAR_CTR = 0;
	  HAL_UART_Receive_IT(&huart1, &UART_CHAR, 1);
		f_InitState = false;
	}


	// USB input Check
	if (f_USB){
		f_USB = false;
		return Filter_USB_String();
	}




	// Interrupt PMCU
	if (f_CheckPMCU){
		HAL_GPIO_WritePin(INT_PMCU_GPIO_Port, INT_PMCU_Pin, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(INT_PMCU_GPIO_Port, INT_PMCU_Pin, GPIO_PIN_RESET);
	}


	// NRST_PMCU
	if (g_Fault_Ctr == 3){
		HAL_GPIO_WritePin(NRST_PMCU_GPIO_Port, NRST_PMCU_Pin, GPIO_PIN_RESET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(NRST_PMCU_GPIO_Port, NRST_PMCU_Pin, GPIO_PIN_SET);
		g_Fault_Ctr = 0;
	}



	// Message from PMCU
	if (f_PMCU_MSG){
		f_PMCU_MSG = false;
		Print_UARTBuffer();
  }

	if (f_PMCU_QRY){
		if (strcmp(UART_Buffer, "Status") == 0){
			xprintf(PMCU, "Attached\r\n");
			f_PMCU_QRY = false;
		}
	}

	//
	if (f_PMCU_CMD){
		f_PMCU_CMD = false;
		Extract_PMCUCommand();
	}

	return e_NONE;
}



uint8_t INIT_State(void){
	HAL_GPIO_WritePin(NRST_PMCU_GPIO_Port, NRST_PMCU_Pin, GPIO_PIN_SET);
	HAL_UART_Receive_IT(&huart1, &UART_CHAR, 1);
	HAL_TIM_Base_Start_IT(arQTimer);


	DTM_DateTime_Set("26/02/04,15:13:00");

/*	if (Retry(Get_DateTime_From_PMCU, 3)){
		DTM_DateTime_Set(RESP_Buffer);
		xprintf(PC, "Date Time: %s\r\n", g_DateTime);
	}*/



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
