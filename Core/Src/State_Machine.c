/*
 * State_Machine.c
 *
 *  Created on: Dec 22, 2025
 *      Author: IanMo
 */


#include "State_Machine.h"
#include "Global_Variables.h"


e_Events g_currentEvent;
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
{s_IDLE, e_DBUG, s_DBUG},
{s_IDLE, e_PRST, s_PRST},
{s_IDLE, e_SMSC, s_SMSC},
{s_IDLE, e_TTGD, s_DATA},

{s_IDLE, e_BDTC, s_IDLE},
{s_IDLE, e_LWBT, s_PWSV},

{s_IDLE, e_KBHT, s_FLTS},


{s_DBUG, e_DBUG, s_DBUG},
{s_DBUG, e_DONE, s_IDLE},

{s_DATA, e_NONE, s_SEND},

{s_SEND, e_NONE, s_IDLE},

{s_PWSV, e_LWBT, s_PWSV},
{s_PWSV, e_CHGD, s_IDLE}};


/******************************************************************************
  * @brief	Perform actions based on current state
  * @param	uint8_t state	Current_State
  * @return None
  * @FVer		1.2.00
  * ***************************************************************************
*/
uint8_t STM_ActionWhileInState(uint8_t state)
{
	switch (state){
		case s_IDLE:{
			return IDLE_State();
			break;
		}
		case s_DBUG:{
			return DEBUG_State();
			break;
		}
		case s_PWSV:{
			return POWERSAVING_State();
			break;
		}
		case s_SMSC:{
			return e_NONE;
			break;
		}
		case s_PMCU:{
			return AA_PMCU();
			break;
		}
		default:
			break;
	}
	return g_currentEvent;
}
/******************************************************************************
  * @brief	Evaluate current state machine based on ocurring events
  * @param	uint8_t event	Current_Event
  * @return None
  * @FVer		1.2.00
  * Test Code: https://onlinegdb.com/3sdQJaDh1
  * **************************************************************************
*/
void STM_StateManager(uint8_t event)
{
	nextState = STM_NextState(currentState, event);


	// Transition
	if (nextState != currentState){
		STM_UponExiting(currentState);
		STM_UponEntering(nextState);

		currentState = nextState;
	}

	if (event != e_Undefined)
		g_currentEvent = STM_ActionWhileInState(currentState);
}


/******************************************************************************
  * @brief	Extra functionality
  * 				Execute code upon entering a state
  * @param	nextState	nextState
  * @return None
  * @FVer		1.2.00
  * ***************************************************************************
*/
void STM_UponEntering(uint8_t nextState)
{
	DTM_DateTime_Get();

	xprintf(SMCU, "\r\nEntering %s: %s\r\n^^", Get_State(nextState), g_DateTime);
	HAL_Delay(100);

	switch (nextState)
	{
		case s_INIT:{
			g_currentEvent = State_ArQ_Init();
			break;
		}

		case s_PWSV:{
			PCM_BoostPins_Disable();
			break;
		}

		case s_FLTS:{
			g_currentEvent = UTL_Filter_PC_String();
		}

		case s_PRST:{
			ARQ_Reset();
			break;
		}
		case s_DATA:		UP_ENT_Data_Gathering();		break;
		case s_SEND:		UP_ENT_Send_Data();					break;
		case s_SMSC:		UP_ENT_SMS_Call();					break;
		case s_MANUAL_AT_MODE:	UP_ENT_MANUAL_AT_MODE(); 		break;
		default:	break;
	}

	HAL_Delay(100);
}


/******************************************************************************
  * @brief	Execute code upon leaving a state
  * @note		Extra functionality
  * @param	currentState	State upon exiting
  * @return None
  * @FVer		1.2.00
  * ***************************************************************************
*/
void STM_UponExiting(uint8_t currentState)
{
	switch (currentState)
	{
		case s_DBUG:
		{
			UP_EXT_Debug_Mode();
			break;
		}
		case s_DATA:
		{
			UP_EXT_Data_Gathering();
			break;
		}
		case s_SEND:
		{
			g_currentEvent = e_NONE;
			f_DataTaken = true;
			break;
		}
		case s_SMSC:
		{
			f_SMS = false;
			break;
		}
		default:
			break;
	}
	HAL_Delay(100);
}


/******************************************************************************
  * @brief	Determine the next state
  * @param	cState Current state
  * @param	evt	occuring event
  * @return uint8_t Next state
  * @FVer		1.2.00
  * ***************************************************************************
*/
uint8_t STM_NextState(uint8_t currentState, uint8_t event)
{
	for (uint8_t i = 0; i < 50; i++){
		if (nState[i].cST == currentState){
			if(nState[i].cEvt == event)
				return nState[i].nST;
		}
	}

	// Default
	return s_IDLE;
}
