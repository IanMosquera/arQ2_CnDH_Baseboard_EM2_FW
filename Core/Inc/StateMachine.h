/*
 * StateMachine.h
 *
 *  Created on: Dec 18, 2025
 *      Author: IanCMosquera
 */

#ifndef INC_STATEMACHINE_H_
#define INC_STATEMACHINE_H_

#include "main.h"






typedef enum
{
	s_CHCK,
	s_DATA,
	s_DBUG,
	s_DBG1,
	s_FLTS,
	s_IDLE,
	s_INIT,
	s_MANUAL_AT_MODE,
	s_MIDNIGHTRESET,
	s_PC,
	s_PMCU,
	s_PRST,
	s_PWSV,
	s_SEND,
	s_SMSC,
	s_STRT,
	s_Undefined
}s_States;


// Define Events
typedef enum
{
	e_BBDT,
	e_BDTC,
	e_CHCK,
	e_CHGD,
	e_DBUG,
	e_DONE,
	e_INIT,
	e_InitDone,
	e_KBHT,
	e_LWBT,
	e_MidNight,
	e_NONE,
	e_PRST,
	e_SMSC,
	e_SRST,
	e_TaskDone,
	e_TTGD,
	e_Undefined,
	e_USBP
}e_Events;

typedef struct
{
	uint8_t cST;
	uint8_t cEvt;
	uint8_t nST;
}s_nextState;


extern e_Events g_CurrentEvent;
extern s_States g_currentState;
extern s_States nextState;


uint8_t CHECK_State(void);
uint8_t DEBUG_State(void);
uint8_t Filter_USB_String(void);
uint8_t IDLE_State(void);
uint8_t INIT_State(void);
uint8_t STM_ActionWhileInState(uint8_t state);
uint8_t STM_DetermineNextState(uint8_t state, uint8_t event);


void Clear_PMCU_Flags(void);
void STM_StateManager(uint8_t event);
void STM_UponEntering(uint8_t nextState);
void STM_UponExiting(uint8_t currentState);

#endif /* INC_STATEMACHINE_H_ */
