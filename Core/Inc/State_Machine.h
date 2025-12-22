/*
 * State_Machine.h
 *
 *  Created on: Dec 22, 2025
 *      Author: IanMo
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_


#include "main.h"


// Define State Machine
typedef enum
{
	s_DATA,
	s_DBUG,
	s_FLTS,
	s_IDLE,
	s_INIT,
	s_MANUAL_AT_MODE,
	s_MIDNIGHTRESET,
	s_PC,
	s_PMCU,
	s_PWSV,
	s_PRST,
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


extern e_Events g_currentEvent;
extern s_States currentState;
extern s_States nextState;

uint8_t STM_ActionWhileInState(uint8_t state);
void STM_UponEntering(uint8_t nextState);
void STM_UponExiting(uint8_t currentState);
void STM_StateManager(uint8_t event);

uint8_t STM_NextState(uint8_t currentState, uint8_t event);

#endif /* INC_STATE_MACHINE_H_ */
