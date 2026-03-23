/*
 * Timer.h
 *
 *  Created on: Dec 18, 2025
 *      Author: IanCMosquera
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include "main.h"

typedef enum
{
	DEF,
	JAN,
	FEB,
	MAR,
	APR,
	MAY,
	JUN,
	JUL,
	AUG,
	SEP,
	OCT,
	NOV,
	DEC
}e_Month_Name;



bool Task_TimeOut(uint16_t seconds);

uint8_t IS_LEAP(uint8_t Year);

void BLE_Timer_Loop(void);
void Task_TimeOut_Start(void);
void TMR_SEC_Count(void);

#endif /* INC_TIMER_H_ */
