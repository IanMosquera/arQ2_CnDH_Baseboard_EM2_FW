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



void TMR_SEC_Count(void);
uint8_t IS_LEAP(uint8_t Year);
bool Task_TimeOut(uint16_t seconds);
void Task_TimeOut_Start(void);

#endif /* INC_TIMER_H_ */
