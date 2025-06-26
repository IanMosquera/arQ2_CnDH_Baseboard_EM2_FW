/*
 * InterruptTimer.h
 *
 *  Created on: Jun 26, 2025
 *      Author: IanCMosquera
 */

#ifndef INC_INTERRUPTTIMER_H_
#define INC_INTERRUPTTIMER_H_

#include "main.h"

#include "stdbool.h"

void DateTime_Init(void);
void Count_arQ_Time(void);
uint8_t IS_LEAP(uint8_t year);

bool Time_To_Get_Sensor_Data(void);

#endif /* INC_INTERRUPTTIMER_H_ */
