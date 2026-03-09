/*
 * DateTime.h
 *
 *  Created on: Mar 9, 2026
 *      Author: IanMo
 */

#ifndef INC_DATETIME_H_
#define INC_DATETIME_H_

#include "main.h"


extern uint8_t ctr;
extern uint8_t DAY;
extern uint8_t HRS_OLD;
extern uint8_t HRS;
extern uint8_t MIN;
extern uint8_t MON;
extern uint8_t PREV_YRS;
extern uint8_t SEC;
extern uint8_t YRS;

void DTM_DateTime_Get(void);
void DTM_DateTime_Set(char *pMsg);

#endif /* INC_DATETIME_H_ */
