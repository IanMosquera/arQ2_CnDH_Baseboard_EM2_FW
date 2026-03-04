/*
 * Debug.h
 *
 *  Created on: Dec 18, 2025
 *      Author: IanCMosquera
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_

#include "main.h"


bool ModifyCancelled(uint8_t maxRetry);

uint8_t Debug_Mode(void);
uint8_t DBG_Change_DateTime(void);
uint8_t DBG_Change_Password(void);
uint8_t DBG_Change_SendingTime(void);
uint8_t DBG_Change_ServerNumber(void);
uint8_t DBG_List_Registered_Numbers(void);

void Print_Modify_Cancel(void);
void Print_Setting_Menu(void);


#endif /* INC_DEBUG_H_ */
