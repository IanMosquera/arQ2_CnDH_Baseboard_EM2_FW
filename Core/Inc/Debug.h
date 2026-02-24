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
uint8_t Change_SendingTime(void);
void Print_Modify_Cancel(void);
void Print_Setting_Menu(void);


#endif /* INC_DEBUG_H_ */
