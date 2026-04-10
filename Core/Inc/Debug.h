/*
 * Debug.h
 *
 *  Created on: Dec 18, 2025
 *      Author: IanCMosquera
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_

#include "main.h"


bool CHAR_is_Not_Within_In_AtoZ(void);
bool Correct_DEBUG_Detected(void);
bool DEBUG_Exit_Detected(void);
bool Valid_Value_Format(char *pVal);

uint8_t Exit_Debug(void);
uint8_t Extract_Value_From_BLE_DebugMessage(char *value);
uint8_t Send_DEBUG_To_PMCU(void);
uint8_t Set_DEBUG_Value_to_PMCU(char *value);


void Print_Setting_Menu(void);


#endif /* INC_DEBUG_H_ */
