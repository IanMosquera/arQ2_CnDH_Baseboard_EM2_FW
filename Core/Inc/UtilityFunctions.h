/*
 * UtilityFunctions.h
 *
 *  Created on: Dec 18, 2025
 *      Author: IanCMosquera
 */

#ifndef INC_UTILITYFUNCTIONS_H_
#define INC_UTILITYFUNCTIONS_H_

#include "main.h"
#include <stdbool.h>

extern char RESP_Buffer[];




bool UTL_CompareEqual(char *pStr1, char *pStr2);

char *Get_Desired_Response(char *Respons, uint8_t timeout);
char UTL_GetChar(uint8_t timeout);

void Reset_PMCU(void);
void UTL_GetString(uint8_t timeout);
#endif /* INC_UTILITYFUNCTIONS_H_ */
