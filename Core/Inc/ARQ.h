/*
 * ARQ.h
 *
 *  Created on: Feb 24, 2026
 *      Author: IanMo
 */

#ifndef INC_ARQ_H_
#define INC_ARQ_H_

#include "main.h"

extern char g_firmwareVer[];



char GetChar(uint8_t timeout);
void Get_Config(void);
#endif /* INC_ARQ_H_ */
