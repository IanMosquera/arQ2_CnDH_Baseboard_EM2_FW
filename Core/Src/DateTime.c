/*
 * DateTime.c
 *
 *  Created on: Mar 9, 2026
 *      Author: IanMo
 */


#include "ARQ.h"
#include "DateTime.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>




uint8_t 	ctr;
uint8_t 	DAY;
uint8_t 	HRS_OLD;
uint8_t 	HRS;
uint8_t 	MIN;
uint8_t 	MON;
uint8_t 	PREV_YRS;
uint8_t 	SEC;
uint8_t 	YRS;






void DTM_DateTime_Get(void){
  sprintf(g_DateTime, "%02d/%02d/%02d,%02d:%02d:%02d",
	  YRS%100, MON%100, DAY%100, HRS%100, MIN%100, SEC%100);
}





/******************************************************************************
  * @brief	Set arQ Date and Time from formatted string buffer
  * @Note		previous InsertTimeToVariable
  * @param	None
  * @return None
  * @FVer		7.0
  * ***************************************************************************
*/
void DTM_DateTime_Set(char *pMsg){
  char *pData = NULL;
  char token[6];

  sprintf(token, "/,:\"");

  pData = strtok(pMsg, token);
  YRS = atoi(pData);

  for (uint8_t i = 0; i < 5; i++){
  	pData = strtok(0, token);

  	if (i == 0) MON = atoi(pData);
  	if (i == 1) DAY = atoi(pData);
  	if (i == 2) HRS = atoi(pData);
  	if (i == 3) MIN = atoi(pData);
  	if (i == 4) SEC = atoi(pData);
  }

  DTM_DateTime_Get();
}








