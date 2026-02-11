/*
 * Timer.c
 *
 *  Created on: Dec 18, 2025
 *      Author: IanCMosquera
 */
#include "Timer.h"


uint8_t 	ctr;
uint8_t 	DAY;
uint8_t 	HRS_OLD;
uint8_t 	HRS;
uint8_t 	MIN;
uint8_t 	MON;
uint8_t 	PREV_YRS;
uint8_t 	SEC;
uint8_t 	YRS;



/******************************************************************************
  * @brief	Timer Elapse Function
  * @note		Previous Name "Timer1_isr"
  * @param	*htim Pointer to a triggering timer
  * @return None
  * @FVer		1.2.00
  * ***************************************************************************
*/
void TMR_SEC_Count(void)
{
	if (++SEC > 59)
	{
		SEC = 0;

		if (++MIN > 59)
		{
			MIN = 0;

			if (++HRS > 23)
			{
				HRS = 0;
				++DAY;

				if((DAY == 29 &&  MON == FEB && !IS_LEAP(YRS)) ||
					(DAY == 30 &&  MON == FEB)    ||
					(DAY == 31 && (MON == APR		||
										MON == JUN   	||
										MON == SEP   	||
										MON == NOV))	||
					 (DAY == 32))
				{
					MON++;
					DAY = 1;
				}

				if (MON == 13)
				{
					YRS++;
					MON = JAN;
				}

			}
			/*if (HRS == 8)
				arQ.Flg.CUM_RAIN_RESET_FLAG = true;*/
		}
	}
}




uint8_t IS_LEAP(uint8_t Year)
{
	uint16_t Year_ = 0;

	Year_ = Year + 2000;
	if (((Year_ % 4 == 0) && (Year % 100 != 0)) || (Year % 400 ==0))
		return 1;
	else
		return 0;
}


bool Task_TimeOut(uint16_t seconds)
{
	if (Process_Ctr >= (seconds * 20)){
		Process_Ctr = 0;
		return true;
	}

	return false;
}
/******************************************************************************
  * @brief	Start task counting
  * @param	None
  * @return true or false
  * @FVer		1.2.00
  * ***************************************************************************
*/
void Task_TimeOut_Start(void){
	Process_Ctr = 0;
}
