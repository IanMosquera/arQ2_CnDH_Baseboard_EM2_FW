/*
 * InterruptTimer.c
 *
 *  Created on: Jun 26, 2025
 *      Author: IanCMosquera
 */

#include "interruptTimer.h"
#include "arQ_CnDH_BaseBoard.h"

extern arQ_t arQ;

void DateTime_Init(void)
{
	arQ.DTm.Sec 			= 45;
	arQ.DTm.Min 			= 9;
	arQ.DTm.Hour 			= 9;
	arQ.DTm.Days 			= 13;
	arQ.DTm.Month 		= 3;
	arQ.DTm.Year 			= 25;
	arQ.DTm.Year_Prev = 24;
	arQ.DTm.Hour_Old 	= 0;

	arQ.Cfg.GetDataTime = 2;

	arQ.Flg.THESAMEMINUTE_FLAG 	= false;
	arQ.DTm.ResetCPU 						= false;
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


void Count_arQ_Time(void)
{
	if (++arQ.DTm.Sec > 59)
	{
		arQ.DTm.Sec = 0;
		arQ.Flg.TEST_FLAG = true;

		if (++arQ.DTm.Min > 59)
		{
			arQ.DTm.Min = 0;
			if (++arQ.DTm.Hour > 23)
			{
				arQ.DTm.Hour = 0;
				++arQ.DTm.Days;

				if((arQ.DTm.Days == 29 && arQ.DTm.Month == 2 && !IS_LEAP(arQ.DTm.Year)) ||
					 (arQ.DTm.Days == 30 && arQ.DTm.Month == 2)                           ||
					 (arQ.DTm.Days == 31 &&
							 (arQ.DTm.Month == 4 ||
								arQ.DTm.Month == 6 ||
								arQ.DTm.Month == 9 ||
								arQ.DTm.Month == 11))																						||
					 (arQ.DTm.Days == 32))
				{
					arQ.DTm.Month++;
					arQ.DTm.Days = 1;
				}
				if (arQ.DTm.Month == 13)
				{
					arQ.DTm.Year++;
					arQ.DTm.Month = 1;
				}
				arQ.Flg.MIDNIGHT_RESET_LTE_FLAG = true;
			}
			/*if (arQ.DTm.Hour == 8)
				arQ.Flg.CUM_RAIN_RESET_FLAG = true;*/
		}
	}

	// Sync Time Every Hour
	if ((arQ.DTm.Hour_Old != arQ.DTm.Hour) &&
			(arQ.DTm.Min == 58))
	{
		arQ.Flg.SYNC_FLAG = true;
		arQ.DTm.Hour_Old = arQ.DTm.Hour;
	}
}


bool Time_To_Get_Sensor_Data(void)
{
	if ((arQ.DTm.Min % arQ.Cfg.GetDataTime) == 0)
		return true;
	else
	{
		arQ.Flg.DATA_TAKEN = false;
		return false;
	}
}
