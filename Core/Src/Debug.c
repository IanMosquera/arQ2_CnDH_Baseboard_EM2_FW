/*
 * Debug.c
 *
 *  Created on: Dec 22, 2025
 *      Author: IanMo
 */


#include "Debug.h"
#include "State_Machine.h"
#include "Global_Variables.h"

void Print_Setting_Menu(void)
{
	xprintf(PC, "\nFIRMWARE Version: %s\r\n", g_firmwareVer);

  for (uint8_t i = 0; i < 26; i++)
  {
  	xprintf(PC, "%s\r\n", Settings_Menu[i]);
  	HAL_Delay(5);
  }

  Clear_USB_Buffers();
  g_currentEvent = e_NONE;
}
