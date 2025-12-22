/*
 * Debug.h
 *
 *  Created on: Dec 22, 2025
 *      Author: IanMo
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_


#include "main.h"

static const char Settings_Menu[26][50] =
{
    "SETTINGS MENU",
    "A.     Get data from the Sensor",
    "B.     Get current system configuration",
    "C.     Display SETTINGS MENU again",
    "D.     Change sending time",
    "E.     -----",
    "F.     Set Server Number",
    "G.     List registered numbers",
    "H.     Synct Time with PAGASA NTP Server",
    "I.     Set Date and Time",
    "J.     Set power board configuration",
    "K.     Get GPS Data",
    "L.     Read Data from Flash memory",
    "M.     Change password",
    "O.     Set sensor type",
    "P.     Send via HTTP",
    "Q.     Send via LoRa",
    "R.     Set phone number",
    "S.     Turn off Watchdog",
    "T.     Set arQ Serial Number",
    "U.     Turn On BLE Debug Mode",
    "V.     Enable hybrid power saving",
    "X.     Delete registered number",
    "Z.     Exit Debug Mode",
    "",
    "Enter Choice"
};



uint8_t DBG_SendingTime_Change(void);
uint8_t Debug_Exiting(void);
uint8_t DEBUG_State(void);


void Debug_Get_Char(void);
void Modify_Command(void);

void Print_For_Deploy(void);
void Print_Modify_Cancel(void);
void Print_Setting_Menu(void);
void Print_Settings_Format(void);
#endif /* INC_DEBUG_H_ */
