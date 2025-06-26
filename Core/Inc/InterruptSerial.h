/*
 * InterruptSerial.h
 *
 *  Created on: Jun 26, 2025
 *      Author: IanCMosquera
 */

#ifndef INC_INTERRUPTSERIAL_H_
#define INC_INTERRUPTSERIAL_H_

#include "main.h"

#include "stdbool.h"

void rda_isr(void);
void rda_isr2(void);
void Clear_UART_Buffers(void);
void Clear_USB_Buffers(void);
void getDataFromPC(void);

void Wait_Data_From_PC(bool serial_f);
char *GetResponse(void);

char *Get_Serial_Response();

void Get_iSCC_Data(void);

// New Fucntion Codes
void Get_Data_From_USB(void);

#endif /* INC_INTERRUPTSERIAL_H_ */
