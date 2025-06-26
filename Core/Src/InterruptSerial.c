/*
 * InterruptSerial.c
 *
 *  Created on: Jun 26, 2025
 *      Author: IanCMosquera
 */


#include "interruptSerial.h"
#include "arQ_CnDH_Baseboard.h"

#include "string.h"
#include "usbd_cdc_if.h"


extern arQ_t arQ;


// "Fires when Serial receives a character"
// [!] Needs to integrate on HAL_UART_ReceiveCallBack function"
void rda_isr(void)
{
	arQ.Flg.USB_SERIAL_FLAG = true;
	// arQ.Buf.RXD_DATA[arQ.Ctr.WRITE_CNTR++] = rxBuffer[0];
}





void rda_isr2(void)
{
	arQ.Flg.UART_SERIAL_FLAG = 1;
	// arQ.Buf.RXD2_DATA[arQ.Ctr.WRITE2_CNTR++] = rxBuffer[0];
}




void Clear_UART_Buffers(void)
{
	memset(arQ.Buf.UART_DATA, '\0', 255);

	arQ.Ctr.WRITE_CNTR = 0;
	arQ.Ctr.READ_CNTR	= 0;
	arQ.Flg.UART_SERIAL_FLAG = false;
}





void Clear_USB_Buffers(void)
{
	memset(arQ.Buf.RXD2_DATA, '\0', 255);

	arQ.Ctr.WRITE2_CNTR  = 0;
	arQ.Flg.USB_SERIAL_FLAG = 0;
}





void getDataFromPC(void)
{
  int while_ = 0;

  while(while_ == 0)
  {
    if (arQ.Flg.UART_SERIAL_FLAG == true)
    {
      HAL_Delay(200);
      arQ.Buf.RXD2_DATA[arQ.Ctr.WRITE2_CNTR - 1] = '\0';
      strcpy(arQ.Buf.FROMSERIALPC, arQ.Buf.RXD2_DATA);
      Clear_USB_Buffers();
      while_ = 1;
    }
  }
}



void Get_Data_From_USB(void)
{
  bool escape = false;

  while(escape == false)
  {
    if (arQ.Flg.UART_SERIAL_FLAG == true)
    {
      HAL_Delay(200);
      //arQ.Buf.RXD2_DATA[arQ.Ctr.WRITE2_CNTR - 1] = '\0';
      strcpy(arQ.Buf.FROMSERIALPC, arQ.Buf.RXD2_DATA);
      Clear_USB_Buffers();
      escape = true;
    }
  }
}



void Wait_Data_From_PC(bool serial_f)
{
  uint8_t whilex = 0;

  while (whilex == 0)
  {
    if (serial_f == true)	// Character present in rxBuff
    {
      HAL_Delay(200);
      serial_f = false;
      whilex = 1;
    }
  }
}




char *Get_Serial_Response()
{
	uint8_t i, j, x;
	uint8_t truthCounter;
	uint8_t bufferLength;
	uint8_t desiredResponseLength;

	bufferLength = strlen(arQ.Buf.UART_DATA);
	desiredResponseLength = strlen(arQ.Buf.DESIRED_RESPONSE);

	for (i = 0; i < bufferLength; i++)
	{
		// First character found
		if (arQ.Buf.UART_DATA[i] == arQ.Buf.DESIRED_RESPONSE[0])
		{
			truthCounter = 0;
			for(j = 0; j < desiredResponseLength; i++, j++)
			{
				if (arQ.Buf.UART_DATA[i] == arQ.Buf.DESIRED_RESPONSE[j])
					truthCounter++;

				if (truthCounter == desiredResponseLength)
				{
					for (x = 0; x < (bufferLength - 1); x++)
						arQ.Buf.GSM_RESPONSE[x] = arQ.Buf.UART_DATA[x+i+1];

					return &arQ.Buf.GSM_RESPONSE[0];
				}
			}
		}
	}
	return NULL;
}





char *GetResponse(void)
{
  /*uint8_t tempCtr = 0;
  uint8_t i = 0;
  uint8_t len = 0;

  len = strlen(arQ.Buf.DESIRED_RESPONSE);

  // "Delay until all character length is read, for less than 3 seconds"
  while(((arQ.Ctr.WRITE_CNTR - arQ.Ctr.READ_CNTR) < len) && ++tempCtr < 30)
  {
    HAL_Delay(100);
  }

  len = arQ.Ctr.READ_CNTR;

  if (tempCtr < 30)
  {
	  tempCtr = 0;
	  do
	  {
		  do
		  {
			  for (; (arQ.Ctr.READ_CNTR != arQ.Ctr.WRITE_CNTR) &&
			         (arQ.Buf.RXD_DATA[arQ.Ctr.READ_CNTR] != arQ.Buf.DESIRED_RESPONSE[0]);
			  				arQ.Ctr.READ_CNTR = (arQ.Ctr.READ_CNTR+1) % 255);

			  for (i = 0; (arQ.Ctr.READ_CNTR != arQ.Ctr.WRITE_CNTR) &&
				    (arQ.Buf.RXD_DATA[arQ.Ctr.READ_CNTR] == arQ.Buf.DESIRED_RESPONSE[i]) &&
						(arQ.Buf.DESIRED_RESPONSE[i] != 0);
				    i++, arQ.Ctr.READ_CNTR = (arQ.Ctr.READ_CNTR+1) % 255);

				if (arQ.Buf.DESIRED_RESPONSE[i] == 0)
				{
					arQ.Buf.RXD_DATA[arQ.Ctr.READ_CNTR] = 0;
					len = (arQ.Ctr.WRITE_CNTR != arQ.Ctr.READ_CNTR);

					for (i = 0; i <= len; i++)
					{
						arQ.Buf.GSM_RESPONSE[i] = arQ.Buf.RXD_DATA[arQ.Ctr.READ_CNTR + i];
					}
					arQ.Ctr.READ_CNTR = arQ.Ctr.WRITE_CNTR;
					return &arQ.Buf.GSM_RESPONSE[0];
				}
		  }
		  while((arQ.Buf.DESIRED_RESPONSE[i] != 0) &&
				    (arQ.Ctr.READ_CNTR != arQ.Ctr.WRITE_CNTR));
		  HAL_Delay(100);
		  arQ.Ctr.READ_CNTR = len;
	  }
	  while (++tempCtr < 10);
  }

  arQ.Ctr.READ_CNTR = arQ.Ctr.WRITE_CNTR;*/
  return NULL;
}




void Get_iSCC_Data(void)
{
	uint8_t counter = 0;

	arQ.Buf.LTC_BUFFER[0] = '\0';
	Clear_USB_Buffers();

	// Catch ISCC OUTPUT
	while (counter < 20)
	{
		strcpy(arQ.Buf.LTC_BUFFER, arQ.Buf.RXD2_DATA);
		Clear_USB_Buffers();
		HAL_Delay(500);
		counter++;
	}
	xprintf(PC, "iSCC:%s\n", arQ.Buf.LTC_BUFFER);
}
