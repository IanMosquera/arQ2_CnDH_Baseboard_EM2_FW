/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"

#include "arQ_CnDH_BaseBoard.h"
#include "InterruptTimer.h"
#include "InterruptSerial.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* SPP */
  uint8_t               Rx_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[512];
uint8_t NotifyCharData[512];
uint16_t Connection_Handle;
/* USER CODE BEGIN PV */
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

extern UART_HandleTypeDef huart1;

extern uint8_t rxChar;

extern arQ_t arQ;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* SPP */
static void Custom_Rx_Update_Char(void);
static void Custom_Rx_Send_Notification(void);

/* USER CODE BEGIN PFP */
void Check_Primary_Board(void);
void Read_Data(void);
static void BLE_Send_String(void);
void bprintf(uint8_t stream, char *FormatString, ...);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* SPP */
    case CUSTOM_STM_TX_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_TX_READ_EVT */

      /* USER CODE END CUSTOM_STM_TX_READ_EVT */
      break;

    case CUSTOM_STM_TX_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_TX_WRITE_NO_RESP_EVT */
    	pNotification->DataTransfered.pPayload[pNotification->DataTransfered.Length] = '\0';
      /* USER CODE END CUSTOM_STM_TX_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_RX_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_RX_READ_EVT */

      /* USER CODE END CUSTOM_STM_RX_READ_EVT */
      break;

    case CUSTOM_STM_RX_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_RX_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_RX_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_RX_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_RX_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_RX_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */

      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */
	UTIL_SEQ_RegTask(1 << CFG_TASK_READ_DATA,		UTIL_SEQ_RFU, Read_Data);
	UTIL_SEQ_RegTask(1 << CFG_TASK_CHECK_PMCU,	UTIL_SEQ_RFU, Check_Primary_Board);
	UTIL_SEQ_RegTask(1 << CFG_TASK_SEND_STR, 		UTIL_SEQ_RFU, BLE_Send_String);
  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
void Read_Data(void)
{
	//sprintf(SystemMessage, "VBAT: 12.56");
	//SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&SystemMessage[0]);
}


void Check_Primary_Board(void)
{
	Clear_UART_Buffers();

  HAL_UART_Receive_IT(&huart1, &rxChar, 1);
  xprintf(MCU, "Check_MCU");
  //HAL_Delay(200);

  if (strcmp(arQ.Buf.UART_DATA, "PMCU_OK") == 0)
  {
  	arQ.Flg.PMCU_STAT_FLAG_OK = true;
  	xprintf(PC, "%s\r\n", arQ.Buf.UART_DATA);
  }
  else
  {
  	xprintf(PC, "PMCU Error\r\n");
  	//bprintf(BLE, "PMCU Error\r\n");
  	arQ.Flg.PMCU_STAT_FLAG_OK = false;
  }

  Clear_UART_Buffers();
}

static void BLE_Send_String(void)
{
	sprintf(arQ.Buf.PC_MSG, "VBAT: 12.56\r\n");
	SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&arQ.Buf.PC_MSG[0]);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == SW1_Pin)
  {
  	sprintf(arQ.Buf.PC_MSG, "SW1 Pressed\r\n");
  	UTIL_SEQ_SetTask(1 << CFG_TASK_SEND_STR, CFG_SCH_PRIO_0);
  }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim16)
	{
		USBSerial_Interrupt_Check();
		if (arQ.Flg.BLE_MODE_FLAG == true)
			xprintf(PC, "BLE Mode Running\r\n");
	}


	if (htim == &htim17) // every 50ms
	{
		if (arQ.Ctr.PROG_CTR >= 250)
			arQ.Ctr.PROG_CTR = 0;
		else
			arQ.Ctr.PROG_CTR++;

		if ((arQ.Ctr.PROG_CTR % 20) == 0) // every 1 Sec
		{
			Count_One_Sec();
		}

		if ((arQ.Ctr.PROG_CTR % 100) == 0) // every 5 Sec
			UTIL_SEQ_SetTask(1 << CFG_TASK_CHECK_PMCU, CFG_SCH_PRIO_0);

		Blink_LED();

	}
}

void bprintf(uint8_t stream, char *FormatString, ...)
{
	va_list args;
	char *sval;
	int  ival;
	float fval;
	char tempSTR[100];
	char cdcSTR[100];
	char format[10];
	int8_t i, j, x;
	uint8_t len;

	len = strlen(FormatString);
	va_start(args, FormatString);

	for (i = 0, j = 0; j < len; i++, j++)
	{
		tempSTR[i] = FormatString[j];

		if (FormatString[j] == '%')
		{
			tempSTR[i] = '\0';
			j++;

			if (stream == PC) CDC_Transmit_FS((uint8_t *)tempSTR, strlen(tempSTR));
			else if (stream == MCU) HAL_UART_Transmit(&huart1, (uint8_t *)tempSTR, strlen(tempSTR), HAL_MAX_DELAY);
			else if (stream == BLE) SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&tempSTR[0]);

			HAL_Delay(10);
			x = 0;
			format[x++] = '%';

			if (FormatString[j] != 's')
			{
				do format[x++] = FormatString[j++];
				while (FormatString[j] != 'd' && FormatString[j] != 'f');
			}

			if (FormatString[j] == 's')
			{
				sval = va_arg(args, char *);

				if (stream == PC) CDC_Transmit_FS((uint8_t *)sval, strlen(sval));
				else if (stream == MCU) HAL_UART_Transmit(&huart1, (uint8_t *)sval, strlen(sval), HAL_MAX_DELAY);
				else if (stream == BLE) SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&tempSTR[0]);
			}
			else if (FormatString[j] == 'd')
			{
				format[x] = 'd';
				format[x+1] = '\0';
				ival = va_arg(args, int);
				sprintf(cdcSTR, format, ival);

				if (stream == PC) CDC_Transmit_FS((uint8_t *)cdcSTR, strlen(cdcSTR));
				else if (stream == MCU) HAL_UART_Transmit(&huart1, (uint8_t *)cdcSTR, strlen(cdcSTR), HAL_MAX_DELAY);
				else if (stream == BLE) SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&tempSTR[0]);
			}
			else if (FormatString[j] == 'f')
			{
				format[x] = 'f';
				format[x+1] = '\0';
				fval = va_arg(args, double);
				sprintf(cdcSTR, format, fval);
				if (stream == PC) CDC_Transmit_FS((uint8_t *)cdcSTR, strlen(cdcSTR));
				else if (stream == MCU) HAL_UART_Transmit(&huart1, (uint8_t *)cdcSTR, strlen(cdcSTR), HAL_MAX_DELAY);
				else if (stream == BLE) SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&tempSTR[0]);
			}
			HAL_Delay(10);
			i = -1;
		}
	}
	tempSTR[i] = '\0';
	if (stream == PC) CDC_Transmit_FS((uint8_t *)tempSTR, strlen(tempSTR));
	else if (stream == MCU) HAL_UART_Transmit(&huart1, (uint8_t *)tempSTR, strlen(tempSTR), HAL_MAX_DELAY);
	else if (stream == BLE) SPP_Update_Char(CUSTOM_STM_RX, (uint8_t *)&tempSTR[0]);
	va_end(args);
}
/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* SPP */
__USED void Custom_Rx_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Rx_UC_1*/

  /* USER CODE END Rx_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_RX, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Rx_UC_Last*/

  /* USER CODE END Rx_UC_Last*/
  return;
}

void Custom_Rx_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Rx_NS_1*/

  /* USER CODE END Rx_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_RX, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Rx_NS_Last*/

  /* USER CODE END Rx_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
