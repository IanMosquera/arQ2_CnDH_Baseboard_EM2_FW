/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"

#include "stm32wbxx_ll_tim.h"
#include "stm32wbxx_ll_bus.h"
#include "stm32wbxx_ll_cortex.h"
#include "stm32wbxx_ll_rcc.h"
#include "stm32wbxx_ll_system.h"
#include "stm32wbxx_ll_utils.h"
#include "stm32wbxx_ll_pwr.h"
#include "stm32wbxx_ll_gpio.h"
#include "stm32wbxx_ll_dma.h"

#include "stm32wbxx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct
{
	char USB_BUFFER[255];
	char RETURN_VAL[255];
} buffers_t;


typedef struct{
	uint8_t ctr;
	uint8_t Sec;
	uint8_t Min;
	uint8_t Hour;
	uint8_t Hour_Old;
	uint8_t Days;
	uint8_t Month;

	uint16_t Year;
	uint16_t Year_Prev;

	char DateTime[30];
	char RTCDateTime[50];

	bool ResetCPU;
}dateTime_t;


typedef struct
{
	bool USBSERIAL_FLAG;
	bool RETURN_FLAG;
}flag_t;


typedef struct
{
	buffers_t			Buf;
	dateTime_t			DTm;
	flag_t				Flg;
}arQ_t;

extern TIM_HandleTypeDef htim17;

extern bool f_PMCU_MSG;
extern bool f_PMCU_QRY;
extern bool f_PMCU_CMD;
extern bool f_USB;

extern char strDisplay[250];
extern char	TEMP_Buffer[100];
extern char UART_Buffer[100];
extern char USB_BUFFER[255];

extern uint8_t CHAR_CTR;
extern uint8_t g_RGAccuTipsData;
extern uint8_t Mili_Sec_Ctr;
extern uint8_t UART_CHAR;

extern uint16_t Process_Ctr;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Check_Primary_Board(void);

//void MAIN_PROGRAM(void);
//void BLE_PROGRAM(void);

void RTC_Init(void);
void RTC_Assign_Date(RTC_DateTypeDef *pDate);
void RTC_Assign_Time(RTC_TimeTypeDef *pTime);
void RTC_ShowDateTime(void);

void BLE_Mode_LED_Stat(void);
void Main_Prog_LED_Stat(void);

void WatchDog_Reset(void);

void USBSerial_Interrupt_Check(void);

void USB_CDC_RxHandler(uint8_t*, uint32_t);
void Clear_USB_Buffers(void);

void xprintf(uint8_t stream, char *FormatString, ...);

void Log_Error(char *pBuffer);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NRST_PMCU_Pin GPIO_PIN_0
#define NRST_PMCU_GPIO_Port GPIOA
#define STAT_Pin GPIO_PIN_4
#define STAT_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define PC			0
#define GSM			1
#define PMCU		2


#define arQTimer &htim17
// Return Message Definitions

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
