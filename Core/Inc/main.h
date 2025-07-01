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
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"

#include "arQ_CnDH_BaseBoard.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/*typedef struct
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
	buffers_t				Buf;
	dateTime_t			DTm;
	flag_t					Flg;
}arQ_t;*/
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


void ArQ_DateTime_Init(void);
void ArQ_ShowDateTime(void);
void ArQ_Sys_Init(void);
void BLE_Mode_LED_Stat(void);
void BLE_Program(void);
void Blink_LED(void);
void Check_Primary_Board(void);
void Log_Error(char *pBuffer);
void Main_Program(void);
void Main_Prog_LED_Stat(void);
void RTC_Assign_Date(RTC_DateTypeDef *pDate);
void RTC_Assign_Time(RTC_TimeTypeDef *pTime);
void RTC_Init(void);
void RTC_ShowDateTime(void);
void USBSerial_Interrupt_Check(void);
void USB_CDC_RxHandler(uint8_t*, uint32_t);
void WatchDog_Reset(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NRST_PMCU_Pin GPIO_PIN_0
#define NRST_PMCU_GPIO_Port GPIOA
#define WDI_Pin GPIO_PIN_1
#define WDI_GPIO_Port GPIOA
#define STAT_Pin GPIO_PIN_4
#define STAT_GPIO_Port GPIOA
#define GPIO4_Pin GPIO_PIN_5
#define GPIO4_GPIO_Port GPIOA
#define IO3_Pin GPIO_PIN_2
#define IO3_GPIO_Port GPIOB
#define GPIO1_Pin GPIO_PIN_0
#define GPIO1_GPIO_Port GPIOB
#define GPIO2_Pin GPIO_PIN_1
#define GPIO2_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_4
#define SW1_GPIO_Port GPIOE
#define GPIO3_Pin GPIO_PIN_4
#define GPIO3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define PC			0
#define GSM			1

// Return Message Definitions

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
