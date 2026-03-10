/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

#include "gpio.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ARQ.h"
#include "StateMachine.h"
#include "DateTime.h"

#include "stdbool.h"
#include "string.h"
#include "Timer.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_USB_Device_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  CHAR_CTR = 0;
  HAL_UART_Receive_IT(&huart1, &UART_CHAR, 1);

  currentState = s_STRT;
  g_CurrentEvent = e_NONE;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  STM_StateManager(g_CurrentEvent);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_LSI1|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSE;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */
  //LL_HSEM_1StepLock( HSEM, 5);
  /* USER CODE END Smps */
}

/* USER CODE BEGIN 4 */


void BLE_Mode_LED_Stat(void)
{
	HAL_GPIO_TogglePin(STAT_GPIO_Port, STAT_Pin);
}


void Main_Prog_LED_Stat(void)
{
	HAL_GPIO_WritePin(STAT_GPIO_Port, STAT_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(STAT_GPIO_Port, STAT_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(STAT_GPIO_Port, STAT_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(STAT_GPIO_Port, STAT_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);
}


void WatchDog_Reset(void){
	xprintf(PC, "Resetting Watchdog\r\n");
	HAL_Delay(10);
	//HAL_IWDG_Refresh(&hiwdg);
}










void Log_Error(char *pBuffer)
{

}







void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len){
	strcpy((char *)USB_BUFFER, (char *)Buf);
	f_USB = true;
}




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == PMCU_INT_Pin)
		f_PMCU_Responds = true;
}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == arQTimer){

		if (Mili_Sec_Ctr == 20){
			Mili_Sec_Ctr = 0;
			TMR_SEC_Count();
		}
		else	Mili_Sec_Ctr++;


		// Task Counter Timer
		if (Process_Ctr >= 65000)	Process_Ctr = 0;
		else	Process_Ctr++;

		//PMCU_Check();
		if ((SEC > 25) && (SEC < 31))
			f_CheckPMCU = true;
		else
			f_CheckPMCU = false;
	}
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart == &huart1){
		//HAL_UART_Receive_IT(&huart1, (uint8_t *)&UART_CHAR, 1);


		// Clear Buffer id starting to zero
		if (CHAR_CTR == 0)
			memset(TEMP_Buffer, '\0', 100);

		// Copy character to buffer
		TEMP_Buffer[CHAR_CTR] = UART_CHAR;

		if (CHAR_CTR > 100)
			CHAR_CTR = 0;
		else
			CHAR_CTR++;

		// reset counter when carriage return encounters
		/*if ((UART_CHAR== '\n') ||
			 ((TEMP_Buffer[CHAR_CTR-2] == '\r') && (TEMP_Buffer[CHAR_CTR-1] == '\n')))
		{
			CHAR_CTR = 0;
			sprintf(UART_Buffer, TEMP_Buffer);
		}*/

		if ((TEMP_Buffer[CHAR_CTR-1] == '^') && (TEMP_Buffer[CHAR_CTR-2] == '^')){
			CHAR_CTR = 0;
			snprintf(UART_Buffer, strlen(TEMP_Buffer)-1, "%s", TEMP_Buffer);
			f_PMCU_MSG = true;
		}

		if ((TEMP_Buffer[CHAR_CTR-1] == '$') && (TEMP_Buffer[CHAR_CTR-2] == '$')){
			CHAR_CTR = 0;
			snprintf(UART_Buffer, strlen(TEMP_Buffer)-1, "%s", TEMP_Buffer);
			f_PMCU_CMD = true;
		}

		if ((TEMP_Buffer[CHAR_CTR-1] == '?') && (TEMP_Buffer[CHAR_CTR-2] == '?')){
			CHAR_CTR = 0;
			snprintf(UART_Buffer, strlen(TEMP_Buffer)-1, "%s", TEMP_Buffer);
			f_PMCU_QRY = true;
		}

		HAL_UART_Receive_IT(&huart1, &UART_CHAR, 1);
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
