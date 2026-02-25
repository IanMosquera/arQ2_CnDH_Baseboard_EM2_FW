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
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "StateMachine.h"
#include "stdarg.h"
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

arQ_t 	arQ;

bool BLE_INIT = false;
bool f_PMCU_MSG = false;
bool f_PMCU_CMD = false;
bool f_PMCU_QRY = false;
bool f_USB = false;
bool f_InitState = true;
bool f_Disable_PMCU_MSG = false;

uint8_t g_RGAccuTipsData;
uint8_t g_RGTipsData;

char strDisplay[250];
char TEMP_Buffer[100];
char UART_Buffer[100];
char USB_BUFFER[255];

uint8_t CHAR_CTR;
uint8_t UART_CHAR;
uint8_t Mili_Sec_Ctr = 0;

uint16_t Process_Ctr = 0;
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

void Check_Primary_Board(void)
{
	if (arQ.Flg.RETURN_FLAG == false)
	{
		Log_Error(arQ.Buf.RETURN_VAL);
		arQ.Flg.RETURN_FLAG = true;
	}
}



/*void MAIN_PROGRAM(void)
{
	WatchDog_Reset();

	USBSerial_Interrupt_Check();

	//xprintf(PC, "Main Program Running\n");

	HAL_Delay(10);
	RTC_ShowDateTime();

	Main_Prog_LED_Stat();

	HAL_Delay(1000);
}*/


/*void BLE_PROGRAM(void)
{
	if (BLE_INIT == false)
	{
		MX_APPE_Init();
		BLE_INIT = true;
	}
	MX_APPE_Process();
}*/


void RTC_Init(void)
{
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};

	RTC_Assign_Date(&sDate);
	RTC_Assign_Time(&sTime);
}

void RTC_Assign_Date(RTC_DateTypeDef *pDate)
{
	pDate->WeekDay		= RTC_WEEKDAY_THURSDAY;
	pDate->Month			= RTC_MONTH_MARCH;
	pDate->Date				= 0x03;
	pDate->Year				= 0x25;
	if (HAL_RTC_SetDate(&hrtc, pDate, RTC_FORMAT_BCD) != HAL_OK) Error_Handler();
}

void RTC_Assign_Time(RTC_TimeTypeDef *pTime)
{
	pTime->Hours 			= 0x09;
	pTime->Minutes		=	0x00;
	pTime->Seconds		= 0x00;
	pTime->SubSeconds	= 0x00;
	pTime->DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	pTime->StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, pTime, RTC_FORMAT_BCD) != HAL_OK) Error_Handler();
}

void RTC_ShowDateTime(void)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;

  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
  HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);

  xprintf(PC, "Date and Time: %02d/%02d/%02d,%02d:%02d:%02d\r\n",
  		sdatestructureget.Year, sdatestructureget.Month, sdatestructureget.Date,
  		stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
}


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


void WatchDog_Reset(void)
{
	xprintf(PC, "Resetting Watchdog\r\n");
	HAL_Delay(10);
	//HAL_IWDG_Refresh(&hiwdg);
}




void Clear_USB_Buffers(void)
{
	memset(USB_BUFFER, '\0', 255);
	f_USB = false;
}



void xprintf(uint8_t stream, char *FormatString, ...){
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
			else if (stream == PMCU) HAL_UART_Transmit(&huart1, (uint8_t *)tempSTR, strlen(tempSTR), HAL_MAX_DELAY);

			HAL_Delay(50);
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
				else if (stream == PMCU) HAL_UART_Transmit(&huart1, (uint8_t *)sval, strlen(sval), HAL_MAX_DELAY);
			}
			else if (FormatString[j] == 'd')
			{
				format[x] = 'd';
				format[x+1] = '\0';
				ival = va_arg(args, int);
				sprintf(cdcSTR, format, ival);

				if (stream == PC) CDC_Transmit_FS((uint8_t *)cdcSTR, strlen(cdcSTR));
				else if (stream == PMCU) HAL_UART_Transmit(&huart1, (uint8_t *)cdcSTR, strlen(cdcSTR), HAL_MAX_DELAY);
			}
			else if (FormatString[j] == 'f')
			{
				format[x] = 'f';
				format[x+1] = '\0';
				fval = va_arg(args, double);
				sprintf(cdcSTR, format, fval);
				if (stream == PC) CDC_Transmit_FS((uint8_t *)cdcSTR, strlen(cdcSTR));
				else if (stream == PMCU) HAL_UART_Transmit(&huart1, (uint8_t *)cdcSTR, strlen(cdcSTR), HAL_MAX_DELAY);
			}
			HAL_Delay(50);
			i = -1;
		}
	}
	tempSTR[i] = '\0';
	if (stream == PC) CDC_Transmit_FS((uint8_t *)tempSTR, strlen(tempSTR));
	else if (stream == PMCU) HAL_UART_Transmit(&huart1, (uint8_t *)tempSTR, strlen(tempSTR), HAL_MAX_DELAY);
	va_end(args);
}


void Log_Error(char *pBuffer)
{

}







void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len){
	strcpy((char *)USB_BUFFER, (char *)Buf);
	f_USB = true;
}





void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == arQTimer){
		if (Mili_Sec_Ctr == 20){
			Mili_Sec_Ctr = 0;
			TMR_SEC_Count();
		}
		else Mili_Sec_Ctr++;

		// Task Counter Timer
		if (Process_Ctr >= 65000)
			Process_Ctr = 0;
		else
			Process_Ctr++;
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
			HAL_GPIO_TogglePin(STAT_GPIO_Port, STAT_Pin);
			f_PMCU_MSG = true;
		}

		if ((TEMP_Buffer[CHAR_CTR-1] == '$') && (TEMP_Buffer[CHAR_CTR-2] == '$')){
			CHAR_CTR = 0;
			snprintf(UART_Buffer, strlen(TEMP_Buffer)-1, "%s", TEMP_Buffer);
			HAL_GPIO_TogglePin(STAT_GPIO_Port, STAT_Pin);
			f_PMCU_CMD = true;
		}

		if ((TEMP_Buffer[CHAR_CTR-1] == '?') && (TEMP_Buffer[CHAR_CTR-2] == '?')){
			CHAR_CTR = 0;
			snprintf(UART_Buffer, strlen(TEMP_Buffer)-1, "%s", TEMP_Buffer);
			HAL_GPIO_TogglePin(STAT_GPIO_Port, STAT_Pin);
			f_PMCU_QRY = true;
		}

		//if (f_Disable_PMCU_MSG ==  false){
			HAL_UART_Receive_IT(&huart1, &UART_CHAR, 1);
		//}

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
