/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdbool.h>

/* Private includes ------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MICROSECONDS_GRANULARITY 4
// œÓÚ≥·Ì‡ ˜‡ÒÚÓÚ‡ ÒÔ‡ˆ˛‚‡Ì¸ Ú‡ÈÏÂ‡ SysTick ‚ „Âˆ‡ı
#define FREQ ((1000000)/(MICROSECONDS_GRANULARITY))
// const uint32_t freq=1000000/microseconds_granularity;
#define TICKS ((SystemCoreClock)/(FREQ))
//const uint32_t ticks=SystemCoreClock/freq;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
inline static void delay_some_us(uint32_t mks)
{
	uint32_t ticks=mks/MICROSECONDS_GRANULARITY;
	int stop_ticks=ticks+HAL_GetTick();
	while (HAL_GetTick() < stop_ticks) {};
}

//! «‡ÚËÏÍ‡ ‚ Ï≥ÍÓÒÂÍÛÌ‰‡ı
inline static void delay_some_us1 (uint32_t dlyTicks) {
	// ¬ÁˇÚÓ Á leaflabs Maple
	dlyTicks *= 6; //STM32_DELAY_US_MULT;

	/* ÕÂ‚ÂÎËÍ‡ ≥ ÌÂ ‰ÛÊÂ ÚÓ˜Ì‡ ÔÓÔ‡‚Í‡ Ì‡ ˜‡Ò ‚ËÍÎËÍÛ ÙÛÌˆ≥ø */
	dlyTicks--;
	asm volatile("   mov r0, %[dlyTicks]          \n\t"
	             "1: subs r0, #1            \n\t"
	             "   bhi 1b                 \n\t"
	             :
	             : [dlyTicks] "r" (dlyTicks)
	             : "r0");
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  if( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) )
   {
    // Помилка -- імпульсу не було, а на Echo вже одиниця
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // Синім позначатимемо помилку
    printf("Error -- Echo line is high, though no impuls was given\n");
    while(1); // Зависаємо
   }

   uint32_t starting_ticks;
   uint32_t timeout_ticks;
   uint32_t distance_mm;

   bool are_echoing=false; // For C needs <stdbool.h>! Some violation of strict C standard here.

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    	if(!are_echoing)
    			{
    				// œ‡ÛÁ‡ Ï≥Ê ‚ËÏ≥‡ÏË, ‰‚≥˜≥ ÔÓ 1/4 ÒÂÍÛÌ‰Ë
    				delay_some_us(500000/2);
    				// √‡ÒËÏÓ Ò‚≥ÚÎÓ‰≥Ó‰Ë
    				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    				delay_some_us(500000/2);
    				are_echoing=true;
    				// œÓÒËÎ‡∫ÏÓ ≥ÏÔÛÎ¸Ò
    				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    				delay_some_us(12);
    				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    				// œÓ˜ËÌ‡∫ÏÓ Á‡Ï≥ˇÚË ˜‡Ò
    				timeout_ticks=HAL_GetTick();
    			}else{
    				// ﬂÍ˘Ó Ê ÔÓ‰‡ÎË -- ˜ÂÍ‡∫ÏÓ Ì‡ ≥ÏÔÛÎ¸Ò ≥ Á‡Ï≥ˇ∫ÏÓ ÈÓ„Ó ÚË‚‡Î≥ÒÚ¸
    				bool measuring=false;
    				uint32_t measured_time;
    				// —Í≥Î¸ÍË ˜ÂÍ‡ÚË, ÔÓÍË ÌÂ ‚Ë≥¯ËÚË, ˘Ó ≥ÏÔÛÎ¸ÒÛ ‚ÊÂ ÌÂ ·Û‰Â
    				uint32_t usoniq_timeout = 100000;
    				while( (HAL_GetTick() - timeout_ticks) < usoniq_timeout )
    				{
    					// œÂÂ‚≥ˇ∫ÏÓ ÎÓ„≥˜ÌËÈ ≥‚ÂÌ¸ Ì‡ Echo
    					//printf("B->IDR %X\n",GPIOB->IDR);
    					if( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) )
    					{
    						// ﬂÍ˘Ó ‡Ì≥¯Â ÒË„Ì‡ÎÛ ÌÂ ·ÛÎÓ, ÔÓ˜ËÌ‡∫ÏÓ Á‡Ï≥ˇÚË ÈÓ„Ó ÚË‚‡Î≥ÒÚ¸
    						if( !measuring )
    						{
    							starting_ticks=HAL_GetTick();
    							measuring=true;
    						}
    					}else{
    						// ﬂÍ˘Ó ÒË„Ì‡Î Ì‡ Echo ·Û‚ ≥ ÁÌËÍ -- Á‡Ï≥ˇ∫ÏÓ ÈÓ„Ó ÚË‚‡Î≥ÒÚ¸
    						if(measuring)
    						{
    							// –ÂÁÛÎ¸Ú‡Ú ·Û‰Â ‚ Ï≥Î≥ÏÂÚ‡ı
    							measured_time = (HAL_GetTick() - starting_ticks)*10*MICROSECONDS_GRANULARITY;
    							distance_mm = measured_time/58;
    							// œÓ‚≥‰ÓÏÎˇ∫ÏÓ ÔÓ ÛÒÔ≥¯ÌËÈ ‚ËÏ≥ ÁÂÎÂÌËÏ Ò‚≥ÚÎÓ‰≥Ó‰ÓÏ
    							HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    							break;
    						}
    					}
    				}
    				if(!measuring)
    				{
    					// ÕÂ ÓÚËÏ‡ÎË ÒË„Ì‡ÎÛ, ÔÓ‚≥‰ÓÏÎˇ∫ÏÓ ÔÓ ÔÓÏËÎÍÛ ≥ ÔÓ·Û∫ÏÓ ˘Â
    					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    					printf("Echo signal not arrived in time\n");
    				}else{
    					printf("Distance: %u mm, measured time: %lu us\n",distance_mm, measured_time/10);
    				}
    				are_echoing=false;

    			}
    	  }
  }
  /* USER CODE END 3 */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
	printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
