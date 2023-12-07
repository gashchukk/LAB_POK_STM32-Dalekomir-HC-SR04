/* USER CODE BEGIN Header */
/**
  **************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  **************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd5110.h"
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
#define LOOP_FREQ (SystemCoreClock/4000000)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void init_timing();
uint32_t get_us();
void udelay(uint32_t useconds);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

typedef enum state_t {
 IDLE_S,
 TRIGGERING_S,
 WAITING_FOR_ECHO_START_S,
 WAITING_FOR_ECHO_STOP_S,
 TRIG_NOT_WENT_LOW_S,
 ECHO_TIMEOUT_S,
 ECHO_NOT_WENT_LOW_S,
 READING_DATA_S,
 ERROR_S
} state_t;

volatile state_t state = IDLE_S;

volatile uint32_t echo_start;
volatile uint32_t echo_finish;
volatile uint32_t measured_time;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
	 if (GPIO_Pin == GPIO_PIN_2 )
	 {
	  switch (state) {
	  case WAITING_FOR_ECHO_START_S: {
	   echo_start =  get_us();
	   state = WAITING_FOR_ECHO_STOP_S;
	   break;
	  }
	  case WAITING_FOR_ECHO_STOP_S: {
	   echo_finish = get_us();
	   measured_time = echo_finish - echo_start;
	   state = READING_DATA_S;
	   break;
	  }
	  default:
	   state = ERROR_S;
	  }
	 }
	}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
LCD5110_display lcd1;

void udelay_asm (uint32_t useconds) {
 useconds *= LOOP_FREQ;

    asm volatile("   mov r0, %[useconds]    \n\t"
                 "1: subs r0, #1            \n\t"
                 "   bhi 1b                 \n\t"
                 :
                 : [useconds] "r" (useconds)
                 : "r0");
}
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */


    if( HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) )
     {
      // Помилка -- імпульсу не було, а на Echo вже одиниця
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // Синім позначатимемо помилку
      printf("Error -- Echo line is high, though no impuls was given\n");
     }
    lcd1.hw_conf.spi_handle = &hspi1;
	lcd1.hw_conf.spi_cs_pin =  LCD1_CS_Pin;
	lcd1.hw_conf.spi_cs_port = LCD1_CS_GPIO_Port;
	lcd1.hw_conf.rst_pin =  LCD1_RST_Pin;
	lcd1.hw_conf.rst_port = LCD1_RST_GPIO_Port;
	lcd1.hw_conf.dc_pin =  LCD1_DC_Pin;
	lcd1.hw_conf.dc_port = LCD1_DC_GPIO_Port;
	lcd1.def_scr = lcd5110_def_scr;
	LCD5110_init(&lcd1.hw_conf, LCD5110_NORMAL_MODE, 0x40, 2, 3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
      while (1)
      {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    	  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    	    udelay_asm(16);
    	    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

    	    while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET );
    	      {}
    	    uint32_t before = HAL_GetTick();
    	    while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET );
    	      {}
    	    uint32_t pulse_time = HAL_GetTick()-before;
    	    //! Увага, не забудьте додати:
    	    // monitor arm semihosting enable
    	    // До  Debug Configurations -> Startup Tab:
    	    LCD5110_print("Distance is : ", BLACK, &lcd1);
  		//! Увага, не забудьте додати:
  		// monitor arm semihosting enable
  		// До  Debug Configurations -> Startup Tab:
  		char buffer[100];
//  		LCD5110_print("Time is %lu us", BLACK, &lcd1);
//  		LCD5110_print("Distance is %lu cm", BLACK, &lcd1);
  		sprintf(buffer, "%d",  pulse_time*343/20);
  		LCD5110_print(buffer, BLACK, &lcd1);
  		memset(buffer, '\0', sizeof(buffer));
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
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
