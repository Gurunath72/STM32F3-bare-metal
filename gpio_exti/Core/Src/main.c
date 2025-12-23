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
#include "stm32f303xe.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
//void SystemClock_Config(void);
void GPIO_Init();
static void EXTI_Init();
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t buttonPress = 0;
uint32_t InterruptStatus= 0;
void EXTI15_10_IRQHandler(void)
{
	InterruptStatus = EXTI->PR;
	if(InterruptStatus & EXTI_PR_PR13)
	{
		InterruptStatus |= EXTI_PR_PR13;
		EXTI->PR = InterruptStatus;
		buttonPress = 1;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	GPIO_Init();
	EXTI_Init();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
//  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if(buttonPress == 1)
	  {
		  buttonPress = 0;
		  GPIOA->ODR ^= (GPIO_ODR_5);
	  }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void GPIO_Init()
{
	// enbale GPIOA & GPIOC peripheral clocks
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

		// set GPIOA_PIN5 as output and GPIOC_PIN13 as input mode
		GPIOA->MODER &= ~(GPIO_MODER_MODER5);
		GPIOA->MODER |= (0x01U << GPIO_MODER_MODER5_Pos);

		GPIOC->MODER &= ~(GPIO_MODER_MODER13); // 0x00 in bits 26 & 27 as input mode for pin 13;

	//	/* disable pull up/down for INPUT pins*/
		GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR5);
		GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR13);

		GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_5);	// push pull mode for GPIOA pin5

		GPIOA->ODR &= ~(GPIO_ODR_5);	// set port A pin 5 output as low
}

static void EXTI_Init()
{
	__disable_irq();	// disable global interrupt
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;	// enable CLK for SYSCFG
	SYSCFG->EXTICR[3] &= ~(SYSCFG_EXTICR4_EXTI13);	    // clear 4 bits for EXTI13 line in register
	SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI13_PC);	// set portC - 0010 (0x02) from bit 4

	EXTI->IMR |= EXTI_IMR_MR13;		// Unmask the EXTI 13 line interrupt
	EXTI->FTSR |= EXTI_FTSR_TR13;	// select falling edge trigger for EXTI 13 interrupt
	NVIC_EnableIRQ(EXTI15_10_IRQn);	// enable EXTI 13 on NVIC;
	__enable_irq();
}

///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

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
