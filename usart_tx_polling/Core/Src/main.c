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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void GPIO_Init();
void UART_Init();
static size_t uart_write(USART_TypeDef *USARTx, uint8_t *txBuffer, size_t len);
static void uart_write_byte(USART_TypeDef *USARTx, uint8_t byte);
static uint8_t uart_read_byte(USART_TypeDef *USARTx);
static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate);
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate);
/* USER CODE END PFP */

///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//int __io_putchar(int ch)
//{
//	ITM_SendChar(ch);
//}

uint8_t usart2_rxBuffer[100]={0};
uint8_t usart3_rxBuffer[100]={0};

uint8_t usart2_txBuffer[100] = "welcome to excel vlsi\r\n";

uint8_t usart2_rxPos = 0;
uint8_t usart3_txPos =0;

void USART2_IRQHandler(void)
{
	if (USART2->ISR & USART_ISR_RXNE) {
		usart2_rxBuffer[] = USART2->RDR; // Read received data
	        // You can add further processing here, e.g., store in a ring buffer
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
	UART_Init();
	uint8_t txBuffer[30] = "Welcome to Excel VLSI\r\n";

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
	size_t datLen = 0;
  /* USER CODE END 2 */
	uint32_t i =0;
	uint8_t bytedata =0;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
//	  datLen =  uart_write(USART2, txBuffer, 25);
	  uart_write_byte(USART2, (uint8_t)'A');
//	  printf("%.*s",25, txBuffer);
//	  for( i=0; i< 4000000; i++){};
	  bytedata = uart_read_byte(USART2);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void GPIO_Init()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	// set GPIOA_PIN2 as AF- USART2_TX pin
	GPIOA->MODER &= ~(GPIO_MODER_MODER2);
//	GPIOA->MODER |= (1 << 5U);
	GPIOA->MODER |= (0x02U << GPIO_MODER_MODER2_Pos);

	/* set GPIOA_PIN3 as AF- USART2_RX pin*/
	GPIOA->MODER &= ~(GPIO_MODER_MODER3);
//	GPIOA->MODER |= (1 << 7U);
	GPIOA->MODER |= (0x02U << GPIO_MODER_MODER3_Pos);

	GPIOB->MODER &= ~(GPIO_MODER_MODER8);
	GPIOB->MODER |= (0x02U << GPIO_MODER_MODER8_Pos);


	GPIOB->MODER &= ~(GPIO_MODER_MODER9);
	GPIOB->MODER |= (0x02U << GPIO_MODER_MODER9_Pos);
	/* set A.F type for pin PA2 and PA3 */
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL2);
	GPIOA->AFR[0] |= ((uint32_t)0x07 << GPIO_AFRL_AFRL2_Pos);

	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL3);
	GPIOA->AFR[0] |= ((uint32_t)0x07 << GPIO_AFRL_AFRL3_Pos);

	/* set A.F type for pin Pb8 and Pb9 as USART3 Rx and Tx */
	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFRH0);
	GPIOA->AFR[1] |= ((uint32_t)0x07 << GPIO_AFRL_AFRL2_Pos);

	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFRH1);
	GPIOA->AFR[1] |= ((uint32_t)0x07 << GPIO_AFRH_AFRH1_Pos);

	/* disable pull up/down for Tx pins*/
//	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR2);
	/*  enable pull up for RX pin*/
//	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR3);
//	GPIOA->PUPDR |= ((uint32_t)0x01 << GPIO_PUPDR_PUPDR3_Pos);

	/* set Tx pin gpio output type as push pull*/
//	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_2);	// push pull mode for GPIOA pin2
}

void UART_Init()
{
	/*enable clock for USART2*/
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	USART2->CR1 &= ~USART_CR1_UE;
	USART3->CR1 &= ~USART_CR1_UE;
	/* Configure baud rate */
	uart_set_baudrate(USART2, 8000000U, 9600U);
	uart_set_baudrate(USART3, 8000000U, 9600U);
//	/* set M[1:0} as 0x00 8 data word length*/
//	USART2->CR1 &= ~(USART_CR1_M0);
//	USART2->CR1 &= ~(USART_CR1_M1);
//	/* Set stop bit as 1 */
//	USART2->CR2 &= ~(USART_CR2_STOP);
//	/* disable parity */
//	USART2->CR1 &= ~(USART_CR1_PCE);


	/* enable Transmitter */
	USART2->CR1 |= USART_CR1_TE;
	USART2->CR1 |= USART_CR1_RE;
	USART2->CR1 |= USART_CR1_RXNEIE;

	USART3->CR1 |= USART_CR1_TE;
	USART3->CR1 |= USART_CR1_RE;
	USART3->CR1 |= USART_CR1_RXNEIE;

	/* enable USART*/
	USART2->CR1 |= USART_CR1_UE;
	USART3->CR1 |= USART_CR1_UE;

	NVIC->ISER[((uint32_t)USART2_IRQn >> 5UL)] = (uint32_t)(1UL << (((uint32_t)USART2_IRQn) & 0x1FUL));
	NVIC->ISER[((uint32_t)USART3_IRQn >> 5UL)] = (uint32_t)(1UL << (((uint32_t)USART3_IRQn) & 0x1FUL));
}


static size_t uart_write(USART_TypeDef *USARTx, uint8_t *txBuffer, size_t len)
{
	size_t txlen = len;
	uint8_t *txDataPtr = txBuffer;
	uint32_t count = 0;
	while(txlen--)
	{
		uart_write_byte(USARTx, (uint8_t)((*txDataPtr) & 0xFFU));
		txDataPtr++;
		count++;
	}
	return count;
}

static void uart_write_byte(USART_TypeDef *USARTx, uint8_t byte)
{
	while(!(USARTx->ISR & USART_ISR_TXE));
	USARTx->TDR = byte;
}

static uint8_t uart_read_byte(USART_TypeDef *USARTx)
{
	uint8_t byte;
	while(!(USARTx->ISR & USART_ISR_RXNE));
	byte = USARTx->RDR;
	return byte;
}

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate)
{
	USARTx->BRR = compute_uart_bd(PeriphClk,BaudRate);
//	USARTx->BRR = (uint16_t)69;
}

static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate)
{
	uint16_t Div =  (PeriphClk + (BaudRate/2U));
	Div /= BaudRate;
	return Div;
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
