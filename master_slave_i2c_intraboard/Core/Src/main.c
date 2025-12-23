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
#define BUFFER_SIZE 		 50
#define I2C2_SLAVE_ADDR		(0x06)
#define I2C_TIMINGR 		((1U << I2C_TIMINGR_PRESC_Pos) | (0x0FU << I2C_TIMINGR_SCLH_Pos) | (0x13U << I2C_TIMINGR_SCLL_Pos) | (0x02 << I2C_TIMINGR_SDADEL_Pos) | (0x04 << I2C_TIMINGR_SCLDEL_Pos))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
volatile uint8_t i2c_slave_buffer[BUFFER_SIZE];		// to receive the master data
volatile uint8_t mem_offset = 0;					// address offset requested by master
volatile uint8_t data_addr_received = 0;			// flag to identify address offset received from master

volatile uint8_t i2c_master_buffer[BUFFER_SIZE];

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void GPIO_Init();
void I2C_Master_Init(void);
void I2C_Slave_Init(void);
void DMA1_Init(uint32_t source, uint32_t destination, uint16_t len);
int i2c_master_write(uint8_t addr, const uint8_t *data, uint16_t len);
int i2c_master_read(uint8_t addr, uint8_t *data, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ---------------- Slave Interrupt Handler ---------------- */
void I2C2_EV_IRQHandler(void)
{
    uint32_t isr = I2C2->ISR;

    if(isr & I2C_ISR_ADDR)
    {
        I2C2->ICR = I2C_ICR_ADDRCF;
        if(!(isr & I2C_ISR_DIR))
        	data_addr_received = 0;
        else
        {
        	 I2C2->TXDR = i2c_slave_buffer[mem_offset++];
        }
    }

    if (isr & I2C_ISR_RXNE)
    {
        uint8_t val = (uint8_t)I2C2->RXDR;

        if (!data_addr_received)
        {
            mem_offset = val % BUFFER_SIZE;
            data_addr_received = 1;
        }
        else
        {
            i2c_slave_buffer[mem_offset++] = val;
            if (mem_offset >= BUFFER_SIZE)
                mem_offset = 0;
        }
    }

    if(isr & I2C_ISR_TXIS)
    {
        I2C2->TXDR = i2c_slave_buffer[mem_offset++];
        if (mem_offset >= BUFFER_SIZE)
            mem_offset = 0;
    }

    if (isr & I2C_ISR_STOPF)
    {
        I2C2->ICR = I2C_ICR_STOPCF;
    }
}

volatile uint8_t rxbuffer[50]= {0};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	volatile uint8_t txbuffer[50] = {0x00, 'h','e','l','l','o'};
	volatile uint8_t txBuffer2[50] = {0x05,'w','o','r','l','d'};

    /* USER CODE BEGIN 1 */
	GPIO_Init();
	I2C_Master_Init();
	I2C_Slave_Init();

	i2c_master_write(I2C2_SLAVE_ADDR, txbuffer, 6);
	i2c_master_write(I2C2_SLAVE_ADDR, txBuffer2, 6);
	i2c_master_write(I2C2_SLAVE_ADDR, txbuffer, 1);
	i2c_master_read(I2C2_SLAVE_ADDR, rxbuffer, 5);
	i2c_master_write(I2C2_SLAVE_ADDR, txBuffer2, 1);
	i2c_master_read(I2C2_SLAVE_ADDR, rxbuffer, 5);
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

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
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
void GPIO_Init()
{
	/* enable clocks for GPIO A AND B*/
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
//	RCC->AHBENR |= RCC_AHBENR_GPIOFEN;

	// set GPIOB_PIN8 as AF - I2C1 SCL
	GPIOB->MODER &= ~(GPIO_MODER_MODER8);
	GPIOB->MODER |= (0x02U << GPIO_MODER_MODER8_Pos);

	// set GPIOB_PIN9 as AF - I2C1 SDA
	GPIOB->MODER &= ~(GPIO_MODER_MODER9);
	GPIOB->MODER |= (0x02U << GPIO_MODER_MODER9_Pos);

//	// set GPIOF_PIN1 as AF - I2C2 SCL
//	GPIOF->MODER &= ~(GPIO_MODER_MODER1);
//	GPIOF->MODER |= (0x02U << GPIO_MODER_MODER1_Pos);
//
//	// set GPIOF_PIN0 as AF - I2C2 SDA
//	GPIOF->MODER &= ~(GPIO_MODER_MODER0);
//	GPIOF->MODER |= (0x02U << GPIO_MODER_MODER0_Pos);

	// set GPIOA_PIN9 as AF - I2C2 SCL
	GPIOA->MODER &= ~(GPIO_MODER_MODER9);
	GPIOA->MODER |= (0x02U << GPIO_MODER_MODER9_Pos);

	// set GPIOA_PIN10 as AF - I2C2 SDA
	GPIOA->MODER &= ~(GPIO_MODER_MODER10);
	GPIOA->MODER |= (0x02U << GPIO_MODER_MODER10_Pos);

	/* set A.F type for pin PB8 and PB9 */
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFRH0);
	GPIOB->AFR[1] |= ((uint32_t)0x04 << GPIO_AFRH_AFRH0_Pos);

	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFRH1);
	GPIOB->AFR[1] |= ((uint32_t)0x04 << GPIO_AFRH_AFRH1_Pos);

//	/* set A.F type for pin PF0 and PF1 as I2C2 SDA AND SCL */
//	GPIOF->AFR[0] &= ~(GPIO_AFRL_AFRL1);
//	GPIOF->AFR[0] |= ((uint32_t)0x04 << GPIO_AFRL_AFRL0_Pos);
//
//	GPIOF->AFR[0] &= ~(GPIO_AFRL_AFRL0);
//	GPIOF->AFR[0] |= ((uint32_t)0x04 << GPIO_AFRL_AFRL0_Pos);

	/* set A.F type for pin PA9 and PA10 as I2C2 SDA AND SCL */
	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFRH1);
	GPIOA->AFR[1] |= ((uint32_t)0x04 << GPIO_AFRH_AFRH1_Pos);

	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFRH2);
	GPIOA->AFR[1] |= ((uint32_t)0x04 << GPIO_AFRH_AFRH2_Pos);

	/* Output type: open-drain */
	GPIOB->OTYPER |= ((GPIO_PIN_8) | (GPIO_PIN_9));

	GPIOA->OTYPER |= ((GPIO_PIN_9) | (GPIO_PIN_10));

	/* ENABLE pull up for SCL and SDA pins in master port*/
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR8);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR9);
	GPIOB->PUPDR |= ((uint32_t)0x01 << GPIO_PUPDR_PUPDR8_Pos);
	GPIOB->PUPDR |= ((uint32_t)0x01 << GPIO_PUPDR_PUPDR9_Pos);

	/* Speed: high (set OSPEEDR to 10) */
	GPIOB->OSPEEDR &= ~( (GPIO_OSPEEDER_OSPEEDR8) | (GPIO_OSPEEDER_OSPEEDR9));
	GPIOB->OSPEEDR |=  (0x01U << GPIO_OSPEEDER_OSPEEDR8_Pos) | (0x01U << GPIO_OSPEEDER_OSPEEDR9_Pos);

	/* Speed: high (set OSPEEDR to 10) */
	GPIOA->OSPEEDR &= ~( (GPIO_OSPEEDER_OSPEEDR9) | (GPIO_OSPEEDER_OSPEEDR10));
	GPIOA->OSPEEDR |=  (0x01U << GPIO_OSPEEDER_OSPEEDR9_Pos) | (0x01U << GPIO_OSPEEDER_OSPEEDR10_Pos);

	/* set Tx pin gpio output type as push pull*/
//	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_2);	// push pull mode for GPIOA pin2
}

/* Initialize I2C1 peripheral as master (basic configuration) */
void I2C_Master_Init(void)
{
	uint32_t i=0;
    /* Enable I2C1 clock on APB1 */
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    I2C1->CR1 &= ~I2C_CR1_PE;
    /* Reset I2C to ensure known state */


    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    for(i=0; i<1000; i++);
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;

    /* Configure timing */
    I2C1->TIMINGR = I2C_TIMINGR;

    /* Enable peripheral (PE = 1) */
    I2C1->CR1 |= I2C_CR1_PE;
}

void I2C_Slave_Init(void)
{
	uint32_t i=0;

	/* Enable I2C2 clock on APB1 */
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	I2C2->CR1 &= ~I2C_CR1_PE;
	/* Reset I2C to ensure known state */
	RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST;
	for(i=0; i<1000; i++);
	RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;

	I2C2->TIMINGR = I2C_TIMINGR;	// configure timing register
	I2C2->OAR1 = ((uint32_t)I2C2_SLAVE_ADDR << 1) | I2C_OAR1_OA1EN;	// Set slave address
	I2C2->CR1 = I2C_CR1_PE | I2C_CR1_ADDRIE | I2C_CR1_RXIE | I2C_CR1_TXIE ;	// enable interrupt for TXIS, RXNE, and ADDRF(address detected)
	NVIC_EnableIRQ(I2C2_EV_IRQn);	// enable interrupt on NVIC
}

void DMA1_Init(uint32_t source, uint32_t destination, uint16_t len)
{
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    DMA1_Channel6->CCR = 0;   // Disable first

//    DMA1_Channel6->CPAR  = (uint32_t)&I2C1->TXDR; // Peripheral register
    DMA1_Channel6->CPAR = destination;
    DMA1_Channel6->CMAR  = source;         // Memory address
    DMA1_Channel6->CNDTR = len;            // Number of bytes

    DMA1_Channel6->CCR =
            DMA_CCR_MINC       |   // Memory increment
            DMA_CCR_DIR        |   // Memory → Peripheral
            DMA_CCR_PL_1;          // Priority high

    // Enable DMA channel
    DMA1_Channel6->CCR |= DMA_CCR_EN;
}

/* Master transmit: blocking write
 * addr: 7-bit slave address
 * data: pointer to data buffer
 * len: number of bytes to be transmitted
 * returns 0 on success, non-zero on error (NACK or timeout)
 */
int i2c_master_write(uint8_t addr, const uint8_t *data, uint16_t len)
{
    /* Ensure peripheral is enabled */
    if (!(I2C1->CR1 & I2C_CR1_PE)) return -1;

    /* Compose CR2: set slave address (7-bit left aligned to bits [7:1]), write direction (RD_WRN=0),
       NBYTES, and START. We don't set AUTOEND so we will generate STOP manually.
    */
    uint32_t cr2 = 0;
    cr2 |= ((uint32_t)addr << 1) & I2C_CR2_SADD;   /* SADD in bits [7:1] */
    cr2 |= ((uint32_t)len << 16) & I2C_CR2_NBYTES; /* NBYTES */
    /* Clear previous flags  */
    I2C1->ICR = 0xFFFFFFFFUL;
    while(I2C1->ISR & I2C_ISR_BUSY);
    I2C1->CR2 = cr2 | I2C_CR2_START;

    for (uint16_t i = 0; i < len; ++i)
    {
        /* Wait until TXIS (transmit interrupt status) = 1 , ACK for previous byte , (TXDR can be written) */
        uint32_t to = 100000;	// timeout for indefinite ACK or NACK flag not set
        while (!(I2C1->ISR & I2C_ISR_TXIS))
        {
            if (I2C1->ISR & I2C_ISR_NACKF)
            {
            	I2C1->ICR = I2C_ICR_NACKCF;
            	I2C1->CR2 |= I2C_CR2_STOP;
            	return -2;
            }
//            if (--to == 0)
//            {
//            	I2C1->CR2 |= I2C_CR2_STOP;
//            	return -3;
//            }
        }

        /* Write data to TXDR */
        I2C1->TXDR = data[i];
    }

    /* Wait for Transfer Complete (TC) flag */
    {
        uint32_t to = 100000;
        while (!(I2C1->ISR & I2C_ISR_TC))
        {
            if (I2C1->ISR & I2C_ISR_NACKF)
            {
            	I2C1->ICR = I2C_ICR_NACKCF;
            	I2C1->CR2 |= I2C_CR2_STOP;
            	return -4;
            }
//            if (--to == 0)
//            {
//            	I2C1->CR2 |= I2C_CR2_STOP;
//            	return -5;
//            }
        }
    }

    /* Generate STOP condition */
    I2C1->CR2 |= I2C_CR2_STOP;

    /* Wait until STOPF is set (stop detected) */
    {
        uint32_t to = 100000;
        while (!(I2C1->ISR & I2C_ISR_STOPF))
        {
//            if (--to == 0) return -6;
        }
    }

    /* Clear STOPF by writing to ICR */
    I2C1->ICR = I2C_ICR_STOPCF;

    return 0;		// successful transaction on i2c bus
}

/* Master receive: blocking read
 * addr: 7-bit slave address
 * data: buffer to store received bytes
 * len: number of bytes to read
 */
int i2c_master_read(uint8_t addr, uint8_t *data, uint16_t len)
{
    /* Prepare CR2 for read (set RD_WRN = 1) */
    uint32_t cr2 = 0;
    cr2 |= ((uint32_t)addr << 1) & I2C_CR2_SADD;
    cr2 |= I2C_CR2_RD_WRN;                       /* read direction */
    cr2 |= ((uint32_t)len << 16) & I2C_CR2_NBYTES;
    I2C1->ICR = 0xFFFFFFFFUL;
    while(I2C1->ISR & I2C_ISR_BUSY);
    I2C1->CR2 = cr2 | I2C_CR2_START;

    for(uint16_t i = 0; i < len; ++i)
    {
        /* Wait until RXNE (receive not empty) */
        uint32_t to = 100000;
        while (!(I2C1->ISR & I2C_ISR_RXNE))
        {
            if(I2C1->ISR & I2C_ISR_NACKF)
            {
            	I2C1->ICR = I2C_ICR_NACKCF;
            	I2C1->CR2 |= I2C_CR2_STOP;
            	return -2;
            }
//            if(--to == 0)
//            {
//            	I2C2->CR2 |= I2C_CR2_STOP;
//            	return -3;
//            }
        }

        /* Read from RXDR */
        data[i] = (uint8_t)I2C1->RXDR;
    }

//    /* Wait for TC */
//    {
//        uint32_t to = 100000;
        while (!(I2C1->ISR & I2C_ISR_TC))
        {
            if (I2C1->ISR & I2C_ISR_NACKF)
            {
            	I2C1->ICR = I2C_ICR_NACKCF;
            	I2C1->CR2 |= I2C_CR2_STOP;
            	return -4;
            }
//            if (--to == 0)
//            {
//            	I2C2->CR2 |= I2C_CR2_STOP;
//            	return -5;
//            }
        }
//    }

    /* STOP */
    I2C1->CR2 |= I2C_CR2_STOP;
    {
        uint32_t to = 100000;
        while (!(I2C1->ISR & I2C_ISR_STOPF))
        {
        	;
//            if (--to == 0) return -6;
        }
    }
    I2C1->ICR = I2C_ICR_STOPCF;

    return 0;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}
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
