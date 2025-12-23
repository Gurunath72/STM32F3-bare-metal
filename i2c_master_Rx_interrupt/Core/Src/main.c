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
#include <stdbool.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE BEGIN PM */
#define BUFFER_SIZE 		 50
#define I2C2_SLAVE_ADDR		(0x06)
#define I2C_TIMINGR 		((1U << I2C_TIMINGR_PRESC_Pos) | (0x0FU << I2C_TIMINGR_SCLH_Pos) | (0x13U << I2C_TIMINGR_SCLL_Pos) | (0x02 << I2C_TIMINGR_SDADEL_Pos) | (0x04 << I2C_TIMINGR_SCLDEL_Pos))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint8_t i2c_master_rx_buffer[BUFFER_SIZE];		// to receive the master data
volatile uint8_t mem_offset = 0;					// address offset requested by master
static volatile uint8_t i2c_rx_buf[256];
static volatile uint16_t i2c_rx_len = 0;
static volatile uint8_t i2c_rx_done =0;
static volatile uint8_t i2c_rx_cnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void GPIO_Init();
void I2C_Master_Init(void);
int i2c_master_write(uint8_t addr, const uint8_t *data, uint16_t len);
int i2c_master_read(uint8_t addr, uint8_t *data, uint16_t len);
void i2c1_master_receive_it(uint8_t addr7, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* -------------------------
   I2C1 event IRQ handler (handles RXNE etc.)
   -------------------------*/
void I2C1_EV_IRQHandler(void)
{
    uint32_t isr = I2C1->ISR;

    /* RXNE: data received in RXDR */
    if (isr & I2C_ISR_RXNE)
    {
        /* read a byte */
        uint8_t b = (uint8_t)I2C1->RXDR;
        i2c_rx_buf[i2c_rx_cnt++] = b;

        /* If we have received all bytes -> done */
        if (i2c_rx_cnt >= i2c_rx_len)
        {
            i2c_rx_done = true;
            /* ensure STOP has been generated for multi-byte case (we generate STOP when 2nd last byte read)
               but if not, try to set STOP now */
            I2C1->CR1 |= I2C_CR2_STOP;
        }

        /* Manage ACK/STOP for when 2 bytes remain: when i2c_rx_cnt == i2c_rx_len - 2
           we must clear ACK so that N-1 byte is not acked, and generate STOP when the penultimate.
           Sequence: byte is received
             - when two bytes remaining, clear ACK
             - when next byte (penultimate) is received, generate STOP (so that last byte will be clocked out by slave).
        */
//        if (i2c_rx_cnt == (i2c_rx_len - 1))
//        {
//            /* Received penultimate byte; generate STOP so last byte will be sent by slave and then RXNE will trigger for last byte */
//            I2C1->CR1 |= I2C_CR1_STOP;
//        }
    }
    if(isr & I2C_ISR_TC)
    {
    	I2C1->CR1 |= I2C_CR2_STOP;
    }
}

/* -------------------------
   I2C1 error IRQ handler
   -------------------------*/
void I2C1_ER_IRQHandler(void)
{
    uint32_t isr = I2C1->ISR;

    /* NACKF */
    if (isr & I2C_ISR_NACKF)
    {
        /* clear NACK flag */
        I2C1->ICR |= I2C_ICR_NACKCF;
        /* generate STOP to free bus */
        I2C1->CR1 |= I2C_CR2_STOP;
        i2c_rx_done = true;
    }

    /* ARLO (arbitration lost) */
    if (isr & I2C_ISR_ARLO)
    {
        I2C1->ICR |= I2C_ICR_ARLOCF;
        i2c_rx_done = true;
    }

    /* BERR (bus error) */
    if (isr & I2C_ISR_BERR)
    {
        I2C1->ICR |= I2C_ICR_BERRCF;
        i2c_rx_done = true;
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
	volatile uint8_t txbuffer[50] = {'h','e','l','l','o','W','o','r','l','d'};

  /* USER CODE END 1 */
	GPIO_Init();
	I2C_Master_Init();

	i2c_master_write(I2C2_SLAVE_ADDR, txbuffer,(uint16_t)10);
//	i2c_master_read(I2C2_SLAVE_ADDR, (uint16_t)10);
	i2c_master_read(I2C2_SLAVE_ADDR, i2c_rx_buf, (uint16_t)10);
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
	  if(i2c_rx_done)
	  {
		  i2c_rx_done = false;
	  }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void GPIO_Init()
{
	/* enable clocks for GPIO A */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	// set GPIOB_PIN8 as AF - I2C1 SCL
	GPIOB->MODER &= ~(GPIO_MODER_MODER8);
	GPIOB->MODER |= (0x02U << GPIO_MODER_MODER8_Pos);

	// set GPIOB_PIN9 as AF - I2C1 SDA
	GPIOB->MODER &= ~(GPIO_MODER_MODER9);
	GPIOB->MODER |= (0x02U << GPIO_MODER_MODER9_Pos);


	/* set A.F type for pin PB8 and PB9 */
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFRH0);
	GPIOB->AFR[1] |= ((uint32_t)0x04 << GPIO_AFRH_AFRH0_Pos);

	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFRH1);
	GPIOB->AFR[1] |= ((uint32_t)0x04 << GPIO_AFRH_AFRH1_Pos);


	/* Output type: open-drain */
	GPIOB->OTYPER |= ((GPIO_PIN_8) | (GPIO_PIN_9));


	/* ENABLE pull up for SCL and SDA pins in master port*/
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR8);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR9);
	GPIOB->PUPDR |= ((uint32_t)0x01 << GPIO_PUPDR_PUPDR8_Pos);
	GPIOB->PUPDR |= ((uint32_t)0x01 << GPIO_PUPDR_PUPDR9_Pos);

	/* Speed: high (set OSPEEDR to 10) */
	GPIOB->OSPEEDR &= ~( (GPIO_OSPEEDER_OSPEEDR8) | (GPIO_OSPEEDER_OSPEEDR9));
	GPIOB->OSPEEDR |=  (0x01U << GPIO_OSPEEDER_OSPEEDR8_Pos) | (0x01U << GPIO_OSPEEDER_OSPEEDR9_Pos);
}

/* Initialize I2C1 peripheral as master (basic configuration) */
void I2C_Master_Init(void)
{
	uint32_t i=0;
    /* Enable I2C1 clock on APB1 */
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    I2C1->CR1 &= ~I2C_CR1_PE;
    /* Reset I2C to ensure known state */
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    for(i=0; i<1000; i++);
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    /* Configure timing */
    I2C1->TIMINGR = I2C_TIMINGR;

    /* Enable I2C interrupts for event and error for receive handling */
//    I2C1->CR2 |= I2C_CR1_RXIE | I2C_CR1_TCIE | I2C_CR1_ERRIE;
//    I2C1->CR2 |= I2C_CR1_RXIE | I2C_CR1_TCIE;

    /* Enable peripheral (PE = 1) */
    I2C1->CR1 |= I2C_CR1_PE;
    /* NVIC: enable IRQs */
//	NVIC_EnableIRQ(I2C1_EV_IRQn);
//	NVIC_SetPriority(I2C1_EV_IRQn, 2);
//	NVIC_EnableIRQ(I2C1_ER_IRQn);
//	NVIC_SetPriority(I2C1_ER_IRQn, 1);
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
    I2C2->ICR = 0xFFFFFFFFUL;
    I2C2->CR2 = cr2 | I2C_CR2_START;

    for(uint16_t i = 0; i < len; ++i)
    {
        /* Wait until RXNE (receive not empty) */
        uint32_t to = 100000;
        while (!(I2C2->ISR & I2C_ISR_RXNE))
        {
            if(I2C2->ISR & I2C_ISR_NACKF)
            {
            	I2C2->ICR = I2C_ICR_NACKCF;
            	I2C2->CR2 |= I2C_CR2_STOP;
            	return -2;
            }
//            if(--to == 0)
//            {
//            	I2C2->CR2 |= I2C_CR2_STOP;
//            	return -3;
//            }
        }

        /* Read from RXDR */
        data[i] = (uint8_t)I2C2->RXDR;
    }

    /* Wait for TC */
    {
        uint32_t to = 100000;
        while (!(I2C2->ISR & I2C_ISR_TC))
        {
            if (I2C2->ISR & I2C_ISR_NACKF)
            {
            	I2C2->ICR = I2C_ICR_NACKCF;
            	I2C2->CR2 |= I2C_CR2_STOP;
            	return -4;
            }
//            if (--to == 0)
//            {
//            	I2C2->CR2 |= I2C_CR2_STOP;
//            	return -5;
//            }
        }
    }

    /* STOP */
    I2C2->CR2 |= I2C_CR2_STOP;
    {
        uint32_t to = 100000;
        while (!(I2C2->ISR & I2C_ISR_STOPF))
        {
        	;
//            if (--to == 0) return -6;
        }
    }
    I2C2->ICR = I2C_ICR_STOPCF;

    return 0;
}

void i2c1_master_receive_it(uint8_t addr7, uint16_t len)
{
	/* Prepare CR2 for read (set RD_WRN = 1) */
	uint32_t cr2 = 0;
	cr2 |= ((uint32_t)addr7 << 1) & I2C_CR2_SADD;
	cr2 |= I2C_CR2_RD_WRN;                       /* read direction */
	cr2 |= ((uint32_t)len << 16) & I2C_CR2_NBYTES;
	I2C2->ICR = 0xFFFFFFFFUL;


    i2c_rx_len = len;
    i2c_rx_cnt = 0;
    i2c_rx_done = false;
    I2C2->CR2 = cr2 | I2C_CR2_START;
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
