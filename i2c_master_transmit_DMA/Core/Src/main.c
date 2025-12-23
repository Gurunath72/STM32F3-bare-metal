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
#define DMA_TIMEOUT         1000000U
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//volatile uint8_t i2c_slave_buffer[BUFFER_SIZE];		// to receive the master data
//volatile uint8_t mem_offset = 0;					// address offset requested by master
//volatile uint8_t data_addr_received = 0;			// flag to identify address offset received from master
//
//volatile uint8_t i2c_master_buffer[BUFFER_SIZE];
//volatile uint8_t rxbuffer[50]= {0};
/* USER CODE END PV */
/* master tx and rx buffers */
uint8_t master_tx[] = { 1, 2, 3, 4, 5 };
uint8_t master_rx[5];

/* slave tx and rx buffers */
uint8_t slave_rx[5];
uint8_t slave_tx[] = { 10, 20, 30, 40, 50 };
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE BEGIN PFP */
void GPIO_Init();
void I2C_Master_Init(void);
void I2C_Slave_Init(void);
void DMA1_I2C1_TX_Init(uint32_t source, uint32_t destination, uint16_t len);
void DMA1_I2C1_RX_Init(uint32_t  periphAddr, uint32_t memAddr , uint16_t len);
void I2C1_Master_TX_DMA(uint16_t len);
int I2C1_Master_RX_DMA(uint8_t slave_addr, uint16_t len);
void I2C2_Slave_TX_DMA( );
void I2C2_Slave_RX_DMA( );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ---------------- Slave Interrupt Handler (for eeprom style access for slave buffer---------------- */
//void I2C2_EV_IRQHandler(void)
//{
//    uint32_t isr = I2C2->ISR;
//
//    if(isr & I2C_ISR_ADDR)
//    {
//        I2C2->ICR = I2C_ICR_ADDRCF;
//        if(!(isr & I2C_ISR_DIR))
//        	data_addr_received = 0;
//        else
//        	I2C2->TXDR = i2c_slave_buffer[mem_offset++];
////        	while(!(isr & I2C_ISR_TXE));
//    }
//
//    if (isr & I2C_ISR_RXNE)
//    {
//        uint8_t val = (uint8_t)I2C2->RXDR;
//
//        if (!data_addr_received)
//        {
//            mem_offset = val % BUFFER_SIZE;
//            data_addr_received = 1;
//        }
//        else
//        {
//            i2c_slave_buffer[mem_offset++] = val;
//            if (mem_offset >= BUFFER_SIZE)
//                mem_offset = 0;
//        }
//    }
//
//    if(isr & I2C_ISR_TXIS)
//    {
//        I2C2->TXDR = i2c_slave_buffer[mem_offset++];
////        while(!(isr & I2C_ISR_TXE));
//        if (mem_offset >= BUFFER_SIZE)
//            mem_offset = 0;
//    }
//
//    if (isr & I2C_ISR_STOPF)
//    {
//        I2C2->ICR = I2C_ICR_STOPCF;
//    }
//
//    /* handle/clear errors (NACKF, BERR, OVR, etc) */
//   if (isr & I2C_ISR_NACKF)
//   {
//	   I2C2->ICR = I2C_ICR_NACKCF;
//   }
//
//   if (isr & I2C_ISR_BERR)
//   {
//	   I2C2->ICR = I2C_ICR_BERRCF;
//   }
//}


void I2C2_EV_IRQHandler(void)
{
	uint32_t isr = I2C2->ISR;

	if(isr & I2C_ISR_ADDR)
	{
		 I2C2->ICR |= I2C_ICR_ADDRCF;
		 if(!(isr & I2C_ISR_DIR))
		 {
			 DMA1_Channel5->CCR |= DMA_CCR_EN;
			 DMA1_Channel6->CCR |= DMA_CCR_EN;
		 }
		 else
		 {
			 DMA1_Channel7->CCR |= DMA_CCR_EN;
			 DMA1_Channel4->CCR |= DMA_CCR_EN;
		 }
	}

	if(isr & I2C_ISR_STOPF)
	{
		I2C2->ICR |= I2C_ICR_STOPCF;
	}

	/* handle/clear errors (NACKF, BERR, OVR, etc) */
   if (isr & I2C_ISR_NACKF)
   {
	   I2C2->ICR |= I2C_ICR_NACKCF;
   }
}

void DMA1_Channel4_IRQHandler(void)
{
	uint32_t isr = DMA1->ISR;
	if(isr & DMA_ISR_TCIF4)
	{
		DMA1->IFCR |= DMA_IFCR_CTCIF4;
	}
}


void DMA1_Channel5_IRQHandler(void)
{
	uint32_t isr = DMA1->ISR;
	if(isr & DMA_ISR_TCIF5)
	{
		DMA1->IFCR |= DMA_IFCR_CTCIF5;
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
	 /* USER CODE BEGIN 1 */
	volatile uint8_t txbuffer[50] = {0x00, 'h','e','l','l','o'};
	volatile uint8_t txBuffer2[50] = {0x05,'w','o','r','l','d'};

	uint8_t i=0;
	GPIO_Init();
	I2C_Master_Init();
	I2C_Slave_Init();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//	HAL_Init();

  /* USER CODE BEGIN Init */
	/* eeprom style access from slave buffer */
	/*---------------------------- write offset and read data ------------------------------------*/
	/* transmit data "hello" with offset 0x00 (sensor or eeprom style access)*/
//	DMA1_I2C1_TX_Init( (uint32_t)&I2C1->TXDR, (uint32_t)txbuffer,(uint16_t)6);	//reconfigure DMA for i2c1_tx
//	I2C1_Master_TX_DMA((uint16_t)6);											// DMA transfer of data from memory to i2c_TXDR

	/* transmit data "world" with offset 0x05 (sensor or eeprom style access */
//	DMA1_I2C1_TX_Init( (uint32_t)&I2C1->TXDR, (uint32_t)txBuffer2,(uint16_t)6);	//reconfigure DMA for i2c1_tx
//	I2C1_Master_TX_DMA((uint16_t)6);											// DMA transfer of data from memory to i2c_TXDR

//	/* set offset as 0x00 and read 5 bytes */
//	DMA1_I2C1_TX_Init( (uint32_t)&I2C1->TXDR, (uint32_t)txbuffer,(uint16_t)1);	//reconfigure DMA for i2c1_tx
//	I2C1_Master_TX_DMA((uint16_t)1);											// DMA transfer of data from memory to i2c_TXDR
//	DMA1_I2C1_RX_Init( (uint32_t)&I2C1->RXDR, (uint32_t)rxbuffer,(uint16_t)10);	// configure DMA for i21_rx
//	I2C1_Master_RX_DMA((uint8_t)I2C2_SLAVE_ADDR, 10);														// DMA transfer of data from RXDR to memory
//
//	for(i=0;i<1000;i++);
//	DMA1_I2C1_TX_Init( (uint32_t)&I2C1->TXDR, (uint32_t)txbuffer,(uint16_t)1);	//reconfigure DMA for i2c1_tx
//	I2C1_Master_TX_DMA((uint16_t)1);											// DMA transfer of data from memory to i2c_TXDR
//	DMA1_I2C1_RX_Init( (uint32_t)&I2C1->RXDR, (uint32_t)rxbuffer,(uint16_t)10);	// configure DMA for i21_rx
//	I2C1_Master_RX_DMA((uint8_t)I2C2_SLAVE_ADDR, 10);
//	/* set offset as 0x05 and read 5 bytes */
//	DMA1_I2C1_TX_Init( (uint32_t)&I2C1->TXDR, (uint32_t)txBuffer2,(uint16_t)1);
//	I2C1_Master_TX_DMA((uint16_t)1);
//	DMA1_I2C1_RX_Init((uint32_t)&I2C1->RXDR, (uint32_t)rxbuffer,(uint16_t)5);
//	I2C1_Master_RX_DMA((uint8_t)I2C2_SLAVE_ADDR, 5);

	/* data exchange of 5 bytes b/w master i2c1 DMA mode and slave I2C2  DMA mode */
	DMA1_I2C1_TX_Init((uint32_t)&I2C1->TXDR, (uint32_t)master_tx, (uint16_t)5);
	DMA1_I2C1_RX_Init((uint32_t)&I2C1->RXDR, (uint32_t)master_rx, (uint16_t)5);
	DMA1_I2C2_TX_Init((uint32_t)&I2C2->TXDR, (uint32_t)slave_tx, (uint16_t)5);
	DMA1_I2C2_RX_Init((uint32_t)&I2C2->RXDR, (uint32_t) slave_rx, (uint16_t)5);

	//start master transmit
	I2C1_Master_TX_DMA((uint16_t)5);

	I2C1_Master_RX_DMA((uint8_t)I2C2_SLAVE_ADDR,(uint16_t)5);

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
//	  DMA1_I2C1_TX_Init((uint32_t)&I2C1->TXDR, (uint32_t)master_tx, (uint16_t)5);
//	  DMA1_I2C2_RX_Init((uint32_t)&I2C2->RXDR, (uint32_t) slave_rx, (uint16_t)5);
//
//	  I2C1_Master_TX_DMA((uint16_t)5);
//
//	  for(uint8_t i=0; i<1)
  }
  /* USER CODE END 3 */
}


void GPIO_Init()
{
	/* enable clocks for GPIO A AND B*/
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// set GPIOB_PIN8 as AF - I2C1 SCL
	GPIOB->MODER &= ~(GPIO_MODER_MODER8);
	GPIOB->MODER |= (0x02U << GPIO_MODER_MODER8_Pos);
	// set GPIOB_PIN9 as AF - I2C1 SDA
	GPIOB->MODER &= ~(GPIO_MODER_MODER9);
	GPIOB->MODER |= (0x02U << GPIO_MODER_MODER9_Pos); // // set GPIOF_PIN1 as AF - I2C2 SCL
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
	GPIOB->AFR[1] |= ((uint32_t)0x04 << GPIO_AFRH_AFRH1_Pos); //

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
	GPIOB->OSPEEDR |= (0x01U << GPIO_OSPEEDER_OSPEEDR8_Pos) | (0x01U << GPIO_OSPEEDER_OSPEEDR9_Pos);
	/* Speed: high (set OSPEEDR to 10) */
	GPIOA->OSPEEDR &= ~( (GPIO_OSPEEDER_OSPEEDR9) | (GPIO_OSPEEDER_OSPEEDR10));
	GPIOA->OSPEEDR |= (0x01U << GPIO_OSPEEDER_OSPEEDR9_Pos) | (0x01U << GPIO_OSPEEDER_OSPEEDR10_Pos);
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
	for(i=0; i<10; i++);
	RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
	/* Configure timing */
	I2C1->TIMINGR = I2C_TIMINGR;
	// Enable DMA request for TX
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	I2C1->CR1 |= I2C_CR1_TXDMAEN;
	I2C1->CR1 |= I2C_CR1_RXDMAEN;

	/* Enable peripheral (PE = 1) */
	I2C1->CR1 |= I2C_CR1_PE  ;
}


void I2C_Slave_Init(void)
{
	uint32_t i=0;
	/* Enable I2C2 clock on APB1 */
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	I2C2->CR1 &= ~I2C_CR1_PE;
	/* Reset I2C to ensure known state */
	RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST;
	for(i=0; i<10; i++);
	RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;
	I2C2->TIMINGR = I2C_TIMINGR;
	// configure timing register
	I2C2->OAR1 = ((uint32_t)I2C2_SLAVE_ADDR << 1) | I2C_OAR1_OA1EN;
	// Set slave address
	I2C2->CR1 = I2C_CR1_PE | I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_ADDRIE; // enable interrupt for TXIS, RXNE, and ADDRF(address detected)
	// Enable DMA request for TX
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	I2C2->CR1 |= I2C_CR1_TXDMAEN;
	I2C2->CR1 |= I2C_CR1_RXDMAEN;
	NVIC_SetPriority(I2C2_EV_IRQn, 2);
	NVIC_EnableIRQ(I2C2_EV_IRQn); // enable interrupt on NVIC

	NVIC_EnableIRQ(DMA1_Channel4_IRQn);          /* !< DMA1 Channel 4 Interrupt */
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);			/* DMA1 channel 5 interrupt enable*/
}

void DMA1_I2C1_TX_Init(uint32_t periphAddr, uint32_t memAddr, uint16_t len)
{
	DMA1_Channel6->CCR = 0; // Disable first
	// DMA1_Channel6->CPAR = (uint32_t)&I2C1->TXDR; // Peripheral register
	DMA1_Channel6->CPAR = periphAddr;
	DMA1_Channel6->CMAR = memAddr; // Memory address
	DMA1_Channel6->CNDTR = len; // Number of bytes
	DMA1_Channel6->CCR = DMA_CCR_MINC | // Memory increment
						 DMA_CCR_DIR | // Memory → Peripheral
						 DMA_CCR_PL_1; // Priority high
	// Enable DMA channel
//	DMA1_Channel6->CCR |= DMA_CCR_EN;
}

void DMA1_I2C1_RX_Init(uint32_t periphAddr, uint32_t memAddr , uint16_t len)
{
	/* Ensure channel disabled while configuring */
	DMA1_Channel7->CCR = 0;
	/* Peripheral address = I2C1->RXDR */
	DMA1_Channel7->CPAR = periphAddr;
	/* Memory address */
	DMA1_Channel7->CMAR = memAddr;
	/* Number of data to transfer */
	DMA1_Channel7->CNDTR = len;
	/* Configure CCR: - MEM increment (MINC) - PERIPHERAL->MEMORY (DIR = 0)- Priority: high (PL bits) - Peripheral & memory data size 8-bit default (PSIZE=MSIZE=00) - No circular, no MSIZE 16/32 */
	DMA1_Channel7->CCR = DMA_CCR_MINC | (DMA_CCR_PL_0); /* PL_0/PL_1 depending on header; here setting PL=01 (medium) */
	/* Clear any pending flags for channel7 */
	DMA1->IFCR =  DMA_IFCR_CTCIF7 | DMA_IFCR_CTEIF7 | DMA_IFCR_CHTIF7 ;
//	/* 2) Enable DMA channel */
//	DMA1_Channel7->CCR |= DMA_CCR_EN;
}

void DMA1_I2C2_TX_Init(uint32_t periphAddr, uint32_t memAddr, uint16_t len)
{
	DMA1_Channel4->CCR = 0; // Disable first
	// DMA1_Channel6->CPAR = (uint32_t)&I2C1->TXDR; // Peripheral register
	DMA1_Channel4->CPAR = periphAddr;
	DMA1_Channel4->CMAR = memAddr; // Memory address
	DMA1_Channel4->CNDTR = len; // Number of bytes
	DMA1_Channel4->CCR = DMA_CCR_MINC | // Memory increment
						 DMA_CCR_DIR | // Memory → Peripheral
						 DMA_CCR_PL_1 | DMA_CCR_TCIE; // Priority high
	/* Clear any pending flags for channel5 */
	DMA1->IFCR = DMA_IFCR_CGIF7 | DMA_IFCR_CTCIF7 | DMA_IFCR_CTEIF7 | DMA_IFCR_CHTIF7 ;
	// Enable DMA channel
//	DMA1_Channel4->CCR |= DMA_CCR_EN;
}


void DMA1_I2C2_RX_Init(uint32_t periphAddr, uint32_t memAddr , uint16_t len)
{
	/* Ensure channel disabled while configuring */
	DMA1_Channel5->CCR = 0;
	/* Peripheral address = I2C1->RXDR */
	DMA1_Channel5->CPAR = periphAddr;
	/* Memory address */
	DMA1_Channel5->CMAR = memAddr;
	/* Number of data to transfer */
	DMA1_Channel5->CNDTR = len;
	/* Configure CCR: - MEM increment (MINC) - PERIPHERAL->MEMORY (DIR = 0)- Priority: high (PL bits) - Peripheral & memory data size 8-bit default (PSIZE=MSIZE=00) - No circular, no MSIZE 16/32 */
	DMA1_Channel5->CCR = DMA_CCR_MINC | (DMA_CCR_PL_0) | DMA_CCR_TCIE; /* PL_0/PL_1 depending on header; here setting PL=01 (medium) */
	/* Clear any pending flags for channel5 */
	DMA1->IFCR = DMA_IFCR_CGIF7 | DMA_IFCR_CTCIF7 | DMA_IFCR_CTEIF7 | DMA_IFCR_CHTIF7 ;
//	/* 2) Enable DMA channel */
//	DMA1_Channel5->CCR |= DMA_CCR_EN;
}

void I2C1_Master_TX_DMA(uint16_t len)
{
	// Configure I2C1 CR2 for write
	I2C1->CR2 = (I2C2_SLAVE_ADDR << 1) | // Slave address
				((uint32_t)len << 16) |  // Number of bytes NBYTES
				I2C_CR2_AUTOEND; // Automatic STOP condition

	uint32_t stat = I2C1->ISR;
	while(stat & I2C_ISR_BUSY);
	// Start condition
	I2C1->CR2 |= I2C_CR2_START;
//	for(uint8_t i=0; i<10;i++);
//	stat = I2C2->ISR;
	// wait until address ACK by slave
//	while(!(stat & I2C_ISR_ADDR));
//	I2C2->ICR = I2C_ICR_ADDRCF;

	// enable channel
//	DMA1_Channel6->CCR |= DMA_CCR_EN;
	// Wait until DMA finishes transferring memory → TXDR
	while (!(DMA1->ISR & DMA_ISR_TCIF6));
	// Clear DMA transfer complete flag
	DMA1->IFCR |= DMA_IFCR_CTCIF6;
	// Disable DMA channel for next use
	DMA1_Channel6->CCR &= ~DMA_CCR_EN;
	// Wait until STOP is detected
	while (!(I2C1->ISR & I2C_ISR_STOPF));
	I2C1->ICR |= I2C_ICR_STOPCF; // Clear STOP flag
}


int I2C1_Master_RX_DMA(uint8_t slave_addr, uint16_t len)
{
	uint32_t to;
	if(len == 0)
		return -1; ////
	/* 1) Prepare DMA channel (disable, set addresses & length) */
	// DMA1_Channel7->CCR &= ~DMA_CCR_EN; /* disable */
	// DMA1_Channel7->CPAR = (uint32_t)&(I2C1->RXDR);
	// DMA1_Channel7->CMAR = (uint32_t)buf;
	// DMA1_Channel7->CNDTR = len;

	/* 2) Enable DMA channel */
//	 DMA1_Channel7->CCR |= DMA_CCR_EN;
	/* 3) Configure I2C CR2 for reception: - SADD: slave address << 1 - NBYTES: len (bits [23:16]) - RD_WRN: 1 (read) - AUTOEND: 1 to auto generate STOP after NBYTES - START: generate start */
	uint32_t cr2 = 0;
	cr2 |= ((uint32_t)slave_addr << 1) & I2C_CR2_SADD;
	cr2 |= ((uint32_t)len << 16) & I2C_CR2_NBYTES;
	cr2 |= I2C_CR2_RD_WRN; /* Read direction */
	cr2 |= I2C_CR2_AUTOEND; /* Auto STOP at end */

	/* Clear DMA flags for channel7 */
	DMA1->IFCR = DMA_IFCR_CGIF7 | DMA_IFCR_CTCIF7 | DMA_IFCR_CTEIF7 | DMA_IFCR_CHTIF7;
	/* 2) check for i2c bus free or busy */
	uint32_t stat = I2C1->ISR;
	while(stat & I2C_ISR_BUSY);

	// Start condition


	I2C1->CR2 = cr2 | I2C_CR2_START;

//	for(uint8_t i=0; i<10; i++);
//	stat = I2C2->ISR;
//	// wait until address ACK by slave
//	while(!(stat & I2C_ISR_ADDR));
//	I2C2->ICR = I2C_ICR_ADDRCF;

	/*  Wait for DMA transfer complete (TCIF7) */
	while (!(DMA1->ISR & DMA_ISR_TCIF7));

	/* Clear DMA transfer complete flag */
	DMA1->IFCR |= DMA_IFCR_CTCIF7;

	/* Disable DMA channel (safe) */
	DMA1_Channel7->CCR &= ~DMA_CCR_EN;

	/*  Wait for STOPF (end of transfer) and clear it */
	while (!(I2C1->ISR & I2C_ISR_STOPF));
	I2C1->ICR = I2C_ICR_STOPCF;	/* Clear STOP flag */
	return 0;
}


//void I2C2_Slave_TX_DMA()
//{
//	// Enable DMA channel
//	DMA1_Channel4->CCR |= DMA_CCR_EN;
//}
//
//void I2C2_Slave_RX_DMA()
//{
//	/* 2) Enable DMA channel */
//	DMA1_Channel5->CCR |= DMA_CCR_EN;
//}

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

  /** Initializes the CPU, AHB and APB buses clocks*/
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
