/*
 * i2c.c
 *
 *  Created on: Nov 28, 2025
 *      Author: Dell
 */

#include "i2c_master.h"

void GPIO_Init()
{
	/* enable clocks for GPIO A AND B*/
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

///* Initialize I2C1 peripheral as master (basic configuration) */
void I2C_Master_Init(I2C_TypeDef *I2C_BASE)
{
	uint32_t i=0;
    /* Enable I2C1 clock on APB1 */
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    I2C1->CR1 &= ~I2C_CR1_PE;
    /* Reset I2C to ensure known state */

    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
//    for(i=0; i<50; i++);
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    /* Configure timing */
    I2C1->TIMINGR = (uint32_t)I2C_TIMINGR;

    /* Enable peripheral (PE = 1) */
    I2C1->CR1 |= I2C_CR1_PE;
}

/* Master transmit: blocking write
 *  I2C_BASE : i2c port or instance
 * slaveAddr: 7-bit slave address
 * data: pointer to data buffer
 * len: number of bytes to be transmitted
 * returns 0 on success, non-zero on error (NACK or timeout)
 */
i2c_tx_state i2c_master_write(I2C_TypeDef *I2C_BASE, uint8_t slaveAddr, const uint8_t *data, uint16_t len)
{
    /* Ensure peripheral is enabled */
    if (!(I2C_BASE->CR1 & I2C_CR1_PE)) return -1;

    /* Compose CR2: set slave address (7-bit left aligned to bits [7:1]), write direction (RD_WRN=0),
       NBYTES, and START. We don't set AUTOEND so we will generate STOP manually.
    */
    I2C_BASE->CR2 = 0x00;
    uint32_t cr2 = 0;
    cr2 |= ((uint32_t)slaveAddr << 1) & I2C_CR2_SADD;   /* SADD in bits [7:1] */
    cr2 |= ((uint32_t)len << 16) & I2C_CR2_NBYTES; /* NBYTES */
    /* Clear previous flags  */
    I2C_BASE->ICR = 0xFFFFFFFFUL;
    while(I2C_BASE->ISR & I2C_ISR_BUSY);
    I2C_BASE->CR2 = cr2 | I2C_CR2_START;

    for (uint16_t i = 0; i < len; ++i)
    {
        /* Wait until TXIS (transmit interrupt status) = 1 , ACK for previous byte , (TXDR can be written) */
        uint32_t to = 100000;	// timeout for indefinite ACK or NACK flag not set
        while (!(I2C_BASE->ISR & I2C_ISR_TXIS))
        {
            if (I2C_BASE->ISR & I2C_ISR_NACKF)
            {
            	I2C_BASE->ICR = I2C_ICR_NACKCF;
            	I2C_BASE->CR2 |= I2C_CR2_STOP;
            	while (!(I2C_BASE->ISR & I2C_ISR_STOPF));
            	I2C_BASE->ICR = I2C_ICR_STOPCF;
            	return TX_NACK;
            }
            if (--to == 0)
            {
            	I2C_BASE->CR2 |= I2C_CR2_STOP;
            	while (!(I2C_BASE->ISR & I2C_ISR_STOPF));
            	I2C_BASE->ICR = I2C_ICR_STOPCF;
            	return TX_TIMEOUT;
            }
        }

        /* Write data to TXDR */
        I2C_BASE->TXDR = data[i];
    }

    /* Wait for Transfer Complete (TC) flag */
    {
        uint32_t to = 100000;
        while (!(I2C_BASE->ISR & I2C_ISR_TC))
        {
            if (I2C_BASE->ISR & I2C_ISR_NACKF)
            {
            	I2C_BASE->ICR = I2C_ICR_NACKCF;
            	I2C_BASE->CR2 |= I2C_CR2_STOP;
            	while (!(I2C_BASE->ISR & I2C_ISR_STOPF));
                I2C_BASE->ICR = I2C_ICR_STOPCF;
            	return TX_NACK;
            }
            if (--to == 0)
            {
            	I2C_BASE->CR2 |= I2C_CR2_STOP;
            	while (!(I2C_BASE->ISR & I2C_ISR_STOPF));
            	I2C_BASE->ICR = I2C_ICR_STOPCF;
            	return TX_TIMEOUT;
            }
        }
    }

    /* Generate STOP condition */
    I2C_BASE->CR2 |= I2C_CR2_STOP;

    /* Wait until STOPF is set (stop detected) */
    {
        uint32_t to = 100000;
        while (!(I2C_BASE->ISR & I2C_ISR_STOPF))
        {
//            if (--to == 0) return -6;
        }
    }

    /* Clear STOPF by writing to ICR */
    I2C_BASE->ICR = I2C_ICR_STOPCF;
    return TX_SUCCESS;		// successful transaction on i2c bus
}


/* Master receive: blocking read
 * I2C_BASE : i2c port or instance
 * slaveAddr: 7-bit slave address
 * data: buffer to store received bytes
 * len: number of bytes to read
 */
i2c_rx_state i2c_master_read(I2C_TypeDef *I2C_BASE, uint8_t slaveAddr, uint8_t *data, uint16_t len)
{
    /* Prepare CR2 for read (set RD_WRN = 1) */
	 I2C_BASE->CR2 =0x00;
    uint32_t cr2 = 0;
    cr2 |= ((uint32_t)slaveAddr << 1) & I2C_CR2_SADD;
    cr2 |= I2C_CR2_RD_WRN;                       /* read direction */
    cr2 |= ((uint32_t)len << 16) & I2C_CR2_NBYTES;
    I2C_BASE->ICR = 0xFFFFFFFFUL;
    while(I2C_BASE->ISR & I2C_ISR_BUSY);
    I2C_BASE->CR2 = cr2 | I2C_CR2_START;

    for(uint16_t i = 0; i < len; ++i)
    {
        /* Wait until RXNE (receive not empty) */
        uint32_t to = 100000;
        while (!(I2C_BASE->ISR & I2C_ISR_RXNE))
        {
            if(I2C_BASE->ISR & I2C_ISR_NACKF)
            {
            	I2C_BASE->ICR = I2C_ICR_NACKCF;
            	I2C_BASE->CR2 |= I2C_CR2_STOP;
            	while (!(I2C_BASE->ISR & I2C_ISR_STOPF));
            	I2C_BASE->ICR = I2C_ICR_STOPCF;
            	return RX_NACK;
            }
            if(--to == 0)
            {
            	I2C2->CR2 |= I2C_CR2_STOP;
            	while (!(I2C_BASE->ISR & I2C_ISR_STOPF));
            	I2C_BASE->ICR = I2C_ICR_STOPCF;
            	return RX_TIMEOUT;
            }
        }

        /* Read from RXDR */
        data[i] = (uint8_t)I2C_BASE->RXDR;
    }

//    /* Wait for TC */
    {
        uint32_t to = 100000;
        while (!(I2C_BASE->ISR & I2C_ISR_TC))
        {
            if (I2C_BASE->ISR & I2C_ISR_NACKF)
            {
            	I2C_BASE->ICR = I2C_ICR_NACKCF;
            	I2C_BASE->CR2 |= I2C_CR2_STOP;
            	while (!(I2C_BASE->ISR & I2C_ISR_STOPF));
            	I2C_BASE->ICR = I2C_ICR_STOPCF;
            	return -4;
            }
            if (--to == 0)
            {
            	I2C2->CR2 |= I2C_CR2_STOP;
            	while (!(I2C_BASE->ISR & I2C_ISR_STOPF));
            	I2C_BASE->ICR = I2C_ICR_STOPCF;
            	return RX_TIMEOUT;
            }
        }
    }

    /* STOP */
    I2C_BASE->CR2 |= I2C_CR2_STOP;
    {
       uint32_t to = 100000;
       while (!(I2C_BASE->ISR & I2C_ISR_STOPF))
       {
    	   if (I2C_BASE->ISR & I2C_ISR_NACKF)
		   {
				I2C_BASE->ICR = I2C_ICR_NACKCF;
		   }
       }
//

    }
    I2C_BASE->ICR = I2C_ICR_STOPCF;

    return RX_SUCCESS;
}
