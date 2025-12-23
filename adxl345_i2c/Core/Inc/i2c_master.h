/*
 * i2c.h
 *
 *  Created on: Nov 28, 2025
 *      Author: Dell
 */

#ifndef INC_I2C_MASTER_H_
#define INC_I2C_MASTER_H_
#include "stm32f3xx_hal.h"
#include <stdlib.h>
#include <stdint.h>

#define BUFFER_SIZE 		 50
#define I2C_SLAVE_ADDR		(0x53)
#define I2C_TIMINGR 		((1U << I2C_TIMINGR_PRESC_Pos) | (0x0FU << I2C_TIMINGR_SCLH_Pos) | (0x13U << I2C_TIMINGR_SCLL_Pos) | (0x02 << I2C_TIMINGR_SDADEL_Pos) | (0x04 << I2C_TIMINGR_SCLDEL_Pos))
#define I2C_PORT		     I2C1

typedef enum
{
	TX_SUCCESS,
	TX_TIMEOUT,
	TX_NACK,
}i2c_tx_state;

typedef enum
{
	RX_SUCCESS,
	RX_TIMEOUT,
	RX_NACK,
}i2c_rx_state;

void GPIO_Init();
void I2C_Master_Init(I2C_TypeDef *I2C_BASE);
i2c_tx_state i2c_master_write(I2C_TypeDef *I2C_BASE, uint8_t slaveAddr, const uint8_t *data, uint16_t len);
i2c_rx_state i2c_master_read(I2C_TypeDef *I2C_BASE, uint8_t slaveAddr, uint8_t *data, uint16_t len);

#endif /* INC_I2C_MASTER_H_ */
