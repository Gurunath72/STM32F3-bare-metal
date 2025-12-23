/*
 * adxl345.h
 *
 *  Created on: Dec 1, 2025
 *      Author: Dell
 */

#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_

#include "i2c_master.h"

#define DEVID_R						0x00
#define DEVIDID_ADDR_7BIT			0x53
#define DATA_FORMAT_R				0x31
#define POWER_CTL_R					0x2D
#define DATA_START_ADDR				0x32
#define DATA_FORMAT_ADDR			0x31
#define SET_MEASURE_BIT				0x08
#define RESET						0x00
#define FOUR_G						0x01


typedef enum
{
	ADXL_WRITE_SUCCESS,
	ADXL_READ_SUCCESS,
	ADXL_WRITE_FAIL,
	ADXL_READ_FAIL,
	ADXL_INIT_FAIL,
	ADXL_INIT_SUCCESS
}adxl_transfer_state;

extern uint8_t AxesData[6];
extern uint16_t x,y,z;	// x, y and z - axes data
extern float xg,yg,zg;	// x, y, and z acceleration constant g data
extern uint8_t deviceID;	// device ID of adxl sensor to be read by i2c operation
extern const float FOUR_G_SCALE_FACT;	// scaling factor for axis raw data

adxl_transfer_state ADXL_Init();

adxl_transfer_state ADXL_Write( uint8_t regAddr, uint8_t *data, uint16_t len);
adxl_transfer_state ADXL_Read( uint8_t regAddr, uint8_t *data, uint16_t len);

#endif /* INC_ADXL345_H_ */
