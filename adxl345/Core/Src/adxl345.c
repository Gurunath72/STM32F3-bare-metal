#include "adxl345.h"

const float FOUR_G_SCALE_FACT = 0.0078;

/* USER CODE BEGIN PV */
uint8_t AxesData[6] = {0};
uint16_t x,y,z = 0;	// x, y and z - axes data
float xg,yg,zg = 0.0;	// x, y, and z acceleration constant g data
uint8_t deviceID = 0x00;


adxl_transfer_state ADXL_Init()
{
	uint8_t RegAddr = 0x00;
	RegAddr = DEVID_R;
	uint8_t dataBuf[10] = {0};

	adxl_transfer_state ret = 0;

	ret = ADXL_Read(RegAddr, &deviceID, (uint16_t)0x01);
	if(ret != ADXL_READ_SUCCESS)
	{
		return ADXL_INIT_FAIL;
	}
	/* read device id to verify i2c operation is working with sensor*/
//	i2c_master_write(I2C1, I2C_SLAVE_ADDR, &RegAddr, (uint16_t)0x01);
//	i2c_master_read(I2C1, I2C_SLAVE_ADDR, &deviceID, 1);

	dataBuf[0] = (uint8_t)DATA_FORMAT_R;
	dataBuf[1] = FOUR_G;

	ret = ADXL_Write(RegAddr, dataBuf, 0x02);
	if(ret != ADXL_WRITE_SUCCESS)
	{
		return ADXL_INIT_FAIL;
	}

	/* Set data format to +/- 4g */
//	RegAddr = DATA_FORMAT_R;
//	i2c_master_write(I2C1, I2C_SLAVE_ADDR, &RegAddr, FOUR_G);

	/* Reset all bits */

	dataBuf[0] = (uint8_t)POWER_CTL_R;
	dataBuf[1] = RESET;

	ret = ADXL_Write(RegAddr, dataBuf, 0x02);
	if(ret != ADXL_WRITE_SUCCESS)
	{
		return ADXL_INIT_FAIL;
	}

//	RegAddr = POWER_CTL_R;
//	i2c_master_write(I2C1, I2C_SLAVE_ADDR, &RegAddr, RESET);
	/* Configure power control reg -  measure bits */

	dataBuf[0] = (uint8_t)POWER_CTL_R;
	dataBuf[1] = SET_MEASURE_BIT;

	ret = ADXL_Write(RegAddr, dataBuf, 0x02);
	if(ret != ADXL_WRITE_SUCCESS)
	{
		return ADXL_INIT_FAIL;
	}

//	RegAddr = POWER_CTL_R;
//	i2c_master_write(I2C1, I2C_SLAVE_ADDR, &RegAddr, SET_MEASURE_BIT);	// set measurement mode in sensor
}


adxl_transfer_state ADXL_Write( uint8_t regAddr,  uint8_t *data, uint16_t datLen)
{
	int ret = 0;
	uint8_t dataBuf[datLen+ 1];
	dataBuf[0] = regAddr;
	memcpy(dataBuf+1, data, datLen);
	ret = i2c_master_write(I2C_PORT, I2C_SLAVE_ADDR, dataBuf, (uint16_t)datLen+ 1);
	if(ret != TX_SUCCESS)
	{
		return ADXL_WRITE_FAIL;
	}
	return ADXL_WRITE_SUCCESS;
}


adxl_transfer_state ADXL_Read( uint8_t regAddr, uint8_t *data, uint16_t len)
{
	int ret = 0;
	uint8_t dataBuf[1] = {regAddr};
	ret = i2c_master_write(I2C_PORT, I2C_SLAVE_ADDR, dataBuf, (uint16_t)0x01);
	if(ret != TX_SUCCESS)
	{
		return ADXL_READ_FAIL;
	}

	ret = i2c_master_read(I2C_PORT, I2C_SLAVE_ADDR, data, len);
	if(ret != RX_SUCCESS)
	{
		return ADXL_READ_FAIL;
	}

	return ADXL_READ_SUCCESS;
}
