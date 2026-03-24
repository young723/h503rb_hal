
#include "stdio.h"
#include "bsp_i2c.h"

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

/* I2C1 init function */

int bsp_i2c1_write(unsigned char slave, unsigned char addr, unsigned char value)
{
	HAL_StatusTypeDef status;
	unsigned char buf[2];
	
	buf[0] = addr;
	buf[1] = value;
	status = HAL_I2C_Master_Transmit(&hi2c1, slave, buf, 2, 100);
	//status = HAL_I2C_Mem_Write(&hi2c2, slave, addr, 1, &value, 1, 100);
	if(status == HAL_OK)
		return 1;
	else
		return 0;
}

int bsp_i2c1_writes(unsigned char slave, unsigned char addr, unsigned char *value, unsigned short len)
{
	HAL_StatusTypeDef status;
	unsigned char buf[2];
	
	buf[0] = addr;
	for(int i=0; i<len; i++)
	{
		buf[i+1] = value[i];
	}
	status = HAL_I2C_Master_Transmit(&hi2c1, slave, buf, len+1, 100);
	//status = HAL_I2C_Mem_Write(&hi2c2, slave, addr, 1, &value, 1, 100);
	if(status == HAL_OK)
		return 1;
	else
		return 0;
}

int bsp_i2c1_read(unsigned char slave, unsigned char addr, unsigned char *buf, unsigned short len)
{
	HAL_StatusTypeDef status;
	
	status = HAL_I2C_Mem_Read(&hi2c1, slave, addr, 1, buf, len, 100);
	if(status == HAL_OK)
		return 1;
	else
		return 0;

}

int bsp_i2c2_write(unsigned char slave, unsigned char addr, unsigned char value)
{
	HAL_StatusTypeDef status;
	unsigned char buf[2];
	
	buf[0] = addr;
	buf[1] = value;
	status = HAL_I2C_Master_Transmit(&hi2c2, slave, buf, 2, 100);
	//status = HAL_I2C_Mem_Write(&hi2c2, slave, addr, 1, &value, 1, 100);
	if(status == HAL_OK)
		return 1;
	else
		return 0;
}

int bsp_i2c2_writes(unsigned char slave, unsigned char addr, unsigned char *value, unsigned short len)
{
	HAL_StatusTypeDef status;
	unsigned char buf[2];
	
	buf[0] = addr;
	for(int i=0; i<len; i++)
	{
		buf[i+1] = value[i];
	}
	status = HAL_I2C_Master_Transmit(&hi2c2, slave, buf, len+1, 100);
	//status = HAL_I2C_Mem_Write(&hi2c2, slave, addr, 1, &value, 1, 100);
	if(status == HAL_OK)
		return 1;
	else
		return 0;
}

int bsp_i2c2_read(unsigned char slave, unsigned char addr, unsigned char *buf, unsigned short len)
{
	HAL_StatusTypeDef status;
	
	status = HAL_I2C_Mem_Read(&hi2c2, slave, addr, 1, buf, len, 100);
	if(status == HAL_OK)
		return 1;
	else
		return 0;

}


int bsp_i2c2_write_buf(unsigned char slave, unsigned char *buf, unsigned short len)
{
	HAL_StatusTypeDef status;
	
	status = HAL_I2C_Master_Transmit(&hi2c2, slave, buf, len, 100);
	if(status == HAL_OK)
		return 1;
	else
		return 0;

}


int bsp_i2c2_read_buf(unsigned char slave, unsigned char *buf, unsigned short len)
{
	HAL_StatusTypeDef status;
	
	status = HAL_I2C_Master_Receive(&hi2c2, slave, buf, len, 100);
	if(status == HAL_OK)
		return 1;
	else
		return 0;

}
