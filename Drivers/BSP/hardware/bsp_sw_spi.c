 /**
  ******************************************************************************
  * @file    bsp_xxx.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   spi bsp 
  ******************************************************************************
  */
  
#include "bsp_sw_spi.h"

//CPOL=0，表示当SCLK=0时处于空闲态，所以有效状态就是SCLK处于高电平时；
//CPOL=1，表示当SCLK=1时处于空闲态，所以有效状态就是SCLK处于低电平时；
//CPHA=0，表示主控数据采样是在第1个边沿，主控数据发送在第2个边沿；
//CPHA=1，表示主控数据采样是在第2个边沿，主控数据发送在第1个边沿。

static void spi_sw_delay(uint32_t nCount)
{
	while(nCount--)
	{
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
	}
}

void spi_sw_init(int wire, int mode)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();


	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pin = SW_SPI_SCK_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SW_SPI_SCK_PORT,&GPIO_InitStruct);

	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pin = SW_SPI_CS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SW_SPI_CS_PORT,&GPIO_InitStruct);

	if(wire == 3)
	{
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Pin = SPI_DATA_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(SPI_DATA_PORT,&GPIO_InitStruct);
	}
	else if(wire == 4)
	{
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Pin = SW_SPI_MOSI_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(SW_SPI_MOSI_PORT,&GPIO_InitStruct);

		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Pin = SW_SPI_MISO_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(SW_SPI_MISO_PORT,&GPIO_InitStruct);
	}

	SPI_CS_HIGH;

	if((mode == 0)||(mode == 1))
		SPI_SCK_LOW;
	else if((mode == 2)||(mode == 3))
		SPI_SCK_HIGH;
	else
		SPI_SCK_LOW;	
}

static void SPI_DATA_INPUT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pin = SPI_DATA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SPI_DATA_PORT, &GPIO_InitStruct);
}

static void SPI_DATA_OUTPUT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pin = SPI_DATA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SPI_DATA_PORT, &GPIO_InitStruct);
}

uint8_t spi_3wire_write_0(uint8_t data)
{
	uint8_t i = 0;

	SPI_DATA_OUTPUT();
	__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();

	for(i=0;i<8;i++)
	{
		if(data&0x80)
		{
			SPI_DATA_HIGH;
		}
		else
		{
			SPI_DATA_LOW;
		}
		data<<=1;

		SPI_SCK_HIGH;
		__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();
		SPI_SCK_LOW;
		//__NOP();__NOP();__NOP();
		//__NOP();__NOP();__NOP();
	}

	SPI_DATA_INPUT();

	return 1;
}

uint8_t spi_3wire_read_0(uint8_t dummy)
{
	uint8_t RecevieData=0,i;

//	SPI_DATA_INPUT();
	__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();

	for(i=0;i<8;i++)
	{
		SPI_SCK_HIGH;
		__NOP();__NOP();__NOP();

		RecevieData <<= 1;
		if(SPI_DATA_STATUS)
		{
			RecevieData |= 1;
		}

		SPI_SCK_LOW;
		__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();
	}

	return RecevieData;
}

uint8_t spi_3wire_write_3(uint8_t data)
{
	uint8_t i = 0;

	SPI_DATA_OUTPUT();
	for(i=0;i<8;i++)
	{
		SPI_SCK_LOW;
		__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();

		if(data&0x80)
		{
			SPI_DATA_HIGH;
		}
		else
		{
			SPI_DATA_LOW;
		}
		data<<=1;		

		SPI_SCK_HIGH;
		__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();
	}
	SPI_DATA_INPUT();

	return 1;
}


uint8_t spi_3wire_read_3(uint8_t dummy)
{
	uint8_t RecevieData=0,i;

//	SPI_DATA_INPUT();
	for(i=0;i<8;i++)
	{
		SPI_SCK_LOW;
		__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();

		SPI_SCK_HIGH;
		__NOP();__NOP();
		RecevieData <<= 1;
		if(SPI_DATA_STATUS)
		{
			RecevieData |= 1;
		}
	}

	return RecevieData;
}


uint8_t spi_4wire_write_read_0(uint8_t data)
{
	uint8_t RecevieData=0,i;

	for(i=0;i<8;i++)
	{
		if(data&0x80)
		{
			SPI_MOSI_HIGH;
		}
		else
		{
			SPI_MOSI_LOW;
		}
		data<<=1;

		SPI_SCK_HIGH;
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();

		RecevieData <<= 1;
		if(SPI_MISO_DATA)
		{
			RecevieData |= 1;
		}

		SPI_SCK_LOW;
		__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();
	}

	SPI_SCK_LOW;

	return RecevieData;
}

uint8_t spi_4wire_write_read_3(uint8_t data)
{
	uint8_t RecevieData=0,i;

	for(i=0;i<8;i++)
	{
		SPI_SCK_LOW;
		__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();

		if(data&0x80)
		{
			SPI_MOSI_HIGH;
		}
		else
		{
			SPI_MOSI_LOW;
		}
		data<<=1;

		SPI_SCK_HIGH;
		__NOP();__NOP();

		RecevieData <<= 1;
		if(SPI_MISO_DATA)
		{
			RecevieData |= 1;
		}
	}

	SPI_SCK_HIGH;

	return RecevieData;
}




// interface
uint8_t qst_sw_spi4_mode0_write(uint8_t Addr, uint8_t Data)
{
	SPI_CS_LOW;
	spi_sw_delay(2);
	spi_4wire_write_read_0(Addr&0x7f);
	spi_4wire_write_read_0(Data);
	spi_sw_delay(2);
	SPI_CS_HIGH;

	return 1;
}

uint8_t qst_sw_spi4_mode0_read(uint8_t addr, uint8_t* buff, uint16_t len)
{
	uint16_t index = 0;

	SPI_CS_LOW;
	spi_sw_delay(2);
	spi_4wire_write_read_0(addr|0x80);
	for(index=0; index<len; index++)
	{
		buff[index] = spi_4wire_write_read_0(0x00);
	}
	spi_sw_delay(2);
	SPI_CS_HIGH;

	return 1;
}

uint8_t qst_sw_spi4_mode3_write(uint8_t Addr, uint8_t Data)
{
	SPI_CS_LOW;
	spi_sw_delay(2);
	spi_4wire_write_read_3(Addr&0x7f);
	spi_4wire_write_read_3(Data);
	spi_sw_delay(2);
	SPI_CS_HIGH;

	return 1;
}

uint8_t qst_sw_spi4_mode3_read(uint8_t addr, uint8_t* buff, uint16_t len)
{
	uint16_t index = 0;

	SPI_CS_LOW;
	spi_sw_delay(2);
	spi_4wire_write_read_3(addr|0x80);
	for(index=0; index<len; index++)
	{
		buff[index] = spi_4wire_write_read_3(0x00);
	}
	spi_sw_delay(2);
	SPI_CS_HIGH;

	return 1;
}



uint8_t qst_sw_spi3_mode0_write(uint8_t Addr, uint8_t Data)
{
	SPI_CS_LOW;
	spi_sw_delay(2);
	spi_3wire_write_0(Addr&0x7f);
	spi_3wire_write_0(Data);
	spi_sw_delay(2);
	SPI_CS_HIGH;

	return 1;
}

uint8_t qst_sw_spi3_mode0_read(uint8_t addr, uint8_t* buff, uint16_t len)
{
	uint16_t index = 0;

	SPI_CS_LOW;
	spi_sw_delay(2);
	spi_3wire_write_0(addr|0x80);
	for(index=0; index<len; index++)
	{
		buff[index] = spi_3wire_read_0(0x00);
	}
	spi_sw_delay(2);
	SPI_CS_HIGH;

	return 1;
}

uint8_t qst_sw_spi3_mode3_write(uint8_t Addr, uint8_t Data)
{
	SPI_CS_LOW;
	spi_sw_delay(2);
	spi_3wire_write_3(Addr&0x7f);
	spi_3wire_write_3(Data);
	spi_sw_delay(2);
	SPI_CS_HIGH;

	return 1;
}

uint8_t qst_sw_spi3_mode3_read(uint8_t addr, uint8_t* buff, uint16_t len)
{
	uint16_t index = 0;

	SPI_CS_LOW;
	spi_sw_delay(2);
	spi_3wire_write_3(addr|0x80);
	for(index=0; index<len; index++)
	{
		buff[index] = spi_3wire_read_3(0x00);
	}
	spi_sw_delay(2);
	SPI_CS_HIGH;

	return 1;
}


