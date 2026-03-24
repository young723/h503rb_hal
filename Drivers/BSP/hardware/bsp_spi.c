
#include "stdio.h"
#include "bsp_spi.h"

#ifdef HAL_SPI_MODULE_ENABLED

#define HAL_SPI1_CS_LOW		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET)
#define HAL_SPI1_CS_HIGH	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET)

SPI_HandleTypeDef hspi1;

void MX_SPI1_Init(int mode)
{
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	if(mode == 0)
	{
		hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
		hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	}
	else if(mode == 3)
	{
		hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
		hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	}
	else
	{
		hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
		hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	}
	hspi1.Init.NSS = SPI_NSS_SOFT;
//	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;	// 7.8125Mbps
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;	// 3.90625Mbps
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 0x7;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
	hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
	hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
	hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
	hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;

	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
}


static void bsp_spi_delay(uint32_t nCount)
{
	while(nCount--)
	{
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
	}
}

int qst_hw_spi_write(unsigned char addr, unsigned char value)
{
	HAL_StatusTypeDef status;
	unsigned char buf[2];
	
	buf[0] = addr&0x7f;
	buf[1] = value;
	
	HAL_SPI1_CS_LOW;
	bsp_spi_delay(2);
	status = HAL_SPI_Transmit(&hspi1, buf, 2, 100);
	bsp_spi_delay(2);
	HAL_SPI1_CS_HIGH;
	if(status == HAL_OK)
		return 1;
	else
		return 0;
}

int qst_hw_spi_read(unsigned char addr, unsigned char *buf, unsigned short len)
{
	HAL_StatusTypeDef status;

	HAL_SPI1_CS_LOW;
	bsp_spi_delay(2);
	addr |= 0x80;
	status = HAL_SPI_Transmit(&hspi1, &addr, 1, 100);
	status = HAL_SPI_Receive(&hspi1, buf, len, 100);
	bsp_spi_delay(2);
	HAL_SPI1_CS_HIGH;
	if(status == HAL_OK)
		return 1;
	else
		return 0;
}

#endif

