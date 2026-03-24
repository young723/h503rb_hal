

#ifndef __BSP_SPI_H
#define	__BSP_SPI_H

#include "stm32h5xx_hal.h"
#include "stm32h5xx_nucleo.h"

extern void MX_SPI1_Init(int mode);

extern int qst_hw_spi_write(unsigned char addr, unsigned char value);
extern int qst_hw_spi_read(unsigned char addr, unsigned char *buf, unsigned short len);

#endif

