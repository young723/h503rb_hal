#ifndef __SPI_SW_H
#define __SPI_SW_H

#include "stm32h5xx_hal.h"
#include "stm32h5xx_nucleo.h"

//CS
#define      SW_SPI_CS_PORT                GPIOC
#define      SW_SPI_CS_PIN                 GPIO_PIN_9
//CLK
#define      SW_SPI_SCK_PORT               GPIOA   
#define      SW_SPI_SCK_PIN                GPIO_PIN_5
//MISO
#define      SW_SPI_MISO_PORT              GPIOA 
#define      SW_SPI_MISO_PIN               GPIO_PIN_6
//MOSI
#define      SW_SPI_MOSI_PORT              GPIOA 
#define      SW_SPI_MOSI_PIN               GPIO_PIN_7

#define SPI_CS_HIGH				HAL_GPIO_WritePin(SW_SPI_CS_PORT, SW_SPI_CS_PIN, GPIO_PIN_SET)				// CS----PB6
#define SPI_CS_LOW				HAL_GPIO_WritePin(SW_SPI_CS_PORT, SW_SPI_CS_PIN, GPIO_PIN_RESET)
#define SPI_SCK_HIGH			HAL_GPIO_WritePin(SW_SPI_SCK_PORT, SW_SPI_SCK_PIN, GPIO_PIN_SET)			//SCLK----PA5
#define SPI_SCK_LOW				HAL_GPIO_WritePin(SW_SPI_SCK_PORT, SW_SPI_SCK_PIN, GPIO_PIN_RESET)
#define SPI_MOSI_HIGH			HAL_GPIO_WritePin(SW_SPI_MOSI_PORT, SW_SPI_MOSI_PIN, GPIO_PIN_SET)			//MOSI----PA7
#define SPI_MOSI_LOW			HAL_GPIO_WritePin(SW_SPI_MOSI_PORT, SW_SPI_MOSI_PIN, GPIO_PIN_RESET)
#define SPI_MISO_DATA			HAL_GPIO_ReadPin(SW_SPI_MISO_PORT, SW_SPI_MISO_PIN)							//MISO----PA6

#define SPI_DATA_PORT			GPIOA
#define SPI_DATA_PIN			GPIO_PIN_7
#define SPI_DATA_HIGH			HAL_GPIO_WritePin(SPI_DATA_PORT,		SPI_DATA_PIN, 	GPIO_PIN_SET)
#define SPI_DATA_LOW			HAL_GPIO_WritePin(SPI_DATA_PORT,		SPI_DATA_PIN, 	GPIO_PIN_RESET)
#define SPI_DATA_STATUS			HAL_GPIO_ReadPin(SPI_DATA_PORT,			SPI_DATA_PIN)


void spi_sw_init(int wire, int mode);

//uint8_t spi_3wire_write_0(uint8_t data);
//uint8_t spi_3wire_read_0(uint8_t dummy);
//uint8_t spi_3wire_write_3(uint8_t data);
//uint8_t spi_3wire_read_3(uint8_t dummy);

//uint8_t spi_4wire_write_read_0(uint8_t data);
//uint8_t spi_4wire_write_read_3(uint8_t data);

uint8_t qst_sw_spi4_mode0_write(uint8_t Addr, uint8_t Data);
uint8_t qst_sw_spi4_mode0_read(uint8_t addr, uint8_t* buff, uint16_t len);
uint8_t qst_sw_spi4_mode3_write(uint8_t Addr, uint8_t Data);
uint8_t qst_sw_spi4_mode3_read(uint8_t addr, uint8_t* buff, uint16_t len);

uint8_t qst_sw_spi3_mode0_write(uint8_t Addr, uint8_t Data);
uint8_t qst_sw_spi3_mode0_read(uint8_t addr, uint8_t* buff, uint16_t len);
uint8_t qst_sw_spi3_mode3_write(uint8_t Addr, uint8_t Data);
uint8_t qst_sw_spi3_mode3_read(uint8_t addr, uint8_t* buff, uint16_t len);

#endif /* __SPI_SW_H */

