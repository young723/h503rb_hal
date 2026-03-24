#ifndef _BSP_SW_I2C_H
#define _BSP_SW_I2C_H

#include "stm32h5xx_hal.h"
#include "stm32h5xx_nucleo.h"

void i2c_sw_gpio_config(uint16_t id);
uint8_t i2c_CheckDevice(uint8_t _Address);
uint8_t qst_sw_writereg(uint8_t slave, uint8_t reg_add,uint8_t reg_dat);
uint8_t qst_sw_writeregs(uint8_t slave, uint8_t reg_add, uint8_t *reg_dat, uint16_t len);
uint8_t qst_sw_readreg(uint8_t slave, uint8_t reg_add, uint8_t *buf, uint16_t num);
uint8_t qst_iic_tx(uint8_t slave, uint8_t *buf, uint16_t num);
uint8_t qst_iic_rx(uint8_t slave, uint8_t *buf, uint16_t num);

#endif
