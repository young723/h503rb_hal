

#ifndef __BSP_FLASH_H
#define	__BSP_FLASH_H

#include "stm32h5xx_hal.h"
#include "stm32h5xx_nucleo.h"

void bsp_flash_write_quadword(uint32_t bufAddr);
void bsp_flash_read_quadword(uint32_t *buf);

#endif
