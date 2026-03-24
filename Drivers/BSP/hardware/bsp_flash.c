
#include "stdio.h"
#include "bsp_flash.h"

#define FLASH_BAS_ADDR	0x08006000

void bsp_flash_write_quadword(uint32_t bufAddr)
{
	HAL_FLASH_Unlock();	//解锁FLASH
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTCHANGEERR|FLASH_FLAG_WRPERR);
#if 1
	FLASH_EraseInitTypeDef f;
	f.TypeErase = FLASH_TYPEERASE_MASSERASE;
	f.Banks = FLASH_BANK_1;
	f.Sector = FLASH_SECTOR_0;	//FLASH_BAS_ADDR;
	f.NbSectors = 1;

	uint32_t PageError =0;
	HAL_FLASHEx_Erase(&f, &PageError);
#endif
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, FLASH_BAS_ADDR, bufAddr);		//对FLASH烧写
	HAL_FLASH_Lock();//锁住 FLASH
}


void bsp_flash_read_quadword(uint32_t *buf)
{
	buf[0] = *((__IO uint32_t*)(0x08006000));
	buf[1] = *((__IO uint32_t*)(0x08006004));
//	buf[2] = *((__IO uint32_t*)(FLASH_BAS_ADDR+8));
//	buf[3] = *((__IO uint32_t*)(FLASH_BAS_ADDR+12));
}
