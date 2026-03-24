#ifndef __MMC5603_H
#define __MMC5603_H

#include "bsp_hardware.h"

/* vendor chip id*/
#define MMC5603_IIC_ADDR		0x30

//MMC56x3NJ Register Addresses
#define MMC56x3NJ_REG_DATA      0x00
#define MMC56x3NJ_REG_TMPT      0x09
#define MMC56x3NJ_REG_STATUS0   0x19
#define MMC56x3NJ_REG_STATUS1   0x18
#define MMC56x3NJ_REG_ODR       0x1A
#define MMC56x3NJ_REG_CTRL0     0x1B
#define MMC56x3NJ_REG_CTRL1     0x1C
#define MMC56x3NJ_REG_CTRL2     0x1D
#define MMC56x3NJ_REG_WHO_AM_I  0x39
#define MMC56x3NJ_REG_THRD      0x3C


#define MMC56x3NJ_REG_TPH0      0x0B
#define MMC56x3NJ_REG_TPH1      0x0A
#define MMC56x3NJ_REG_TURR      0x0C
#define MMC56x3NJ_REG_DT0       0x0D



/*MMC56x3NJ Measurement Command*/
#define MMC56x3NJ_BW_100HZ      0x00     
#define MMC56x3NJ_BW_300HZ      0x01     
#define MMC56x3NJ_BW_500HZ      0x02
#define MMC56x3NJ_BW_1000HZ     0x03
#define MMC56x3NJ_SOFT_RESET    0x80
#define MMC56x3NJ_CMM_CONF      0xA0
#define MMC56x3NJ_CONT_MEASURE  0x1D
#define MMC56x3NJ_AUTO_SET      0x10
#define MMC56x3NJ_MANU_TM       0x01
#define MMC56x3NJ_MANU_TEMP     0x02
#define MMC56x3NJ_MANU_SET      0x08
#define MMC56x3NJ_MANU_RESET    0x10
#define MMC56x3NJ_SFTEST_POSEN  0x20


#define MMC56x3NJ_WHOAMI_VALUE		0x10	
#define MMC56x3NJ_OTP_READ_DONE		0x10

#define MMC56x3NJ_SINGLE_TM_TIME	10 //ms
#define MMC56x3NJ_SINGLE_ST_TM		2  //ms
#define MMC56x3NJ_SINGLE_SET_TIME	1  //ms
#define MMC56x3NJ_ST_DELTA_VALUE	100//count
#define MMC56x3NJ_STABLE_DELAY		20

#define MMC56x3NJ_NUM_AXES			3
#define MMC56x3NJ_CMD_TMM			0x01
#define MMC56x3NJ_CMD_TMT			0x02
#define MMC56x3NJ_CMD_START_MDT		0x04
#define MMC56x3NJ_CMD_SET			0x08
#define MMC56x3NJ_CMD_RESET			0x10
#define MMC56x3NJ_CMD_AUTO_SR_EN	0x20
#define MMC56x3NJ_CMD_AUTO_ST_EN	0x40
#define MMC56x3NJ_CMD_CMM_FREQ_EN	0x80
#define MMC56x3NJ_CMD_CMM_EN		0x10

/* Bit definition for control register 1 0x1C */
#define MMC56x3NJ_CMD_BW00      0x00
#define MMC56x3NJ_CMD_BW01      0x01
#define MMC56x3NJ_CMD_BW10      0x02
#define MMC56x3NJ_CMD_BW11      0x03
#define MMC56x3NJ_CMD_ST_ENP    0x20
#define MMC56x3NJ_CMD_ST_ENM    0x40
#define MMC56x3NJ_CMD_SW_RST    0x80


/* Bit definition for control register ODR 0x1A */
#define MMC56x3NJ_CMD_ODR_1HZ   0x01
#define MMC56x3NJ_CMD_ODR_5HZ   0x05
#define MMC56x3NJ_CMD_ODR_10HZ  0x0A
#define MMC56x3NJ_CMD_ODR_50HZ  0x32
#define MMC56x3NJ_CMD_ODR_100HZ 0x64
#define MMC56x3NJ_CMD_ODR_200HZ 0xC8
#define MMC56x3NJ_CMD_ODR_255HZ 0xFF



#define MMC56x3NJ_REG_X_THD     0x1E
#define MMC56x3NJ_REG_Y_THD     0x1F
#define MMC56x3NJ_REG_Z_THD     0x20

#define MMC56x3NJ_REG_ST_X_VAL  0x27
#define MMC56x3NJ_REG_ST_Y_VAL  0x28
#define MMC56x3NJ_REG_ST_Z_VAL  0x29
#define MMC56x3NJ_SAT_SENSOR    0x20


#define MMC56x3X_NUM_DATA_HXL	6

/** Soft reset */
#define MMC56x3X_SOFT_RESET     0x80


int mmc5603_write_reg(unsigned char addr, unsigned char data);
int mmc5603_read_block(unsigned char addr, unsigned char *data, unsigned char len);

int mmc5603_init(void);
int mmc5603_enable(int en);
void mmc5603_soft_reset(void);
int mmc5603_read_mag_xyz(float *data);
int mmc5603_do_selftest(short *data);
int mmc5603_config_mode(unsigned char mode);
int mmc5603_config_odr(unsigned char odr);
int mmc5603_config_range(unsigned char range);
int mmc5603_config_setrst(unsigned char setrst);
int mmc5603_self_test(void);

#endif

