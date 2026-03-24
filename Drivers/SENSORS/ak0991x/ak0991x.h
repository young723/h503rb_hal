#ifndef __AK0991X_H
#define __AK0991X_H

#include "bsp_hardware.h"


/* vendor chip id*/
#define AK0991X_IIC_ADDR				0x0e
#define AK0991X_REG_WHOAMI				0x00
#define AK0991X_REG_ST1					0x10
#define AK0991X_REG_HXL					0x11
#define AK0991X_REG_HXH					0x12
#define AK0991X_REG_HYL					0x13
#define AK0991X_REG_HYH					0x14
#define AK0991X_REG_HZL 				0x15
#define AK0991X_REG_HZH					0x16
#define AK0991X_REG_TMPS 				0x17
#define AK0991X_REG_ST2					0x18
#define AK0991X_REG_CNTL1				0x30
#define AK0991X_REG_CNTL2				0x31
#define AK0991X_REG_CNTL3				0x32

#define AK0991X_WHOAMI_COMPANY_ID		0x48	
#define AK09917_WHOAMI_DEV_ID			0x0d
#define AK09918_WHOAMI_DEV_ID			0x0c
#define AK09919_WHOAMI_DEV_ID			0x0e

#define AK0991X_SDR_DISABLE				(0<<6)
#define AK0991X_SDR_ENABLE				(1<<6)
#define AK0991X_ITS_OFF					(0<<5)
#define AK0991X_ITS_LOW					(1<<5)
#define AK0991X_ITS_HIGH				(2<<5)

typedef enum
{
	UNKNOW = -1,
	AK09911,
	AK09912,
	AK09913,
	AK09915C,
	AK09915D,
	AK09916C,
	AK09916D,
	AK09917,
	AK09918,
	AK09919,
	SUPPORTED_DEVICES
} akm_device_type;

typedef enum
{
	AK0991X_MAG_ODR_OFF = 0x00,      /* power down output data rate */
	AK0991X_MAG_ODR_SNG_MEAS = 0x01, /* single measurement mode */
	AK0991X_MAG_ODR10 = 0x02,        /* 10 Hz output data rate */
	AK0991X_MAG_ODR20 = 0x04,        /* 20 Hz output data rate */
	AK0991X_MAG_ODR50 = 0x06,        /* 50 Hz output data rate */
	AK0991X_MAG_ODR100 = 0x08,       /* 100 Hz output data rate */
	AK0991X_MAG_ODR200 = 0x0A,       /* 200 Hz output data rate */
	AK0991X_MAG_ODR1 = 0x0C,         /* 1 Hz output data rate */
	AK0991X_MAG_ODR5 = 0x0E,         /* 5 Hz output data rate */
	AK0991X_MAG_SELFTEST = 0x10,     /* selftest */
	AK0991X_MAG_FUSEROM = 0x1F,      /* FUSE ROM access mode */
} ak0991x_mag_odr;

typedef struct
{
	akm_device_type		type;
	float				resolution;
	unsigned char		fifo_wmk;
	float				l_data[3];
} akm_sensor_t;

int ak0991x_write_reg(unsigned char addr, unsigned char data);
int ak0991x_read_reg(unsigned char addr, unsigned char *data, unsigned short len);
int ak0991x_init(void);
int ak0991x_read_mag_xyz(float *data);
int ak0991x_read_mag_fifo(unsigned char *data);

#endif

