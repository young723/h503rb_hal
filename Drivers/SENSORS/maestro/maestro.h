#ifndef __MAESTRO_H
#define __MAESTRO_H

#include "bsp_hardware.h"

//#define MAESTRO_0

#define MAESTRO_RECOVER
//#define MAESTRO_KXKY

#define MAESTRO_OK			1
#define MAESTRO_FAIL		0

#define MAESTRO_IIC_ADDR				0x0c

/* chip id*/
#define MAESTRO1V_CHIP_ID				0x10
#define MAESTRO2H_CHIP_ID				0x20
#define MAESTRO_FPGA_CHIP_ID			0xa5
/*data output register*/
#define MAESTRO_CHIP_ID_REG				0x00
#define MAESTRO_DATA_OUT_X_LSB_REG		0x01
#define MAESTRO_DATA_OUT_X_MSB_REG		0x02
#define MAESTRO_DATA_OUT_Y_LSB_REG		0x03
#define MAESTRO_DATA_OUT_Y_MSB_REG		0x04
#define MAESTRO_DATA_OUT_Z_LSB_REG		0x05
#define MAESTRO_DATA_OUT_Z_MSB_REG		0x06
#define MAESTRO_TEMP_OUT_LSB_REG		0x07
#define MAESTRO_TEMP_OUT_MSB_REG		0x08

#define MAESTRO_DATA_OUT_ST_X			0x13

/*Status registers */
#define MAESTRO_STATUS_REG				0x09
/* configuration registers */
#define MAESTRO_CTL_REG_ONE				0x0A  /* Contrl register one */
#define MAESTRO_CTL_REG_TWO				0x0B  /* Contrl register two */
#define MAESTRO_CTL_REG_PSET			0x0C
//#define MAESTRO_CTL_REG_AAF_CTRL		0x0D
#define MAESTRO_CTL_REG_SELFTEST		0x0E
#define MAESTRO_CTL_IBI					0x21

#define MAESTRO_FIFO_REG_STATUS			0x20
#define MAESTRO_FIFO_REG_CTRL			0x2E
#define MAESTRO_FIFO_REG_DATA			0x2F

#define MAESTRO_DATA_OFFSET_X_LSB_REG	0x22

// maestro0 set/reset@reg0e
#define MAESTRO0_SR_OFF					0	// 0 or 1
#define MAESTRO0_SR_ON1					2	// reset at temp_sinc_enable
#define MAESTRO0_SR_ON2					3	// reset at afe_rstb
// maestro1 set/reset
#define MAESTRO_SET_RESET_ON 			1
#define MAESTRO_SET_RESET_OFF 			0
#define MAESTRO_SET_ON 					1
#define MAESTRO_RESET_ON 				3

#define MAESTRO_RNG_32G					0
#define MAESTRO_RNG_16G					1
#define MAESTRO_RNG_8G					2
#define MAESTRO1V_RNG_20G				0

#define MAESTRO_MODE_SUSPEND			0
#define MAESTRO_MODE_NORMAL				1
#define MAESTRO_MODE_SINGLE				2
#define MAESTRO_MODE_HPFM				3

// maestro0 odr
#define MAESTRO0_ODR_HPF				0
#define MAESTRO0_ODR_1HZ				0
#define MAESTRO0_ODR_10HZ				1
#define MAESTRO0_ODR_50HZ				2
#define MAESTRO0_ODR_100HZ				3
#define MAESTRO0_ODR_200HZ				4	// 4~7
// maestro1 odr
#define MAESTRO_ODR_HPF					0
#define MAESTRO_ODR_1HZ					0
#define MAESTRO_ODR_10HZ				1
#define MAESTRO_ODR_20HZ				2
#define MAESTRO_ODR_50HZ				3
#define MAESTRO_ODR_100HZ				4
#define MAESTRO_ODR_200HZ				5
#define MAESTRO_ODR_400HZ				6
#define MAESTRO_ODR_1000HZ				7

// CIC filter OSR: 512/256/128/64(0~3)
#define MAESTRO_OSR1_8					0		// 512
#define MAESTRO_OSR1_4					1		// 256
#define MAESTRO_OSR1_2					2		// 128
#define MAESTRO_OSR1_1					3		// 64
#define MAESTRO_OSR1_16					4		// 1024	maestro1
#define MAESTRO_OSR1_32					5		// 2048	maestro1
#define MAESTRO_OSR1_64					6		// 4096	maestro1
#define MAESTRO_OSR1_128				7		// 8192	maestro1

#define MAESTRO1V_OSR1_1				1
#define MAESTRO1V_OSR1_2				0
#define MAESTRO1V_OSR1_4				4
#define MAESTRO1V_OSR1_8				5
#define MAESTRO1V_OSR1_16				6
#define MAESTRO1V_OSR1_32				7

// Moving average depth normal mode the maestro0:depth:16/8/4/2/1(7~4/3/2/1/0) maestro1:depth:8/4/2/1(3/2/1/0). Single Mode also use this filter.
#define MAESTRO_OSR2_1					0
#define MAESTRO_OSR2_2					1
#define MAESTRO_OSR2_4					2
#define MAESTRO_OSR2_8					3
#define MAESTRO_OSR2_16					4		// 4~7	maestro0

#define MAESTRO_STATUS_DRDY				0x01
#define MAESTRO_STATUS_OVFL				0x02
#define MAESTRO_STATUS_ST_RDY			0x04
#define MAESTRO_STATUS_NVM_RDY			0x08
#define MAESTRO_STATUS_NVM_LOAD_DONE	0x10

#define MAESTRO_FIFO_STATUS_OR			0x04
#define MAESTRO_FIFO_STATUS_WMK			0x02
#define MAESTRO_FIFO_STATUS_FULL		0x01

#define MAESTRO_SELFTEST_MAX_X			(50)
#define MAESTRO_SELFTEST_MIN_X			(1)
#define MAESTRO_SELFTEST_MAX_Y			(50)
#define MAESTRO_SELFTEST_MIN_Y			(1)
#define MAESTRO_SELFTEST_MAX_Z			(50)
#define MAESTRO_SELFTEST_MIN_Z			(1)

#define MAESTRO_ABS(X) 					((X) < 0 ? (-1 * (X)) : (X))
#define MAESTRO_ABSF(X) 				((X) < 0.0f ? (-1.0 * (X)) : (X))
#define MAESTRO_MIN(x, y)				((x) < (y) ? (x) : (y))
#define MAESTRO_MAX(x, y)				((x) > (y) ? (x) : (y))


#define MAESTRO_LOG						printf
#define MAESTRO_CHECK_ERR(ret)			do {\
											if((ret) != MAESTRO_OK)	\
											MAESTRO_LOG("maestro error:%d line:%d\r\n",ret, __LINE__);	\
										}while(0)

//#define MAESTRO_MODE_SWITCH
typedef enum
{
	MAESTRO_FIFO_MODE_BYPASS = (0<<6),
	MAESTRO_FIFO_MODE_FIFO = (1<<6),
	MAESTRO_FIFO_MODE_STREAM = (2<<6),
	MAESTRO_FIFO_MODE_DEFAULT = (3<<6)
} maestro_fifo_mode;

typedef enum
{
	MAESTRO_IBI_OFF = 0x00,
	MAESTRO_IBI_DRDY = 0x01,
	MAESTRO_IBI_OVFL = 0x02,
	MAESTRO_IBI_ST_RDY = 0x04,
	MAESTRO_IBI_FIFO_FULL = 0x08,
	MAESTRO_IBI_FIFO_WMK = 0x10,
	MAESTRO_IBI_EN = 0x20
} maestro_fifo_ibi;

typedef struct
{
	char			mode;
	unsigned short	count;
} maestro_sr_ctl_t;

typedef union
{
	struct
	{
		unsigned char mode:2;
		unsigned char zdbl_enb:1;
		unsigned char osr1:2;
		unsigned char osr2:3;
	}bit;
	unsigned char value;
} maestro0_ctrla;

typedef union
{
	struct
	{
		unsigned char set_rst:2;
		unsigned char range:2;
		unsigned char odr:3;
		unsigned char soft_rst:1;
	}bit;
	unsigned char value;
} maestro0_ctrlb;


typedef union
{
	struct
	{
		unsigned char mode:2;
		unsigned char osr1:3;
		unsigned char osr2:2;
		unsigned char rev:1;
	}bit;
	unsigned char value;
} maestro1_ctrla;

typedef union
{
	struct
	{
		unsigned char set_rst:2;
		unsigned char range:2;
		unsigned char odr:3;
		unsigned char soft_rst:1;
	}bit;
	unsigned char value;
} maestro1_ctrlb;

typedef union
{
	struct
	{
		unsigned char period_set:6;
		unsigned char user_set:1;
		unsigned char rev:1;
	}bit;
	unsigned char value;
} maestro_pset;

typedef union
{
	struct
	{
		signed short kx:7;	
		signed short ky:9;
	}bit;
	unsigned short value;
} maestro_kxky;

typedef union
{
	struct
	{
		signed short ky:9;
		signed short rev:7;
	}bit;
	unsigned short value;
} maestro_ky;


typedef struct
{
	unsigned char			protocol;
	unsigned char			slave_addr;
	unsigned short			ssvt;
	short					last_data[3];
#if defined(MAESTRO_0)
	maestro0_ctrla			ctrla;
	maestro0_ctrlb			ctrlb;
#else
	maestro1_ctrla			ctrla;
	maestro1_ctrlb			ctrlb;
#endif
	unsigned char			fifo_ctrl;
	float					kx;
	float					ky;
#if defined(MAESTRO_MODE_SWITCH)
	maestro_sr_ctl_t		set_ctl;
#endif
} maestro_data_t;


int maestro_init(int protocol);
int maestro_read_mag_xyz(float *ut);
void maestro_fifo_config(maestro_fifo_mode mode, unsigned char wmk);
int maestro_fifo_read(unsigned char *f_data);
int maestro_self_test(void);

#endif

