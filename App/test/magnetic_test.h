
#ifndef _MAGNETIC_TEST_H_
#define _MAGNETIC_TEST_H_

// chip
//#define AK0991X_MMC5603
#define QMC6308

//#define FPGA
#if defined(FPGA)

//#define QMC6309V_X7

//#define MAESTRO0

//#define MAESTRO1
//#define MAESTRO1V
//#define MAESTRO2H
#endif


#include "bsp_hardware.h"
#include "qst_sensor_id.h"
#include "MyFilter.h"
#include "qmc6309.h"
#include "qmc6309v.h"
#include "maestro.h"
#ifdef QMC6308
#include "qmc6308.h"
#endif
#ifdef AK0991X_MMC5603
#include "ak0991x.h"
#include "mmc5603.h"
#endif

#define SENSOR_NUM		1
//#define MAG_TEST_STD_OUT
//#define CUSTOMER_CONFIG_REG
//#define MAG_SOFT_COMPENSATE
//#define QMC630H_3_3V_0X40
#define MAG_OTP_LOCK_UNLOCK

#if defined(MAESTRO1)
//#define MAESTRO1_1_SET
//#define MAESTRO1_PERIOD_SET
//#define MAESTRO1_USER_SET	// 0x0c<5:0>=0x3f, mode:HPF
#endif


#define MAG_TEST_HW_I2C_SUPPORT
#define MAG_TEST_I3C_SUPPORT

#define DEBUG_USART		USART3
//#define TIM2			TIMER2
#if defined(MAG_TEST_HW_I2C_SUPPORT)
#define HW_IIC_CONFIG				MX_I2C1_Init
#define HW_IIC_WRITE				bsp_i2c1_write
#define HW_IIC_READ					bsp_i2c1_read
#endif
#ifdef HAL_SPI_MODULE_ENABLED
#define HW_SPI_WRITE				qst_hw_spi_write
#define HW_SPI_READ					qst_hw_spi_read
#else
#define HW_SPI_WRITE(...)			0
#define HW_SPI_READ(...)			0
#endif

#define MAG_DATA_OUT_X_LSB_REG		0x01
#define MAG_TEMP_OUT_LSB_REG		0x07
#define MAG_CTL_REG_ONE				0x0A
#define MAG_CTL_REG_TWO				0x0B
#define MAG_CTL_REG_THREE			0x0C
#define MAG_CTL_REG_PSET			0x0C
#define MAG_STATUS_REG				0x09
#define MAG_FIFO_REG_STATUS			0x20
#define MAG_FIFO_REG_CTRL			0x2E
#define MAG_FIFO_REG_DATA			0x2F
#define MAG_STATUS_DRDY				0x01
#define MAG_STATUS_OVFL				0x02
#define MAG_STATUS_STRDY			0x04

#define MAG_REG_OFFSET_X_LSB		0x22
#define MAG_REG_OFFSET_X_MSB		0x23
#define MAG_REG_OFFSET_Y_LSB		0x24
#define MAG_REG_OFFSET_Y_MSB		0x25
#define MAG_REG_OFFSET_Z_LSB		0x26
#define MAG_REG_OFFSET_Z_MSB		0x27

#define MAG_REG_GAIN_X				0x34
#define MAG_REG_GAIN_Y				0x35
#define MAG_REG_GAIN_Z				0x36

#define MAG_REG_SR_CTRL				0x40
#define MAG_REG_LOT_ID_L			0x41

#define MAG_REG_TCS_X				0x47
#define MAG_REG_TCS_Y				0x48
#define MAG_REG_TCS_Z				0x49
#define MAG_REG_TCO_X				0x4a
#define MAG_REG_TCO_Y				0x4b
#define MAG_REG_TCO_Z				0x4c

#define MAG_CROSS_KYZ				0x3A
#define MAG_CROSS_KXZ				0x3B

#define MAG_CROSS_KXY				0x46
#define MAG_CROSS_KYX				0x4E
#define MAG_CROSS_KXX				0x4F
#define MAG_CROSS_KYY				0x50

#define MAG_CROSS_KZZ				0x51
#define MAG_CROSS_KZY				0x52
#define MAG_CROSS_KZX				0x53

#define MAG_STATUS_FIFO_FULL_INT	0x01
#define MAG_STATUS_FIFO_WMK_INT		0x02
#define MAG_STATUS_FIFO_OVL_INT		0x04

#define MAG_OK						1
#define MAG_FAIL					0

#define QST_ABS(X) 					((X) < 0 ? (-1 * (X)) : (X))
#define QST_FABS(X) 				((X) < 0.0f ? (-1.0f * (X)) : (X))

/*
typedef enum
{
	INTERFACE_I2C_SW = 0,
	INTERFACE_I2C_HW,
	INTERFACE_I2C_HW_1M,
	INTERFACE_I3C_400,
	INTERFACE_I3C_625,
	INTERFACE_I3C_1250,
	INTERFACE_SPI,

	INTERFACE_TOTAL
} qst_sensor_interface;
*/

typedef enum
{
	TEST_POLL_DATA_MANUAL = 0,
	TEST_POLL_DATA_AUTO,
	TEST_POLL_FIFO_MANUAL,
	TEST_POLL_FIFO_AUTO,
	TEST_POLL_ODR,

	TEST_WRITE_READ_REGISTER,
	TEST_WORK_CURRENT,
	TEST_WORK_MODE_SWITCH,
	TEST_SETRESET_SWITCH,
	TEST_SELFTEST,
	TEST_SOFT_RESET,
	TEST_OTP,
	TEST_FACTORY,

	TEST_IBI_TO_DRDY,
	TEST_IBI_TO_FIFO_FULL,
	TEST_IBI_TO_FIFO_WMK,
	TEST_IBI_TO_DATA_OVL,
	TEST_IBI_TO_SELFTEST,
	TEST_I3C_CCC,

#if defined(FPGA)
	TEST_GAIN_OFFSET,	// fpga only
	TEST_TCO,			// fpga only
	TEST_TCS,			// fpga only
	TEST_KMTX,			// fpga only
#endif
	
	TEST_MISC,
	TEST_MAX
} qst_sensor_test_item;

typedef enum
{
	MAG_FIFO_MODE_BYPASS = (0<<6),
	MAG_FIFO_MODE_FIFO = (1<<6),
	MAG_FIFO_MODE_STREAM = (2<<6),
	MAG_FIFO_MODE_DEFAULT = (3<<6)
} mag_fifo_mode;

typedef union
{
	struct
	{
		signed char t0:5;	
		signed char rev:3;
	};
	unsigned char value;
} qst_t0_t;

typedef union
{
	struct
	{
		unsigned char sr_half:1;
		unsigned char sr_slop:2;
		unsigned char sr_pulse:2;
		unsigned char io_pwr_on:1;
		unsigned char high_pwr_ldo:1;
		unsigned char io_set_vbat:1;
	};
	unsigned char value;
} reg_40_t;

typedef union
{
	struct
	{
		signed char rev:2;	
		signed char kc_xiyo:6;
	};
	unsigned char value;
} reg_46_t;

typedef union
{
	struct
	{
		signed char rev:2;	
		signed char ky:6;
	};
	unsigned char value;
} reg_3a_t;

typedef union
{
	struct
	{
		signed char rev:2;	
		signed char kx:6;
	};
	unsigned char value;
} reg_3b_t;


typedef struct
{
	unsigned char	mode;
	unsigned char	odr;
	unsigned int	freq;
	short			delay;
//	unsigned int	sample;	
//	unsigned int	ibi_sample;
} mag_odr_t;


typedef struct
{
	const unsigned char		*range;
	int						range_num;
	const mag_odr_t			*odr;
	int						odr_num;
	const unsigned char		*osr1;
	int						osr1_num;
	const unsigned char		*osr2;
	int						osr2_num;
	const unsigned char		*sr;
	int						sr_num;
}evb_mag_set_t;

typedef struct
{
	int 				index;
	char				support;
	char *				info;
}mag_item_info;

typedef struct
{
	int					wait_cfg;

	int 				mag_sensor;	
	int					mag_num;
	evb_mag_set_t		set;

	int					sel_mag_i;
	int 				sel_inf;
	int 				sel_test_i;
	int					sel_range;
	int 				sel_odr;
	int 				sel_osr1;
	int 				sel_osr2;
	int					sel_zdbl;
	int					sel_sr;	
	int					sel_auto_test;
	int 				sel_delay;

	int					fifo_en;
	int 				ibi_en;
	int 				st_en;

	unsigned short 		config_id;

	unsigned char		chipid;	
	unsigned char		v_id;
	unsigned char		w_id;
	unsigned short		d_id;
	unsigned short		l_id;
	reg_40_t			val_40;
	unsigned char		reg_a;
	unsigned char		reg_b;
	unsigned char		reg_c;
	unsigned char		status1;	// 09h
	unsigned char		status2;	// 20h
	unsigned char		fifo_mode;
	unsigned char		fifo_wmk;	
	unsigned char		fifo_full;
	unsigned char		fifo_ctrl;	
	unsigned char		fifo_size;
	unsigned char		fifo_lv_shift;
	unsigned char		reg_max;
	unsigned char		reg;

	short				raw_data[3][3];
	short				raw_temp[3];

	unsigned int 		data_i;
	unsigned int 		data_max;
	unsigned int		data_fail;
	unsigned short 		delay;	
	unsigned int		set_reset_num;

	unsigned short		drdy_fail_num;
	unsigned char		exit_flag;
	unsigned char		otp_bypass;
	unsigned char		user_set_reg;

	const mag_item_info		*item;
} qst_mag_test_t;

typedef struct
{
	unsigned char			mode;
	unsigned int			count;
	unsigned int			count2;

	unsigned char			swich_mode;
	int						switch_count;
	int						switch_threshold;
	int						switch_suspend_delay;
	int						one_sr_delay;
} mag_sr_ctl_t;

typedef struct
{
	float			mag[3];
	float			std[3];	
	float			std_avg[3];
	float			selftest[3];
	short			raw[3];
	short			offset[3];
	short			temp_raw;
	short			mag_lsb;
	short			temp_lsb;
	float			kx;
	float			ky;
	mag_sr_ctl_t	sr_ctl;
} qst_mag_out_t;

typedef void (*evb_mag_write)(unsigned char addr, unsigned char data);
typedef void (*evb_mag_read)(unsigned char addr, unsigned char *data, unsigned short len);

void qst_evb_init_port(int sel_inf);
int qst_evb_mag_write_reg(unsigned char addr, unsigned char data);
int qst_evb_mag_read_reg(unsigned char addr, unsigned char *data, unsigned short len);
void qst_evb_mag_soft_reset(void);
void qst_evb_mag_test_entry(int interface);

#endif


