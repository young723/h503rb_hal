#ifndef __QMC6309V_H
#define __QMC6309V_H

#include "bsp_hardware.h"

#define QMC6309V_RECOVER

#define QMC6309V_OK			1
#define QMC6309V_FAIL		0

#define QMC6309V_IIC_ADDR				0x0c
//#define QMC6309V_0X40_CFG				// vdd=3.3v open
#define QMC6309V_X7

/* chip id*/
#define QMC6309V_CHIP_ID				0x91

/*data output register*/
#define QMC6309V_CHIP_ID_REG			0x00
#define QMC6309V_DATA_OUT_X_LSB_REG		0x01
#define QMC6309V_DATA_OUT_X_MSB_REG		0x02
#define QMC6309V_DATA_OUT_Y_LSB_REG		0x03
#define QMC6309V_DATA_OUT_Y_MSB_REG		0x04
#define QMC6309V_DATA_OUT_Z_LSB_REG		0x05
#define QMC6309V_DATA_OUT_Z_MSB_REG		0x06
#define QMC6309V_TEMP_OUT_LSB_REG		0x07
#define QMC6309V_TEMP_OUT_MSB_REG		0x08

#define QMC6309V_DATA_OUT_ST_X			0x13

/*Status registers */
#define QMC6309V_STATUS_REG				0x09
/* configuration registers */
#define QMC6309V_CTL_REG_ONE			0x0A  /* Contrl register one */
#define QMC6309V_CTL_REG_TWO			0x0B  /* Contrl register two */
#define QMC6309V_CTL_REG_THREE			0x0C
#define QMC6309V_CTL_IBI				0x21

#define QMC6309V_FIFO_REG_STATUS		0x20
#define QMC6309V_FIFO_REG_CTRL			0x2E
#define QMC6309V_FIFO_REG_DATA			0x2F

#define QMC6309V_SET_RESET_ON 			0
#define QMC6309V_SET_ON 				1
#define QMC6309V_RESET_ON 				2
#define QMC6309V_SET_RESET_OFF 			3

#define QMC6309V_RNG_32G				0
#define QMC6309V_RNG_16G				1
#define QMC6309V_RNG_8G					2

#define QMC6309V_MODE_SUSPEND			0
#define QMC6309V_MODE_NORMAL			1
#define QMC6309V_MODE_SINGLE			2
#define QMC6309V_MODE_HPFM				3

#define QMC6309V_ODR_HPFM				0
#define QMC6309V_ODR_1HZ				0
#define QMC6309V_ODR_10HZ				1
#define QMC6309V_ODR_50HZ				2
#define QMC6309V_ODR_100HZ				3
#define QMC6309V_ODR_200HZ				4

// CIC filter OSR: 512/256/128/64(0~3)
#define QMC6309V_OSR1_8					0
#define QMC6309V_OSR1_4					1
#define QMC6309V_OSR1_2					2
#define QMC6309V_OSR1_1					3

// Moving average depth normal mode the depth:/8/4/2/1(/3/2/1/0). Single Mode also use this filter.	
#define QMC6309V_OSR2_1					0
#define QMC6309V_OSR2_2					1
#define QMC6309V_OSR2_4					2
#define QMC6309V_OSR2_8					3

//ZDBL_ENB	0: OSR1_Z=2*OSR_XY   1:OSR1_Z=OSR_XY
#define QMC6309V_ZDBL_ENB_ON			1
#define QMC6309V_ZDBL_ENB_OFF			0

#define QMC6309V_STATUS_DRDY			0x01
#define QMC6309V_STATUS_OVFL			0x02

#define QMC6309V_FIFO_STATUS_OR			0x04
#define QMC6309V_FIFO_STATUS_WMK		0x02
#define QMC6309V_FIFO_STATUS_FULL		0x01

#define QMC6309V_SELFTEST_MAX_X			(50)
#define QMC6309V_SELFTEST_MIN_X			(1)
#define QMC6309V_SELFTEST_MAX_Y			(50)
#define QMC6309V_SELFTEST_MIN_Y			(1)
#define QMC6309V_SELFTEST_MAX_Z			(50)
#define QMC6309V_SELFTEST_MIN_Z			(1)

#define QMC6309V_ABS(X) 				((X) < 0 ? (-1 * (X)) : (X))
#define QMC6309V_ABSF(X) 				((X) < 0.0f ? (-1.0 * (X)) : (X))

#define QMC6309V_LOG					qst_logi
#define QMC6309V_CHECK_ERR(ret)	do {\
									if((ret) != QMC6309V_OK)	\
									QMC6309V_LOG("qmc6309v error:%d line:%d\r\n",ret, __LINE__);	\
									}while(0)

//#define QMC6309V_MODE_SWITCH

typedef enum
{
	QMC6309V_FIFO_MODE_BYPASS = (0<<6),
	QMC6309V_FIFO_MODE_FIFO = (1<<6),
	QMC6309V_FIFO_MODE_STREAM = (2<<6),
	QMC6309V_FIFO_MODE_DEFAULT = (3<<6)
} qmc6309v_fifo_mode;


typedef enum
{
	QMC6309V_IBI_OFF = 0x00,
	QMC6309V_IBI_DRDY = 0x01,
	QMC6309V_IBI_OVFL = 0x02,
	QMC6309V_IBI_ST_RDY = 0x04,
	QMC6309V_IBI_FIFO_FULL = 0x08,
	QMC6309V_IBI_FIFO_WMK = 0x10,
} qmc6309v_fifo_ibi;

typedef struct
{
	char			mode;
	unsigned short	count;
} qmc6309v_set_ctl_t;

typedef union
{
	struct
	{
		unsigned char mode:2;
		unsigned char zdbl_enb:1;
		unsigned char osr1:2;
		unsigned char osr2:2;
		unsigned char rev:1;
	}bit;
	unsigned char value;
} qmc6309v_ctrlreg1;

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
} qmc6309v_ctrlreg2;

typedef union
{
	struct
	{
		unsigned char nvm_bank_num:1;
		unsigned char rev1:1;
		unsigned char bank_scan_done:1;
		unsigned char osr1_z:2;
		unsigned char rev2:3;
	}bit;
	unsigned char value;
} qmc6309v_ctrlreg3;


//typedef union
//{
//	struct
//	{
//		unsigned char sr_half:1;
//		unsigned char sr_slop_ctl:2;
//		unsigned char sr_pulse_ctl:2;
//		unsigned char bgacttrim:1;
//		unsigned char io_1p8:1;
//		unsigned char io_vbat:1;
//	}bit;
//	unsigned char value;
//} qmc6309_reg40;


typedef struct
{
	char					protocol;
	unsigned char			slave_addr;
	unsigned short			ssvt;
	short					last_data[3];

	unsigned char			ctl1_val;
	unsigned char			ctl2_val;
	unsigned char			ctl3_val;
	unsigned char			fifo_ctrl;
	unsigned short			fail_num;
#if defined(QMC6309V_MODE_SWITCH)
	qmc6309v_set_ctl_t		set_ctl;
#endif
}qmc6309v_data_t;


int qmc6309v_init(int protocol);
int qmc6309v_disable(void);
int qmc6309v_enable(void);
void qmc6309v_soft_reset(void);
#if defined(QMC6309V_RECOVER)
int qmc6309v_recover(void);
#endif
void qmc6309v_dump_reg(void);
int qmc6309v_read_mag_xyz(float uT[3]);
int qmc6309v_read_mag_raw(short data[3]);
#if defined(QMC6309V_MODE_SWITCH)
void qmc6309v_setrst_auto_mode(short hw_d[3]);
#endif

int qmc6309v_self_test(void);
int qmc6309v_fifo_config(qmc6309v_fifo_mode mode, unsigned char wmk);
int qmc6309v_fifo_read(unsigned char *f_data);
int qmc6309v_enable_ibi(qmc6309v_fifo_ibi flag);

#endif

