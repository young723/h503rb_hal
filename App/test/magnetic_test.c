/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    I3C/I3C_Controller_InBandInterrupt_IT/Src/main.c
  * @author  MCD Application Team
  * @brief This sample code shows how to use STM32H5xx I3C HAL API to
  *        manage an In-Band-Interrupt procedure between a Controller
  *        and Targets with a communication process based on Interrupt transfer.
  *        The communication is done using 2 or 3 Boards.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include <string.h>
#include "magnetic_test.h"

#define MAG_CHECK_ERR(ret)	do {\
								if((ret) != 1)	\
								{ \
									qst_logi("mag i2c/i3c error\r\n");	\
								} \
							}while(0)

#define OUT_F6				"%.1f,%.1f,%.1f,std,%.2f,%.2f,%.2f"
#define OUT_F3				"%.1f,%.1f,%.1f"
#define OUT_D3 				"%d,%d,%d\r\n"
//#define OUT_F3_T			"%.2f,%.2f,%.2f,%d\r\n"

#define HPF_DELAY			5
#define N200_DELAY			6
#define N100_DELAY			11

#define N50_DELAY			(N100_DELAY*2)
#define N20_DELAY			(N100_DELAY*5)
#define N10_DELAY			(N100_DELAY*10)
#define N1_DELAY			(N100_DELAY*100)
#define SINGLE_DELAY		20
#define ONETEST_DURATION	10		// second

#if defined(MAG_SOFT_COMPENSATE)
const float evb_softmag[3][3]={
    {1.0,  0.0,  0.0},
    {0.0,  1.0,  0.0},
    {0.0,  0.0,  1.0}
};
#endif

static unsigned char mag_slave = 0x0c;
const unsigned char mag_slave_array[] = {0x0c, 0x7c, 0x2c, 0x1c, 0x3c, 0x4c, 0x5c, 0x6c};

const mag_item_info interface_array[INTERFACE_TOTAL] = 
{
	{INTERFACE_I2C_SW,		1,	"I2C-SW"},
#if defined(MAG_TEST_HW_I2C_SUPPORT)	
	{INTERFACE_I2C_HW,		1,	"I2C-HW400K"},
	{INTERFACE_I2C_HW_1M,	1,	"I2C-HW1M"},
#else
	{INTERFACE_I2C_HW,		0,	"I2C-HW400K"},
	{INTERFACE_I2C_HW_1M,	0,	"I2C-HW1M"},
#endif
#if defined(MAG_TEST_I3C_SUPPORT)
	{INTERFACE_I3C_4M,		1,	"I3C-4.0M"},
	{INTERFACE_I3C_6_25M,	1,	"I3C-6.25M"},
	{INTERFACE_I3C_10M, 	1,	"I3C-10.0M"},
	{INTERFACE_I3C_12_5M,	1,	"I3C-12.5M"},
#else
	{INTERFACE_I3C_4M,		0,	"I3C-4.0M"},
	{INTERFACE_I3C_6_25M,	0,	"I3C-6.25M"},
	{INTERFACE_I3C_10M, 	0,	"I3C-10.0M"},
	{INTERFACE_I3C_12_5M,	0,	"I3C-12.5M"},
#endif
	{INTERFACE_SPI_HW4, 	0,	"SPI-HW-4WIRE"},
	{INTERFACE_SPI_HW3, 	0,	"SPI-HW-3WIRE"},
	{INTERFACE_SPI_SW4, 	0,	"SPI-SW-4WIRE"},
	{INTERFACE_SPI_SW3, 	0,	"SPI-SW-3WIRE"},
};

const mag_item_info mag_test_item[] = 
{	
	{TEST_POLL_DATA_MANUAL,		1,	"poll data"},
	{TEST_POLL_DATA_AUTO,		1,	"poll data auto"},
	{TEST_POLL_FIFO_MANUAL,		1,	"poll fifo"},
	{TEST_POLL_FIFO_AUTO,		1,	"poll fifo auto"},
	{TEST_POLL_ODR,				1,	"test odr"},
	{TEST_WRITE_READ_REGISTER,	1,	"write/read reg"},	
	{TEST_WORK_CURRENT,			1,	"current"},
	{TEST_WORK_MODE_SWITCH,		1,	"mode switch"},
	{TEST_SETRESET_SWITCH,		1,	"set/reset switch"},
	{TEST_SELFTEST,				1,	"self test"},
	{TEST_SOFT_RESET,			1,	"soft reset"},
	{TEST_OTP,					1,	"otp test"},
	{TEST_FACTORY,				1,	"factory"},
#if defined(MAG_TEST_I3C_SUPPORT)
	{TEST_IBI_TO_DRDY,			1,	"ibi drdy"},
	{TEST_IBI_TO_FIFO_FULL,		1,	"ibi fifo full"},
	{TEST_IBI_TO_FIFO_WMK,		1,	"ibi fifo wmk"},
	{TEST_IBI_TO_DATA_OVL,		1,	"ibi ovl"},
	{TEST_IBI_TO_SELFTEST,		1,	"ibi selftest"},
	{TEST_I3C_CCC,				1,	"i3c ccc"},
#endif
#if defined(FPGA)
	{TEST_GAIN_OFFSET,			1,	"gain offset"},
	{TEST_TCO					1,	"tco"},
	{TEST_TCS					1,	"tcs"},
	{TEST_KMTX					1,	"kmtx"},
#endif
	{TEST_MISC,					1,	"misc"},
	{TEST_MAX,					0,	"max"}
};

const mag_item_info mag_test_item_qmc6308[] = 
{	
	{TEST_POLL_DATA_MANUAL,		1,	"poll data"},
	{TEST_WRITE_READ_REGISTER,	1,	"write/read reg"},	
	//{TEST_WORK_CURRENT,			1,	"current"},
	{TEST_SELFTEST,				1,	"self test"},
	{TEST_SOFT_RESET,			1,	"soft reset"},
	//{TEST_FACTORY,				1,	"factory"},
	{TEST_MAX,					0,	"max"}
};

const short mag_svvt_array[] = {1000, 2000, 4000};

const mag_odr_t qmc6309_odr[] = 
{
	{QMC6309_MODE_HPFM, 	QMC6309_ODR_HPFM, 	250,	HPF_DELAY,		},
	{QMC6309_MODE_NORMAL, 	QMC6309_ODR_200HZ, 	200,	N200_DELAY,		},
	{QMC6309_MODE_NORMAL,	QMC6309_ODR_100HZ,	100,	N100_DELAY,		},
	{QMC6309_MODE_NORMAL, 	QMC6309_ODR_50HZ, 	50,		N50_DELAY,		},
	{QMC6309_MODE_NORMAL, 	QMC6309_ODR_10HZ, 	10,		N10_DELAY,		},
	{QMC6309_MODE_NORMAL, 	QMC6309_ODR_1HZ,	1,		N1_DELAY,		},
	{QMC6309_MODE_SINGLE,	QMC6309_ODR_HPFM,	0,		SINGLE_DELAY,	},
	{QMC6309_MODE_SUSPEND,	QMC6309_ODR_HPFM,	0,		SINGLE_DELAY,	}
};
const unsigned char qmc6309_range[] = {QMC6309_RNG_32G, QMC6309_RNG_16G, QMC6309_RNG_8G};	//{QMC6309_RNG_32G, QMC6309_RNG_16G, QMC6309_RNG_8G};
const unsigned char qmc6309_osr1[] = {QMC6309_OSR1_1, QMC6309_OSR1_2, QMC6309_OSR1_4, QMC6309_OSR1_8};
const unsigned char qmc6309_osr2[] = {QMC6309_OSR2_1, QMC6309_OSR2_2, QMC6309_OSR2_4, QMC6309_OSR2_8, QMC6309_OSR2_16};
const unsigned char qmc6309_sr[] = {QMC6309_SET_RESET_ON, QMC6309_SET_ON, QMC6309_RESET_ON, QMC6309_SET_RESET_OFF};


const mag_odr_t qmc6309v_odr[] = 
{
	{QMC6309V_MODE_HPFM, 	QMC6309V_ODR_HPFM, 	250,	HPF_DELAY,		},
	{QMC6309V_MODE_NORMAL, 	QMC6309V_ODR_200HZ, 200,	N200_DELAY,		},
	{QMC6309V_MODE_NORMAL,	QMC6309V_ODR_100HZ,	100,	N100_DELAY,		},
	{QMC6309V_MODE_NORMAL, 	QMC6309V_ODR_50HZ, 	50,		N50_DELAY,		},
	{QMC6309V_MODE_NORMAL, 	QMC6309V_ODR_10HZ, 	10,		N10_DELAY,		},
	{QMC6309V_MODE_NORMAL, 	QMC6309V_ODR_1HZ,	1,		N1_DELAY,		},
	{QMC6309V_MODE_SINGLE,	QMC6309V_ODR_HPFM,	0,		SINGLE_DELAY,	},
	{QMC6309V_MODE_SUSPEND,	QMC6309V_ODR_HPFM,	0,		SINGLE_DELAY,	}
};
const unsigned char qmc6309v_range[] = {QMC6309V_RNG_32G, QMC6309V_RNG_16G, QMC6309V_RNG_8G};
const unsigned char qmc6309v_osr1[] = {QMC6309V_OSR1_1, QMC6309V_OSR1_2, QMC6309V_OSR1_4, QMC6309V_OSR1_8};
const unsigned char qmc6309v_osr2[] = {QMC6309V_OSR2_1, QMC6309V_OSR2_2, QMC6309V_OSR2_4, QMC6309V_OSR2_8};
const unsigned char qmc6309v_sr[] = {QMC6309V_SET_RESET_ON, QMC6309V_SET_ON, QMC6309V_RESET_ON, QMC6309V_SET_RESET_OFF};


const mag_odr_t maestro_odr[] = 
{
	{MAESTRO_MODE_HPFM, 	MAESTRO_ODR_HPF,	1500,	HPF_DELAY,		},
	{MAESTRO_MODE_NORMAL, 	MAESTRO_ODR_1000HZ, 1000,	HPF_DELAY,		},
	{MAESTRO_MODE_NORMAL, 	MAESTRO_ODR_400HZ, 	400,	HPF_DELAY,		},
	{MAESTRO_MODE_NORMAL, 	MAESTRO_ODR_200HZ, 	200,	N200_DELAY,		},
	{MAESTRO_MODE_NORMAL,	MAESTRO_ODR_100HZ,	100,	N100_DELAY,		}, 
	{MAESTRO_MODE_NORMAL, 	MAESTRO_ODR_50HZ, 	50,		N50_DELAY,		},
	{MAESTRO_MODE_NORMAL, 	MAESTRO_ODR_20HZ, 	20,		N20_DELAY,		},
	{MAESTRO_MODE_NORMAL, 	MAESTRO_ODR_10HZ, 	10,		N10_DELAY,		},
	{MAESTRO_MODE_NORMAL, 	MAESTRO_ODR_1HZ,	1,		N1_DELAY,		},
	{MAESTRO_MODE_SINGLE,	MAESTRO_ODR_HPF,	0,		SINGLE_DELAY,	},
	{MAESTRO_MODE_SUSPEND,	MAESTRO_ODR_HPF,	0,		SINGLE_DELAY,	}
};

const unsigned char maestro_range[] = {MAESTRO1V_RNG_20G};
const unsigned char maestro_osr1[] = {MAESTRO1V_OSR1_1, MAESTRO1V_OSR1_2, MAESTRO1V_OSR1_4, MAESTRO1V_OSR1_8, MAESTRO1V_OSR1_16, MAESTRO1V_OSR1_32};
const unsigned char maestro_osr2[] = {MAESTRO_OSR2_1, MAESTRO_OSR2_2, MAESTRO_OSR2_4, MAESTRO_OSR2_8};
const unsigned char maestro_sr[] = {MAESTRO_SET_RESET_OFF, MAESTRO_SET_ON, MAESTRO_RESET_ON};


#if defined(QMC6308)
#define QMC6308_NORMAL_MODE_MAX_ODR 	200
const mag_odr_t qmc6308_odr[] = 
{
	{QMC6308_MODE_CONTINUOUS, 	0, 				280,	HPF_DELAY,		},
	{QMC6308_MODE_NORMAL, 	QMC6308_ODR_200HZ, 	200,	N200_DELAY,		},
	{QMC6308_MODE_NORMAL,	QMC6308_ODR_100HZ,	100,	N100_DELAY,		},
	{QMC6308_MODE_NORMAL, 	QMC6308_ODR_50HZ, 	50,		N50_DELAY,		},
	{QMC6308_MODE_NORMAL, 	QMC6308_ODR_10HZ, 	10,		N10_DELAY,		},
};
const unsigned char qmc6308_range[] = {QMC6308_RNG_30G,QMC6308_RNG_12G, QMC6308_RNG_8G, QMC6308_RNG_2G};	// , QMC6308_RNG_12G, QMC6308_RNG_8G, QMC6308_RNG_2G
const unsigned char qmc6308_osr1[] = {QMC6308_OSR1_1, QMC6308_OSR1_2, QMC6308_OSR1_4, QMC6308_OSR1_8};
const unsigned char qmc6308_osr2[] = {QMC6308_OSR2_1, QMC6308_OSR2_2, QMC6308_OSR2_4, QMC6308_OSR2_8};
const unsigned char qmc6308_sr[] = {QMC6308_SET_RESET_ON, QMC6308_SET_ON, QMC6308_SET_RESET_OFF};
//const short qmc6308_svvt[] = {1000, 2500, 3750, 15000};
#endif


static QstStd3				mStd3;
static qst_mag_test_t		t_mag;
static qst_mag_out_t		g_m_out;
#define QST_BUF_LEN			256
static unsigned char		g_buf[QST_BUF_LEN];
static char					g_config_info[128];
static char					g_chip_info[50];

static unsigned char		g_reg_tbl[132];
static maestro_pset			pset;

#if defined(FPGA)
	#if defined(MAESTRO1)
static unsigned short		mag_gain[3] = {512, 512, 512};			// 0 ~ 1023	(0 ~ 2)
	#else
static signed char			mag_gain[3] = {0, 0, 0};				// -128 ~ 127	(0.5 ~ 1.5x)
	#endif	
const unsigned char 		mag_coarse_gain[4] = {0, 1, 2, 3};		// 0 ~ 3	XY(1x/2x/4x/0.5x) Z(1x/2x/4x/8x)

static short				mag_offset[3] = {0, 0, 0};				// -32768 ~ 32767
static signed char			mag_tco[3] = {0, 0, 0};		 			// -128 ~ 127
static signed char			mag_tcs[3] = {0, 0, 0};					// -128 ~ 127
static int					mag_otp_tco_in = 0;						// -128 ~ 127
static int					mag_otp_tcs_in = 0;						// -128 ~ 127
static signed char			mag_kmtx_xy[4] = {0, 0, 0, 0};			// -32 ~ 31/-128 ~ 127
static signed char			mag_kmtx_xy_z[2] = {0, 0};				// -32 ~ 31/-128 ~ 127
static signed char			g_t0 = 0;	//-32;						// -32 ~ 31

static signed char			mag_kc_xiyo = 0;
static signed char			mag_kx_ky = 0;
//static signed char		mag_ky = 0;

#if defined(MAESTRO1)||defined(QMC6309V_X7)
#define FPGA_T0_MAX			31
#define FPGA_T0_MIN			-32
#else
#define FPGA_T0_MAX			15
#define FPGA_T0_MIN			-16
#endif

static void qst_evb_mag_apply_t0(signed char t0);
#if defined(MAESTRO1)
static void qst_evb_mag_apply_offset_gain(unsigned short	gain[3], short offset[3]);
#else
static void qst_evb_mag_apply_offset_gain(signed char	gain[3], short offset[3]);
#endif
static void qst_evb_mag_apply_tco_tcs(signed char tco[3], signed char tcs[3]);
static void qst_evb_mag_apply_kmtx(bool enable, signed char xy[4], signed char xy_z[2]);

static void qst_evb_mag_tco_test(unsigned int bypass);
static void qst_evb_mag_tcs_test(unsigned int bypass);
#endif
static void qst_mag_auto_sel_range_osr(int sel_odr);
#if defined(MAG_TEST_I3C_SUPPORT)
static void qst_evb_mag_enable_ibi(unsigned char flag);
#endif


void qst_evb_mag_key1_hdlr(void)
{
	t_mag.wait_cfg = 0;
}

void qst_evb_mag_key1_exit_hdlr(void)
{
	t_mag.exit_flag = 1;
}

void qst_evb_mag_uart_rx_hdlr(unsigned char *rx_buf, unsigned short len)
{
	if(strncmp((const char*)rx_buf, "rst", 3) == 0)
	{
		qst_evb_mag_soft_reset();
		NVIC_SystemReset();
	}	
	else if(strncmp((const char*)rx_buf, "exit", 1) == 0)
	{
		t_mag.wait_cfg = 0;
	}
	else if(strncmp((const char*)rx_buf, "start", 5) == 0)
	{
		if(rx_buf[5] == ',')
		{
			sscanf((char *)rx_buf, "start,%d,%d", &t_mag.sel_delay,&t_mag.sel_mag_i);
		}
		t_mag.wait_cfg = 0;
	}
	else if(strncmp((const char*)rx_buf, "ma,", 3) == 0)
	{	
		int fifo_mode, fifo_wmk, data_max;

		sscanf((char *)rx_buf, "ma,%d,%d,%d,%d,%d", &t_mag.sel_test_i, &t_mag.fifo_en, &fifo_mode, &fifo_wmk, &data_max);
		if(data_max > 0)
		{
			t_mag.data_max = data_max;
		}
		t_mag.fifo_mode = (unsigned char)fifo_mode;
		t_mag.fifo_wmk = (unsigned char)fifo_wmk;

	}
	else if(strncmp((const char*)rx_buf, "mb,", 3) == 0)
	{
		sscanf((char *)rx_buf, "mb,%d,%d,%d,%d,%d", &t_mag.sel_odr, &t_mag.sel_range, &t_mag.sel_osr1, &t_mag.sel_osr2, &t_mag.sel_sr);
	}	
	else if(strncmp((const char*)rx_buf, "set", 3) == 0)
	{
		sscanf((char *)rx_buf, "set,%d", &t_mag.set_reset_num);
		void qst_evb_mag_enable(void);

		qst_evb_mag_enable();
	}
	else if(strncmp((const char*)rx_buf, "r,", 2) == 0)
	{
		int reg, value;
		sscanf((char *)rx_buf, "r,0x%x,0x%x", &reg, &value);
		qst_logi("0x%02x=0x%02x\r\n", (unsigned char)reg, (unsigned char)value);
		//g_reg_tbl[reg] = value;
		qst_evb_mag_write_reg((unsigned char)reg, (unsigned char)value);
		t_mag.user_set_reg = 1;
	}
}

int qst_evb_mag_read_reg(unsigned char addr, unsigned char *data, unsigned short len)
{
	int ret = 0;
	int retry = 0;

	while((ret!=1) && (retry++ < 5))
	{
		if((t_mag.sel_inf>=INTERFACE_I3C_4M)&&(t_mag.sel_inf<=INTERFACE_I3C_12_5M))
		{
#if defined(MAG_TEST_I3C_SUPPORT)
			if(0)//(t_mag.mag_sensor == QST_MAG_QMC6309)
			{
				int i=0;
				if(MAG_FIFO_REG_DATA == addr)		// fifo data
				{
					// read data one by one, test
					for(i=0;i<len;i++)
					{
						ret = bsp_i3c_read(addr, &data[i], 1);
						//ret = bsp_i3c_read(addr+i, &data[i], 1);
					}
				}
				else
				{
					for(i=0;i<len;i++)
					{
						ret = bsp_i3c_read(addr+i, &data[i], 1);
					}
				}
			}
			else
			{
				ret = bsp_i3c_read(addr, data, len);
			}
#endif
		}
		else if((t_mag.sel_inf>=INTERFACE_I2C_HW)&&(t_mag.sel_inf<=INTERFACE_I2C_HW_1M))
		{
#if defined(MAG_TEST_HW_I2C_SUPPORT)
			ret = HW_IIC_READ(mag_slave<<1, addr, data, len);
#endif
		}
		else if(t_mag.sel_inf==INTERFACE_SPI_HW4)
		{
			ret = HW_SPI_READ(addr|0x80, data, len);
		}
		else
		{
			ret = qst_sw_readreg(mag_slave<<1, addr, data, len);
		}
		if(ret!=1)
		{
			qst_delay_us(50);
		}
	}

	return ret;
}

int qst_evb_mag_write_reg(unsigned char addr, unsigned char data)
{
	int ret = 0;
	int retry = 0;

	__disable_irq();
	while((ret!=1) && (retry++ < 5))
	{
		if((t_mag.sel_inf>=INTERFACE_I3C_4M)&&(t_mag.sel_inf<=INTERFACE_I3C_12_5M))
		{
#if defined(MAG_TEST_I3C_SUPPORT)
			ret = bsp_i3c_write(addr, data);
#endif
		}
		else if((t_mag.sel_inf>=INTERFACE_I2C_HW)&&(t_mag.sel_inf<=INTERFACE_I2C_HW_1M))
		{
#if defined(MAG_TEST_HW_I2C_SUPPORT)
			ret = HW_IIC_WRITE(mag_slave<<1, addr, data);
#endif
		}
		else if(t_mag.sel_inf==INTERFACE_SPI_HW4)
		{
			ret = HW_SPI_WRITE(addr&0x7f, data);
		}
		else
		{
			ret = qst_sw_writereg(mag_slave<<1, addr, data);
		}
	}
	__enable_irq();

	return ret;
}


#ifdef AK0991X_MMC5603
void qst_evb_mag_other_read_data(void)
{
	if(t_mag.mag_sensor == QST_MAG_AK0991X)
	{
		int res = 0;
		unsigned char mag_data[8];
		short hw_d[3] = {0};
		//float uT[3];
		int t1 = 0;
		unsigned char rdy = 0;

		res = qst_evb_mag_read_reg(AK0991X_REG_ST1, &rdy, 1);
		while(!(rdy & 0x01)&&(t1++ < 3))
		{
			qst_delay_ms(1);
			res = qst_evb_mag_read_reg(AK0991X_REG_ST1, &rdy, 1);
		}
		if(!(rdy & 0x01))
		{
			qst_logi("drdy fail!\n");
			return;
		}
		res = qst_evb_mag_read_reg(AK0991X_REG_HXL, mag_data, 8);
		if(!res)
		{
			qst_logi("ak09919 read data fail!\n");
		}
		if(1)	/*((g_akm.type == AK09917)||(g_akm.type == AK09919))*/
		{
			hw_d[0] = (short)((mag_data[0] << 8) | mag_data[1]);
			hw_d[1] = (short)((mag_data[2] << 8) | mag_data[3]);
			hw_d[2] = (short)((mag_data[4] << 8) | mag_data[5]);
		}
		else
		{
			hw_d[0] = (short)((mag_data[1] << 8) | mag_data[0]);
			hw_d[1] = (short)((mag_data[3] << 8) | mag_data[2]);
			hw_d[2] = (short)((mag_data[5] << 8) | mag_data[4]);
		}

//		uT[0] = (float) hw_d[0]*0.15f;
//		uT[1] = (float) hw_d[1]*0.15f;
//		uT[2] = (float) hw_d[2]*0.15f;
		qst_logi("ak09919 %d	%d	%d\r\n", hw_d[0], hw_d[1], hw_d[2]);
	}
	else if(t_mag.mag_sensor == QST_MAG_MMC5603)
	{

	}
}

void qst_evb_test_other_reg_wr(void)
{
	static unsigned int fail_num[12] = {0};	
	uint8_t reg_value[12];
	int	ret = 0;
	int len = 0;
	char *buf = (char*)g_buf;

	if(t_mag.mag_sensor == QST_MAG_AK0991X)
	{
		#define AK_MAX_REG		6
		const unsigned char 	ak_reg_array[AK_MAX_REG][2] = 
		{
			{0x00, 0x48},
			{0x01, 0x0e},
			{0x02, 0x20},
			{0x03, 0x00},
			{0x30, 0x20},
			{0x31, 0x48}
		};		

		for(int i=0; i<AK_MAX_REG; i++)
		{
			ret = qst_evb_mag_read_reg(ak_reg_array[i][0], &reg_value[i], 1);
			if(ret != 1)
				qst_logi("read reg fail \r\n");
		}

		len = 0;
		len += snprintf(buf+len, 256-len, "%d ", t_mag.data_i++);
		for(int i=0; i<AK_MAX_REG; i++)
		{
			if(reg_value[i] !=	ak_reg_array[i][1])
			{
				fail_num[i]++;
			}
			len += snprintf(buf+len, 256-len, "0x%02x=0x%02x f:%d, ", ak_reg_array[i][0], reg_value[i], fail_num[i]);
		}
		len += snprintf(buf+len, 256-len, "\r\n");
		buf[len] = '\0';
		qst_logi("%s", buf);

		qst_evb_mag_write_reg(ak_reg_array[4][0], ak_reg_array[4][1]);
		qst_evb_mag_write_reg(ak_reg_array[5][0], ak_reg_array[5][1]);
	}
	else if(t_mag.mag_sensor == QST_MAG_MMC5603)
	{
		#define MMC_MAX_REG		5
		const unsigned char 	mmc_reg_array[MMC_MAX_REG][2] = 
		{
			{0x39, 0x10},	
			{0x1A, 0x65},
			{0x1B, 0x65},
			{0x1C, 0x65},
			{0x1D, 0x65},
		};

		for(int i=0; i<MMC_MAX_REG; i++)
		{
			ret = qst_evb_mag_read_reg(mmc_reg_array[i][0], &reg_value[i], 1);
			if(ret != 1)
				qst_logi("read reg fail \r\n");
		}

		len = 0;
		len += snprintf(buf+len, 256-len, "%d ", t_mag.data_i++);
		for(int i=0; i<MMC_MAX_REG; i++)
		{
			if(reg_value[i] !=	mmc_reg_array[i][1])
			{
				fail_num[i]++;
			}
			len += snprintf(buf+len, 256-len, "0x%02x=0x%02x f:%d, ", mmc_reg_array[i][0], reg_value[i], fail_num[i]);
		}
		len += snprintf(buf+len, 256-len, "\r\n");
		buf[len] = '\0';
		qst_logi("%s", buf);
	}
}

void qst_evb_mag_entry_other(void)
{
	mag_slave = 0x0e;
	qst_evb_mag_read_reg(0x01, &t_mag.chipid, 1);
	if(t_mag.chipid == 0x0e)
	{		
		mag_slave = 0x0e;
		t_mag.mag_sensor = QST_MAG_AK0991X;
		qst_logi("test akm09919\r\n");
	
		if(t_mag.sel_test_i < 0)
		{
			while(1)
			{
				qst_logi("Select function:\r\n");
				qst_logi("[0]: test poll data\r\n");
				qst_logi("[5]: test write/read register\r\n");
	
				scanf("%d", &t_mag.sel_test_i);
				if((t_mag.sel_test_i ==TEST_POLL_DATA_MANUAL)||(t_mag.sel_test_i == TEST_WRITE_READ_REGISTER))
				{
					qst_logi("User select function: %d\r\n\r\n", t_mag.sel_test_i);
					break;
				}
				else
				{
					qst_logi("User select function: %d error\r\n\r\n", t_mag.sel_test_i);
				}
			}
		}
	
		switch(t_mag.sel_test_i)
		{
			case TEST_POLL_DATA_MANUAL:
				qst_evb_mag_write_reg(AK0991X_REG_CNTL1, 0x00|AK0991X_ITS_LOW);
				qst_delay_ms(1);
				qst_evb_mag_write_reg(AK0991X_REG_CNTL2, (AK0991X_SDR_ENABLE|AK0991X_MAG_ODR100)&0x7f);
				qst_delay_ms(1);
				evb_setup_timer(TIM2, qst_evb_mag_other_read_data, 10, ENABLE);
				break;
			case TEST_WRITE_READ_REGISTER:
				qst_evb_mag_write_reg(0x30, 0x20);
				qst_delay_ms(2);
				qst_evb_mag_write_reg(0x31, 0x48);
				qst_delay_ms(2);
				evb_setup_timer(TIM2, qst_evb_test_other_reg_wr, 10, ENABLE);
				break;
			default:
				break;
		}
	
		return;
	}

	mag_slave = 0x30;
	qst_evb_mag_read_reg(0x39, &t_mag.chipid, 1);
	if(t_mag.chipid == 0x10)
	{	
		qst_logi("test mmc5633\r\n");
		mag_slave = 0x30;
		t_mag.mag_sensor = QST_MAG_MMC5603;
		qst_evb_mag_write_reg(0x1C, 0x01);
		qst_evb_mag_write_reg(0x1A, 150);
		qst_evb_mag_write_reg(0x1B, 0xA0);
		qst_evb_mag_write_reg(0x1D, 0x10);
		evb_setup_timer(TIM2, qst_evb_test_other_reg_wr, 10, ENABLE);
		return;
	}
	qst_logi("mag communication fail! chipid=0x%02x\r\n", t_mag.chipid);
}
#endif

static int qst_evb_mag_get_maestor_info(void)
{
	int len = 0;
	maestro1_ctrla	ctrl1;
	maestro1_ctrlb	ctrl2;
	
	ctrl1.value = t_mag.reg_a;
	ctrl2.value = t_mag.reg_b;

	memset(g_config_info, 0, sizeof(g_config_info));
	if(t_mag.mag_num > 1)
		len += sprintf(g_config_info+len, "ID-%d ", t_mag.sel_mag_i);
	if(ctrl2.bit.range == MAESTRO1V_RNG_20G)
	{
		g_m_out.mag_lsb = 1000;
		len += sprintf(g_config_info+len, "20G");
	}
	else
	{
		g_m_out.mag_lsb = 1000;
		len += sprintf(g_config_info+len, "?G");
	}
	if(ctrl1.bit.mode == MAESTRO_MODE_HPFM)
	{
		len += sprintf(g_config_info+len, "HPF");
	}
	else if(ctrl1.bit.mode == MAESTRO_MODE_NORMAL)
	{
		if(ctrl2.bit.odr == MAESTRO_ODR_1000HZ)
			len += sprintf(g_config_info+len, "1K");
		else if(ctrl2.bit.odr == MAESTRO_ODR_400HZ)
			len += sprintf(g_config_info+len, "400");
		else if(ctrl2.bit.odr == MAESTRO_ODR_200HZ)
			len += sprintf(g_config_info+len, "200");
		else if(ctrl2.bit.odr == MAESTRO_ODR_100HZ)
			len += sprintf(g_config_info+len, "100");
		else if(ctrl2.bit.odr == MAESTRO_ODR_50HZ)
			len += sprintf(g_config_info+len, "50");
		else if(ctrl2.bit.odr == MAESTRO_ODR_20HZ)
			len += sprintf(g_config_info+len, "20");
		else if(ctrl2.bit.odr == MAESTRO_ODR_10HZ)
			len += sprintf(g_config_info+len, "10");
		else if(ctrl2.bit.odr == MAESTRO_ODR_1HZ)
			len += sprintf(g_config_info+len, "1");
	}
	else if(ctrl1.bit.mode == MAESTRO_MODE_SINGLE)
	{
		len += sprintf(g_config_info+len, "SING");
	}
	else
	{
		len += sprintf(g_config_info+len, "SUSPEND");
	}
	
	len += sprintf(g_config_info+len, " ");
	
	if(ctrl1.bit.osr1 == MAESTRO1V_OSR1_1)
		len += sprintf(g_config_info+len, "1");
	else if(ctrl1.bit.osr1 == MAESTRO1V_OSR1_2)
		len += sprintf(g_config_info+len, "2");
	else if(ctrl1.bit.osr1 == MAESTRO1V_OSR1_4)
		len += sprintf(g_config_info+len, "4");
	else if(ctrl1.bit.osr1 == MAESTRO1V_OSR1_8)
		len += sprintf(g_config_info+len, "8");
	else if(ctrl1.bit.osr1 == MAESTRO1V_OSR1_16)
		len += sprintf(g_config_info+len, "16");
	else if(ctrl1.bit.osr1 == MAESTRO1V_OSR1_32)
		len += sprintf(g_config_info+len, "32");
	else
		len += sprintf(g_config_info+len, "?");
	
	if(ctrl1.bit.osr2 == MAESTRO_OSR2_1)
		len += sprintf(g_config_info+len, "-1");
	else if(ctrl1.bit.osr2 == MAESTRO_OSR2_2)
		len += sprintf(g_config_info+len, "-2");
	else if(ctrl1.bit.osr2 == MAESTRO_OSR2_4)
		len += sprintf(g_config_info+len, "-4");
	else if(ctrl1.bit.osr2 == MAESTRO_OSR2_8)
		len += sprintf(g_config_info+len, "-8");
	else
		len += sprintf(g_config_info+len, "-?");
	
	len += sprintf(g_config_info+len, " ");
	
	if(ctrl2.bit.set_rst == MAESTRO_SET_ON)
		len += sprintf(g_config_info+len, "S1");
	else if(ctrl2.bit.set_rst == MAESTRO_RESET_ON)
		len += sprintf(g_config_info+len, "R1");
	else if(ctrl2.bit.set_rst == MAESTRO_SET_RESET_ON)
		len += sprintf(g_config_info+len, "SR1");
	else
		len += sprintf(g_config_info+len, "SR0");
	
	len += sprintf(g_config_info+len, " ");

	if(t_mag.ibi_en)
	{
		len += sprintf(g_config_info+len, "IBI-");
		if(t_mag.fifo_en)
		{
			if(t_mag.fifo_ctrl & MAESTRO_FIFO_MODE_STREAM)
				len += sprintf(g_config_info+len, "STR-");
			else if(t_mag.fifo_ctrl & MAESTRO_FIFO_MODE_FIFO)
				len += sprintf(g_config_info+len, "FIFO-");
			else
				len += sprintf(g_config_info+len, "BP-");

			if(t_mag.sel_test_i == TEST_IBI_TO_FIFO_FULL)
				len += sprintf(g_config_info+len, "FULL");
			else if(t_mag.sel_test_i == TEST_IBI_TO_FIFO_WMK)
				len += sprintf(g_config_info+len, "WMK%d", t_mag.fifo_wmk);
		}
		else
		{
			if(TEST_IBI_TO_DATA_OVL == t_mag.sel_test_i)
				len += sprintf(g_config_info+len, "OVL");
			else
				len += sprintf(g_config_info+len, "DRDY");
		}
		len += sprintf(g_config_info+len, " ");
	}
	else if(t_mag.fifo_en)
	{
		if(t_mag.fifo_ctrl & MAESTRO_FIFO_MODE_STREAM)
			len += sprintf(g_config_info+len, "STR-");
		else if(t_mag.fifo_ctrl & MAESTRO_FIFO_MODE_FIFO)
			len += sprintf(g_config_info+len, "FIFO-");
		else
			len += sprintf(g_config_info+len, "BP-");

		if(t_mag.fifo_full)
			len += sprintf(g_config_info+len, "FULL");
		else
			len += sprintf(g_config_info+len, "WMK%d", t_mag.fifo_wmk);
		len += sprintf(g_config_info+len, " ");
	}

	return len;
}

static int qst_evb_mag_get_qmc6309x_info(void)
{
	int len = 0;
	qmc6309_ctrlreg1	ctrl1;
	qmc6309_ctrlreg2	ctrl2;

	ctrl1.value = t_mag.reg_a;
	ctrl2.value = t_mag.reg_b;
	memset(g_config_info, 0, sizeof(g_config_info));	
	if(t_mag.mag_num > 1)
		len += sprintf(g_config_info+len, "ID-%d ", t_mag.sel_mag_i);
	if(ctrl2.bit.range == QMC6309_RNG_32G)
	{
		g_m_out.mag_lsb = 1000;
		len += sprintf(g_config_info+len, "32G");
	}
	else if(ctrl2.bit.range == QMC6309_RNG_16G)
	{
		g_m_out.mag_lsb = 2000;
		len += sprintf(g_config_info+len, "16G");
	}
	else if(ctrl2.bit.range == QMC6309_RNG_8G)
	{
		g_m_out.mag_lsb = 4000;
		len += sprintf(g_config_info+len, "8G");
	}
	else
	{
		g_m_out.mag_lsb = 1000;
		len += sprintf(g_config_info+len, "?G");
	}
	//len += sprintf(g_config_info+len, " ");
	if(ctrl1.bit.mode == QMC6309_MODE_HPFM)
	{
		len += sprintf(g_config_info+len, "HPF");
	}
	else if(ctrl1.bit.mode == QMC6309_MODE_NORMAL)
	{
		if(ctrl2.bit.odr == QMC6309_ODR_200HZ)
			len += sprintf(g_config_info+len, "200");
		else if(ctrl2.bit.odr == QMC6309_ODR_100HZ)
			len += sprintf(g_config_info+len, "100");
		else if(ctrl2.bit.odr == QMC6309_ODR_50HZ)
			len += sprintf(g_config_info+len, "50");
		else if(ctrl2.bit.odr == QMC6309_ODR_10HZ)
			len += sprintf(g_config_info+len, "10");
		else if(ctrl2.bit.odr == QMC6309_ODR_1HZ)
			len += sprintf(g_config_info+len, "1");
	}
	else if(ctrl1.bit.mode == QMC6309_MODE_SINGLE)
	{
		len += sprintf(g_config_info+len, "SING");
	}
	else
	{
		len += sprintf(g_config_info+len, "SUSPEND");
	}

	len += sprintf(g_config_info+len, " ");

	if(ctrl1.bit.osr1 == QMC6309_OSR1_1)
		len += sprintf(g_config_info+len, "1");
	else if(ctrl1.bit.osr1 == QMC6309_OSR1_2)
		len += sprintf(g_config_info+len, "2");
	else if(ctrl1.bit.osr1 == QMC6309_OSR1_4)
		len += sprintf(g_config_info+len, "4");
	else if(ctrl1.bit.osr1 == QMC6309_OSR1_8)
		len += sprintf(g_config_info+len, "8");
	else		
		len += sprintf(g_config_info+len, "?");

	if(ctrl1.bit.osr2 == QMC6309_OSR2_1)
		len += sprintf(g_config_info+len, "-1");
	else if(ctrl1.bit.osr2 == QMC6309_OSR2_2)
		len += sprintf(g_config_info+len, "-2");
	else if(ctrl1.bit.osr2 == QMC6309_OSR2_4)
		len += sprintf(g_config_info+len, "-4");
	else if(ctrl1.bit.osr2 == QMC6309_OSR2_8)
		len += sprintf(g_config_info+len, "-8");
	else if(ctrl1.bit.osr2 == QMC6309_OSR2_16)
		len += sprintf(g_config_info+len, "-16");
	else
		len += sprintf(g_config_info+len, "-?");

	len += sprintf(g_config_info+len, " ");
#if 1
	if(ctrl2.bit.set_rst == QMC6309_SET_RESET_ON)
		len += sprintf(g_config_info+len, "SR1");
	else if(ctrl2.bit.set_rst == QMC6309_SET_ON)
		len += sprintf(g_config_info+len, "S1");
	else if(ctrl2.bit.set_rst == QMC6309_RESET_ON)
		len += sprintf(g_config_info+len, "R1");
	else if(ctrl2.bit.set_rst == QMC6309_SET_RESET_OFF)
		len += sprintf(g_config_info+len, "SR0");
	len += sprintf(g_config_info+len, " ");
#endif

//	if(ctrl1.bit.zdbl_enb == QMC6309H_ZDBL_ENB_ON)
//		len += sprintf(g_config_info+len, "ZD1");
//	else
//		len += sprintf(g_config_info+len, "ZD0");
//	len += sprintf(g_config_info+len, " ");

	if(t_mag.ibi_en)
	{
		len += sprintf(g_config_info+len, "IBI-");
		if(t_mag.fifo_en)
		{
			if(t_mag.fifo_ctrl & QMC6309_FIFO_MODE_STREAM)
				len += sprintf(g_config_info+len, "STR-");
			else if(t_mag.fifo_ctrl & QMC6309_FIFO_MODE_FIFO)
				len += sprintf(g_config_info+len, "FIFO-");
			else
				len += sprintf(g_config_info+len, "BP-");

			if(t_mag.sel_test_i == TEST_IBI_TO_FIFO_FULL)
				len += sprintf(g_config_info+len, "FULL");
			else if(t_mag.sel_test_i == TEST_IBI_TO_FIFO_WMK)
				len += sprintf(g_config_info+len, "WMK%d", t_mag.fifo_wmk);
		}
		else
		{
			if(TEST_IBI_TO_DATA_OVL == t_mag.sel_test_i)
				len += sprintf(g_config_info+len, "OVL");
			else
				len += sprintf(g_config_info+len, "DRDY");
		}
		len += sprintf(g_config_info+len, " ");
	}
	else if(t_mag.fifo_en)
	{
		if(t_mag.fifo_ctrl & QMC6309_FIFO_MODE_STREAM)
			len += sprintf(g_config_info+len, "STR-");
		else if(t_mag.fifo_ctrl & QMC6309_FIFO_MODE_FIFO)
			len += sprintf(g_config_info+len, "FIFO-");
		else
			len += sprintf(g_config_info+len, "BP-");

		if(t_mag.fifo_full)
			len += sprintf(g_config_info+len, "FULL");
		else
			len += sprintf(g_config_info+len, "WMK%d", t_mag.fifo_wmk);
		len += sprintf(g_config_info+len, " ");
	}

	return len;
}


static int qst_evb_mag_get_qmc6309v_info(void)
{
	int len = 0;
	qmc6309v_ctrlreg1	ctrl1;
	qmc6309v_ctrlreg2	ctrl2;

	ctrl1.value = t_mag.reg_a;
	ctrl2.value = t_mag.reg_b;
	memset(g_config_info, 0, sizeof(g_config_info));	
	if(t_mag.mag_num > 1)
		len += sprintf(g_config_info+len, "ID-%d ", t_mag.sel_mag_i);
	if(ctrl2.bit.range == QMC6309_RNG_32G)
	{
		g_m_out.mag_lsb = 1000;
		len += sprintf(g_config_info+len, "32G");
	}
	else if(ctrl2.bit.range == QMC6309_RNG_16G)
	{
		g_m_out.mag_lsb = 2000;
		len += sprintf(g_config_info+len, "16G");
	}
	else if(ctrl2.bit.range == QMC6309_RNG_8G)
	{
		g_m_out.mag_lsb = 4000;
		len += sprintf(g_config_info+len, "8G");
	}
	else
	{
		g_m_out.mag_lsb = 1000;
		len += sprintf(g_config_info+len, "?G");
	}
	//len += sprintf(g_config_info+len, " ");
	if(ctrl1.bit.mode == QMC6309_MODE_HPFM)
	{
		len += sprintf(g_config_info+len, "HPF");
	}
	else if(ctrl1.bit.mode == QMC6309_MODE_NORMAL)
	{
		if(ctrl2.bit.odr == QMC6309_ODR_200HZ)
			len += sprintf(g_config_info+len, "200");
		else if(ctrl2.bit.odr == QMC6309_ODR_100HZ)
			len += sprintf(g_config_info+len, "100");
		else if(ctrl2.bit.odr == QMC6309_ODR_50HZ)
			len += sprintf(g_config_info+len, "50");
		else if(ctrl2.bit.odr == QMC6309_ODR_10HZ)
			len += sprintf(g_config_info+len, "10");
		else if(ctrl2.bit.odr == QMC6309_ODR_1HZ)
			len += sprintf(g_config_info+len, "1");
	}
	else if(ctrl1.bit.mode == QMC6309_MODE_SINGLE)
	{
		len += sprintf(g_config_info+len, "SING");
	}
	else
	{
		len += sprintf(g_config_info+len, "SUSPEND");
	}

	len += sprintf(g_config_info+len, " ");

	if(ctrl1.bit.osr1 == QMC6309_OSR1_1)
		len += sprintf(g_config_info+len, "1");
	else if(ctrl1.bit.osr1 == QMC6309_OSR1_2)
		len += sprintf(g_config_info+len, "2");
	else if(ctrl1.bit.osr1 == QMC6309_OSR1_4)
		len += sprintf(g_config_info+len, "4");
	else if(ctrl1.bit.osr1 == QMC6309_OSR1_8)
		len += sprintf(g_config_info+len, "8");
	else		
		len += sprintf(g_config_info+len, "?");

	if(ctrl1.bit.osr2 == QMC6309_OSR2_1)
		len += sprintf(g_config_info+len, "-1");
	else if(ctrl1.bit.osr2 == QMC6309_OSR2_2)
		len += sprintf(g_config_info+len, "-2");
	else if(ctrl1.bit.osr2 == QMC6309_OSR2_4)
		len += sprintf(g_config_info+len, "-4");
	else if(ctrl1.bit.osr2 == QMC6309_OSR2_8)
		len += sprintf(g_config_info+len, "-8");
	else if(ctrl1.bit.osr2 == QMC6309_OSR2_16)
		len += sprintf(g_config_info+len, "-16");
	else
		len += sprintf(g_config_info+len, "-?");

	len += sprintf(g_config_info+len, " ");
#if 1
	if(ctrl2.bit.set_rst == QMC6309_SET_RESET_ON)
		len += sprintf(g_config_info+len, "SR1");
	else if(ctrl2.bit.set_rst == QMC6309_SET_ON)
		len += sprintf(g_config_info+len, "S1");
	else if(ctrl2.bit.set_rst == QMC6309_RESET_ON)
		len += sprintf(g_config_info+len, "R1");
	else if(ctrl2.bit.set_rst == QMC6309_SET_RESET_OFF)
		len += sprintf(g_config_info+len, "SR0");
	len += sprintf(g_config_info+len, " ");
#endif

//	if(ctrl1.bit.zdbl_enb == QMC6309H_ZDBL_ENB_ON)
//		len += sprintf(g_config_info+len, "ZD1");
//	else
//		len += sprintf(g_config_info+len, "ZD0");
//	len += sprintf(g_config_info+len, " ");

	if(t_mag.ibi_en)
	{
		len += sprintf(g_config_info+len, "IBI-");
		if(t_mag.fifo_en)
		{
			if(t_mag.fifo_ctrl & QMC6309_FIFO_MODE_STREAM)
				len += sprintf(g_config_info+len, "STR-");
			else if(t_mag.fifo_ctrl & QMC6309_FIFO_MODE_FIFO)
				len += sprintf(g_config_info+len, "FIFO-");
			else
				len += sprintf(g_config_info+len, "BP-");

			if(t_mag.sel_test_i == TEST_IBI_TO_FIFO_FULL)
				len += sprintf(g_config_info+len, "FULL");
			else if(t_mag.sel_test_i == TEST_IBI_TO_FIFO_WMK)
				len += sprintf(g_config_info+len, "WMK%d", t_mag.fifo_wmk);
		}
		else
		{
			if(TEST_IBI_TO_DATA_OVL == t_mag.sel_test_i)
				len += sprintf(g_config_info+len, "OVL");
			else
				len += sprintf(g_config_info+len, "DRDY");
		}
		len += sprintf(g_config_info+len, " ");
	}
	else if(t_mag.fifo_en)
	{
		if(t_mag.fifo_ctrl & QMC6309_FIFO_MODE_STREAM)
			len += sprintf(g_config_info+len, "STR-");
		else if(t_mag.fifo_ctrl & QMC6309_FIFO_MODE_FIFO)
			len += sprintf(g_config_info+len, "FIFO-");
		else
			len += sprintf(g_config_info+len, "BP-");

		if(t_mag.fifo_full)
			len += sprintf(g_config_info+len, "FULL");
		else
			len += sprintf(g_config_info+len, "WMK%d", t_mag.fifo_wmk);
		len += sprintf(g_config_info+len, " ");
	}

	return len;
}


#if defined(QMC6308)
static int qst_evb_mag_get_qmc6308_info(void)
{
	int len = 0;
	ctrl_reg1	ctrl1;
	ctrl_reg2	ctrl2;

	ctrl1.value = t_mag.reg_a;
	ctrl2.value = t_mag.reg_b;
	memset(g_config_info, 0, sizeof(g_config_info));	
	if(t_mag.mag_num > 1)
		len += sprintf(g_config_info+len, "Id[%d] ", t_mag.sel_mag_i);
	if(ctrl2.bit.range == QMC6308_RNG_30G)
	{
		g_m_out.mag_lsb = 1000;
		len += sprintf(g_config_info+len, "30G");
	}
	else if(ctrl2.bit.range == QMC6308_RNG_12G)
	{
		g_m_out.mag_lsb = 2500;
		len += sprintf(g_config_info+len, "12G");
	}
	else if(ctrl2.bit.range == QMC6308_RNG_8G)
	{
		g_m_out.mag_lsb = 3750;
		len += sprintf(g_config_info+len, "8G");
	}
	else if(ctrl2.bit.range == QMC6308_RNG_2G)
	{
		g_m_out.mag_lsb = 15000;
		len += sprintf(g_config_info+len, "2G");
	}
	else
	{
		g_m_out.mag_lsb = 1000;
		len += sprintf(g_config_info+len, "?G");
	}
	//len += sprintf(g_config_info+len, " ");
	if(ctrl1.bit.mode == QMC6308_MODE_CONTINUOUS)
	{
		len += sprintf(g_config_info+len, "HPF");
	}
	else if(ctrl1.bit.mode == QMC6308_MODE_NORMAL)
	{
		if(ctrl1.bit.odr == QMC6308_ODR_200HZ)
			len += sprintf(g_config_info+len, "200");
		else if(ctrl1.bit.odr == QMC6308_ODR_100HZ)
			len += sprintf(g_config_info+len, "100");
		else if(ctrl1.bit.odr == QMC6308_ODR_50HZ)
			len += sprintf(g_config_info+len, "50");
		else if(ctrl1.bit.odr == QMC6308_ODR_10HZ)
			len += sprintf(g_config_info+len, "10");
		else
			len += sprintf(g_config_info+len, "?");
	}
	else if(ctrl1.bit.mode == QMC6308_MODE_SINGLE)
	{
		len += sprintf(g_config_info+len, "SING");
	}
	else
	{
		len += sprintf(g_config_info+len, "SUSPEND");
	}

	len += sprintf(g_config_info+len, " ");

	if(ctrl1.bit.osr1 == QMC6308_OSR1_1)
		len += sprintf(g_config_info+len, "1");
	else if(ctrl1.bit.osr1 == QMC6308_OSR1_2)
		len += sprintf(g_config_info+len, "2");
	else if(ctrl1.bit.osr1 == QMC6308_OSR1_4)
		len += sprintf(g_config_info+len, "4");
	else if(ctrl1.bit.osr1 == QMC6308_OSR1_8)
		len += sprintf(g_config_info+len, "8");
	else		
		len += sprintf(g_config_info+len, "?");

	if(ctrl1.bit.osr2 == QMC6308_OSR2_1)
		len += sprintf(g_config_info+len, "-1");
	else if(ctrl1.bit.osr2 == QMC6308_OSR2_2)
		len += sprintf(g_config_info+len, "-2");
	else if(ctrl1.bit.osr2 == QMC6308_OSR2_4)
		len += sprintf(g_config_info+len, "-4");
	else if(ctrl1.bit.osr2 == QMC6308_OSR2_8)
		len += sprintf(g_config_info+len, "-8");
	else
		len += sprintf(g_config_info+len, "-?");

	len += sprintf(g_config_info+len, " ");
#if 1
	if(ctrl2.bit.setrst == QMC6308_SET_RESET_ON)
		len += sprintf(g_config_info+len, "SR1");
	else if(ctrl2.bit.setrst == QMC6308_SET_ON)
		len += sprintf(g_config_info+len, "S1");
	else if(ctrl2.bit.setrst == QMC6308_SET_RESET_OFF)
		len += sprintf(g_config_info+len, "SR0");
	len += sprintf(g_config_info+len, " ");
#endif

	return len;
}
#endif

static void qst_evb_mag_get_config_info(void)
{
	int len = 0;
	int len_ext = 0;

	(void)(len_ext);
	(void)(len);
	memset(g_chip_info, 0, sizeof(g_chip_info));
	len_ext = sprintf(g_chip_info, "0x%02x 0x%02x 0x%02x 0x%04x %s ", t_mag.chipid, t_mag.v_id, t_mag.w_id, t_mag.d_id, interface_array[t_mag.sel_inf].info);

	if((t_mag.mag_sensor==QST_MAG_MAESTRO_1V)||(t_mag.mag_sensor==QST_MAG_MAESTRO_2H))
	{
		len = qst_evb_mag_get_maestor_info();
	}
	else if(t_mag.mag_sensor==QST_MAG_QMC6309)
	{
		len = qst_evb_mag_get_qmc6309x_info();
	}
	else if(t_mag.mag_sensor==QST_MAG_QMC6309V)
	{
		len = qst_evb_mag_get_qmc6309v_info();
	}
#if defined(QMC6308)
	else if(t_mag.mag_sensor==QST_MAG_QMC6308)
	{
		len = qst_evb_mag_get_qmc6308_info();
	}
#endif
	else
	{
		len += sprintf(g_config_info+len, "unknow");
	}

	switch(t_mag.sel_test_i)
	{
		case TEST_FACTORY:
			return;
		default:
			break;
	}
#if defined(QST_MAG_USE_SWJ)
//	qst_logi("$:$%c%s\r\n", (uint8_t)(len-1), g_config_info);
	qst_logi("$:$%c%s%s\r\n", (uint8_t)(len_ext), g_chip_info,g_config_info);
	qst_logi("$:$%c%s%s\r\n", (uint8_t)(len_ext), g_chip_info,g_config_info);
#endif
}

void qst_evb_mag_calc_misc(void)
{
	t_mag.delay = t_mag.set.odr[t_mag.sel_odr].delay;

	if(t_mag.fifo_en)
	{
		if(t_mag.fifo_wmk <= 0)
		{
			qst_logi("error t_mag.fifo_wmk = %d set default value=1\r\n", t_mag.fifo_wmk);
			t_mag.fifo_wmk = 1;
		}
		if(t_mag.set.odr[t_mag.sel_odr].freq)
		{
			if(t_mag.fifo_full)
				t_mag.delay = (1000*(t_mag.fifo_size)/t_mag.set.odr[t_mag.sel_odr].freq)+3;
			else
				t_mag.delay = (1000*(t_mag.fifo_wmk)/t_mag.set.odr[t_mag.sel_odr].freq)+3;
		}
	}
	else
	{
		if((t_mag.mag_sensor==QST_MAG_MAESTRO_1V)||(t_mag.mag_sensor==QST_MAG_MAESTRO_2H))
		{
			if(t_mag.set.odr[t_mag.sel_odr].mode == MAESTRO_MODE_SINGLE)
			{
				if(t_mag.set.osr1[t_mag.sel_osr1] == MAESTRO_OSR1_128)
				{
					t_mag.delay = 50;
				}
			}
		}
		else if(t_mag.mag_sensor==QST_MAG_QMC6309V)
		{
			if(t_mag.set.odr[t_mag.sel_odr].mode == QMC6309V_MODE_SINGLE)
			{
				if(t_mag.set.osr2[t_mag.sel_osr2] > QMC6309V_OSR2_4)
				{
					t_mag.delay = 20 * (t_mag.set.osr2[t_mag.sel_osr2]-QMC6309V_OSR2_4+1);
				}
			}
		}
		else
		{
			if(t_mag.set.odr[t_mag.sel_odr].mode == QMC6309_MODE_SINGLE)
			{
				if(t_mag.set.osr2[t_mag.sel_osr2] > QMC6309_OSR2_4)
				{
					t_mag.delay = 20 * (t_mag.set.osr2[t_mag.sel_osr2]-QMC6309_OSR2_4+1);
				}
			}
		}
	}

	if(t_mag.sel_delay > 0)
	{
		t_mag.delay = t_mag.sel_delay;
	}

	t_mag.data_i = 0;
	t_mag.data_max = t_mag.set.odr[t_mag.sel_odr].freq * ONETEST_DURATION;
	if(t_mag.data_max < 40)
	{
		t_mag.data_max = 40;
	}
}


void maestro1_do_set_reset(void)
{
	maestro1_ctrla		ctrlA;
	maestro1_ctrlb		ctrlB;

	if(t_mag.sel_sr > 0)
	{
		return;
	}
	#define SET_RESET_DELAY		2
	ctrlA.bit.mode = t_mag.set.odr[t_mag.sel_odr].mode;
	ctrlA.bit.osr1 = t_mag.set.osr1[t_mag.sel_osr1];
	ctrlA.bit.osr2 = t_mag.set.osr2[t_mag.sel_osr2];
	ctrlB.bit.set_rst = t_mag.set.sr[t_mag.sel_sr];
	ctrlB.bit.range = t_mag.set.range[t_mag.sel_range];
	ctrlB.bit.odr = t_mag.set.odr[t_mag.sel_odr].odr;
	ctrlB.bit.soft_rst = 0;
	if(ctrlA.bit.mode != MAESTRO_MODE_HPFM)
	{
		ctrlB.bit.odr = MAESTRO_ODR_1000HZ;	// normal mode set use 1000 Hz!
	}

	//qst_logi("set_reset count[%d]\r\n", t_mag.set_reset_num, ctrlA.value, ctrlB.value);
	for(int i=0; i<t_mag.set_reset_num; i++)
	{
		// do set
		qst_evb_mag_write_reg(MAESTRO_CTL_REG_ONE, 0x00);
		//qst_delay_ms(1);
		ctrlB.bit.set_rst = MAESTRO_SET_ON;
		qst_evb_mag_write_reg(MAG_CTL_REG_TWO, ctrlB.value);
		qst_evb_mag_write_reg(MAG_CTL_REG_ONE, ctrlA.value);
		qst_delay_ms(SET_RESET_DELAY);
		qst_logi("[%d] maestro SET [0x0a=0x%02x 0x0b=0x%02x] | ",(i+1), ctrlA.value, ctrlB.value);
#if 1
		// do reset
		qst_evb_mag_write_reg(MAESTRO_CTL_REG_ONE, 0x00);
		//qst_delay_ms(1);
		ctrlB.bit.set_rst = MAESTRO_RESET_ON;
		qst_evb_mag_write_reg(MAG_CTL_REG_TWO, ctrlB.value);
		qst_evb_mag_write_reg(MAG_CTL_REG_ONE, ctrlA.value);
		qst_delay_ms(SET_RESET_DELAY);
		qst_logi("RESET [0x0a=0x%02x 0x0b=0x%02x]\r\n", ctrlA.value, ctrlB.value);
#endif
	}

	qst_evb_mag_write_reg(MAESTRO_CTL_REG_ONE, 0x00);
	//qst_delay_ms(2);
}

#if 0
void maestro1_calc_kxky(unsigned char *preg)
{
#if 1
	signed char kx_calc = 0;
	signed short ky_calc = 0;
	
	unsigned char kx_code = preg[0] & 0x7f;
	unsigned short ky_code = ((preg[0] & 0x80) << 1) + (preg[1]);
	
	if (kx_code > 63){
		kx_calc = kx_code - 128;
	}
	else{
		kx_calc = kx_code;
	}
	kx_calc = MAESTRO_MAX(-64, MAESTRO_MIN(kx_calc, 63));
	
	if (ky_code > 255){
		ky_calc = ky_code - 512;
	}
	else{
		ky_calc = ky_code;
	}
	ky_calc = MAESTRO_MAX(-256, MAESTRO_MIN(ky_calc, 255));
	qst_logi("1 0x4e-0x4f[0x%02x 0x%02x] kxky[%d %d]\r\n",preg[0], preg[1], kx_calc, ky_calc);
	g_m_out.kx = (float)kx_calc / 128.f;
	g_m_out.ky = (float)ky_calc / 128.f;
#endif
#if 0
	maestro_kxky	kxky;

	kxky.bit.kx = preg[0]&0x7f;
	kxky.bit.ky = (short)(((preg[0]&0x80)<<1)|preg[1]);
	MAESTRO_LOG("2 0x4e-0x4f[0x%02x 0x%02x] kxky[%d %d]\r\n", preg[0], preg[1], kxky.bit.kx, kxky.bit.ky);
	g_m_out.kx = (float)kxky.bit.kx / 128.f;
	g_m_out.ky = (float)kxky.bit.ky / 128.f;
#endif
	
	qst_logi("maestro_calc_kxky [%f %f]  \r\n", g_m_out.kx, g_m_out.ky);
}
#endif

void qst_evb_mag_enable_single(void)
{
	if((t_mag.mag_sensor==QST_MAG_MAESTRO_1V)||(t_mag.mag_sensor==QST_MAG_MAESTRO_2H))
	{
		if((t_mag.reg_a & 0x03) == MAESTRO_MODE_SINGLE)
		{
			qst_evb_mag_write_reg(MAG_CTL_REG_ONE, t_mag.reg_a);
		}
	}
	else if(t_mag.mag_sensor==QST_MAG_QMC6309V)
	{
		if((t_mag.reg_a & 0x03) == QMC6309V_MODE_SINGLE)
		{
			qst_evb_mag_write_reg(MAG_CTL_REG_ONE, t_mag.reg_a);
		}
	}
	else
	{
		if((t_mag.reg_a & 0x03) == QMC6309_MODE_SINGLE)
		{
			qst_evb_mag_write_reg(MAG_CTL_REG_ONE, t_mag.reg_a);
		}
	}
}

void qst_evb_mag_enable(void)
{
	int ret = 0;

	if(t_mag.sel_odr < 0)
	{
		return;
	}
	if(t_mag.set.odr == maestro_odr)
	{
		maestro1_do_set_reset();
	}

	ret = qst_evb_mag_write_reg(MAG_CTL_REG_ONE, 0x00);
	if(!ret)
	{
		qst_logi("qst_evb_mag_enable write 0x0a fail1!\r\n");
	}
	qst_delay_ms(1);

	if((t_mag.mag_sensor == QST_MAG_MAESTRO_1V)||(t_mag.mag_sensor == QST_MAG_MAESTRO_2H))
	{
		maestro1_ctrla	ctrl1;
		maestro1_ctrlb	ctrl2;

		ctrl1.bit.mode = t_mag.set.odr[t_mag.sel_odr].mode;
		ctrl1.bit.osr1 = t_mag.set.osr1[t_mag.sel_osr1];
		ctrl1.bit.osr2 = t_mag.set.osr2[t_mag.sel_osr2];

		ctrl2.bit.set_rst = t_mag.set.sr[t_mag.sel_sr];
		ctrl2.bit.range = t_mag.set.range[t_mag.sel_range];
		ctrl2.bit.odr = t_mag.set.odr[t_mag.sel_odr].odr;
		ctrl2.bit.soft_rst = 0;

		t_mag.reg_a = ctrl1.value;
		t_mag.reg_b = ctrl2.value;
	}
	else if(t_mag.mag_sensor == QST_MAG_QMC6309V)
	{
		qmc6309v_ctrlreg1	ctrl1;
		qmc6309v_ctrlreg2	ctrl2;
		qmc6309v_ctrlreg3	ctrl3;

		ctrl1.bit.mode = t_mag.set.odr[t_mag.sel_odr].mode;
		ctrl1.bit.osr1 = t_mag.set.osr1[t_mag.sel_osr1];
		ctrl1.bit.osr2 = t_mag.set.osr2[t_mag.sel_osr2];
		ctrl1.bit.rev = 0;
		ctrl1.bit.zdbl_enb = (t_mag.sel_zdbl) ? QMC6309H_ZDBL_ENB_ON : QMC6309H_ZDBL_ENB_OFF;

		ctrl2.bit.set_rst = t_mag.set.sr[t_mag.sel_sr];
		ctrl2.bit.range = t_mag.set.range[t_mag.sel_range];
		ctrl2.bit.odr = t_mag.set.odr[t_mag.sel_odr].odr;
		ctrl2.bit.soft_rst = 0;

		ctrl3.value = t_mag.reg_c;
		ctrl3.bit.osr1_z = t_mag.set.osr1[t_mag.sel_osr1];

		t_mag.reg_a = ctrl1.value;
		t_mag.reg_b = ctrl2.value;
		t_mag.reg_c = ctrl3.value;
	}
	else if(t_mag.mag_sensor == QST_MAG_QMC6309)
	{
		qmc6309_ctrlreg1	ctrl1;
		qmc6309_ctrlreg2	ctrl2;

		ctrl1.bit.mode = t_mag.set.odr[t_mag.sel_odr].mode;
		ctrl1.bit.osr1 = t_mag.set.osr1[t_mag.sel_osr1];
		ctrl1.bit.osr2 = t_mag.set.osr2[t_mag.sel_osr2];
		ctrl1.bit.zdbl_enb = (t_mag.sel_zdbl) ? QMC6309H_ZDBL_ENB_ON : QMC6309H_ZDBL_ENB_OFF;

		ctrl2.bit.set_rst = t_mag.set.sr[t_mag.sel_sr];
		ctrl2.bit.range = t_mag.set.range[t_mag.sel_range];
		ctrl2.bit.odr = t_mag.set.odr[t_mag.sel_odr].odr;
		ctrl2.bit.soft_rst = 0;

		t_mag.reg_a = ctrl1.value;
		t_mag.reg_b = ctrl2.value;
	}
#if defined(QMC6308)
	else if(t_mag.mag_sensor == QST_MAG_QMC6308)
	{
		ctrl_reg1 ctrl1;
		ctrl_reg2 ctrl2;

		ctrl1.bit.mode = t_mag.set.odr[t_mag.sel_odr].mode;
		ctrl1.bit.odr = t_mag.set.odr[t_mag.sel_odr].odr;
		ctrl1.bit.osr1 = t_mag.set.osr1[t_mag.sel_osr1];
		ctrl1.bit.osr2 = t_mag.set.osr2[t_mag.sel_osr2];

		ctrl2.bit.setrst = t_mag.set.sr[t_mag.sel_sr];
		ctrl2.bit.range = t_mag.set.range[t_mag.sel_range];
		ctrl2.bit.rev = 0;
		ctrl2.bit.selftest = 0;
		ctrl2.bit.softrst = 0;

		t_mag.reg_a = ctrl1.value;
		t_mag.reg_b = ctrl2.value;
		t_mag.fifo_en = 0;
	}
#endif
	if(t_mag.fifo_en)
	{
		if(t_mag.mag_sensor == QST_MAG_QMC6309)
		{				
			t_mag.fifo_ctrl = (t_mag.fifo_mode|QMC6309_FIFO_CH_ALL|(t_mag.fifo_wmk<<3));
		}
		else
		{
			t_mag.fifo_ctrl = (t_mag.fifo_mode|t_mag.fifo_wmk);
		}
	}
	else
	{
		t_mag.fifo_ctrl = 0x00;
	}
	qst_evb_mag_calc_misc();
	qst_evb_mag_get_config_info();
	if(t_mag.st_en == 0)
	{
#if defined(MAESTRO0)
		qst_evb_mag_write_reg(0x0e, 0x04);		//qst_evb_mag_write_reg(0x0f, 0x06);
		qst_delay_ms(2);
#endif
		if(t_mag.mag_sensor == QST_MAG_MAESTRO_2H)
		{
			pset.value = 0x00;
#if defined(MAESTRO1_1_SET)
			pset.bit.period_set = 0x3f;
			pset.bit.user_set = 0;
			pset.bit.rev = 0;
			qst_evb_mag_write_reg(MAG_CTL_REG_PSET, pset.value);
			qst_delay_ms(1);
#elif defined(MAESTRO1_PERIOD_SET)
			pset.bit.period_set = 0x05;
			pset.bit.user_set = 0;
			pset.bit.rev = 0;
			qst_evb_mag_write_reg(MAG_CTL_REG_PSET, pset.value);
			qst_delay_ms(1);
#elif defined(MAESTRO1_USER_SET)
			pset.bit.period_set = 0x3f;
			pset.bit.user_set = 1;
			pset.bit.rev = 0;
			qst_evb_mag_write_reg(MAG_CTL_REG_PSET, pset.value);	// 0x00 0x3f
			qst_delay_ms(1);
#else
			(void)pset;
#endif
			// 0x0c 	OSR1		ODR
			// 0x00 	00			3521
			// 0x00 	01			5212
			// 0x00 	10			6843
			// 0x00 	11			8547
			// 0x3f 	00			16276
			// 0x3f 	01			32522
			// 0x3f 	10			65104
			// 0x3f 	11			170000
		}

		// write fifo config
		qst_evb_mag_write_reg(MAG_FIFO_REG_CTRL, t_mag.fifo_ctrl);
		// write fifo config
		if(t_mag.mag_sensor == QST_MAG_QMC6309V)
		{
			ret = qst_evb_mag_write_reg(MAG_CTL_REG_THREE, t_mag.reg_c);
			if(!ret)
			{
				qst_logi("qst_evb_mag_enable write 0x0c fail!\r\n");
				return;
			}
		}
		ret = qst_evb_mag_write_reg(MAG_CTL_REG_TWO, t_mag.reg_b);
		if(!ret)
		{
			qst_logi("qst_evb_mag_enable write 0x0b fail!\r\n");
			return;
		}
		qst_delay_ms(1);
		ret = qst_evb_mag_write_reg(MAG_CTL_REG_ONE, t_mag.reg_a);
		if(!ret)
		{
			qst_logi("qst_evb_mag_enable write 0x0a fail!\r\n");
			return;
		}
	}

	if(0)
	{
	unsigned char reg[6];

	qst_logi("mag enable write [0x0a=0x%02x 0x0b=0x%02x 0x2e=0x%02x]\r\n", t_mag.reg_a, t_mag.reg_b, t_mag.fifo_ctrl);
	qst_evb_mag_read_reg(MAG_CTL_REG_ONE, &reg[0], 2);
	qst_evb_mag_read_reg(MAG_FIFO_REG_CTRL, &reg[2], 1);
	qst_logi("mag enable read [0x0a=0x%02x 0x0b=0x%02x 0x2e=0x%02x]\r\n", reg[0], reg[1], reg[2]);
	}

#if defined(MAG_TEST_I3C_SUPPORT)
	if(t_mag.sel_test_i == TEST_IBI_TO_DRDY)
	{
		qst_evb_mag_enable_ibi(QMC6309_IBI_DRDY);
	}
	else if(t_mag.sel_test_i == TEST_IBI_TO_FIFO_FULL)
	{
		qst_evb_mag_enable_ibi(QMC6309_IBI_FIFO_FULL);
	}
	else if(t_mag.sel_test_i == TEST_IBI_TO_FIFO_WMK)
	{
		qst_evb_mag_enable_ibi(QMC6309_IBI_FIFO_WMK);
	}
	else if(t_mag.sel_test_i == TEST_IBI_TO_DATA_OVL)
	{
		qst_evb_mag_enable_ibi(QMC6309_IBI_OVFL);
	}
	else if(t_mag.sel_test_i == TEST_IBI_TO_SELFTEST)
	{
		qst_evb_mag_enable_ibi(QMC6309_IBI_ST_RDY);
	}
	if(t_mag.ibi_en)
	{
		qst_evb_i3c_enable_ibi(1);
	}
#endif
}

void qst_evb_mag_disable(void)
{
	qst_evb_mag_write_reg(MAG_FIFO_REG_CTRL, 0x00);
	qst_delay_ms(1);
	qst_evb_mag_write_reg(MAG_CTL_REG_TWO, 0x00);
	qst_delay_ms(1);
	qst_evb_mag_write_reg(MAG_CTL_REG_ONE, 0x00);
	qst_delay_ms(2);
}

void qst_evb_mag_soft_reset(void)
{
	unsigned char	status = 0;
//	qst_logi("qst_evb_mag_soft_reset! ");

	if(t_mag.user_set_reg)
	{
		qst_evb_mag_disable();	// when user config reg, softreset ignore!
		qst_logi("qst_evb_mag_soft_reset ignore!\r\n");
		return;
	}

	qst_evb_mag_write_reg(MAG_CTL_REG_TWO, 0x80);
	if((t_mag.mag_sensor == QST_MAG_QMC6309)||(t_mag.mag_sensor == QST_MAG_QMC6308))
	{
		qst_evb_mag_write_reg(MAG_CTL_REG_TWO, 0x00);
	}
	qst_delay_ms(5);

	int retry1 = 0;
	while(retry1++ < 10)
	{
		qst_evb_mag_read_reg(MAG_STATUS_REG, &status, 1);
		//qst_logi("mag status 0x09 = 0x%x\r\n", status);
		if((status & 0x10)&&(status & 0x08))
		{
			break;
		}
		qst_delay_ms(1);
	}
	if((status & 0x10)&&(status & 0x08))
	{	
		//qst_logi("NVM load done! 0x09=0x%02x\r\n", status);
	}
	else
	{
		qst_logi("NVM load fail! 0x09=0x%02x\r\n", status);
	}

#if defined(MAG_DIS_I3C)
	t_mag.reg = 0;
	qst_evb_mag_read_reg(0x3c, &t_mag.reg, 1);
	qst_evb_mag_write_reg(0x3c, t_mag.reg|0x20);
#endif

#if defined(QMC630H_3_3V_0X40)	// yang test
	if((t_mag.mag_sensor == QST_MAG_QMC6309)||(t_mag.mag_sensor == QST_MAG_QMC6309V))
	{
		#define SR_PULSE_250NS				0
		#define SR_PULSE_125NS				1
		#define SR_PULSE_62_5NS				2
		#define SR_PULSE_31_25NS			3
		#define SR_SLOP_100NS				0
		#define SR_SLOP_50NS				1
		#define SR_SLOP_25NS				2
		#define SR_SLOP_0NS					3

		// 0x40=0x0d	180mv
		// 0x40=0x15	120mv
		qst_evb_mag_read_reg(0x40, &(t_mag.val_40.value), 1);
		//t_mag.val_40.value = (t_mag.val_40.value&0xe0)|0x0d;		
		qst_logi("read 0x40=0x%02x	", t_mag.val_40.value);
		//t_mag.val_40.bit.sr_pulse_ctl = SR_PULSE_62_5NS;		// 0x15
		t_mag.val_40.sr_pulse = SR_PULSE_125NS;					// 0x0d
		t_mag.val_40.sr_slop = SR_SLOP_25NS;
		t_mag.val_40.sr_half = 1;
	
		t_mag.val_40.value |= 0x80;
		qst_logi("write 0x40=0x%02x\r\n", t_mag.val_40.value);
		qst_evb_mag_write_reg(0x40, t_mag.val_40.value);
	}
#endif

#if defined(FPGA)
	qst_evb_mag_apply_t0(0);
#endif
}

void qst_evb_mag_reload_otp(void)
{
	int ret, count, retry;
	unsigned char status = 0x00;

	count = 5;
	while(count > 0)
	{
		count--;
		ret = qst_evb_mag_write_reg(0x28, 0x02);
		qst_delay_ms(5);
		retry = 0;
		while(retry++ < 5)
		{
			qst_delay_ms(1);
			status = 0;
			ret = qst_evb_mag_read_reg(MAG_STATUS_REG, &status, 1);
			//qst_logi("mag status 0x%x\r\n", status);
			if(ret && (status & 0x10))
			{
				count = 0;
				break;
			}
		}
	}

#if defined(MAESTRO1)
	if(status & 0x20)
	{
		qst_logi("mag NVM load crc check fail\r\n");
	}
#endif

	if(ret && (status & 0x10))	
		qst_logi("qst_evb_mag_reload_otp done 0x09=0x%02x\r\n", status);
	else
		qst_logi("qst_evb_mag_reload_otp fail 0x09=0x%02x\r\n", status);	

//#if defined(FPGA)
//	#if defined(MAESTRO1)
//	qst_evb_mag_apply_kmtx(false, mag_kmtx_xy, mag_kmtx_xy_z);
//	qst_evb_mag_apply_offset_gain(mag_gain, mag_offset);
//	#elif defined(QMC6309V_X7)
//	qst_evb_mag_apply_offset_gain(mag_gain, mag_offset);
//	#endif
//#endif
}


void qst_evb_mag_programme_otp(void)
{
	int ret, count, retry;
	unsigned char status = 0x00;

#if defined(MAG_OTP_LOCK_UNLOCK)
	#define MAG_OTP_LOCK_UNLOCK_REG1				0x0e
	#define MAG_OTP_LOCK_UNLOCK_REG2				0x10

	unsigned char reg_v[2];
	if(t_mag.mag_sensor == QST_MAG_QMC6309V)
	{
#if defined(QMC6309V_X7)	// unlock nvram
		ret = qst_evb_mag_read_reg(MAG_OTP_LOCK_UNLOCK_REG1, &reg_v[0], 1);
		reg_v[1] = (reg_v[0] & 0xe0)|0x0b;
		ret = qst_evb_mag_write_reg(MAG_OTP_LOCK_UNLOCK_REG1, reg_v[1]);
#endif
	}
	else if(t_mag.mag_sensor == QST_MAG_QMC6309)
	{
		ret = qst_evb_mag_read_reg(MAG_OTP_LOCK_UNLOCK_REG2, &reg_v[0], 1);
		reg_v[1] = (reg_v[0] & 0xe0)|0x05;
		ret = qst_evb_mag_write_reg(MAG_OTP_LOCK_UNLOCK_REG2, reg_v[1]);
	}
#endif

	count = 5;
	while(count > 0)
	{
		count--;
		ret = qst_evb_mag_write_reg(0x28, 0x01);
		qst_delay_ms(5);
		retry = 0;
		while(retry++ < 5)
		{
			qst_delay_ms(1);
			status = 0;
			ret = qst_evb_mag_read_reg(MAG_STATUS_REG, &status, 1);
			//qst_logi("mag status 0x%x\r\n", status);
			if(ret && (status & 0x10))	
			{
				count = 0;
				break;
			}
		}
	}

	if(ret && (status & 0x10))	
		qst_logi("qst_evb_mag_programme_otp done 0x09=0x%02x\r\n", status);
	else
		qst_logi("qst_evb_mag_programme_otp fail 0x09=0x%02x\r\n", status);	

#if defined(MAG_OTP_LOCK_UNLOCK)
	if(t_mag.mag_sensor == QST_MAG_QMC6309V)
	{
#if defined(QMC6309V_X7)
		ret = qst_evb_mag_write_reg(MAG_OTP_LOCK_UNLOCK_REG1, reg_v[0]);
#endif
	}
	else if(t_mag.mag_sensor == QST_MAG_QMC6309)
	{
		ret = qst_evb_mag_write_reg(MAG_OTP_LOCK_UNLOCK_REG2, reg_v[0]);
	}
#endif
}


void mag_dump_reg(int flag)
{
	int i=0;

	memset(g_reg_tbl, 0, sizeof(g_reg_tbl));
	for(i=0; i<=t_mag.reg_max; i++)
	{
		qst_evb_mag_read_reg((unsigned char)i, &g_reg_tbl[i], 1);
		qst_delay_us(100);
		if(flag==1)
		{
			if((t_mag.mag_sensor == QST_MAG_MAESTRO_1V)||(t_mag.mag_sensor == QST_MAG_MAESTRO_2H))
			{
				if((i>=0x22 && i<=0x27) || (i>=0x30 && i<=0x53))
					qst_logi("%02x ", g_reg_tbl[i]);
			}
			else if(t_mag.mag_sensor == QST_MAG_QMC6309)
			{
				if(i>=0x30 && i<=0x49)
					qst_logi("%02x ", g_reg_tbl[i]);
			}
			else if(t_mag.mag_sensor == QST_MAG_QMC6309V)
			{
				if(i>=0x30 && i<=0x47)
					qst_logi("%02x ", g_reg_tbl[i]);
			}
		}
	}	

	t_mag.v_id = g_reg_tbl[0x12];
	t_mag.w_id = g_reg_tbl[0x37];
	t_mag.d_id = (unsigned short)((g_reg_tbl[0x39]<<8)|g_reg_tbl[0x38]);
	t_mag.l_id = (unsigned short)(((g_reg_tbl[0x42]&0x3f)<<8)|g_reg_tbl[0x41]);
	t_mag.reg_c = g_reg_tbl[0x0c];

	if(flag)
	{	
		qst_logi("\r\n");
		if((t_mag.mag_sensor == QST_MAG_MAESTRO_1V)||(t_mag.mag_sensor == QST_MAG_MAESTRO_2H))
		{
			qst_logi("CRC check: %02x %02x\t%s\r\n", g_reg_tbl[0x54], g_reg_tbl[0x55], (g_reg_tbl[0x09]&0x20)?"fail":"pass");
		}
		else if(t_mag.mag_sensor == QST_MAG_QMC6309)
		{
			qst_logi("CRC check: %02x %02x\t%s\r\n", g_reg_tbl[0x4A], g_reg_tbl[0x4B],	(g_reg_tbl[0x09]&0x20)?"fail":"pass");
		}
		else if(t_mag.mag_sensor == QST_MAG_QMC6309V)
		{
			qst_logi("CRC check: %02x %02x NVM_BANK_STATUS: 0x%02x\t%s\r\n", g_reg_tbl[0x48], g_reg_tbl[0x49], g_reg_tbl[0x0c], (g_reg_tbl[0x09]&0x20)?"fail":"pass");
		}
		// send config
		memset(g_chip_info, 0, sizeof(g_chip_info));
		int len_ext = sprintf(g_chip_info, "0x%02x 0x%02x 0x%02x 0x%04x", t_mag.chipid, t_mag.v_id, t_mag.w_id, t_mag.d_id);
		qst_logi("$:i%c%s\r\n", (uint8_t)(len_ext), g_chip_info);
		// send config
		qst_logi("********ver[0x%02x] wafer[0x%02x] die[0x%04x] lot[0x%04x]********\r\n", t_mag.v_id, t_mag.w_id, t_mag.d_id, t_mag.l_id);
	}

	qst_logi("********qst reg dump(hex)********\r\n");
	qst_logi("dump |  0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
	qst_logi("______________________________________________________\r\n");
	unsigned char dis_buf[60];
	for(int i=0; i<6; i++)
	{
		int index = i*16;
		memset(dis_buf, 0, sizeof(dis_buf));
		sprintf((char*)dis_buf, "0x%xx | %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\r\n", i,
																g_reg_tbl[index+0],g_reg_tbl[index+1],g_reg_tbl[index+2],g_reg_tbl[index+3],
																g_reg_tbl[index+4],g_reg_tbl[index+5],g_reg_tbl[index+6],g_reg_tbl[index+7],
																g_reg_tbl[index+8],g_reg_tbl[index+9],g_reg_tbl[index+10],g_reg_tbl[index+11],
																g_reg_tbl[index+12],g_reg_tbl[index+13],g_reg_tbl[index+14],g_reg_tbl[index+15]);
		qst_logi("%s", dis_buf);
	}	
	qst_logi("\r\n");
	//if(t_mag.mag_sensor == QST_MAG_MAESTRO_1V)
	//{
	//	maestro1_calc_kxky(&g_reg_tbl[0x4e]);
	//}
}


static int qst_evb_mag_select_config(int id)
{
	int a,b,c,d;
	int	curr_id = 0;

	qst_evb_mag_soft_reset();
	qst_delay_ms(10);

	for(a=0; a<t_mag.set.range_num; a++)
	{
		for(b=0; b<t_mag.set.odr_num; b++)
		{
			if(t_mag.set.odr == maestro_odr)
			{
				if((t_mag.set.odr[b].mode == MAESTRO_MODE_SINGLE)||(t_mag.set.odr[b].mode == MAESTRO_MODE_SUSPEND))
				{
					continue;
				}
			}
			else if(t_mag.set.odr == qmc6309_odr)
			{
				if((t_mag.set.odr[b].mode == QMC6309_MODE_SINGLE)||(t_mag.set.odr[b].mode == QMC6309_MODE_SUSPEND))
				{
					continue;
				}
			}
			else if(t_mag.set.odr == qmc6309v_odr)
			{
				if((t_mag.set.odr[b].mode == QMC6309V_MODE_SINGLE)||(t_mag.set.odr[b].mode == QMC6309V_MODE_SUSPEND))
				{
					continue;
				}
			}
#if defined(QMC6308)
			else if(t_mag.set.odr == qmc6308_odr)
			{
				if((t_mag.set.odr[b].mode == QMC6308_MODE_SINGLE)||(t_mag.set.odr[b].mode == QMC6308_MODE_SUSPEND))
				{
					continue;
				}
			}
#endif
#if 0
			if(((TEST_POLL_DATA_AUTO == t_mag.sel_test_i) || (TEST_POLL_FIFO_AUTO == t_mag.sel_test_i)))
			{
				if((t_mag.set.odr[b].freq < 100))
				{
					continue;
				}
			}
#endif
			for(c=0; c<t_mag.set.osr1_num; c++)
			{
				if(t_mag.set.odr == maestro_odr)
				{
					if((t_mag.set.odr[b].odr == MAESTRO_ODR_1000HZ) && ((t_mag.set.osr1[c] == MAESTRO1V_OSR1_16) || (t_mag.set.osr1[c] == MAESTRO1V_OSR1_32)))
					{
						continue;
					}
					else if((t_mag.set.odr[b].odr == MAESTRO_ODR_400HZ) && (t_mag.set.osr1[c] == MAESTRO1V_OSR1_32))
					{
						continue;
					}
				}
				for(d=0; d<t_mag.set.osr2_num; d++)
				{
					if(id == curr_id)
					{
						t_mag.sel_range = a;
						t_mag.sel_odr = b;
						t_mag.sel_osr1 = c;
						t_mag.sel_osr2 = d;

						qst_evb_mag_enable();

						return 1;
					}
					else
					{
						curr_id++;
					}
				}
			}
		}
	}

	//qst_logi("mag config id=%d return 0\r\n",id);
	return 0;
}


#if defined(MAG_TEST_I3C_SUPPORT)
static int qst_evb_mag_select_config_lite(int id, unsigned char osr1, unsigned char osr2)
{
	int a,b,c,d;
	int	curr_id = 0;

	qst_evb_mag_soft_reset();
	qst_delay_ms(10);

	for(a=0; a<t_mag.set.range_num; a++)
	{
		for(b=0; b<t_mag.set.odr_num; b++)
		{
			if(t_mag.set.odr == maestro_odr)
			{
				if((t_mag.set.odr[b].mode == MAESTRO_MODE_SINGLE)||(t_mag.set.odr[b].mode == MAESTRO_MODE_SUSPEND))
				{
					continue;
				}
				if((TEST_IBI_TO_DRDY == t_mag.sel_test_i) && (t_mag.set.odr[b].mode == MAESTRO_MODE_HPFM))		// mcu ability, can not support ibi too fast
				{
					continue;
				}
			}
			else if(t_mag.set.odr == qmc6309_odr)
			{
				if((t_mag.set.odr[b].mode == QMC6309_MODE_SINGLE)||(t_mag.set.odr[b].mode == QMC6309_MODE_SUSPEND))
				{
					continue;
				}
				if((TEST_IBI_TO_DRDY == t_mag.sel_test_i) && (t_mag.set.odr[b].mode == QMC6309_MODE_HPFM))		// mcu ability, can not support ibi too fast
				{
					continue;
				}
			}
			else if(t_mag.set.odr == qmc6309v_odr)
			{
				if((t_mag.set.odr[b].mode == QMC6309V_MODE_SINGLE)||(t_mag.set.odr[b].mode == QMC6309V_MODE_SUSPEND))
				{
					continue;
				}
				if((TEST_IBI_TO_DRDY == t_mag.sel_test_i) && (t_mag.set.odr[b].mode == QMC6309V_MODE_HPFM))		// mcu ability, can not support ibi too fast
				{
					continue;
				}
			}

			for(c=0; c<t_mag.set.osr1_num; c++)
			{
				if(t_mag.set.odr == maestro_odr)
				{
					if((t_mag.set.odr[b].odr == MAESTRO_ODR_1000HZ) && ((t_mag.set.osr1[c] == MAESTRO1V_OSR1_16) || (t_mag.set.osr1[c] == MAESTRO1V_OSR1_32)))
					{
						continue;
					}
					else if((t_mag.set.odr[b].odr == MAESTRO_ODR_400HZ) && (t_mag.set.osr1[c] == MAESTRO1V_OSR1_32))
					{
						continue;
					}
				}
				if(t_mag.set.osr1[c] != osr1)
				{
					continue;
				}
				for(d=0; d<t_mag.set.osr2_num; d++)
				{
					if(t_mag.set.osr2[d] != osr2)
					{
						continue;
					}
					if(id == curr_id)
					{
						t_mag.sel_range = a;
						t_mag.sel_odr = b;
						t_mag.sel_osr1 = c;
						t_mag.sel_osr2 = d;

						qst_logi("\r\n\r\nmag config find id=%d\r\n",id);
						qst_evb_mag_enable();
						return 1;
					}
					else
					{
						curr_id++;
					}
				}
			}
		}
	}	
	qst_logi("mag config not find id=%d\r\n",id);

	return 0;
}
#endif

int mag_get_drdy(void)
{
	int ret = 0;
	int t1 = 0;
	unsigned char rdy = 0;

	ret = qst_evb_mag_read_reg(MAG_STATUS_REG, &rdy, 1);
	while(!(rdy & MAG_STATUS_DRDY) && (t1++ < 5))
	{
		qst_delay_ms(1);
		ret = qst_evb_mag_read_reg(MAG_STATUS_REG, &rdy, 1);
	}
	if((ret == 0)||(!(rdy & MAG_STATUS_DRDY)))
	{
		ret = qst_evb_mag_read_reg(MAG_CTL_REG_ONE, g_buf, 2);
		ret = qst_evb_mag_read_reg(0x40, &g_buf[2], 1);
		qst_logi("mag drdy fail(ret=%d)! 0x09=0x%02x 0x0a=0x%02x 0x0b=0x%02x 0x40=0x%02x\r\n", ret, rdy, g_buf[0], g_buf[1], g_buf[2]);
		return 0;
	}
	else
	{
		return 1;
	}
}


void mag_read_raw(void)
{
	int ret = 0;

#if defined(OUT_F3_T)
	ret = qst_evb_mag_read_reg(MAG_DATA_OUT_X_LSB_REG, g_buf, 8);	// read mag & t
#else
	ret = qst_evb_mag_read_reg(MAG_DATA_OUT_X_LSB_REG, g_buf, 6);	// read mag
#endif
	if(ret == MAG_FAIL)
	{
		qst_logi("mag_read_raw read 0x01 fail! ret=%d", ret);
		return;
	}

	g_m_out.raw[0] = (short)(((g_buf[1]) << 8) | g_buf[0]);
	g_m_out.raw[1] = (short)(((g_buf[3]) << 8) | g_buf[2]);
	g_m_out.raw[2] = (short)(((g_buf[5]) << 8) | g_buf[4]);
#if defined(OUT_F3_T)
	g_m_out.temp_raw = ((short)(((g_buf[7]) << 8) | g_buf[6]))>>2;
	qst_logi("%d,%d,%d,T,%d,M-%d\r\n", g_m_out.raw[0], g_m_out.raw[1], g_m_out.raw[2], g_m_out.temp_raw, t_mag.data_i++);
#else
	qst_logi("%d,%d,%d,M-%d\r\n", g_m_out.raw[0], g_m_out.raw[1], g_m_out.raw[2], t_mag.data_i++);
#endif
	qst_evb_mag_enable_single();

#if defined(MAESTRO1_USER_SET)
	pset.bit.period_set = 0x3f;
	pset.bit.user_set = 1;
	qst_evb_mag_write_reg(MAG_CTL_REG_PSET, pset.value);
#endif
}


void mag_read_temp(short *temp)
{
	int ret = 0;
	unsigned char mag_data[2];
	short temp_raw = 0;

	ret = qst_evb_mag_read_reg(MAG_TEMP_OUT_LSB_REG, mag_data, 2);
	if(ret == MAG_FAIL)
	{
		qst_logi("mag_read_temp read 0x07 fail! res=%d", ret);
		return;
	}

	temp_raw = (short)((mag_data[1]<<8)|mag_data[0]);
	temp_raw = (temp_raw >> 6);
	*temp = temp_raw;
}

int mag_read_fifo(unsigned char *f_data)
{
	unsigned char fifo_level = 0;
	int ret = 1;

	ret = qst_evb_mag_read_reg(MAG_FIFO_REG_STATUS, &t_mag.status2, 1);
	fifo_level = (t_mag.status2 >> t_mag.fifo_lv_shift);

	if(fifo_level && ret)
	{
		ret = qst_evb_mag_read_reg(MAG_FIFO_REG_DATA, (unsigned char*)f_data, 6*fifo_level);
		ret = qst_evb_mag_write_reg(MAG_FIFO_REG_CTRL, t_mag.fifo_ctrl);
#if 1
		qst_logi("st:0x%x level:%d\r\n", t_mag.status2, fifo_level);
		for(int i=0; i<fifo_level; i++)
		{
			g_m_out.raw[0] = (short)(((f_data[1+i*6]) << 8) | f_data[0+i*6]);
			g_m_out.raw[1] = (short)(((f_data[3+i*6]) << 8) | f_data[2+i*6]);
			g_m_out.raw[2] = (short)(((f_data[5+i*6]) << 8) | f_data[4+i*6]);
			qst_logi("%d,%d,%d,M-%d,\r\n", g_m_out.raw[0], g_m_out.raw[1], g_m_out.raw[2], t_mag.data_i++);
		}
#endif
	}
	else
	{
		qst_logi("%s read fifo error status:0x%x \r\n", g_config_info, t_mag.status2);
	}

	return (int)fifo_level;
}

void qst_evb_mag_misc_hdlr(void)
{
	if(mag_get_drdy())
	{
		mag_read_raw();
	}
}

void qst_evb_mag_sensor_data(void)
{
	int ret = 0;
	int t1 = 0;
	unsigned char rdy = 0;
	unsigned char std_out = 0;
#if defined(MAG_SOFT_COMPENSATE)
	float mag_raw[3];
#endif

#if defined(MAG_TEST_STD_OUT)
	std_out = 1;
#else
	if(t_mag.sel_test_i == TEST_POLL_DATA_AUTO)
	{
		std_out = 1;
	}
#endif
	ret = qst_evb_mag_read_reg(MAG_STATUS_REG, &rdy, 1);
	while(!(rdy & MAG_STATUS_DRDY) && (t1++ < 5))
	{
		qst_delay_ms(1);
		ret = qst_evb_mag_read_reg(MAG_STATUS_REG, &rdy, 1);
	}

	if((ret == MAG_FAIL)||(!(rdy & MAG_STATUS_DRDY)))
	{	
		t_mag.data_fail++;

		ret = qst_evb_mag_read_reg(MAG_CTL_REG_ONE, g_buf, 2);
		ret = qst_evb_mag_read_reg(MAG_REG_SR_CTRL, &g_buf[2], 1);
		qst_logi("mag drdy fail(ret=%d)! 0x09=0x%02x 0x0a=0x%02x 0x0b=0x%02x 0x40=0x%02x\r\n", ret, rdy, g_buf[0], g_buf[1], g_buf[2]);
		if(t_mag.drdy_fail_num++ >= 5)
		{
			t_mag.drdy_fail_num = 0;
			qst_evb_mag_soft_reset();
			qst_evb_mag_enable();
		}
		return;
	}
	else
	{
		t_mag.drdy_fail_num = 0;
#if defined(OUT_F3_T)
		ret = qst_evb_mag_read_reg(MAG_DATA_OUT_X_LSB_REG, g_buf, 8);	// read mag & temp
#else
		ret = qst_evb_mag_read_reg(MAG_DATA_OUT_X_LSB_REG, g_buf, 6);	// read mag
#endif
		if(ret == MAG_FAIL)
		{
			t_mag.data_fail++;
			qst_logi("qst_evb_mag_sensor_data read 0x01 fail! ret=%d", ret);
			return;
		}
		qst_evb_mag_enable_single();

#if defined(MAESTRO1_USER_SET)
		pset.bit.period_set = 0x3f;
		pset.bit.user_set = 1;
		qst_evb_mag_write_reg(MAG_CTL_REG_PSET, pset.value);
#endif
		g_m_out.raw[0] = (short)(((g_buf[1]) << 8) | g_buf[0]);
		g_m_out.raw[1] = (short)(((g_buf[3]) << 8) | g_buf[2]);
		g_m_out.raw[2] = (short)(((g_buf[5]) << 8) | g_buf[4]);	
#if defined(OUT_F3_T)
		g_m_out.temp_raw = ((short)(((g_buf[7]) << 8) | g_buf[6]))>>2;
#endif
		g_m_out.mag[0] = (float)(g_m_out.raw[0]*100.0f/g_m_out.mag_lsb);
		g_m_out.mag[1] = (float)(g_m_out.raw[1]*100.0f/g_m_out.mag_lsb);
		g_m_out.mag[2] = (float)(g_m_out.raw[2]*100.0f/g_m_out.mag_lsb);
		//if(t_mag.mag_sensor == QST_MAG_MAESTRO_1V)
		//{
		//	g_m_out.mag[2] = g_m_out.mag[2] - (g_m_out.kx*g_m_out.mag[0]) - (g_m_out.ky*g_m_out.mag[1]);
		//}

#if defined(MAG_SOFT_COMPENSATE)
		mag_raw[0] = g_m_out.mag[0];
		mag_raw[1] = g_m_out.mag[1];
		mag_raw[2] = g_m_out.mag[2];
		g_m_out.mag[0] = (float)(mag_raw[0]*evb_softmag[0][0] + mag_raw[1]*evb_softmag[0][1] + mag_raw[2]*evb_softmag[0][2]);
		g_m_out.mag[1] = (float)(mag_raw[0]*evb_softmag[1][0] + mag_raw[1]*evb_softmag[1][1] + mag_raw[2]*evb_softmag[1][2]);
		g_m_out.mag[2] = (float)(mag_raw[0]*evb_softmag[2][0] + mag_raw[1]*evb_softmag[2][1] + mag_raw[2]*evb_softmag[2][2]);
#endif
		if(std_out)
		{
			if(t_mag.data_i > 3)
			{
				getStandardDeviation3(&mStd3, g_m_out.mag, g_m_out.std);
				if(g_m_out.std[0] < 5.0f)
				{
					if(g_m_out.std_avg[0] > 0.0f)
					{
						g_m_out.std_avg[0] = (g_m_out.std_avg[0] + g_m_out.std[0])/2.0f;
						g_m_out.std_avg[1] = (g_m_out.std_avg[1] + g_m_out.std[1])/2.0f;
						g_m_out.std_avg[2] = (g_m_out.std_avg[2] + g_m_out.std[2])/2.0f;
					}
					else
					{
						g_m_out.std_avg[0] = g_m_out.std[0];
						g_m_out.std_avg[1] = g_m_out.std[1];
						g_m_out.std_avg[2] = g_m_out.std[2];
					}
				}
			}
			else
			{
				g_m_out.std[0] = g_m_out.std[1] = g_m_out.std[2] = 255;
			}
			qst_logi(OUT_F6",M-%d", g_m_out.mag[0],g_m_out.mag[1],g_m_out.mag[2],g_m_out.std[0],g_m_out.std[1],g_m_out.std[2],t_mag.data_i);
		}
		else
		{
#if defined(OUT_F3_T)
			qst_logi(OUT_F3_T",M-%d", g_m_out.mag[0],g_m_out.mag[1],g_m_out.mag[2],g_m_out.temp_raw,t_mag.data_i);
#else
			qst_logi(OUT_F3",M-%d", g_m_out.mag[0],g_m_out.mag[1],g_m_out.mag[2],t_mag.data_i);
#endif
		}
		if((t_mag.data_i % t_mag.set.odr[t_mag.sel_odr].freq)==0)	// print config
			qst_logi(",%s\r\n", g_config_info);
		else
			qst_logi("\r\n");
	}
	t_mag.data_i++;

	if((t_mag.exit_flag)||(t_mag.data_i>=t_mag.data_max))
	{
		t_mag.exit_flag = 0;
		evb_setup_timer(TIM2, NULL, t_mag.delay, DISABLE);
		qst_logi("\r\nmag poll data fail[%d]/total[%d]\r\n", t_mag.data_fail, t_mag.data_i);
	}
}


void qst_evb_mag_auto_sensor_data(void)
{
	qst_evb_mag_sensor_data();
	if(t_mag.data_i >= t_mag.data_max)
	{	
		qst_logi("$:w Id[%d][%d/%d] Noise-%d %s average std,%f,%f,%f\r\n",t_mag.sel_mag_i,t_mag.data_fail,t_mag.data_max, t_mag.config_id,g_config_info,g_m_out.std_avg[0],g_m_out.std_avg[1],g_m_out.std_avg[2]);	// send test end to write file
		evb_setup_timer(TIM2, NULL, t_mag.delay, DISABLE);
		memset(&mStd3, 0, sizeof(mStd3));
		g_m_out.std_avg[0] = g_m_out.std_avg[1] = g_m_out.std_avg[2] = 0.0f;
		t_mag.data_i = t_mag.data_fail = 0;
		t_mag.config_id++;
		if(qst_evb_mag_select_config(t_mag.config_id))
		{
			evb_setup_timer(TIM2, qst_evb_mag_auto_sensor_data, t_mag.delay, ENABLE);
		}
		else
		{		
			qst_logi("qst_evb_mag_auto_sensor_data end!\r\n");
			t_mag.config_id = 0;
			t_mag.sel_mag_i++;
			if(t_mag.sel_mag_i < t_mag.mag_num)
			{
				qst_evb_mag_test_entry(t_mag.sel_inf);
			}
		}
	}
}

void qst_evb_mag_sensor_fifo_data(void)
{
	unsigned char std_out = 0;
	unsigned char level = 0;
	int ret = 1;
#if defined(MAG_SOFT_COMPENSATE)
	float mag_raw[3];
#endif

	ret = qst_evb_mag_read_reg(MAG_FIFO_REG_STATUS, &t_mag.status2, 1);
	level = (t_mag.status2 >> t_mag.fifo_lv_shift);

	if(level && ret)
	{
		ret = qst_evb_mag_read_reg(MAG_FIFO_REG_DATA, (unsigned char*)g_buf, 6*level);
		ret = qst_evb_mag_write_reg(MAG_FIFO_REG_CTRL, t_mag.fifo_ctrl);
#if 0
		ret = qst_evb_mag_read_reg(MAG_FIFO_REG_STATUS, &t_mag.status1, 1);
		if((t_mag.status1 >> t_mag.fifo_lv_shift))
		{
			qst_logi("%s error! after read fifo fifo level != 0, status=0x%02x\r\n", g_config_info, t_mag.status1); 
		}
#endif

#if defined(MAG_TEST_STD_OUT)
		std_out = 1;
#else
		if(t_mag.sel_test_i == TEST_POLL_FIFO_AUTO)
		{
			std_out = 1;
		}
#endif
		int config_flag = (((t_mag.data_i/level) % ((t_mag.set.odr[t_mag.sel_odr].freq/level)+1))==0);
		//qst_logi("%s0x%x L-%d\r\n", g_config_info, t_mag.status2, level);
		for(int i=0; i<level; i++)
		{
			g_m_out.raw[0] = (short)(((g_buf[1+i*6]) << 8) | g_buf[0+i*6]);
			g_m_out.raw[1] = (short)(((g_buf[3+i*6]) << 8) | g_buf[2+i*6]);
			g_m_out.raw[2] = (short)(((g_buf[5+i*6]) << 8) | g_buf[4+i*6]);
			g_m_out.mag[0] = (float)(g_m_out.raw[0]*100.0f/g_m_out.mag_lsb);
			g_m_out.mag[1] = (float)(g_m_out.raw[1]*100.0f/g_m_out.mag_lsb);
			g_m_out.mag[2] = (float)(g_m_out.raw[2]*100.0f/g_m_out.mag_lsb);
#if defined(MAG_SOFT_COMPENSATE)
			mag_raw[0] = g_m_out.mag[0];
			mag_raw[1] = g_m_out.mag[1];
			mag_raw[2] = g_m_out.mag[2];
			g_m_out.mag[0] = (float)(mag_raw[0]*evb_softmag[0][0] + mag_raw[1]*evb_softmag[0][1] + mag_raw[2]*evb_softmag[0][2]);
			g_m_out.mag[1] = (float)(mag_raw[0]*evb_softmag[1][0] + mag_raw[1]*evb_softmag[1][1] + mag_raw[2]*evb_softmag[1][2]);
			g_m_out.mag[2] = (float)(mag_raw[0]*evb_softmag[2][0] + mag_raw[1]*evb_softmag[2][1] + mag_raw[2]*evb_softmag[2][2]);
#endif
			if(std_out)
			{
				if(t_mag.data_i > 3)
				{
					getStandardDeviation3(&mStd3, g_m_out.mag, g_m_out.std);
					if(g_m_out.std[0] < 5.0f)
					{
						if(g_m_out.std_avg[0] > 0.0f)
						{
							g_m_out.std_avg[0] = (g_m_out.std_avg[0] + g_m_out.std[0])/2.0f;
							g_m_out.std_avg[1] = (g_m_out.std_avg[1] + g_m_out.std[1])/2.0f;
							g_m_out.std_avg[2] = (g_m_out.std_avg[2] + g_m_out.std[2])/2.0f;
						}
						else
						{
							g_m_out.std_avg[0] = g_m_out.std[0];
							g_m_out.std_avg[1] = g_m_out.std[1];
							g_m_out.std_avg[2] = g_m_out.std[2];
						}
					}
				}
				else
				{
					g_m_out.std[0] = g_m_out.std[1] = g_m_out.std[2] = 255;
				}
				if(config_flag)
					qst_logi(OUT_F6",M-%d,%s\r\n", g_m_out.mag[0],g_m_out.mag[1],g_m_out.mag[2],g_m_out.std[0],g_m_out.std[1],g_m_out.std[2],t_mag.data_i+i, g_config_info);
				else
					qst_logi(OUT_F6",M-%d\r\n", g_m_out.mag[0],g_m_out.mag[1],g_m_out.mag[2],g_m_out.std[0],g_m_out.std[1],g_m_out.std[2],t_mag.data_i+i);
			}
			else
			{
				if(config_flag)
					qst_logi(OUT_F3",M-%d,%s\r\n", g_m_out.mag[0], g_m_out.mag[1], g_m_out.mag[2],t_mag.data_i+i, g_config_info);
				else
					qst_logi(OUT_F3",M-%d\r\n", g_m_out.mag[0], g_m_out.mag[1], g_m_out.mag[2],t_mag.data_i+i);
			}
			//qst_logi("\r\n");
		}

		t_mag.drdy_fail_num = 0;
		t_mag.data_i += level;
	}
	else
	{
		t_mag.data_i += t_mag.fifo_wmk;
		t_mag.data_fail++;
		t_mag.drdy_fail_num++;
		qst_evb_mag_read_reg(MAG_CTL_REG_ONE, &g_buf[0], 2);
		qst_evb_mag_read_reg(MAG_FIFO_REG_CTRL, &g_buf[2], 1);
		qst_logi("read fifo status error:0x%02x  0x0A-0B[0x%02x 0x%02x] 0x2E[0x%02x]\r\n", t_mag.status2, g_buf[0], g_buf[1], g_buf[2]);
		if(t_mag.drdy_fail_num >= 2)
		{
			evb_setup_timer(TIM2, NULL, t_mag.delay, DISABLE);
			t_mag.drdy_fail_num = 0;
			qst_evb_mag_soft_reset();
			qst_evb_mag_enable();
			evb_setup_timer(TIM2, qst_evb_mag_sensor_fifo_data, t_mag.delay, ENABLE);
		}
	}

	if(t_mag.exit_flag)
	{
		t_mag.exit_flag = 0;
		evb_setup_timer(TIM2, NULL, t_mag.delay, DISABLE);
		qst_logi("\r\n mag poll fifo data fail[%d]\r\n", t_mag.data_fail);
	}
}


void qst_evb_mag_sensor_fifo_auto_data(void)
{
	qst_evb_mag_sensor_fifo_data();

	if(t_mag.data_i > t_mag.data_max)
	{	
		qst_logi("$:w Id[%d][%d/%d] Noise-%d %s average std,%f,%f,%f\r\n",t_mag.sel_mag_i,t_mag.data_fail,t_mag.data_max, t_mag.config_id,g_config_info,g_m_out.std_avg[0],g_m_out.std_avg[1],g_m_out.std_avg[2]);	// send test end to write file
		evb_setup_timer(TIM2, NULL, t_mag.delay, DISABLE);
		memset(&mStd3, 0, sizeof(mStd3));
		g_m_out.std_avg[0] = g_m_out.std_avg[1] = g_m_out.std_avg[2] = 0.0f;
		t_mag.data_i = t_mag.data_fail = 0;
		t_mag.config_id++;
		if(qst_evb_mag_select_config(t_mag.config_id))
		{
			evb_setup_timer(TIM2, qst_evb_mag_sensor_fifo_auto_data, t_mag.delay, ENABLE);
		}
		else
		{
			qst_logi("qst_evb_mag_sensor_fifo_auto_data end!\r\n");
			t_mag.config_id = 0;
			t_mag.sel_mag_i++;
			if(t_mag.sel_mag_i < t_mag.mag_num)
			{
				qst_evb_mag_test_entry(t_mag.sel_inf);
			}
		}
	}
}


#define QST_EVB_REG_WR_PLUS
void qst_evb_test_reg_wr_plus(void)
{
	#define MAX_REG 	13

	int ret = 0;
	static unsigned int fail_num[MAX_REG];
	static unsigned char reg_array[MAX_REG][2] = 
	{
		{0x00, 0x00},		// 0
		{0x0a, 0x00},
		{0x0b, 0x00},
		{0x0c, 0x00},
		{0x11, 0x00},
		{0x12, 0x36},		// 5
		{0x37, 0x00},
		{0x38, 0x00},
		{0x39, 0x00},
		{0x40, 0x00},
		{0x41, 0x00},		// 10
		{0x0e, 0x00},
		{0x45, 0x00}
	};		
	uint8_t	reg_value[MAX_REG]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	int len = 0;
	char *buf = (char*)g_buf;

	reg_array[0][1] = t_mag.chipid;
	reg_array[1][1] = t_mag.reg_a;
	reg_array[2][1] = t_mag.reg_b;

	reg_array[3][1] = g_reg_tbl[reg_array[3][0]];
	reg_array[4][1] = g_reg_tbl[reg_array[4][0]];
	reg_array[5][1] = g_reg_tbl[reg_array[5][0]];

	reg_array[6][1] = g_reg_tbl[reg_array[6][0]];
	reg_array[7][1] = g_reg_tbl[reg_array[7][0]];
	reg_array[8][1] = g_reg_tbl[reg_array[8][0]];
	reg_array[9][1] = g_reg_tbl[reg_array[9][0]];
	reg_array[10][1] = g_reg_tbl[reg_array[10][0]];
	reg_array[11][1] = g_reg_tbl[reg_array[11][0]];
	reg_array[12][1] = g_reg_tbl[reg_array[12][0]];
#if 1
	ret = qst_evb_mag_read_reg(reg_array[0][0], &reg_value[0], 1);
	MAG_CHECK_ERR(ret);
	ret = qst_evb_mag_read_reg(reg_array[1][0], &reg_value[1], 3);
	MAG_CHECK_ERR(ret);
	ret = qst_evb_mag_read_reg(reg_array[4][0], &reg_value[4], 2);
	MAG_CHECK_ERR(ret);
	ret = qst_evb_mag_read_reg(reg_array[6][0], &reg_value[6], 3);
	MAG_CHECK_ERR(ret);
	ret = qst_evb_mag_read_reg(reg_array[9][0], &reg_value[9], 2);
	MAG_CHECK_ERR(ret);
	ret = qst_evb_mag_read_reg(reg_array[11][0], &reg_value[11], 1);
	MAG_CHECK_ERR(ret);
	ret = qst_evb_mag_read_reg(reg_array[12][0], &reg_value[12], 1);
	MAG_CHECK_ERR(ret);
#else
	for(int i=0; i<MAX_REG; i++)
	{
		ret = qst_evb_mag_read_reg(reg_array[i][0], &reg_value[i], 1);
		MAG_CHECK_ERR(ret);
	}
#endif

	len = 0;
	if(!(t_mag.data_i%1600))
	{
		len += snprintf(buf+len, 256-len, "\r\nREG[ ");
		for(int i=0; i<MAX_REG; i++)
		{
			len += snprintf(buf+len, 256-len, "0x%02x ", reg_array[i][0]);
		}
		len += snprintf(buf+len, 256-len, "]\r\n");
		qst_logi("%s", buf);		
		len = 0;
	}
	for(int i=0; i<MAX_REG; i++)
	{
		if(reg_value[i] !=	reg_array[i][1])
		{
			fail_num[i]++;
		}
	}	

	if((t_mag.data_i % 100)	== 0)
	{
		len = 0;
		len += snprintf(buf+len, 256-len, "VAL[ ");
		for(int i=0; i<MAX_REG; i++)
		{
			len += snprintf(buf+len, 256-len, "0x%02x ", reg_value[i]);
		}
		len += snprintf(buf+len, 256-len, "]");

		len += snprintf(buf+len, 256-len, " | FAIL[ ");
		for(int i=0; i<MAX_REG; i++)
		{
			len += snprintf(buf+len, 256-len, "%d ", fail_num[i]);
		}	
		len += snprintf(buf+len, 256-len, "]");
		//qst_evb_mag_get_config_info();
		len += snprintf(buf+len, 256-len, "| [%d %s%s]\r\n", t_mag.data_i, g_config_info, interface_array[t_mag.sel_inf].info);
		buf[len] = '\0';

		qst_logi("%s", buf);
	}

	//qst_evb_mag_write_reg(0x0b, reg_array[2][1]);
	//qst_evb_mag_write_reg(0x0a, reg_array[1][1]);

	if(++t_mag.data_i >= t_mag.data_max)
	{	
		qst_evb_mag_disable();
		evb_setup_timer(TIM2, NULL, 8, DISABLE);
		if(t_mag.sel_odr < 0)
			qst_logi("%s %s reg w/r error rate:\r\n", "standy mode", interface_array[t_mag.sel_inf].info);
		else
			qst_logi("%s reg w/r error rate:\r\n", g_config_info);
		len = 0;
		for(int i=0; i<MAX_REG; i++)
		{
			len += snprintf(buf+len, QST_BUF_LEN-len, "0x%02x %.3f%%\r\n", reg_array[i][0], ((float)fail_num[i]*100.f/t_mag.data_max));
		}
		buf[len] = '\0';
		qst_logi("%s\r\n", buf);
		memset(fail_num, 0, sizeof(fail_num));
#if defined(QST_EVB_REG_WR_PLUS)
		evb_setup_timer(TIM3, NULL, 8, DISABLE);
		//t_mag.sel_odr++;
		while(++t_mag.sel_odr < t_mag.set.odr_num)
		{
			int flag = 0;
			if((t_mag.set.odr == qmc6309_odr)||(t_mag.set.odr == qmc6309v_odr))
			{
				if(t_mag.set.odr[t_mag.sel_odr].freq == 200)
					flag = 1;
			}
			else if(t_mag.set.odr == maestro_odr)
			{
				if(t_mag.set.odr[t_mag.sel_odr].freq == 400)
					flag = 1;				
			}
			if(flag)
			{
				qst_mag_auto_sel_range_osr(t_mag.sel_odr);
				qst_evb_mag_enable();
				t_mag.data_max = 50000;
				evb_setup_timer(TIM3, qst_evb_test_reg_wr_plus, 5, ENABLE);
				return;
			}
		}

		t_mag.sel_odr = 0;
		qst_logi("qst_evb_test_reg_wr done\r\n");
#endif
	}
}

static void qst_evb_test_work_mode_switch(void)
{
	int a,b,c,d;

	t_mag.config_id = 0;
	for(a=0; a<t_mag.set.range_num; a++)
	{
		for(b=0; b<t_mag.set.odr_num; b++)
		{			
			qst_delay_ms(500);
			for(c=0; c<t_mag.set.osr1_num; c++)
			{
				for(d=0; d<1/*t_mag.set.osr2_num*/; d++)
				{
					qst_evb_mag_soft_reset();
					qst_delay_ms(100);
					t_mag.sel_range = a;
					t_mag.sel_odr = b;
					t_mag.sel_osr1 = c;
					t_mag.sel_osr2 = d;

					qst_evb_mag_enable();
					qst_logi("\r\n\r\ntest index = %d	%s\r\n", ++t_mag.config_id, g_config_info);

					while(t_mag.data_i++ <= t_mag.data_max)
					{
						qst_delay_ms(t_mag.delay);
						if(mag_get_drdy())
						{						
							mag_read_raw();
						}					
					}
				}
			}
		}
	}

	qst_logi("qst_evb_test_work_mode_switch end!\n");
}

static void qst_evb_mag_test_poll_odr(void)
{
	unsigned char rdy = 0;
	unsigned char fifo_level = 0;
	short ms = 1;

	qst_evb_mag_soft_reset();

	t_mag.data_i = 0;

	if(t_mag.fifo_en)
	{
		t_mag.fifo_mode = (unsigned char)QMC6309_FIFO_MODE_FIFO;
		switch(t_mag.set.odr[t_mag.sel_odr].freq)
		{
			case 1500:
			case 1000:
				t_mag.fifo_wmk = t_mag.fifo_size-2;
				ms = t_mag.fifo_wmk*1;
				break;
			case 400:
				t_mag.fifo_wmk = t_mag.fifo_size/2;
				ms = t_mag.fifo_wmk*2.5;
				break;				
			case 250:
				t_mag.fifo_wmk = t_mag.fifo_size/3;
				ms = t_mag.fifo_wmk*4;
				break;
			case 200:
				t_mag.fifo_wmk = t_mag.fifo_size/3;
				ms = t_mag.fifo_wmk*5;
				break;
			case 100:
				t_mag.fifo_wmk = t_mag.fifo_size/3;
				ms = t_mag.fifo_wmk*10;
				break;
			case 50:
				t_mag.fifo_wmk = t_mag.fifo_size/4;
				ms = t_mag.fifo_wmk*20;
				break;
			case 20:
				t_mag.fifo_wmk = t_mag.fifo_size/4;
				ms = t_mag.fifo_wmk*50;
				break;
			case 10:
				t_mag.fifo_wmk = t_mag.fifo_size/4;
				ms = t_mag.fifo_wmk*100;
				break;
			case 1:
				t_mag.fifo_wmk = t_mag.fifo_size/8;
				ms = t_mag.fifo_wmk*1000;
				break;
		}
		
		ms = 2;
	}
	else
	{
		t_mag.fifo_mode = 0;
		t_mag.fifo_wmk = 0;
		ms = 1;
	}
	qst_evb_mag_enable();
	t_mag.data_max = t_mag.set.odr[t_mag.sel_odr].freq * 30;		// 30 second
	while(1)
	{		
		if(t_mag.fifo_en)
		{
			qst_evb_mag_read_reg(MAG_FIFO_REG_STATUS, &t_mag.status2, 1);
			fifo_level = (t_mag.status2 >> t_mag.fifo_lv_shift);
			if((fifo_level >=t_mag.fifo_wmk)&&(t_mag.status2 & (QMC6309_FIFO_STATUS_WMK|QMC6309_FIFO_STATUS_FULL)))
			{
				t_mag.data_i += fifo_level;
				qst_evb_mag_read_reg(MAG_FIFO_REG_DATA, g_buf, 6*fifo_level);
				qst_evb_mag_write_reg(MAG_FIFO_REG_CTRL, t_mag.fifo_ctrl);
				//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
				qst_logi("M-%d\r\n",t_mag.data_i);
			}
		}
		else
		{
			qst_evb_mag_read_reg(MAG_STATUS_REG, &rdy, 1);
			if(rdy & MAG_STATUS_DRDY)
			{
				t_mag.data_i += 1;
				//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
				qst_logi("M-%d\r\n",t_mag.data_i);
			}
		}
	
		if(t_mag.data_i > t_mag.data_max)
		{
			break;
		}
		qst_delay_ms(ms);
	}

	qst_delay_ms(10);
	qst_logi("$:w\r\n");	// send test end to write file
	qst_logi("\r\n\r\n\r\n\r\n\r\n");
	qst_delay_ms(10);
}

#if defined(MAG_TEST_I3C_SUPPORT)
void qst_evb_i3c_ibi_odr(void)
{
	int ret = 0;

	if(t_mag.fifo_en)
	{
		unsigned char fifo_level = 0;
		qst_evb_mag_read_reg(MAG_FIFO_REG_STATUS, &t_mag.status2, 1);
		fifo_level = (t_mag.status2 >> t_mag.fifo_lv_shift);
		qst_evb_mag_read_reg(MAG_FIFO_REG_DATA, g_buf, 6*fifo_level);
		qst_evb_mag_write_reg(MAG_FIFO_REG_CTRL, t_mag.fifo_ctrl);
		t_mag.data_i += fifo_level;
	}
	else
	{
		ret = qst_evb_mag_read_reg(MAG_DATA_OUT_X_LSB_REG, g_buf, 6);
		if(ret == MAG_FAIL)
		{
			qst_logi("qst_evb_i3c_ibi_odr read 0x01 fail! ret=%d", ret);
		}
		t_mag.data_i += 1;
		
		ret = qst_evb_mag_write_reg(0x38, 0x55);
	}

	qst_logi("IBI M-%d\r\n",t_mag.data_i);
}
#endif

static void qst_evb_mag_test_odr(int ibi)
{
	int index = 0;

	if(ibi)
	{
		for(index=0; index<t_mag.set.odr_num; index++)
		{
			if(t_mag.set.odr == maestro_odr)
			{
				if((t_mag.set.odr[index].mode == MAESTRO_MODE_SINGLE)||(t_mag.set.odr[index].mode == MAESTRO_MODE_SUSPEND)||(t_mag.set.odr[index].mode == MAESTRO_MODE_HPFM))
				{
					continue;
				}
			}
			else if(t_mag.set.odr == qmc6309v_odr)
			{
				if((t_mag.set.odr[index].mode == QMC6309V_MODE_SINGLE)||(t_mag.set.odr[index].mode == QMC6309V_MODE_SUSPEND)||(t_mag.set.odr[index].mode == QMC6309V_MODE_HPFM))
				{
					continue;
				}
			}
			else
			{
				if((t_mag.set.odr[index].mode == QMC6309_MODE_SINGLE)||(t_mag.set.odr[index].mode == QMC6309_MODE_SUSPEND)||(t_mag.set.odr[index].mode == QMC6309_MODE_HPFM))
				{
					continue;
				}
			}
			//qst_evb_mag_soft_reset();
			t_mag.sel_odr = index;
			qst_mag_auto_sel_range_osr(index);

			t_mag.ibi_en = 1;
			t_mag.sel_test_i = TEST_IBI_TO_DRDY;
			qst_evb_mag_enable();
#if defined(MAG_TEST_I3C_SUPPORT)
			qst_evb_reg_i3c_ibi_hdlr(qst_evb_i3c_ibi_odr);
			while(t_mag.data_i < t_mag.data_max)
			{
				qst_i3c_ibi_handle();
			}
#endif
			qst_logi("$:w\r\n\r\n\r\n\r\n\r\n");	// send test end to write file
		}
		
		t_mag.ibi_en = 0;
		t_mag.fifo_en = 0;
		qst_evb_mag_soft_reset();
	}
	else
	{
		for(index=0; index<t_mag.set.odr_num; index++)
		{
			if(t_mag.set.odr == maestro_odr)
			{
				if((t_mag.set.odr[index].mode == MAESTRO_MODE_SINGLE)||(t_mag.set.odr[index].mode == MAESTRO_MODE_SUSPEND)||(t_mag.set.odr[index].mode == MAESTRO_MODE_HPFM))
				{
					continue;
				}
			}
			else if(t_mag.set.odr == qmc6309v_odr)
			{
				if((t_mag.set.odr[index].mode == QMC6309V_MODE_SINGLE)||(t_mag.set.odr[index].mode == QMC6309V_MODE_SUSPEND)||(t_mag.set.odr[index].mode == QMC6309V_MODE_HPFM))
				{
					continue;
				}
			}
			else
			{
				if((t_mag.set.odr[index].mode == QMC6309_MODE_SINGLE)||(t_mag.set.odr[index].mode == QMC6309_MODE_SUSPEND)||(t_mag.set.odr[index].mode == QMC6309_MODE_HPFM))
				{
					continue;
				}
			}
	
			t_mag.sel_odr = index;
			qst_mag_auto_sel_range_osr(index);
			t_mag.fifo_en = 0;
			qst_evb_mag_test_poll_odr();	// polling drdy
			t_mag.fifo_en = 1;
			qst_evb_mag_test_poll_odr();	// polling fifo
		}
	}

	qst_logi("qst_evb_mag_test_odr complete!\r\n");
}

static void qst_evb_mag_current_test(void)
{
	int a,b,c,d;
	#define CURRENT_TEST_ONE_DUR		1	// second

	evb_setup_uart_rx(DEBUG_USART, NULL);
	t_mag.config_id = 0;
	for(a=0; a<t_mag.set.range_num; a++)
	{
		for(b=0; b<t_mag.set.odr_num; b++)
		{
			if((t_mag.mag_sensor==QST_MAG_MAESTRO_1V)||(t_mag.mag_sensor==QST_MAG_MAESTRO_2H))
			{
				if((MAESTRO_MODE_SINGLE == t_mag.set.odr[b].mode)||(MAESTRO_MODE_SUSPEND == t_mag.set.odr[b].mode))
				{
					continue;
				}
			}
			else if(t_mag.mag_sensor==QST_MAG_QMC6309V)
			{
				if((QMC6309V_MODE_SINGLE == t_mag.set.odr[b].mode)||(QMC6309V_MODE_SUSPEND == t_mag.set.odr[b].mode))
				{
					continue;
				}
			}
			else
			{
				if((QMC6309_MODE_SINGLE == t_mag.set.odr[b].mode)||(QMC6309_MODE_SUSPEND == t_mag.set.odr[b].mode))
				{
					continue;
				}
			}
			for(c=0; c<t_mag.set.osr1_num; c++)
			{
				if((t_mag.mag_sensor==QST_MAG_MAESTRO_1V)||(t_mag.mag_sensor==QST_MAG_MAESTRO_2H))
				{
					if((t_mag.set.odr[b].odr == MAESTRO_ODR_1000HZ) && ((t_mag.set.osr1[c] == MAESTRO1V_OSR1_16) || (t_mag.set.osr1[c] == MAESTRO1V_OSR1_32)))
					{
						continue;
					}
					else if((t_mag.set.odr[b].odr == MAESTRO_ODR_400HZ) && (t_mag.set.osr1[c] == MAESTRO1V_OSR1_32))
					{
						continue;
					}
				}
				if((t_mag.set.osr1 == qmc6309_osr1)||(t_mag.set.osr1 == qmc6309v_osr1))		// test osr1 = 1 or 8
				{
					if((c != 0) && (c != t_mag.set.osr1_num-1))
					{
						continue;
					}
				}
				for(d=0; d<1/*t_mag.set.osr2_num*/; d++)
				{
					qst_evb_mag_soft_reset();
					t_mag.sel_range = a;
					t_mag.sel_odr = b;
					t_mag.sel_osr1 = c;
					t_mag.sel_osr2 = d;
					qst_evb_mag_enable();
					t_mag.data_i = 0;
					t_mag.data_max = (1500/t_mag.delay)+1;	//t_mag.set.odr[b].freq*CURRENT_TEST_ONE_DUR;

					qst_logi("******current test-%d	[%s]******\r\n", ++t_mag.config_id, g_config_info);
					while(t_mag.data_i++ <= t_mag.data_max)
					{
						qst_delay_ms(t_mag.delay);
						if(mag_get_drdy())
						{
							qst_evb_mag_read_reg(MAG_DATA_OUT_X_LSB_REG, g_buf, 6);
							qst_logi("M-%d	", t_mag.data_i);
							if((t_mag.data_i % 20) == 0)
							{
								qst_logi("\r\n");
							}
						}
						else
						{
							qst_evb_mag_soft_reset();
							qst_evb_mag_enable();
						}
					}					
					qst_logi("\r\n******current test-%d	[%s]******  done!\r\n", t_mag.config_id, g_config_info);
#if 0
					while(1)
					{					
						unsigned char	singal = 0;
						qst_logi("please input n/N to test next config:");
						singal = 0;
						scanf("%c", &singal);
						if((singal=='n')||(singal=='N'))
						{
							break;
						}
						qst_logi("\r\n");
					}
#else
					qst_logi("please press blue-key to test next config\r\n\r\n");
					evb_setup_user_key(1, qst_evb_mag_key1_hdlr);
					t_mag.wait_cfg = 1;
					while(t_mag.wait_cfg)
					{
						evb_key_handle();
					}
#endif
				}
			}
		}
	}

	qst_logi("qst_evb_mag_current_test end!\n");
}

#if 0
static void qst_evb_mag_setreset_test(void)
{
	int b = 0;
	#define SETRESET_ONE_DUR		10

	t_mag.config_id = 0;

	for(b=0; b<t_mag.set.odr_num; b++)
	{
		t_mag.sel_odr = b;
		qst_evb_mag_soft_reset();
		qst_delay_ms(500);

		//qst_mag_entry_sel_range_osr();
		qst_mag_auto_sel_range_osr(b);
		qst_logi("\r\n\r\ntest mode = %d\r\n", b);
		for(t_mag.sel_sr=0; t_mag.sel_sr<t_mag.set.sr_num; t_mag.sel_sr++)
		{
			qst_evb_mag_disable();
			qst_delay_ms(100);
			if(t_mag.set.odr[b].mode == QMC6309_MODE_SUSPEND)
			{
				break;
			}
			qst_evb_mag_enable();
			t_mag.data_max = t_mag.set.odr[b].odr*SETRESET_ONE_DUR;
			while(t_mag.data_i++ <= t_mag.data_max)
			{
				qst_delay_ms(t_mag.delay);
				qst_evb_mag_sensor_data();
			}
		}
	}

	qst_logi("qst_evb_mag_setreset_test end!\n");
}
#endif

static void qst_evb_mag_set_sr_reg(unsigned char sr)
{
	if(t_mag.mag_sensor == QST_MAG_QMC6309)
	{
		qst_evb_mag_write_reg(QMC6309_CTL_REG_ONE, 0x00);		// enter suspend
		qst_delay_ms(g_m_out.sr_ctl.switch_suspend_delay);

		t_mag.reg_b = ((t_mag.reg_b & 0xfc) | sr);
		qst_logi("qst_evb_mag_set_sr_reg! 0x0a=0x%02x 0x0b=0x%02x\r\n", t_mag.reg_a, t_mag.reg_b);
		qst_evb_mag_write_reg(QMC6309_CTL_REG_TWO, t_mag.reg_b);
		qst_delay_ms(1);
		qst_evb_mag_write_reg(QMC6309_CTL_REG_ONE, t_mag.reg_a);
		qst_delay_ms(1);
	}
}

static void qst_evb_qmc6309_sr_mode_switch(short raw[3])
{
	short temp_raw[3];

	temp_raw[0] = QMC6309_ABS(raw[0]);
	temp_raw[1] = QMC6309_ABS(raw[1]);
	temp_raw[2] = QMC6309_ABS(raw[2]);
	
	if(g_m_out.sr_ctl.mode == QMC6309_SET_RESET_ON)
	{	
		qst_logi("Mode[%d]	%d %d %d	count[%d]\r\n", QMC6309_SET_RESET_ON, temp_raw[0],temp_raw[1],temp_raw[2],g_m_out.sr_ctl.count);
		if((temp_raw[0] > g_m_out.sr_ctl.switch_threshold)||(temp_raw[1] > g_m_out.sr_ctl.switch_threshold)||(temp_raw[2] > g_m_out.sr_ctl.switch_threshold))
		{
			g_m_out.sr_ctl.count++;
			if(g_m_out.sr_ctl.count > g_m_out.sr_ctl.switch_count)
			{
				qst_evb_mag_set_sr_reg(g_m_out.sr_ctl.swich_mode);
				g_m_out.sr_ctl.mode = g_m_out.sr_ctl.swich_mode;
				g_m_out.sr_ctl.count = 0;
				g_m_out.sr_ctl.count2 = 0;
				qst_logi("Mode[%d] switch to Mode[%d]\r\n", QMC6309_SET_RESET_ON, g_m_out.sr_ctl.swich_mode);
			}
		}
		else
		{
			g_m_out.sr_ctl.count = 0;
		}
	}
	else if(g_m_out.sr_ctl.mode == g_m_out.sr_ctl.swich_mode)
	{
		g_m_out.sr_ctl.count++;
		qst_logi("Mode[%d]	%d %d %d count[%d %d]\r\n", g_m_out.sr_ctl.mode, temp_raw[0],temp_raw[1],temp_raw[2],g_m_out.sr_ctl.count,g_m_out.sr_ctl.count2);
		if(g_m_out.sr_ctl.count >= 500)
		{
			int wait_n_odr = 5;
			int retry = 0;
			unsigned char status = 0x00;

			qst_evb_mag_set_sr_reg(QMC6309_SET_RESET_ON);		// set-reset on one time
			wait_n_odr = g_m_out.sr_ctl.one_sr_delay;
			while(wait_n_odr > 0)
			{
				status = 0x00;
				retry = 0;
				while(!(status & 0x03))
				{
					qst_delay_ms(1);
					qst_evb_mag_read_reg(MAG_STATUS_REG, &status, 1);
					retry++;
				}
				wait_n_odr--;
			}
			qst_evb_mag_set_sr_reg(g_m_out.sr_ctl.mode);
			g_m_out.sr_ctl.count = 0;
			qst_logi("Mode[%d] do set-reset on every 500 data\r\n", g_m_out.sr_ctl.mode);
		}

		if((temp_raw[0] < g_m_out.sr_ctl.switch_threshold)&&(temp_raw[1] < g_m_out.sr_ctl.switch_threshold)&&(temp_raw[2] < g_m_out.sr_ctl.switch_threshold))
		{
			g_m_out.sr_ctl.count2++;
			if(g_m_out.sr_ctl.count2 >= g_m_out.sr_ctl.switch_count)
			{
				qst_logi("Mode[%d] switch to Mode[%d]\r\n", g_m_out.sr_ctl.mode, QMC6309_SET_RESET_ON);
				qst_evb_mag_set_sr_reg(QMC6309_SET_RESET_ON);
				g_m_out.sr_ctl.mode = QMC6309_SET_RESET_ON;
				g_m_out.sr_ctl.count = 0;
				g_m_out.sr_ctl.count2 = 0;
			}
		}
		else
		{
			g_m_out.sr_ctl.count2 = 0;
		}

		if((temp_raw[0] == 32767)&&(temp_raw[1] == 32767)&&(temp_raw[2] == 32767))
		{
			qst_evb_mag_set_sr_reg(QMC6309_SET_RESET_ON);
			g_m_out.sr_ctl.mode = QMC6309_SET_RESET_ON;
			g_m_out.sr_ctl.count = 0;
			g_m_out.sr_ctl.count2 = 0;
			qst_logi("overflow1 switch to Mode[%d]\r\n", QMC6309_SET_RESET_ON);
		}
		else if((temp_raw[0] <= 10)&&(temp_raw[1] <= 10)&&(temp_raw[2] <= 10))
		{
			qst_evb_mag_set_sr_reg(QMC6309_SET_RESET_ON);
			g_m_out.sr_ctl.mode = QMC6309_SET_RESET_ON;
			g_m_out.sr_ctl.count = 0;
			g_m_out.sr_ctl.count2 = 0;
			qst_logi("overflow2 switch to Mode[%d]\r\n", QMC6309_SET_RESET_ON);
		}
	}
}


static void qst_evb_mag_sr_mode_switch_data(void)
{
	int ret = 0;
	int t1 = 0;
	unsigned char rdy = 0;

	ret = qst_evb_mag_read_reg(MAG_STATUS_REG, &rdy, 1);
	while(!(rdy & MAG_STATUS_DRDY) && (t1++ < 5))
	{
		qst_delay_ms(1);
		ret = qst_evb_mag_read_reg(MAG_STATUS_REG, &rdy, 1);
	}

	if((ret == MAG_FAIL)||(!(rdy & MAG_STATUS_DRDY)))
	{	
		t_mag.data_fail++;

		ret = qst_evb_mag_read_reg(MAG_CTL_REG_ONE, g_buf, 2);
		ret = qst_evb_mag_read_reg(MAG_REG_SR_CTRL, &g_buf[2], 1);
		qst_logi("mag drdy fail(ret=%d)! 0x09=0x%02x 0x0a=0x%02x 0x0b=0x%02x 0x40=0x%02x\r\n", ret, rdy, g_buf[0], g_buf[1], g_buf[2]);
		if(t_mag.drdy_fail_num++ >= 5)
		{
			t_mag.drdy_fail_num = 0;
			qst_evb_mag_soft_reset();
			qst_evb_mag_enable();
		}
		return;
	}
	else if(rdy & MAG_STATUS_OVFL)
	{
		g_m_out.raw[0] = 32767;
		g_m_out.raw[1] = 32767;
		g_m_out.raw[2] = 32767;	
	}
	else
	{
		t_mag.drdy_fail_num = 0;
		ret = qst_evb_mag_read_reg(MAG_DATA_OUT_X_LSB_REG, g_buf, 6);	// read mag
		if(ret == MAG_FAIL)
		{
			t_mag.data_fail++;
			qst_logi("qst_evb_mag_sensor_data read 0x01 fail! ret=%d", ret);
			return;
		}
		qst_evb_mag_enable_single();

		g_m_out.raw[0] = (short)(((g_buf[1]) << 8) | g_buf[0]);
		g_m_out.raw[1] = (short)(((g_buf[3]) << 8) | g_buf[2]);
		g_m_out.raw[2] = (short)(((g_buf[5]) << 8) | g_buf[4]);	
	}

	g_m_out.mag[0] = (float)(g_m_out.raw[0]*100.0f/g_m_out.mag_lsb);
	g_m_out.mag[1] = (float)(g_m_out.raw[1]*100.0f/g_m_out.mag_lsb);
	g_m_out.mag[2] = (float)(g_m_out.raw[2]*100.0f/g_m_out.mag_lsb);

	t_mag.data_i++;
	if(t_mag.mag_sensor==QST_MAG_QMC6309)
	{
		qst_evb_qmc6309_sr_mode_switch(&g_m_out.raw[0]);
	}
	else
	{
		qst_logi("%f,%f,%f,M-%d\r\n", g_m_out.mag[0], g_m_out.mag[1], g_m_out.mag[2], t_mag.data_i);
	}
}

void qst_mag_entry_sel_sr_switch(void)
{
	int items_data = 0;

	if(t_mag.set.odr != qmc6309_odr)
	{
		return;
	}

	g_m_out.sr_ctl.mode = QMC6309_SET_RESET_ON;
	while(1)
	{
		items_data = 0;
		qst_logi("mag sr switch threshold:\r\n");
		qst_logi("[0]: 15 Gauss\r\n");
		qst_logi("[1]: 20 Gauss\r\n");
		qst_logi("[2]: 25 Gauss\r\n");
		scanf("%d", &items_data);
		if((items_data >= 0)||(items_data <= 3))
		{
			g_m_out.sr_ctl.switch_threshold = (items_data+3)*5000;
			qst_logi("User select mag sr switch threshold : %d Gauss\r\n\r\n", (items_data+3)*5);
			break;
		}
		else
		{
			qst_logi("User select mag sr switch threshold error!!!\r\n");
		}
	}

	while(1)
	{
		items_data = 0;
		qst_logi("input number of (data > threshold) to switch sr(500,..)\r\n");
		scanf("%d", &items_data);
		if((items_data > 0))
		{
			g_m_out.sr_ctl.switch_count = items_data;
			qst_logi("inupt number : %d \r\n\r\n", g_m_out.sr_ctl.switch_count);
			break;
		}
		else
		{
			qst_logi("input number error!!!\r\n");
		}
	}

	while(1)
	{
		items_data = 0;
		qst_logi("When data > threshold switch to:\r\n");
		qst_logi("[%d]: set on only\r\n",		QMC6309_SET_ON);
		qst_logi("[%d]: set/reset off\r\n",		QMC6309_SET_RESET_OFF);
		scanf("%d", &items_data);
		if((items_data == QMC6309_SET_ON)||(items_data == QMC6309_SET_RESET_OFF))
		{
			g_m_out.sr_ctl.swich_mode = (unsigned char)items_data;
			qst_logi("User select switch sr mode : %d\r\n\r\n", items_data);
			break;
		}
		else
		{
			qst_logi("User select mag odr : %d error!!!\r\n", t_mag.sel_sr);	
		}
	}

	while(1)
	{
		items_data = 0;
		qst_logi("input suspend delay(ms)  (when switch sr mode):\r\n");
		scanf("%d", &items_data);
		if((items_data > 0))
		{
			g_m_out.sr_ctl.switch_suspend_delay = items_data;
			qst_logi("inupt delay : %d \r\n\r\n", g_m_out.sr_ctl.switch_suspend_delay);
			break;
		}
		else
		{
			qst_logi("input delay error!!!\r\n");
		}
	}

	while(1)
	{
		items_data = 0;
		qst_logi("input do 1 set-reset delay odr number:\r\n");
		scanf("%d", &items_data);
		if((items_data > 0))
		{
			g_m_out.sr_ctl.one_sr_delay = items_data;
			qst_logi("input do 1 set-reset delay odr number : %d \r\n\r\n", items_data);
			break;
		}
		else
		{
			qst_logi("input do 1 set-reset delay odr number error!!!\r\n");
		}
	}

	
	qst_logi("switch threshold %d mGauss\r\n", g_m_out.sr_ctl.switch_threshold);
	qst_logi("switch to mode %d [1 set on only][3 set/reset off] \r\n", g_m_out.sr_ctl.swich_mode);
	qst_logi("switch suspend delay %d ms\r\n", g_m_out.sr_ctl.switch_suspend_delay);
	qst_logi("switch do 1 set-reset delay odr number: %d \r\n", g_m_out.sr_ctl.one_sr_delay);
}


static void qst_evb_mag_softreset_test(void)
{
	t_mag.fifo_en = 1;
	t_mag.fifo_mode = MAG_FIFO_MODE_STREAM;
	t_mag.fifo_wmk = 6;
	t_mag.data_i = t_mag.data_fail = 0;
	memcpy(g_buf, g_reg_tbl, sizeof(g_reg_tbl));
	while(t_mag.data_i++ < t_mag.data_max)
	{
		t_mag.sel_odr = 0;
		qst_mag_auto_sel_range_osr(t_mag.sel_odr);
		qst_evb_mag_enable();
		qst_delay_ms(10);
		mag_dump_reg(1);
		qst_delay_ms(30);
		qst_logi("qst_evb_mag_soft_reset\r\n");
		qst_evb_mag_soft_reset();
		mag_dump_reg(1);

		for(int i=0; i<t_mag.reg_max; i++)
		{
			if(g_buf[i] != g_reg_tbl[i])
			{
				t_mag.data_fail++;
				break;
			}
		}
		qst_logi("***v[0x%02x]w[0x%02x]d[0x%04x]lot[0x%04x]	softreset count[%d]fail[%d]***\r\n\r\n", t_mag.v_id,t_mag.w_id,t_mag.d_id,t_mag.l_id,t_mag.data_i,t_mag.data_fail);
	}
}

static void qst_evb_mag_enable_reg_protect(unsigned char flag)
{
	unsigned char reg_v = 0;

	qst_evb_mag_read_reg(0x21, &reg_v, 1);
	if(flag)
	{
		reg_v |= 0x80;	// bit7=1
		reg_v &= 0xdf;	// bit5=0
	}
	else
	{
		reg_v &= 0x5f;	// bit5=0 bit7=0
	}	
	qst_evb_mag_write_reg(0x21, reg_v);
}

static void qst_evb_mag_otp_test(void)
{
	memcpy(g_buf, g_reg_tbl, sizeof(g_reg_tbl));
	t_mag.data_i = t_mag.data_fail = 0;

	t_mag.sel_test_i = -1;
	while((t_mag.sel_test_i < 0) || (t_mag.sel_test_i > 1))
	{
		evb_setup_uart_rx(DEBUG_USART, NULL);
		qst_logi("Please select:\r\n");
		qst_logi("[0] reload otp\r\n");
		qst_logi("[1] program otp\r\n");
		qst_logi("[2] reg protect\r\n");
		scanf("%d", &t_mag.sel_test_i);
		if(t_mag.sel_test_i >= 0)
		{
			qst_logi("\r\nUser set counter %d\r\n\r\n", t_mag.sel_test_i);
			break;
		}
	}

	if(t_mag.sel_test_i == 1)
	{
		qst_evb_mag_write_reg(MAG_CTL_REG_ONE, 0x03);	// qst0103 only in work mode
		qst_delay_ms(50);

		if(t_mag.set.odr == maestro_odr)
		{
			g_reg_tbl[MAG_REG_OFFSET_X_LSB] = 0x11;
			g_reg_tbl[MAG_REG_OFFSET_X_MSB] = 0x22;
			g_reg_tbl[MAG_REG_OFFSET_Y_LSB] = 0x33;
			g_reg_tbl[MAG_REG_OFFSET_Y_MSB] = 0x44;
			g_reg_tbl[MAG_REG_OFFSET_Z_LSB] = 0x55;
			g_reg_tbl[MAG_REG_OFFSET_Z_MSB] = 0x66;
			g_reg_tbl[MAG_REG_GAIN_X] = 0x11;
			g_reg_tbl[MAG_REG_GAIN_Y] = 0x22;
			g_reg_tbl[MAG_REG_GAIN_Z] = 0x33;
			g_reg_tbl[MAG_REG_TCS_X] = 0x44;
			g_reg_tbl[MAG_REG_TCS_Y] = 0x55;
			g_reg_tbl[MAG_REG_TCS_Z] = 0x66;
			g_reg_tbl[MAG_REG_TCO_X] = 0x77;
			g_reg_tbl[MAG_REG_TCO_Y] = 0x88;
			g_reg_tbl[MAG_REG_TCO_Z] = 0x99;

			qst_evb_mag_write_reg(MAG_REG_OFFSET_X_LSB, g_reg_tbl[MAG_REG_OFFSET_X_LSB]);
			qst_evb_mag_write_reg(MAG_REG_OFFSET_X_MSB, g_reg_tbl[MAG_REG_OFFSET_X_MSB]);
			qst_evb_mag_write_reg(MAG_REG_OFFSET_Y_LSB, g_reg_tbl[MAG_REG_OFFSET_Y_LSB]);
			qst_evb_mag_write_reg(MAG_REG_OFFSET_Y_MSB, g_reg_tbl[MAG_REG_OFFSET_Y_MSB]);
			qst_evb_mag_write_reg(MAG_REG_OFFSET_Z_LSB, g_reg_tbl[MAG_REG_OFFSET_Z_LSB]);
			qst_evb_mag_write_reg(MAG_REG_OFFSET_Z_MSB, g_reg_tbl[MAG_REG_OFFSET_Z_MSB]);
			qst_evb_mag_write_reg(MAG_REG_GAIN_X, g_reg_tbl[MAG_REG_GAIN_X]);
			qst_evb_mag_write_reg(MAG_REG_GAIN_Y, g_reg_tbl[MAG_REG_GAIN_Y]);
			qst_evb_mag_write_reg(MAG_REG_GAIN_Z, g_reg_tbl[MAG_REG_GAIN_Z]);
			qst_evb_mag_write_reg(MAG_REG_TCS_X, g_reg_tbl[MAG_REG_TCS_X]);
			qst_evb_mag_write_reg(MAG_REG_TCS_Y, g_reg_tbl[MAG_REG_TCS_Y]);
			qst_evb_mag_write_reg(MAG_REG_TCS_Z, g_reg_tbl[MAG_REG_TCS_Z]);
			qst_evb_mag_write_reg(MAG_REG_TCO_X, g_reg_tbl[MAG_REG_TCO_X]);
			qst_evb_mag_write_reg(MAG_REG_TCO_Y, g_reg_tbl[MAG_REG_TCO_Y]);
			qst_evb_mag_write_reg(MAG_REG_TCO_Z, g_reg_tbl[MAG_REG_TCO_Z]);
		}
		else if(t_mag.set.odr == qmc6309_odr)
		{
			g_reg_tbl[0x34] = 0x12;
			g_reg_tbl[0x35] = 0x34;
			g_reg_tbl[0x36] = 0x56;
			g_reg_tbl[0x38] = 0x12;
			g_reg_tbl[0x39] = 0x34;
			g_reg_tbl[0x41] = 0x56;
			g_reg_tbl[0x42] = 0x78;
			qst_evb_mag_write_reg(0x34, g_reg_tbl[0x34]);
			qst_evb_mag_write_reg(0x35, g_reg_tbl[0x35]);
			qst_evb_mag_write_reg(0x36, g_reg_tbl[0x36]);
			qst_evb_mag_write_reg(0x38, g_reg_tbl[0x38]);
			qst_evb_mag_write_reg(0x39, g_reg_tbl[0x39]);
			qst_evb_mag_write_reg(0x41, g_reg_tbl[0x41]);
			qst_evb_mag_write_reg(0x42, g_reg_tbl[0x42]);
		}
		qst_logi("otp program after write reg:\r\n");
		mag_dump_reg(1);
		qst_evb_mag_programme_otp();
		qst_delay_ms(50);
		qst_evb_mag_soft_reset();		
		qst_logi("otp program after 0x28=0x01 and softreset:\r\n");
		mag_dump_reg(1);
	}
	else if(t_mag.sel_test_i == 2)
	{
		t_mag.data_max = 10;

		qst_evb_mag_write_reg(MAG_CTL_REG_ONE, 0x03);	// qst0103 only in work mode
		qst_delay_ms(50);
		qst_evb_mag_enable_reg_protect(1);
		while(t_mag.data_i++ < t_mag.data_max)
		{		
			qst_logi("wirte reg 0x38=0x11 0x39=0x22 0x41=0x55 0x42=0x66\r\n");
			qst_evb_mag_write_reg(0x38, 0x11);
			qst_evb_mag_write_reg(0x39, 0x22);
			qst_evb_mag_write_reg(0x41, 0x55);
			qst_evb_mag_write_reg(0x42, 0x66);
			mag_dump_reg(0);
		}
		qst_evb_mag_enable_reg_protect(0);
	}
	else
	{
		unsigned int data_i = 0;

		while(data_i++ < t_mag.data_max)
		{
			qst_evb_mag_enable();
			qst_delay_ms(30);

			qst_evb_mag_reload_otp();
			qst_delay_ms(50);
			mag_dump_reg(1);
			qst_delay_ms(10);
			for(int i=0x30; i<t_mag.reg_max; i++)
			{
				if(g_buf[i] != g_reg_tbl[i])
				{
					t_mag.data_fail++;
					break;
				}
			}
			qst_logi("***chipId[0x%02x] v[0x%02x]w[0x%02x]d[0x%04x] reload otp count[%d]fail[%d]***\r\n\r\n", t_mag.chipid,t_mag.v_id,t_mag.w_id,t_mag.d_id,data_i,t_mag.data_fail);
		}
	}
}

static int qst_evb_mag_selftest(void)
{
	int selftest_result = 0;
	int selftest_retry = 0;
	signed char  st_data[3];
	unsigned char abs_data[3];
	unsigned char rdy = 0x00;
	int t1 = 0;
	int ret = 0;

	selftest_retry = 0;
	selftest_result = 0;
	while((selftest_result == 0)&&(selftest_retry<3))
	{
		selftest_retry++;
		ret = qst_evb_mag_write_reg(MAG_CTL_REG_ONE, 0x00);
		//qst_delay_ms(2);
		if((t_mag.mag_sensor == QST_MAG_MAESTRO_1V)||(t_mag.mag_sensor == QST_MAG_MAESTRO_2H))
		{
			ret = qst_evb_mag_write_reg(MAG_CTL_REG_TWO, 0x01);
			//qst_delay_ms(2);
			ret = qst_evb_mag_write_reg(MAG_CTL_REG_ONE, 0x17);
		}
		else
		{
			ret = qst_evb_mag_write_reg(MAG_CTL_REG_TWO, 0x00);
			//qst_delay_ms(2);
			ret = qst_evb_mag_write_reg(MAG_CTL_REG_ONE, 0x03);
		}
		qst_delay_ms(20);
		ret = qst_evb_mag_write_reg(0x0e, 0x80);
		if(ret == MAG_FAIL)
		{
			continue;
		}

		rdy = 0x00;
		t1 = 0;
		while(!(rdy & 0x04))
		{
			qst_delay_ms(10);
			qst_evb_mag_read_reg(MAG_STATUS_REG, &rdy, 1);
			if(t1++ > 50)
			{
				break;
			}
		}

		if(rdy & 0x04)
		{			
			ret = qst_evb_mag_read_reg(QMC6309_DATA_OUT_ST_X, (unsigned char*)st_data, 3);
			if(ret == MAG_FAIL)
			{
				continue;
			}
		}
		else
		{
			qst_logi("mag selftest drdy fail[0x%02x]!\r\n", rdy);
			st_data[0] = st_data[1] = st_data[2] = 0;
		}
		abs_data[0] = QMC6309_ABS(st_data[0]);
		abs_data[1] = QMC6309_ABS(st_data[1]);
		abs_data[2] = QMC6309_ABS(st_data[2]);

#if defined(FPGA)
		selftest_result = 1;
		(void)abs_data[0];
		(void)abs_data[1];
		(void)abs_data[2];
#else
		if(	((abs_data[0] < QMC6309_SELFTEST_MAX_X) && (abs_data[0] > QMC6309_SELFTEST_MIN_X))
			&& ((abs_data[1] < QMC6309_SELFTEST_MAX_Y) && (abs_data[1] > QMC6309_SELFTEST_MIN_Y))
			&& ((abs_data[2] < QMC6309_SELFTEST_MAX_Z) && (abs_data[2] > QMC6309_SELFTEST_MIN_Z)) )
	    {
	        selftest_result = 1;
	    }
		else
		{
			selftest_result = 0;
		}
#endif
	}
	qst_evb_mag_write_reg(MAG_CTL_REG_TWO, 0x00);
	qst_evb_mag_write_reg(MAG_CTL_REG_ONE, 0x00);
#if defined(FPGA)
	qst_logi("%d,%d,%d,M-%d, Selftest[fpga]-%s\r\n",st_data[0],st_data[1],st_data[2],t_mag.data_i,(selftest_result?"pass":"fail"));
#else
	qst_logi("%d,%d,%d,M-%d, Selftest-%s\r\n",st_data[0],st_data[1],st_data[2],t_mag.data_i,(selftest_result?"pass":"fail"));
#endif

	return selftest_result;
}


static void qst_evb_mag_selftest_hldr(void)
{
	qst_evb_mag_selftest();	
	if(t_mag.data_i++ >= t_mag.data_max)
	{
		evb_setup_timer(TIM2, NULL, 500, DISABLE);
		qst_logi("mag selftest done!\r\n");
	}
}

#if defined(QMC6308)
void qst_evb_mag_selftest_qmc6308(void)
{
	int selftest_result = 0;
	int selftest_retry = 0;
	int raw_a[3];
	int raw_b[3];
	int hdata[3];
	unsigned char rx_buf[6] = {0};
	unsigned char rdy = 0x00;
	int t1 = 0;
	int ret = 0;

	while((selftest_result == 0)&&(selftest_retry<3))
	{
		selftest_retry++;

		qst_evb_mag_write_reg(MAG_CTL_REG_ONE, 0x00);
		qst_evb_mag_write_reg(MAG_CTL_REG_TWO, 0x00);
		qst_evb_mag_write_reg(MAG_CTL_REG_ONE, 0x03);
		qst_delay_ms(1);
	
		while(!(rdy & 0x03))
		{
			rdy = QMC6308_STATUS_REG;
			ret = qst_evb_mag_read_reg(QMC6308_STATUS_REG, &rdy, 1);
			qst_delay_ms(1);
			if(t1++ > 50)
				continue;
		}
	
		ret = qst_evb_mag_read_reg(MAG_DATA_OUT_X_LSB_REG, rx_buf, 6);
		if(ret == 0)
			continue;
	
		raw_a[0] = (short)(((rx_buf[1]) << 8) | rx_buf[0]);
		raw_a[1] = (short)(((rx_buf[3]) << 8) | rx_buf[2]);
		raw_a[2] = (short)(((rx_buf[5]) << 8) | rx_buf[4]);
	
		qst_evb_mag_write_reg(MAG_CTL_REG_ONE, 0x00);
		qst_delay_ms(1);
		qst_evb_mag_write_reg(0x0b, 0x40);
		qst_evb_mag_write_reg(MAG_CTL_REG_ONE, 0x03);
		qst_delay_ms(10);
		
		t1 = 0;
		rdy = 0;
		while(!(rdy & 0x03))
		{
			rdy = QMC6308_STATUS_REG;
			ret = qst_evb_mag_read_reg(QMC6308_STATUS_REG, &rdy, 1);
			qst_delay_ms(1);
			if(t1++ > 50)
				continue;
		}
		ret = qst_evb_mag_read_reg(MAG_DATA_OUT_X_LSB_REG, rx_buf, 6);
		if(ret == 0)
			continue;

		raw_b[0] = (short)(((rx_buf[1]) << 8) | rx_buf[0]);
		raw_b[1] = (short)(((rx_buf[3]) << 8) | rx_buf[2]);
		raw_b[2] = (short)(((rx_buf[5]) << 8) | rx_buf[4]);
	
		hdata[0] = QST_ABS(raw_a[0]-raw_b[0]);
		hdata[1] = QST_ABS(raw_a[1]-raw_b[1]);
		hdata[2] = QST_ABS(raw_a[2]-raw_b[2]);
	
		qst_evb_mag_write_reg(MAG_CTL_REG_ONE, 0x00);
		if( ((hdata[0] < QMC6308_SELFTEST_MAX_X) && (hdata[0] > QMC6308_SELFTEST_MIN_X))
		&&	((hdata[1] < QMC6308_SELFTEST_MAX_Y) && (hdata[1] > QMC6308_SELFTEST_MIN_Y))
		&&	((hdata[2] < QMC6308_SELFTEST_MAX_Z) && (hdata[2] > QMC6308_SELFTEST_MIN_Z)) )
	    {
	        selftest_result = 1;
	    }
		else
		{
			selftest_result = 0;
		}
	}	
	qst_logi("%d,%d,%d,M-%d, Selftest-%s\r\n",hdata[0],hdata[1],hdata[2],t_mag.data_i,(selftest_result?"pass":"fail"));
	if(t_mag.data_i++ >= t_mag.data_max)
	{
		evb_setup_timer(TIM2, NULL, 500, DISABLE);
		qst_logi("mag selftest done!\r\n");
	}
}
#endif


static void qst_evb_mag_factory_test(void)
{
#define FACTORY_SELFTEST
#define FACTORY_NOISE
#define FACTORY_NOISE_MAX	1.0f
#define FACTORY_LOOP_NUM	1

	short i = 0;
	unsigned char srt_out = 1;	
	unsigned char crc_out = 1;
	unsigned char std_out = 1;
	unsigned char sft_out = 1;

	t_mag.sel_mag_i = 0;
	for(t_mag.sel_mag_i=0; t_mag.sel_mag_i<t_mag.mag_num; t_mag.sel_mag_i++)
	{
		srt_out = 1;
		crc_out = 1;
		std_out = 1;
		sft_out = 1;
		i2c_sw_gpio_config(t_mag.sel_mag_i);

		// softreset
		memset(g_reg_tbl, 0, sizeof(g_reg_tbl));
		qst_evb_mag_soft_reset();
		for(i=0; i<=t_mag.reg_max; i++)
		{
			qst_evb_mag_read_reg((unsigned char)i, &g_reg_tbl[i], 1);
			qst_delay_us(100);
		}
		t_mag.v_id = g_reg_tbl[0x12];
		t_mag.w_id = g_reg_tbl[0x37];
		t_mag.d_id = (unsigned short)((g_reg_tbl[0x39]<<8)|g_reg_tbl[0x38]);
		t_mag.l_id = (unsigned short)(((g_reg_tbl[0x42]&0x3f)<<8)|g_reg_tbl[0x41]);

		t_mag.sel_odr = 0;
		qst_mag_auto_sel_range_osr(t_mag.sel_odr);
		qst_evb_mag_enable();
		qst_delay_ms(2);

		t_mag.data_i = 0;
		t_mag.data_max = FACTORY_LOOP_NUM;
		while((t_mag.data_i++ < t_mag.data_max) && srt_out && crc_out)
		{
			memset(g_buf, 0, sizeof(g_buf));
			qst_evb_mag_soft_reset();
			for(i=0; i<=t_mag.reg_max; i++)
			{
				qst_evb_mag_read_reg((unsigned char)i, &g_buf[i], 1);
				qst_delay_us(10);
				if(g_buf[i] != g_reg_tbl[i])
				{
					srt_out = 0;
					break;
				}
			}					
			if(g_buf[0x09] & 0x20)
			{
				crc_out = 0;
			}
		}		
		qst_logi("[%s]******SOFTRESET[%d]CRC[%d]\r\n",g_config_info, srt_out, crc_out);
		// softreset
#if defined(FACTORY_NOISE)
		// Noise
		for(t_mag.sel_odr=0; t_mag.sel_odr<t_mag.set.odr_num; t_mag.sel_odr++)
		{
			int ret = 0;
			unsigned char t1=0;
			unsigned char rdy = 0;

			qst_evb_mag_soft_reset();
			if(t_mag.set.odr[t_mag.sel_odr].freq < 100)
			{
				break;
			}
			qst_mag_auto_sel_range_osr(t_mag.sel_odr);
			qst_evb_mag_enable();

			t_mag.data_i = 0;
			t_mag.data_max = t_mag.set.odr[t_mag.sel_odr].freq * 3;
				
			while(t_mag.data_i++ <= t_mag.data_max)
			{			
				rdy = 0x00;
				t1 = 0;
				qst_delay_ms(t_mag.delay);
				ret = qst_evb_mag_read_reg(MAG_STATUS_REG, &rdy, 1);
				while(!(rdy & MAG_STATUS_DRDY) && (t1++ < 5))
				{
					qst_delay_ms(1);
					ret = qst_evb_mag_read_reg(MAG_STATUS_REG, &rdy, 1);
				}
				
				if((ret == MAG_FAIL)||(!(rdy & MAG_STATUS_DRDY)))
				{
					t_mag.drdy_fail_num++;
					qst_evb_mag_read_reg(MAG_CTL_REG_ONE, g_buf, 2);
					qst_evb_mag_read_reg(0x40, &g_buf[2], 1);
					qst_logi("mag drdy fail(ret=%d)! 0x09=0x%02x 0x0a=0x%02x 0x0b=0x%02x 0x40=0x%02x\r\n", ret, rdy, g_buf[0], g_buf[1], g_buf[2]);
					if(t_mag.drdy_fail_num >= 5)
					{
						t_mag.drdy_fail_num = 0;
						std_out = 0;
						break;
					}
					t_mag.data_fail++;
				}
				else
				{
					ret = qst_evb_mag_read_reg(MAG_DATA_OUT_X_LSB_REG, g_buf, 6);	// read mag
					if(ret == MAG_FAIL)
					{
						t_mag.data_fail++;
						qst_logi("qst_evb_mag_factory_test read 0x01 fail! ret=%d", ret);
						continue;
					}
					qst_evb_mag_enable_single();

#if defined(MAESTRO1_USER_SET)
					pset.bit.period_set = 0x3f;
					pset.bit.user_set = 1;
					qst_evb_mag_write_reg(MAG_CTL_REG_PSET, pset.value);
#endif
					g_m_out.raw[0] = (short)(((g_buf[1]) << 8) | g_buf[0]);
					g_m_out.raw[1] = (short)(((g_buf[3]) << 8) | g_buf[2]);
					g_m_out.raw[2] = (short)(((g_buf[5]) << 8) | g_buf[4]);
					g_m_out.mag[0] = (float)(g_m_out.raw[0]*100.0f/g_m_out.mag_lsb);
					g_m_out.mag[1] = (float)(g_m_out.raw[1]*100.0f/g_m_out.mag_lsb);
					g_m_out.mag[2] = (float)(g_m_out.raw[2]*100.0f/g_m_out.mag_lsb);
					getStandardDeviation3(&mStd3, g_m_out.mag, g_m_out.std);
					t_mag.data_fail = 0;
				}
			}

			qst_evb_mag_write_reg(MAG_CTL_REG_ONE, 0x00);
			qst_delay_ms(2);
			qst_logi("[%s]******NOISE,%f,%f,%f\r\n",g_config_info, g_m_out.std[0], g_m_out.std[1], g_m_out.std[2]);
			if((g_m_out.std[0] > FACTORY_NOISE_MAX) || (g_m_out.std[1] > FACTORY_NOISE_MAX) || (g_m_out.std[2] > FACTORY_NOISE_MAX))
			{
				std_out = 0;
			}
		}
		// noise
#endif
#if defined(FACTORY_SELFTEST)
		// selftest
		t_mag.data_i = 0;
		t_mag.data_max = FACTORY_LOOP_NUM;
		while(t_mag.data_i++ < t_mag.data_max)
		{
			sft_out = qst_evb_mag_selftest();
			if(sft_out == 0)
			{
				break;
			}
			qst_delay_ms(5);
		}
		// selftest
#endif
		qst_evb_mag_soft_reset();

		qst_logi("%s W-Id[0x%02x]D-Id[0x%04x]L-Id[0x%04x] ", g_config_info, t_mag.w_id, t_mag.d_id, t_mag.l_id);
		qst_logi("SOFTRESET[%s]CRC[%s]",srt_out?"PASS":"FAIL", crc_out?"PASS":"FAIL");
#if defined(FACTORY_NOISE)
		qst_logi("NOISE[%s]",std_out?"PASS":"FAIL");
#endif
#if defined(FACTORY_SELFTEST)
		qst_logi("SELFTEST[%s]",sft_out?"PASS":"FAIL");
#endif
		qst_logi("\r\n\r\n");
	}

	((void)srt_out);
	((void)std_out);
	((void)sft_out);
}


#if defined(FPGA)

static void qst_evb_mag_set_bypass(unsigned char bypass_value, int bypass_en)
{
#if defined(MAESTRO1)
	qst_evb_mag_read_reg(0x3d, &t_mag.reg, 1);
	t_mag.otp_bypass = bypass_en;
	if(bypass_en)
	{
		qst_evb_mag_write_reg(0x3d, (t_mag.reg|bypass_value));
	}
	else
	{
		qst_evb_mag_write_reg(0x3d, (t_mag.reg&(~bypass_value)));
	}
#endif
}


static void qst_evb_mag_auto_apply_coarse_gain(void)
{
	t_mag.data_i++;
	if(t_mag.data_i > 2000)
	{
		unsigned char xy_gain2 = 0;		
		unsigned char z_gain2 = 0;

		t_mag.data_i = 0;
		t_mag.config_id++;

		if(t_mag.config_id < sizeof(mag_coarse_gain)/sizeof(mag_coarse_gain[0]))
		{
			qst_evb_mag_read_reg(0x3a, &xy_gain2, 1);
			qst_evb_mag_read_reg(0x37, &z_gain2, 1);

			xy_gain2 &= 0xfc;
			z_gain2 &= 0x3f;

			xy_gain2 |= mag_coarse_gain[t_mag.config_id];			
			z_gain2 |= (mag_coarse_gain[t_mag.config_id] << 6);
			
			qst_evb_mag_write_reg(0x3a, xy_gain2);
			qst_evb_mag_write_reg(0x37, z_gain2);
		}
		else
		{
			qst_logi("apply coarse gain test done!");			
			evb_setup_timer(TIM2, NULL, t_mag.delay, DISABLE);
		}
	}
	else
	{	
		qst_logi("coarse gain,%d,", mag_coarse_gain[t_mag.config_id]);
		if(mag_get_drdy())
		{
			mag_read_raw();
		}
	}
}

static void qst_evb_mag_auto_apply_gain(void)
{
	#define GAIN_STEP 	1
#if defined(MAESTRO1)
	#define GAIN_MAX	1023
	#define GAIN_MIN	-1024
#else
	#define GAIN_MAX	127
	#define GAIN_MIN	-128
#endif

	if(mag_gain[0] >= GAIN_MAX)
	{
		mag_gain[0] = GAIN_MIN;
		mag_gain[1] = GAIN_MIN;
		mag_gain[2] = GAIN_MIN;	
	}
	else if((mag_gain[0]+GAIN_STEP) >= GAIN_MAX)
	{
		mag_gain[0] = GAIN_MAX;
		mag_gain[1] = GAIN_MAX;
		mag_gain[2] = GAIN_MAX;
		t_mag.config_id++;
	}
	else
	{
		mag_gain[0] += GAIN_STEP;
		mag_gain[1] += GAIN_STEP;
		mag_gain[2] += GAIN_STEP;
	}

#if defined(MAESTRO1)
	qst_evb_mag_apply_offset_gain(mag_gain, mag_offset);
#else
	qst_evb_mag_write_reg(MAG_REG_GAIN_X, (unsigned char)(mag_gain[0] & 0xff));
	qst_evb_mag_write_reg(MAG_REG_GAIN_Y, (unsigned char)(mag_gain[1] & 0xff));
	qst_evb_mag_write_reg(MAG_REG_GAIN_Z, (unsigned char)(mag_gain[2] & 0xff));
#endif

	if(t_mag.otp_bypass)
		qst_logi("bypass-gain,%d,%d,%d,", mag_gain[0],mag_gain[1],mag_gain[2]);
	else		
		qst_logi("gain,%d,%d,%d,", mag_gain[0],mag_gain[1],mag_gain[2]);

	if(mag_get_drdy())
	{
		mag_read_raw();
	}
	if(t_mag.config_id >= 3)
	{
		t_mag.config_id = 0;
		qst_logi("apply gain test done!");
		evb_setup_timer(TIM2, NULL, t_mag.delay, DISABLE);
	}
}

static void qst_evb_mag_auto_apply_offset(void)
{
	#define OFFSET_STEP		16

	if(mag_offset[0] == 32767)
	{
		mag_offset[0] = -32768;
		mag_offset[1] = -32768;
		mag_offset[2] = -32768;
	}
	else if((int)(mag_offset[0]+OFFSET_STEP) > 32767)
	{
		mag_offset[0] = 32767;
		mag_offset[1] = 32767;
		mag_offset[2] = 32767;
		t_mag.config_id++;
	}
	else
	{
		mag_offset[0] += OFFSET_STEP;
		mag_offset[1] += OFFSET_STEP;
		mag_offset[2] += OFFSET_STEP;
	}
	qst_evb_mag_write_reg(MAG_REG_OFFSET_X_LSB, (unsigned char)(mag_offset[0]&0xff));
	qst_evb_mag_write_reg(MAG_REG_OFFSET_X_MSB, (unsigned char)((mag_offset[0]>>8)&0xff));
	qst_evb_mag_write_reg(MAG_REG_OFFSET_Y_LSB, (unsigned char)(mag_offset[1]&0xff));
	qst_evb_mag_write_reg(MAG_REG_OFFSET_Y_MSB, (unsigned char)((mag_offset[1]>>8)&0xff));
	qst_evb_mag_write_reg(MAG_REG_OFFSET_Z_LSB, (unsigned char)(mag_offset[2]&0xff));
	qst_evb_mag_write_reg(MAG_REG_OFFSET_Z_MSB, (unsigned char)((mag_offset[2]>>8)&0xff));
	if(t_mag.otp_bypass)
		qst_logi("offset-bypass,%d,%d,%d,", mag_offset[0],mag_offset[1],mag_offset[2]);
	else
		qst_logi("offset,%d,%d,%d,", mag_offset[0],mag_offset[1],mag_offset[2]);
	qst_delay_ms(1);
	if(mag_get_drdy())
	{
		mag_read_raw();
	}

	if(t_mag.config_id >= 3)
	{
		t_mag.config_id = 0;
		qst_logi("apply offset test done!");
		evb_setup_timer(TIM2, NULL, t_mag.delay, DISABLE);
	}
}

static void qst_evb_mag_tco_test(unsigned int bypass)
{
	#define MAG_OTP_TCO_STEP		16
	g_t0 = FPGA_T0_MIN;
	mag_otp_tco_in = -128;

	for(mag_otp_tco_in=-128; mag_otp_tco_in<=(128+MAG_OTP_TCO_STEP); mag_otp_tco_in+=MAG_OTP_TCO_STEP)
	{
		if(mag_otp_tco_in > 127)
		{
			mag_otp_tco_in = 127;
		}
		qst_delay_ms(2);
		if(mag_get_drdy())
		{
			mag_read_raw();
		}

		t_mag.raw_data[2][0] = g_m_out.raw[0];
		t_mag.raw_data[2][1] = g_m_out.raw[1];
		t_mag.raw_data[2][2] = g_m_out.raw[2];
		t_mag.raw_temp[2] = g_m_out.temp_raw;

		qst_evb_mag_soft_reset();
		mag_tco[0] = mag_tco[1] = mag_tco[2] = mag_otp_tco_in;
		if(bypass)
			qst_evb_mag_set_bypass(0x10, 1);
		else
			qst_evb_mag_set_bypass(0x10, 0);
			
		qst_evb_mag_apply_tco_tcs(mag_tco, mag_tcs);
		qst_delay_ms(2);
		qst_evb_mag_enable();
	
		for(g_t0=FPGA_T0_MIN; g_t0<=FPGA_T0_MAX; g_t0++)
		{
			qst_evb_mag_apply_t0(g_t0);
			qst_delay_ms(1);
			if(t_mag.otp_bypass)
				qst_logi("bypass tco-test,otp,%d,t0,%d,",mag_otp_tco_in, g_t0);
			else
				qst_logi("tco-test,otp,%d,t0,%d,",mag_otp_tco_in, g_t0);
			if(mag_get_drdy())
			{
				mag_read_raw();
			}
			else
			{
				return;
			}

			if(g_t0 == FPGA_T0_MIN)
			{
				t_mag.raw_data[0][0] = g_m_out.raw[0];
				t_mag.raw_data[0][1] = g_m_out.raw[1];
				t_mag.raw_data[0][2] = g_m_out.raw[2];
				t_mag.raw_temp[0] = g_m_out.temp_raw;
			}
			else if(g_t0 == FPGA_T0_MAX)
			{
				t_mag.raw_data[1][0] = g_m_out.raw[0];
				t_mag.raw_data[1][1] = g_m_out.raw[1];
				t_mag.raw_data[1][2] = g_m_out.raw[2];
				t_mag.raw_temp[1] = g_m_out.temp_raw;
			//	qst_logi("tco-calc,otp,%d,t0,%d,M-ORI,%d,%d,%d,M-OFT,%d,%d,%d,T-OFT,%d\r\n",	
			//			mag_otp_tco_in,	g_t0, t_mag.raw_data[2][0],t_mag.raw_data[2][1],t_mag.raw_data[2][2],
			//			(t_mag.raw_data[1][0]-t_mag.raw_data[0][0]),(t_mag.raw_data[1][1]-t_mag.raw_data[0][1]),(t_mag.raw_data[1][2]-t_mag.raw_data[0][2]),
			//			(t_mag.raw_temp[1]-t_mag.raw_temp[0]));
			}
			qst_delay_ms(5);
		}

		if(mag_otp_tco_in >= 127)
		{
			break;
		}
	}

	qst_logi("qst_evb_mag_tco_test done\r\n");
}


static void qst_evb_mag_tcs_test(unsigned int bypass)
{
#if defined(MAESTRO1)
	#define MAG_OTP_TCS_STEP		16
	g_t0 = FPGA_T0_MIN;
	mag_otp_tcs_in = -128;

	for(mag_otp_tcs_in=-128; mag_otp_tcs_in<=(128+MAG_OTP_TCO_STEP); mag_otp_tcs_in+=MAG_OTP_TCO_STEP)
	{
		if(mag_otp_tcs_in > 127)
		{
			mag_otp_tcs_in = 127;
		}
		qst_delay_ms(2);
		if(mag_get_drdy())
		{
			mag_read_raw();
		}

		t_mag.raw_data[2][0] = g_m_out.raw[0];
		t_mag.raw_data[2][1] = g_m_out.raw[1];
		t_mag.raw_data[2][2] = g_m_out.raw[2];
		t_mag.raw_temp[2] = g_m_out.temp_raw;

		qst_evb_mag_soft_reset();
		mag_tcs[0] = mag_tcs[1] = mag_tcs[2] = mag_otp_tcs_in;
		if(bypass)
			qst_evb_mag_set_bypass(0x10, 1);
		else
			qst_evb_mag_set_bypass(0x10, 0);
		qst_evb_mag_apply_tco_tcs(mag_tco, mag_tcs);
		qst_delay_ms(2);
		qst_evb_mag_enable();
	
		for(g_t0=FPGA_T0_MIN; g_t0<=FPGA_T0_MAX; g_t0++)
		{
			qst_evb_mag_apply_t0(g_t0);
			qst_delay_ms(1);
			if(t_mag.otp_bypass)
				qst_logi("bypass tcs-test,otp,%d,t0,%d,",mag_otp_tcs_in, g_t0);
			else
				qst_logi("tcs-test,otp,%d,t0,%d,",mag_otp_tcs_in, g_t0);
			if(mag_get_drdy())
			{
				mag_read_raw();
			}
			else
			{
				return;
			}

			if(g_t0 == FPGA_T0_MIN)
			{
				t_mag.raw_data[0][0] = g_m_out.raw[0];
				t_mag.raw_data[0][1] = g_m_out.raw[1];
				t_mag.raw_data[0][2] = g_m_out.raw[2];
				t_mag.raw_temp[0] = g_m_out.temp_raw;
			}
			else if(g_t0 == FPGA_T0_MAX)
			{
				t_mag.raw_data[1][0] = g_m_out.raw[0];
				t_mag.raw_data[1][1] = g_m_out.raw[1];
				t_mag.raw_data[1][2] = g_m_out.raw[2];
				t_mag.raw_temp[1] = g_m_out.temp_raw;
			//	qst_logi("tco-calc,otp,%d,t0,%d,M-ORI,%d,%d,%d,M-OFT,%d,%d,%d,T-OFT,%d\r\n",	
			//			mag_otp_tco_in,	g_t0, t_mag.raw_data[2][0],t_mag.raw_data[2][1],t_mag.raw_data[2][2],
			//			(t_mag.raw_data[1][0]-t_mag.raw_data[0][0]),(t_mag.raw_data[1][1]-t_mag.raw_data[0][1]),(t_mag.raw_data[1][2]-t_mag.raw_data[0][2]),
			//			(t_mag.raw_temp[1]-t_mag.raw_temp[0]));
			}
			qst_delay_ms(5);
		}

		if(mag_otp_tcs_in >= 127)
		{
			break;
		}
	}
#else
	int count = 0;
	unsigned char v_3d,v_46;

	qst_evb_mag_soft_reset();
	qst_delay_ms(2);

	qst_evb_mag_enable();
	qst_delay_ms(50);
	qst_logi("OTP 0:\r\n");
	for(count=0; count<5; count++)
	{
		qst_delay_ms(5);
		if(mag_get_drdy())
		{
			mag_read_raw();
		}
	}

	//qst_evb_mag_write_reg(0x37, 0x00);
	//qst_evb_mag_write_reg(0x3F, 0x00);
	//qst_evb_mag_write_reg(0x45, 0x00);

	//v_3d = 0x40;
	//v_46 = 0x01;
	v_3d = 0x00;
	v_46 = 0x03;
	qst_evb_mag_write_reg(0x3d, v_3d);
	qst_evb_mag_write_reg(0x46, v_46);

	qst_logi("OTP 0x3d=0x%02x 0x46=0x%02x:\r\n", v_3d, v_46);
	for(g_t0=FPGA_T0_MIN; g_t0<=FPGA_T0_MAX; g_t0++)
	{
		qst_evb_mag_apply_t0(g_t0);
		qst_delay_ms(10);
		//if(t_mag.otp_bypass)
		//	qst_logi("bypass tcs-test,otp,%d,t0,%d,",mag_otp_tcs_in, g_t0);
		//else
		//	qst_logi("tcs-test,otp,%d,t0,%d,",mag_otp_tcs_in, g_t0);
		qst_logi("tcs-test,t0,%d,", g_t0);
		if(mag_get_drdy())
		{
			mag_read_raw();
			//mag_read_temp(&g_m_out.temp_raw);
			//qst_logi("%d\r\n", g_m_out.temp_raw);
		}
		//qst_delay_ms(5);
	}

#endif

	qst_logi("qst_evb_mag_tco_test done\r\n");
}

static void qst_evb_mag_kmtx_test(void)
{
	#define KMTX_XY
	#define KMTX_AVG_NUM		10
	int kmtx_value = 0;

#if defined(MAESTRO2H)
	for(kmtx_value=-128; kmtx_value<=127; kmtx_value++)
#else
	for(kmtx_value=-32; kmtx_value<=31; kmtx_value++)
#endif
	{
		int	raw_sum[3] = {0, 0, 0};
		int count = 0;
		short calc_data[3] = {0, 0, 0};

		qst_evb_mag_soft_reset();
		//qst_evb_mag_write_reg(MAG_CTL_REG_TWO, 0x80);
		//qst_evb_mag_write_reg(MAG_CTL_REG_TWO, 0x00);
		//qst_delay_ms(5);

	#if defined(MAESTRO1)
		qst_evb_mag_apply_kmtx(false, mag_kmtx_xy, mag_kmtx_xy_z);
		qst_evb_mag_apply_offset_gain(mag_gain, mag_offset);
	#endif
		qst_evb_mag_enable();

		t_mag.raw_data[2][0] = t_mag.raw_data[2][1] = t_mag.raw_data[2][2] = 0;
		raw_sum[0] = raw_sum[1] = raw_sum[2] = 0;
		for(count=0; count<KMTX_AVG_NUM; count++)
		{
			qst_delay_ms(20);
			if(mag_get_drdy())
			{
				mag_read_raw();
			}
			t_mag.raw_data[2][0] = g_m_out.raw[0];
			t_mag.raw_data[2][1] = g_m_out.raw[1];
			t_mag.raw_data[2][2] = g_m_out.raw[2];
			t_mag.raw_temp[2] = g_m_out.temp_raw;

			raw_sum[0] += g_m_out.raw[0];
			raw_sum[1] += g_m_out.raw[1];
			raw_sum[2] += g_m_out.raw[2];
		}

		t_mag.raw_data[2][0] = raw_sum[0]/KMTX_AVG_NUM;
		t_mag.raw_data[2][1] = raw_sum[1]/KMTX_AVG_NUM;
		t_mag.raw_data[2][2] = raw_sum[2]/KMTX_AVG_NUM;

#if defined(KMTX_XY)
		mag_kmtx_xy[0] = mag_kmtx_xy[1] = mag_kmtx_xy[2] = mag_kmtx_xy[3] = kmtx_value;
		qst_evb_mag_apply_kmtx(true, mag_kmtx_xy, mag_kmtx_xy_z);
		qst_delay_ms(20);	
		calc_data[0] = t_mag.raw_data[2][0] - (t_mag.raw_data[2][0]*mag_kmtx_xy[0]/512) - (t_mag.raw_data[2][1]*mag_kmtx_xy[1]/128);
		calc_data[1] = t_mag.raw_data[2][1] - (t_mag.raw_data[2][0]*mag_kmtx_xy[2]/128) - (t_mag.raw_data[2][1]*mag_kmtx_xy[3]/512);
		qst_logi("KMTX-XY,OTP,%d,ORI,%d,%d,%d,EXCEPT,%d,%d,", kmtx_value,t_mag.raw_data[2][0], t_mag.raw_data[2][1],t_mag.raw_data[2][2], calc_data[0], calc_data[1]);
#else
		mag_kmtx_xy_z[0] = mag_kmtx_xy_z[1] = kmtx_value;
		qst_evb_mag_apply_kmtx(true, mag_kmtx_xy, mag_kmtx_xy_z);
		qst_delay_ms(20);
		calc_data[0] = t_mag.raw_data[2][2] - (t_mag.raw_data[2][0]*mag_kmtx_xy_z[0]/128) - (t_mag.raw_data[2][1]*mag_kmtx_xy_z[1]/128);
		qst_logi("KMTX-XY2Z,OTP,%d,ORI,%d,%d,%d,EXCEPT,%d,", kmtx_value,t_mag.raw_data[2][0],t_mag.raw_data[2][1],t_mag.raw_data[2][2], calc_data[0]);
#endif
		if(mag_get_drdy())
		{
			mag_read_raw();
		}
	}
}

// y_out = y_raw - ((kc_xiyo/256.f)*x_raw);
// z_out = z_raw - ((kx/128.f)*x_raw) - ((ky/128.f)*y_raw);
static void qst_evb_mag_cross_sensitivity_test(void)
{
	#define CROSS_SVVT_MIN		-32
	#define CROSS_SVVT_MAX		31
	#define CROSS_AVG_NUM		20

	reg_46_t	reg_46;
	reg_3a_t	reg_3a;	
	reg_3b_t	reg_3b;
	short calc_data[3] = {0, 0, 0};
	int raw_sum[3] = {0, 0, 0};
	int count = 0;

	while(count++ < 5)
	{
		qst_evb_mag_soft_reset();
		qst_evb_mag_enable();
		raw_sum[0] = raw_sum[1] = raw_sum[2] = 0;
		for(int i=0; i<CROSS_AVG_NUM; i++)
		{
			qst_delay_ms(20);
			if(mag_get_drdy())
			{
				mag_read_raw();
			}
		
			raw_sum[0] += g_m_out.raw[0];
			raw_sum[1] += g_m_out.raw[1];
			raw_sum[2] += g_m_out.raw[2];
		}
		t_mag.raw_data[2][0] = raw_sum[0]/CROSS_AVG_NUM;
		t_mag.raw_data[2][1] = raw_sum[1]/CROSS_AVG_NUM;
		t_mag.raw_data[2][2] = raw_sum[2]/CROSS_AVG_NUM;


		qst_evb_mag_read_reg(0x46, &reg_46.value, 1);
		for(mag_kc_xiyo=CROSS_SVVT_MIN; mag_kc_xiyo<=CROSS_SVVT_MAX; mag_kc_xiyo++)
		{
			reg_46.kc_xiyo = mag_kc_xiyo;
			qst_evb_mag_write_reg(0x46, reg_46.value);
			calc_data[1] = t_mag.raw_data[2][1] - t_mag.raw_data[2][0]*mag_kc_xiyo/256;
			qst_logi("cross kc_xiyo,%d,y_calc,%d,", mag_kc_xiyo, calc_data[1]);
			qst_delay_ms(20);
			if(mag_get_drdy())
			{
				mag_read_raw();
			}
		}
		reg_46.kc_xiyo = 0;
		qst_evb_mag_write_reg(0x46, reg_46.value);
		qst_delay_ms(20);


		qst_evb_mag_read_reg(0x3a, &reg_3a.value, 1);
		qst_evb_mag_read_reg(0x3b, &reg_3b.value, 1);
		for(mag_kx_ky=CROSS_SVVT_MIN; mag_kx_ky<=CROSS_SVVT_MAX; mag_kx_ky++)
		{
			reg_3b.kx = mag_kx_ky;
			reg_3a.ky = mag_kx_ky;
			qst_evb_mag_write_reg(0x3a, reg_3a.value);
			qst_evb_mag_write_reg(0x3b, reg_3b.value);
			calc_data[2] = t_mag.raw_data[2][2] - (short)(t_mag.raw_data[2][0]*mag_kx_ky/128) - (short)(t_mag.raw_data[2][1]*mag_kx_ky/128);
			qst_logi("cross kxky,%d,z_calc,%d,", mag_kx_ky, calc_data[2]);
			qst_delay_ms(20);
			if(mag_get_drdy())
			{
				mag_read_raw();
			}
		}
		reg_3b.kx = 0;
		reg_3a.ky = 0;
		qst_evb_mag_read_reg(0x3a, &reg_3a.value, 1);
		qst_evb_mag_read_reg(0x3b, &reg_3b.value, 1);
		qst_delay_ms(20);
		
		qst_logi("\r\n\r\n");
	}	
	qst_logi("cross sensitivity test done\r\n");
}

#endif

#if defined(MAG_TEST_I3C_SUPPORT)
void qst_evb_mag_i3c_loop_status(void)
{
	unsigned char status = 0x00;
	int ret = 0;

	ret = qst_evb_mag_read_reg(MAG_STATUS_REG, &status, 1);
	qst_logi("loop status 0x%02x = 0x%02x\r\n", MAG_STATUS_REG, status);
}

void qst_evb_mag_i3c_switch(void)
{
	unsigned char reg_value = 0x00;

	qst_evb_mag_read_reg(0x3c, &reg_value, 1);
	if(reg_value & 0x20)
	{
		qst_logi("mag i3c status[disable], turn on i3c!\r\n");
		reg_value = (reg_value & (~0x20));
		qst_evb_mag_write_reg(0x3c, reg_value);
		t_mag.sel_inf = INTERFACE_I3C_6_25M;
		MX_I3C1_Init(0x13, 0x13);
		qst_delay_ms(300);
		qst_evb_enry_i3c();
	}
	else
	{
		qst_logi("mag i3c status[enable], turn off i3c!\r\n");
		reg_value = (reg_value | 0x20);
		qst_evb_mag_write_reg(0x3c, reg_value);
		t_mag.sel_inf = INTERFACE_I2C_SW;
		i2c_sw_gpio_config(0);
		qst_delay_ms(300);
	}
}

void qst_evb_mag_selftest_enable()
{
	qst_evb_mag_write_reg(MAG_CTL_REG_ONE, 0x00);
	if(t_mag.set.odr == maestro_odr)
	{
		qst_evb_mag_write_reg(MAG_CTL_REG_TWO, 0x01);
	}
	else
	{
		qst_evb_mag_write_reg(MAG_CTL_REG_TWO, 0x00);
	}
	qst_delay_ms(2);
	qst_evb_mag_write_reg(MAG_CTL_REG_ONE, 0x03);
	qst_delay_ms(20);
	qst_evb_mag_write_reg(0x0e, 0x80);
}

static void qst_evb_i3c_ibi_mag_selftest(void)
{
	signed char  st_data[3];
	unsigned char abs_data[3];
	unsigned char rdy = 0x00;
	int ret = 0;

	rdy = 0x00;
	ret = qst_evb_mag_read_reg(MAG_STATUS_REG, &rdy, 1);
	if(rdy & 0x04)
	{
		ret = qst_evb_mag_read_reg(QMC6309_DATA_OUT_ST_X, (unsigned char*)st_data, 3);
	}
	else
	{
		qst_logi("qmc6309 selftest drdy fail! status=0x%02x\r\n", rdy);
	}

	if(ret)
	{
		abs_data[0] = QMC6309_ABS(st_data[0]);
		abs_data[1] = QMC6309_ABS(st_data[1]);
		abs_data[2] = QMC6309_ABS(st_data[2]);
		
		if( ((abs_data[0] < QMC6309_SELFTEST_MAX_X) && (abs_data[0] > QMC6309_SELFTEST_MIN_X))
			&& ((abs_data[1] < QMC6309_SELFTEST_MAX_Y) && (abs_data[1] > QMC6309_SELFTEST_MIN_Y))
			&& ((abs_data[2] < QMC6309_SELFTEST_MAX_Z) && (abs_data[2] > QMC6309_SELFTEST_MIN_Z)) )
		{
			//qst_logi("selftest OK! status[0x%x]data[%d	%d	%d]\r\n",rdy,st_data[0],st_data[1],st_data[2]);
		}
		else
		{
#if defined(FPGA)
			//qst_evb_mag_read_reg(0x0e, abs_data, 1);
			st_data[0] = st_data[1] = st_data[2] = 0;
			//qst_logi("%d selftest OK! status[0x09=0x%02x][0x0e==0x%02x]data[%d	%d	%d]\r\n",t_mag.counter[0],rdy,abs_data[0],st_data[0],st_data[1],st_data[2]);
#else
			t_mag.data_fail++;
#endif
		}
	}
	else
	{
		//qst_logi("selftest read data fail!\r\n");		
		t_mag.data_fail++;
	}
	t_mag.data_i++;
	qst_logi("ibi selftest count[%d]fail[%d] status[0x%x]data[%d	%d	%d]\r\n",t_mag.data_i,t_mag.data_fail,rdy,st_data[0],st_data[1],st_data[2]);

	qst_evb_mag_selftest_enable();
}

static void qst_evb_mag_enable_ibi(unsigned char flag)
{
	qst_evb_mag_write_reg(QMC6309_CTL_IBI, flag);
//	flag = 0x00;
//	qst_evb_mag_read_reg(QMC6309_CTL_IBI, &flag, 1);
//	qst_logi("qst_evb_mag_enable_ibi read 0x21 = 0x%02x\r\n", flag);
}

void qst_evb_i3c_ibi_drdy_test_odr(void)
{
#if defined(MAESTRO1_USER_SET)
	pset.bit.period_set = 0x3f;
	pset.bit.user_set = 1;
	qst_evb_mag_write_reg(MAG_CTL_REG_PSET, pset.value);
#endif
}

void qst_evb_i3c_ibi_drdy(void)
{
	int ret = MAG_OK;
#if defined(QST_MAG_USE_SWJ)
#if defined(MAG_SOFT_COMPENSATE)
	float mag_raw[3];
#endif
#endif

//	ret = qst_evb_mag_read_reg(MAG_STATUS_REG, &t_mag.reg, 1);
//	if(ret == MAG_FAIL)
//	{
//		qst_logi("qst_evb_i3c_ibi_drdy read 0x09 fail! ret=%d", ret);
//	}
	ret = qst_evb_mag_read_reg(MAG_DATA_OUT_X_LSB_REG, g_buf, 6);
	if(ret == MAG_FAIL)
	{
		t_mag.data_fail++;
		qst_logi("qst_evb_i3c_ibi_drdy read 0x01 fail! ret=%d", ret);
	}
	ret = qst_evb_mag_read_reg(MAG_STATUS_REG, &t_mag.reg, 1);
	if(ret == MAG_FAIL)
	{
		t_mag.data_fail++;
		qst_logi("qst_evb_i3c_ibi_drdy read 0x09 fail! ret=%d", ret);
	}
	if(TEST_IBI_TO_DATA_OVL == t_mag.sel_test_i)
	{	
		qst_logi("%s ovl status[0x%02x]\r\n", g_config_info, t_mag.reg);
	}

	qst_evb_mag_enable_single();

	g_m_out.raw[0] = (short)(((g_buf[1]) << 8) | g_buf[0]);
	g_m_out.raw[1] = (short)(((g_buf[3]) << 8) | g_buf[2]);
	g_m_out.raw[2] = (short)(((g_buf[5]) << 8) | g_buf[4]);
#if defined(QST_MAG_USE_SWJ)
	g_m_out.mag[0] = (float)(g_m_out.raw[0]*100.0f/g_m_out.mag_lsb);
	g_m_out.mag[1] = (float)(g_m_out.raw[1]*100.0f/g_m_out.mag_lsb);
	g_m_out.mag[2] = (float)(g_m_out.raw[2]*100.0f/g_m_out.mag_lsb);
#if 0
	if(t_mag.mag_sensor == QST_MAG_MAESTRO_1V)
	{
		g_m_out.mag[2] = g_m_out.mag[2] - (g_m_out.kx*g_m_out.mag[0]) - (g_m_out.ky*g_m_out.mag[1]);
	}
#endif
#if defined(MAG_SOFT_COMPENSATE)
	mag_raw[0] = g_m_out.mag[0];
	mag_raw[1] = g_m_out.mag[1];
	mag_raw[2] = g_m_out.mag[2];
	g_m_out.mag[0] = (float)(mag_raw[0]*evb_softmag[0][0] + mag_raw[1]*evb_softmag[0][1] + mag_raw[2]*evb_softmag[0][2]);
	g_m_out.mag[1] = (float)(mag_raw[0]*evb_softmag[1][0] + mag_raw[1]*evb_softmag[1][1] + mag_raw[2]*evb_softmag[1][2]);
	g_m_out.mag[2] = (float)(mag_raw[0]*evb_softmag[2][0] + mag_raw[1]*evb_softmag[2][1] + mag_raw[2]*evb_softmag[2][2]);
#endif
	#if defined(MAG_TEST_STD_OUT)
	getStandardDeviation3(&mStd3, g_m_out.mag, g_m_out.std);
	qst_logi("%sM-%d,"OUT_F6, g_config_info, t_mag.data_i,g_m_out.mag[0],g_m_out.mag[1],g_m_out.mag[2],g_m_out.std[0],g_m_out.std[1],g_m_out.std[2]);
	#else
	qst_logi("%sM-%d,"OUT_F3, g_config_info,t_mag.data_i,g_m_out.mag[0], g_m_out.mag[1], g_m_out.mag[2]);
	#endif
#else
	if((t_mag.data_i % t_mag.set.odr[t_mag.sel_odr].freq)==0)
	{
		qst_logi("%d,%d,%d, 	M-%d,%s\r\n", g_m_out.raw[0], g_m_out.raw[1], g_m_out.raw[2], t_mag.data_i, g_config_info);
	}
	else
	{
		qst_logi("%d,%d,%d\r\n", g_m_out.raw[0], g_m_out.raw[1], g_m_out.raw[2]);
	}
#endif
	t_mag.data_i++;

	if(t_mag.data_i > t_mag.data_max)
	{
		//qst_evb_mag_get_config_info();
		qst_logi("TEST[%d] %s, RESULT[%s],IBI_COUNT[%d] FAIL_COUNT[%d]", t_mag.config_id, g_config_info, (t_mag.data_fail?"FAIL":"PASS"), t_mag.data_i, t_mag.data_fail);
		t_mag.config_id++;
		t_mag.data_i = 0;
		t_mag.data_fail = 0;

		unsigned char s_osr1, s_osr2;
		if(t_mag.set.odr == maestro_odr)
		{
			s_osr1 = maestro_osr1[3];
			s_osr2 = maestro_osr2[0];
		}
		else if(t_mag.set.odr == qmc6309_odr)
		{
			s_osr1 = qmc6309_osr1[3];
			s_osr2 = qmc6309_osr2[0];
		}
		else if(t_mag.set.odr == qmc6309v_odr)
		{
			s_osr1 = qmc6309v_osr1[3];
			s_osr2 = qmc6309v_osr2[0];
		}
		if(qst_evb_mag_select_config_lite(t_mag.config_id, s_osr1, s_osr2))
		{
			//qst_evb_i3c_enable_ibi(1);
			if((t_mag.reg_a & 0x03) == 2 /*MAESTRO_MODE_SINGLE*/)
			{
				qst_evb_mag_write_reg(MAG_CTL_REG_ONE, t_mag.reg_a);
			}
		}
		else
		{
			qst_logi("TEST IBI DRDY DONE!\r\n");
		}
	}

	if(t_mag.exit_flag)
	{
		t_mag.exit_flag = 0;
		qst_evb_reg_i3c_ibi_hdlr(NULL);
		qst_evb_mag_soft_reset();
		qst_logi("%s ibi drdy fail[%d] total[%d]\r\n", g_config_info, t_mag.data_fail, t_mag.data_i);
	}
}

void qst_evb_i3c_ibi_fifo(void)
{
	unsigned char level = 0;
	int ret = 1;
#if defined(QST_MAG_USE_SWJ)
#if defined(MAG_SOFT_COMPENSATE)
	float mag_raw[3];
#endif
#endif

	ret = qst_evb_mag_read_reg(MAG_FIFO_REG_STATUS, &t_mag.status2, 1);
	if(!ret)
	{
		t_mag.data_fail++;
		level = 0;
	}
	else
	{
		level = (t_mag.status2 >> t_mag.fifo_lv_shift);
	}
	if(level && ret)
	{
		ret = qst_evb_mag_read_reg(MAG_FIFO_REG_DATA, (unsigned char*)g_buf, 6*level);
		if(!ret)
		{
			t_mag.data_fail++;
			qst_logi("ERROR READ FIFO DATA!\r\n");
		}
		ret = qst_evb_mag_write_reg(MAG_FIFO_REG_CTRL, t_mag.fifo_ctrl);
		//qst_logi("L=%d\r\n", level);

		int config_flag = (((t_mag.data_i/level) % ((t_mag.set.odr[t_mag.sel_odr].freq/level)+1))==0);
		for(int i=0; i<level; i++)
		{
			g_m_out.raw[0] = (short)(((g_buf[1+i*6]) << 8) | g_buf[0+i*6]);
			g_m_out.raw[1] = (short)(((g_buf[3+i*6]) << 8) | g_buf[2+i*6]);
			g_m_out.raw[2] = (short)(((g_buf[5+i*6]) << 8) | g_buf[4+i*6]);
#if defined(QST_MAG_USE_SWJ)
			g_m_out.mag[0] = (float)(g_m_out.raw[0]*100.0f/g_m_out.mag_lsb);
			g_m_out.mag[1] = (float)(g_m_out.raw[1]*100.0f/g_m_out.mag_lsb);
			g_m_out.mag[2] = (float)(g_m_out.raw[2]*100.0f/g_m_out.mag_lsb);

			//if(t_mag.mag_sensor == QST_MAG_MAESTRO_1V)
			//{
			//	g_m_out.mag[2] = g_m_out.mag[2] - (g_m_out.kx*g_m_out.mag[0]) - (g_m_out.ky*g_m_out.mag[1]);
			//}

#if defined(MAG_SOFT_COMPENSATE)
			mag_raw[0] = g_m_out.mag[0];
			mag_raw[1] = g_m_out.mag[1];
			mag_raw[2] = g_m_out.mag[2];
			g_m_out.mag[0] = (float)(mag_raw[0]*evb_softmag[0][0] + mag_raw[1]*evb_softmag[0][1] + mag_raw[2]*evb_softmag[0][2]);
			g_m_out.mag[1] = (float)(mag_raw[0]*evb_softmag[1][0] + mag_raw[1]*evb_softmag[1][1] + mag_raw[2]*evb_softmag[1][2]);
			g_m_out.mag[2] = (float)(mag_raw[0]*evb_softmag[2][0] + mag_raw[1]*evb_softmag[2][1] + mag_raw[2]*evb_softmag[2][2]);
#endif
		#if defined(MAG_TEST_STD_OUT)
			getStandardDeviation3(&mStd3, g_m_out.mag, g_m_out.std);
			qst_logi(OUT_F6",M-%d,%s\r\n", g_m_out.mag[0],g_m_out.mag[1],g_m_out.mag[2],g_m_out.std[0],g_m_out.std[1],g_m_out.std[2],t_mag.data_i+i,g_config_info);
		#else
			qst_logi(OUT_F3",M-%d,%s\r\n", g_m_out.mag[0], g_m_out.mag[1], g_m_out.mag[2], t_mag.data_i+i, g_config_info);
		#endif
#else
			if(config_flag)
			{
				qst_logi("%d,%d,%d,		M-%d,ST[0x%02x],%s\r\n", g_m_out.raw[0], g_m_out.raw[1], g_m_out.raw[2], t_mag.data_i+i, t_mag.status2, g_config_info);
				config_flag = 0;
			}
			else
			{
				qst_logi("%d,%d,%d\r\n", g_m_out.raw[0], g_m_out.raw[1], g_m_out.raw[2]);
			}
#endif
		}
		t_mag.data_i += level;
	}
	else
	{
		t_mag.data_fail++;
		qst_logi("ERROR READ FIFO STATUS 0x%02x RET=%d\r\n", t_mag.status2, ret);
		ret = qst_evb_mag_write_reg(MAG_FIFO_REG_CTRL, t_mag.fifo_ctrl);
	}

	if(t_mag.sel_test_i == TEST_IBI_TO_FIFO_FULL)
	{
		if(level != t_mag.fifo_size)
		{
			qst_logi("ERROR FIFO-FULL %s LEVEL(%d) != %d \r\n", g_config_info, level, t_mag.fifo_size);
			t_mag.data_fail++;
		}
		if((t_mag.status2 & MAG_STATUS_FIFO_FULL_INT) == 0)
		{
			qst_logi("ERROR FIFO-FULL %s STATUS[0x%02x]\r\n", g_config_info, t_mag.status2);
			t_mag.data_fail++;
		}		
		level = t_mag.fifo_size;
	}
	else if(t_mag.sel_test_i == TEST_IBI_TO_FIFO_WMK)
	{
		if(level != t_mag.fifo_wmk)
		{
			qst_logi("ERROR FIFO-WMK %s LEVEL(%d) != %d \r\n", g_config_info, level, t_mag.fifo_wmk);
			t_mag.data_fail++;
		}
		if((t_mag.status2 & MAG_STATUS_FIFO_WMK_INT) == 0)
		{
			qst_logi("ERROR FIFO-WMK %s STATUS[0x%02x]\r\n", g_config_info, t_mag.status2);
			t_mag.data_fail++;
		}
		level = t_mag.fifo_wmk;
	}

	if(t_mag.data_i > t_mag.data_max)
	{
		unsigned char s_osr1, s_osr2;
		//qst_evb_mag_get_config_info();
		if(level <= 0)
		{
			level = t_mag.fifo_size;
		}
		qst_logi("TEST[%d] %s, RESULT[%s],IBI_COUNT[%d] FAIL_COUNT[%d]", t_mag.config_id, g_config_info, (t_mag.data_fail?"FAIL":"PASS"), t_mag.data_i/level, t_mag.data_fail);
		t_mag.config_id++;
		t_mag.data_i = t_mag.data_fail = 0;

		if(t_mag.set.odr == maestro_odr)
		{
			s_osr1 = maestro_osr1[3];
			s_osr2 = maestro_osr2[0];
		}
		else if(t_mag.set.odr == qmc6309_odr)
		{
			s_osr1 = qmc6309_osr1[3];
			s_osr2 = qmc6309_osr2[0];
		}
		else if(t_mag.set.odr == qmc6309v_odr)
		{
			s_osr1 = qmc6309v_osr1[3];
			s_osr2 = qmc6309v_osr2[0];
		}
		if(qst_evb_mag_select_config_lite(t_mag.config_id, s_osr1, s_osr2))
		{
			//qst_evb_i3c_enable_ibi(1);
			//qst_evb_reg_i3c_ibi_hdlr(qst_evb_i3c_ibi_fifo);
		}
		else
		{
			if(t_mag.sel_test_i == TEST_IBI_TO_FIFO_FULL)
			{				
				qst_logi("TEST IBI FIFO-FULL DONE!\r\n");
			}
			else if(t_mag.sel_test_i == TEST_IBI_TO_FIFO_WMK)
			{
				qst_logi("TEST IBI FIFO-WMK DONE!\r\n");
			}
		}
	}

	if(t_mag.exit_flag)
	{
		t_mag.exit_flag = 0;
		qst_evb_reg_i3c_ibi_hdlr(NULL);
		qst_evb_mag_soft_reset();
		if(level <= 0)
		{
			level = t_mag.fifo_size;
		}
		qst_logi("TEST[%d] %s, IBI_COUNT[%d] FAIL_COUNT[%d]", t_mag.config_id, g_config_info, t_mag.data_i/level, t_mag.data_fail);
	}
}


void qst_evb_i3c_ibi_drdy_test(void)
{
	int ret = MAG_OK;

	ret = qst_evb_mag_read_reg(MAG_DATA_OUT_X_LSB_REG, g_buf, 6);
	if(ret == MAG_FAIL)
	{
		t_mag.data_fail++;
		qst_logi("qst_evb_i3c_ibi_drdy read 0x01 fail! ret=%d", ret);
	}

	ret = qst_evb_mag_read_reg(MAG_STATUS_REG, &t_mag.reg, 1);
	if(ret == MAG_FAIL)
	{
		qst_logi("qst_evb_i3c_ibi_drdy read 0x09 fail! ret=%d", ret);
	}

	if(TEST_IBI_TO_DATA_OVL == t_mag.sel_test_i)
	{	
		qst_logi("%s ovl status[0x%02x]\r\n", g_config_info, t_mag.reg);
	}

	qst_evb_mag_enable_single();

	g_m_out.raw[0] = (short)(((g_buf[1]) << 8) | g_buf[0]);
	g_m_out.raw[1] = (short)(((g_buf[3]) << 8) | g_buf[2]);
	g_m_out.raw[2] = (short)(((g_buf[5]) << 8) | g_buf[4]);

	qst_logi("%sM-%d,%d,%d,%d\r\n", "IBI-DRDY",t_mag.data_i,g_m_out.raw[0],g_m_out.raw[1],g_m_out.raw[2]);
	if(t_mag.data_i++ > t_mag.data_max)
	{		
		//qst_evb_reg_i3c_ibi_hdlr(NULL);
		t_mag.config_id++;
		t_mag.data_i = 0;
		if(qst_evb_mag_select_config(t_mag.config_id))
		{
			//qst_evb_reg_i3c_ibi_hdlr(qst_evb_i3c_ibi_drdy);
			qst_evb_i3c_enable_ibi(1);
			//if(ctrl1.bit.mode == QMC6309_MODE_SINGLE)
			//{
			//	qst_evb_mag_write_reg(MAG_CTL_REG_ONE, ctrl1.value);
			//}
		}
		else
		{
			qst_logi("ibi drdy fail[%d] total[%d]\r\n", t_mag.data_fail, t_mag.data_i);
		}
	}

	if((t_mag.data_i % 8) == 0)
	{
		if(t_mag.fifo_en)
		{
			qst_evb_mag_read_reg(MAG_FIFO_REG_STATUS, &t_mag.status2, 1);
			if((t_mag.status2 & QMC6309_FIFO_STATUS_FULL))
			{
				int fifo_level = (t_mag.status2 >> t_mag.fifo_lv_shift);
				qst_evb_mag_read_reg(MAG_FIFO_REG_DATA, g_buf, 6*fifo_level);
				qst_evb_mag_write_reg(MAG_FIFO_REG_CTRL, t_mag.fifo_ctrl);
				//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);	// qst0103 20250815
				for(int i=0; i<fifo_level; i++)
				{
					g_m_out.raw[0] = (short)(((g_buf[1+i*6]) << 8) | g_buf[0+i*6]);
					g_m_out.raw[1] = (short)(((g_buf[3+i*6]) << 8) | g_buf[2+i*6]);
					g_m_out.raw[2] = (short)(((g_buf[5+i*6]) << 8) | g_buf[4+i*6]);
					qst_logi("M-%d,%d,%d,%d\r\n", i,g_m_out.raw[0], g_m_out.raw[1], g_m_out.raw[2]);
				}
			}
		}
	}
}

#endif

#if defined(FPGA)
static void qst_evb_mag_apply_t0(signed char t0)
{
#if defined(MAESTRO1)
	unsigned char reg_value = 0;
	unsigned char write_value = 0;

	qst_evb_mag_read_reg(0x3c, &reg_value, 1);
	reg_value &= 0xc0;

	if(t0 & 0x80)
	{
		write_value |= 0x20;
	}	
	write_value |= t0&0x1f;

	if(reg_value & 0x80)
		write_value |= 0x80;
	else
		write_value &= 0x7f;

	if(reg_value & 0x40)
		write_value |= 0x40;
	else
		write_value &= 0xbf;

//	qst_logi("t0=%d 0x3c=0x%02x\r\n", t0, write_value);
	qst_evb_mag_write_reg(0x3c, write_value);
#elif defined(QMC6309V_X7)
	unsigned char reg_value = 0;

	reg_value = t0 & 0x9f;	
	qst_evb_mag_write_reg(0x3c, reg_value);
#else
	qst_t0_t t0_in;
	qst_evb_mag_read_reg(0x3c, &t0_in.value, 1);
	t0_in.t0 = t0;
	t0_in.rev = 0;
	qst_evb_mag_write_reg(0x3c, t0_in.value);
	//qst_logi("apply t0 = %d \r\n", t0);
#endif
}

static void qst_evb_mag_apply_tco_tcs(signed char tco[3], signed char tcs[3])
{
#if defined(MAESTRO1)
	qst_evb_mag_write_reg(MAG_REG_TCS_X, (unsigned char)(tcs[0]));
	qst_evb_mag_write_reg(MAG_REG_TCS_Y, (unsigned char)(tcs[1]));
	qst_evb_mag_write_reg(MAG_REG_TCS_Z, (unsigned char)(tcs[2]));
	
	qst_evb_mag_write_reg(MAG_REG_TCO_X, (unsigned char)(tco[0]));
	qst_evb_mag_write_reg(MAG_REG_TCO_Y, (unsigned char)(tco[1]));
	qst_evb_mag_write_reg(MAG_REG_TCO_Z, (unsigned char)(tco[2]));
#else
	qst_evb_mag_write_reg(MAG_REG_TCS_X, (unsigned char)(tcs[0]));
	qst_evb_mag_write_reg(MAG_REG_TCS_Y, (unsigned char)(tcs[1]));
	qst_evb_mag_write_reg(MAG_REG_TCS_Z, (unsigned char)(tcs[2]));
#endif
}

#if defined(MAESTRO1)
static void qst_evb_mag_apply_offset_gain(unsigned short	gain[3], short offset[3])
#else
static void qst_evb_mag_apply_offset_gain(signed char gain[3], short offset[3])
#endif
{
#if defined(MAESTRO1)
	unsigned char	reg[4];

	qst_evb_mag_read_reg(0x3D, reg, 2);
	reg[0] &= 0xf0;
	reg[0] |= (unsigned char)((gain[0]>>8)&0x03);
	reg[0] |= (unsigned char)(((gain[1]>>8)&0x03)<<2);

//	reg[0] |= 0x40;		// kmtx bypass
	qst_evb_mag_write_reg(0x3D, reg[0]);

	reg[1] &= 0xfc;
	reg[1] |= (unsigned char)((gain[2]>>8)&0x03);
	qst_evb_mag_write_reg(0x3E, reg[1]);
	
	qst_evb_mag_write_reg(MAG_REG_GAIN_X, (unsigned char)(gain[0] & 0xff));
	qst_evb_mag_write_reg(MAG_REG_GAIN_Y, (unsigned char)(gain[1] & 0xff));
	qst_evb_mag_write_reg(MAG_REG_GAIN_Z, (unsigned char)(gain[2] & 0xff));

	qst_evb_mag_write_reg(MAG_REG_OFFSET_X_LSB, (unsigned char)(offset[0]&0xff));
	qst_evb_mag_write_reg(MAG_REG_OFFSET_X_MSB, (unsigned char)((offset[0]>>8)&0xff));
	qst_evb_mag_write_reg(MAG_REG_OFFSET_Y_LSB, (unsigned char)(offset[1]&0xff));
	qst_evb_mag_write_reg(MAG_REG_OFFSET_Y_MSB, (unsigned char)((offset[1]>>8)&0xff));
	qst_evb_mag_write_reg(MAG_REG_OFFSET_Z_LSB, (unsigned char)(offset[2]&0xff));
	qst_evb_mag_write_reg(MAG_REG_OFFSET_Z_MSB, (unsigned char)((offset[2]>>8)&0xff));
#elif defined(QMC6309V_X7)
	qst_evb_mag_write_reg(MAG_REG_GAIN_X, (unsigned char)(gain[0] & 0xff));
	qst_evb_mag_write_reg(MAG_REG_GAIN_Y, (unsigned char)(gain[1] & 0xff));
	qst_evb_mag_write_reg(MAG_REG_GAIN_Z, (unsigned char)(gain[2] & 0xff));

	qst_evb_mag_write_reg(MAG_REG_OFFSET_X_LSB, (unsigned char)(offset[0]&0xff));
	qst_evb_mag_write_reg(MAG_REG_OFFSET_X_MSB, (unsigned char)((offset[0]>>8)&0xff));
	qst_evb_mag_write_reg(MAG_REG_OFFSET_Y_LSB, (unsigned char)(offset[1]&0xff));
	qst_evb_mag_write_reg(MAG_REG_OFFSET_Y_MSB, (unsigned char)((offset[1]>>8)&0xff));
	qst_evb_mag_write_reg(MAG_REG_OFFSET_Z_LSB, (unsigned char)(offset[2]&0xff));
	qst_evb_mag_write_reg(MAG_REG_OFFSET_Z_MSB, (unsigned char)((offset[2]>>8)&0xff));
#endif
}

static void qst_evb_mag_apply_kmtx(bool enable, signed char xy[4], signed char xy_z[2])
{
#if defined(MAESTRO1)
	static unsigned char val_3d = 0;
	qst_evb_mag_read_reg(0x3D, &val_3d, 1);

	if(enable)
	{
		val_3d &= (~0x40); 	// kmtx enable
		qst_evb_mag_write_reg(0x3D, val_3d);
	}
	else
	{
		val_3d |= 0x40;		// kmtx bypass
		qst_evb_mag_write_reg(0x3D, val_3d);
	}
	
#if defined(MAESTRO2H)
	qst_evb_mag_write_reg(MAG_CROSS_KXX, xy[0]);
	qst_evb_mag_write_reg(MAG_CROSS_KYX, xy[1]);

	qst_evb_mag_write_reg(MAG_CROSS_KXY, xy[2]);
	qst_evb_mag_write_reg(MAG_CROSS_KYY, xy[3]);

	qst_evb_mag_write_reg(MAG_CROSS_KXZ, xy_z[0]);
	qst_evb_mag_write_reg(MAG_CROSS_KYZ, xy_z[1]);
#else
	qst_evb_mag_write_reg(MAG_CROSS_KXX, xy[0]<<2);
	qst_evb_mag_write_reg(MAG_CROSS_KYX, xy[1]<<2);

	qst_evb_mag_write_reg(MAG_CROSS_KXY, xy[2]<<2);
	qst_evb_mag_write_reg(MAG_CROSS_KYY, xy[3]<<2);

	qst_evb_mag_write_reg(MAG_CROSS_KXZ, xy_z[0]<<2);
	qst_evb_mag_write_reg(MAG_CROSS_KYZ, xy_z[1]<<2);
#endif

#endif
}
#endif

#if defined(MAG_TEST_I3C_SUPPORT)
void qst_mag_entry_sel_ccc(void)
{
	int items = 0;
	int					sel_ccc;

	while(1)
	{
		items = 0;
		qst_logi("Select I3C CCC COMMAND :\r\n");
		qst_logi("[%d]: GET PID \r\n", items++);
		qst_logi("[%d]: GET BCR \r\n", items++);
		qst_logi("[%d]: GET DCR \r\n", items++);
		qst_logi("[%d]: GET STATUS \r\n", items++);
		qst_logi("[%d]: SET MWL \r\n", items++);
		qst_logi("[%d]: SET MRL \r\n", items++);
		qst_logi("[%d]: GET MWL \r\n", items++);
		qst_logi("[%d]: GET MRL \r\n", items++);
		qst_logi("[%d]: SET DISEC \r\n", items++);
		qst_logi("[%d]: SET ENEC \r\n", items++);

		scanf("%d", &sel_ccc);
		if((sel_ccc >= 0)&&(sel_ccc < items))
		{
			qst_logi("User select ccc command : %d\r\n\r\n", sel_ccc);
			break;
		}
		else
		{
			qst_logi("User select ccc command : %d error!!!\r\n", sel_ccc);	
		}
	}

	switch(sel_ccc)
	{
		case 0:
			t_mag.reg = Direct_GETPID;
			break;
		case 1:
			t_mag.reg = Direct_GETBCR;
			break;
		case 2:
			t_mag.reg = Direct_GETDCR;
			break;
		case 3:
			t_mag.reg = Direct_GETSTATUS;
			break;
		case 4:
			t_mag.reg = Direct_SETMWL;
			break;
		case 5:
			t_mag.reg = Direct_SETMRL;
			break;
		case 6:
			t_mag.reg = Direct_GETMWL;
			break;
		case 7:
			t_mag.reg = Direct_GETMRL;
			break;
		case 8:
			t_mag.reg = Direct_DISEC;
			break;
		case 9:
			t_mag.reg = Direct_ENEC;
			break;
		default:
			break;
	}
}
#endif

void qst_mag_entry_sel_interface(void)
{
	if(t_mag.sel_inf < 0)
	{
		while(1)
		{
			int items = 0;
			qst_logi("Select communication procotol:\r\n");
			
			for(items=0; items<(sizeof(interface_array)/sizeof(interface_array[0])); items++)
			{
				if(interface_array[items].support)
				{
					qst_logi("[%d]: %s\r\n", interface_array[items].index, interface_array[items].info);
				}
			}
		
			scanf("%d", &t_mag.sel_inf);
			if((t_mag.sel_inf>=INTERFACE_I2C_SW)&&(t_mag.sel_inf<INTERFACE_TOTAL))
			{
				break;
			}
			else
			{
				qst_logi("Select communication procotol:%d error!!!\r\n", t_mag.sel_inf);
			}
		}

		qst_evb_init_port(t_mag.sel_inf);
	}
}

static void qst_mag_auto_sel_range_osr(int sel_odr)
{	
	t_mag.sel_range = 0;
	if(t_mag.set.odr == maestro_odr)
	{	
		t_mag.sel_osr1 = 3;
		t_mag.sel_osr2 = 0;
	}
	else if((t_mag.set.odr == qmc6309_odr)||(t_mag.set.odr == qmc6309v_odr)
#if defined(QMC6308)
			||(t_mag.set.odr == qmc6308_odr)
#endif
		)
	{
		if(sel_odr == 0)			// HPF
		{				
			t_mag.sel_osr1 = 3;
			t_mag.sel_osr2 = 2;
		}
		else if(sel_odr == 1)		// 200Hz
		{
			t_mag.sel_osr1 = 3;
			t_mag.sel_osr2 = 1;
		}
		else if(sel_odr == 2)		// 100Hz
		{
			t_mag.sel_osr1 = 3;
			t_mag.sel_osr2 = 1;
		}
		else if(sel_odr == 6)						// single mode
		{
			t_mag.sel_osr1 = 2;
			t_mag.sel_osr2 = 1;
		}
		else
		{
			t_mag.sel_osr1 = 3;
			t_mag.sel_osr2 = 0;
		}
	}
}

static void qst_mag_entry_sel_range_osr(void)
{
	int items = 0;
	// select range 	
	if(t_mag.sel_range < 0)
	{
		while(1)
		{
			items = 0;
			qst_logi("Select mag rang:\r\n");
			qst_logi("[%d]: 32 Gauss \r\n", items++);
			qst_logi("[%d]: 16 Gauss \r\n", items++);
			qst_logi("[%d]: 8  Gauss \r\n", items++);
	
			scanf("%d", &t_mag.sel_range);
			if((t_mag.sel_range >= 0)&&(t_mag.sel_range < items))
			{
				qst_logi("User select mag range: %d\r\n\r\n", t_mag.sel_range);
				break;
			}
			else
			{
				qst_logi("User select mag range: %d error!!!\r\n", t_mag.sel_range);	
			}
		}
	}
	// select range
	// select osr
	if(t_mag.sel_osr1 < 0)
	{
		while(1)
		{
			items = 0;
			int auto_i = 0;

			qst_logi("Select mag osr1 :\r\n");
			for(items=0; items<t_mag.set.osr1_num; items++)
			{
				qst_logi("[%d]: osr1 = %d \r\n", items, 1<<items);
			}

			auto_i = items++;
			qst_logi("[%d]: auto osr \r\n", auto_i);
			scanf("%d", &t_mag.sel_osr1);
			if((t_mag.sel_osr1 >= 0)&&(t_mag.sel_osr1 < items))
			{
				qst_logi("User select mag osr1 : %d\r\n\r\n", t_mag.sel_osr1);
				if(t_mag.sel_osr1 == auto_i)
				{
					qst_mag_auto_sel_range_osr(t_mag.sel_odr);
				}
				break;
			}
			else
			{
				qst_logi("User select mag osr1 : %d error!!!\r\n", t_mag.sel_osr1); 
			}
		}
	}
	if(t_mag.sel_osr2 < 0)
	{
		while(1)
		{
			items = 0;
			qst_logi("Select mag osr2 :\r\n");
			for(items=0; items<t_mag.set.osr2_num; items++)
			{
				qst_logi("[%d]: osr2 = %d \r\n", items, 1<<items);
			}
			scanf("%d", &t_mag.sel_osr2);
			if((t_mag.sel_osr2 >= 0)&&(t_mag.sel_osr2 < items))
			{
				qst_logi("User select mag osr2 : %d\r\n\r\n", t_mag.sel_osr2);
				break;
			}
			else
			{
				qst_logi("User select mag osr2 : %d error!!!\r\n", t_mag.sel_osr2); 
			}
		}
	}
	if(t_mag.sel_zdbl < 0)
	{
		while(1)
		{
			items = 0;
			qst_logi("Select mag zdbl :\r\n");
			qst_logi("[%d]: zdbl = 0 \r\n", items++);
			qst_logi("[%d]: zdbl = 1 \r\n", items++);

			scanf("%d", &t_mag.sel_zdbl);
			if((t_mag.sel_zdbl >= 0)&&(t_mag.sel_zdbl < items))
			{
				qst_logi("User select zdbl : %d\r\n\r\n", t_mag.sel_zdbl);
				break;
			}
			else
			{
				qst_logi("User select mag zdbl : %d error!!!\r\n", t_mag.sel_zdbl); 
			}
		}		
	}
}


void qst_mag_entry_sel_odr(void)
{
	if(t_mag.sel_odr < 0)
	{
		int items = 0;
		while(1)
		{
			items = 0;
			qst_logi("Select mag odr :\r\n");
			for(items=0; items<t_mag.set.odr_num; items++)
			{
				if(items == 0)					//(t_mag.set.odr[items].freq > NORMAL_MODE_MAX_ODR)
					qst_logi("[%d]: hpf mode! \r\n", items);
				else if(t_mag.set.odr[items].freq > 0)
					qst_logi("[%d]: normal mode %dHz\r\n", items, t_mag.set.odr[items].freq);
				else if(t_mag.set.odr[items].mode == 2)
					qst_logi("[%d]: single mode\r\n", items);				
				else if(t_mag.set.odr[items].mode == 0)
					qst_logi("[%d]: suspend mode\r\n", items);
			}

			scanf("%d", &t_mag.sel_odr);
			if((t_mag.sel_odr >= 0)&&(t_mag.sel_odr < items))
			{
				qst_logi("User select mag odr : %d\r\n\r\n", t_mag.sel_odr);
				break;
			}
			else
			{
				qst_logi("User select mag odr : %d error!!!\r\n", t_mag.sel_odr);	
			}
		}
	}
}

void qst_mag_entry_sel_set_reset(void)
{
	if(t_mag.sel_sr < 0)
	{
		int items = 0;
		while(1)
		{
			items = 0;
			qst_logi("Select mag set :\r\n");
			items = t_mag.set.sr_num;
			if(items == 1)
			{
				qst_logi("[%d]: set off\r\n",		0);
			}
			else if(items == 2)
			{
				qst_logi("[%d]: set off\r\n",		0);
				qst_logi("[%d]: set on\r\n",		1);
			}
			else if(items == 3)
			{
				qst_logi("[%d]: set/reset off\r\n",		0);
				qst_logi("[%d]: set on\r\n",			1);
				qst_logi("[%d]: reset on\r\n",			2);
			}
			else
			{
				qst_logi("[%d]: set/reset on\r\n",		0);
				qst_logi("[%d]: set on only\r\n",		1);
				qst_logi("[%d]: reset on only\r\n",		2);
				qst_logi("[%d]: set/reset off\r\n",		3);
			}
			scanf("%d", &t_mag.sel_sr);
			if((t_mag.sel_sr >= 0)&&(t_mag.sel_sr < items))
			{
				qst_logi("User select mag set/reset : %d\r\n\r\n", t_mag.sel_sr);
				break;
			}
			else
			{
				qst_logi("User select mag odr : %d error!!!\r\n", t_mag.sel_sr);	
			}
		}
	}
}

void qst_mag_entry_sel_fifo_mode_wmk(int mode, int wmk)
{
	int sel_item = 0;

	t_mag.fifo_en = 1;
	if((t_mag.fifo_wmk > 0)&&(t_mag.fifo_mode > MAG_FIFO_MODE_BYPASS))
	{
		return;
	}
	while(1 && mode)
	{
		qst_logi("Select fifo work mode:\r\n");
		qst_logi("[%d]: bypass mode\r\n", 0);
		qst_logi("[%d]: fifo mode\r\n", 1);
		qst_logi("[%d]: stream mode\r\n", 2);
		scanf("%d", &sel_item);
		if(sel_item == 0)
		{
			t_mag.fifo_mode = MAG_FIFO_MODE_BYPASS;
			qst_logi("user select fifo mode: bypass\r\n\r\n");
			break;
		}
		if(sel_item == 1)
		{
			t_mag.fifo_mode = MAG_FIFO_MODE_FIFO;
			qst_logi("user select fifo mode: fifo\r\n\r\n");
			break;
		}
		if(sel_item == 2)
		{
			t_mag.fifo_mode = MAG_FIFO_MODE_STREAM;
			qst_logi("user select fifo mode: stream\r\n\r\n");
			break;
		}
		else
		{
			qst_logi("user select fifo mode: error!!!\r\n\r\n");
		}
	}

	if(t_mag.fifo_wmk <= 0)
	{
		while(1 && wmk)
		{
			qst_logi("Please input fifo watermark(1-%d)/fifo full(%d):", t_mag.fifo_size-1, t_mag.fifo_size);
			scanf("%d", &sel_item);
			if((sel_item>0) && (sel_item<t_mag.fifo_size))
			{
				t_mag.fifo_wmk = (unsigned char)sel_item;
				break;
			}
			else if(sel_item == t_mag.fifo_size)
			{
				t_mag.fifo_wmk = 7;
				t_mag.fifo_full = 1;
				break;
			}
			else
			{
				qst_logi("user input fifo watermark(%d): error!!!\r\n\r\n", sel_item);
			}
		}
	}
}

void qst_mag_entry_sel_manual_auto(void)
{
	if(t_mag.sel_auto_test < 0)
	{
		while(1)
		{
			qst_logi("Select manual or auto test:\r\n");
			qst_logi("[%d]: manual test\r\n", 0);
			qst_logi("[%d]: auto test\r\n", 1);
			scanf("%d", &t_mag.sel_auto_test);
			if(t_mag.sel_auto_test == 0)
			{
				qst_logi("user select manual test\r\n");
				break;
			}
			else if(t_mag.sel_auto_test == 1)
			{
				qst_logi("user select auto test\r\n");
				break;
			}
			else
			{
				qst_logi("\r\n\r\n");
			}
		}
	}
}

void qst_mag_entry_sel_test_item(void)
{
	if(t_mag.wait_cfg)
	{
		qst_logi("wait config from UART or press BLUE KEY to start manual\r\n");
		evb_setup_uart_rx(DEBUG_USART, qst_evb_mag_uart_rx_hdlr);
		evb_setup_user_key(1, qst_evb_mag_key1_hdlr);
		while(t_mag.wait_cfg)
		{
			evb_key_handle();
			evb_usart_rx_handle();
		}
		evb_setup_uart_rx(DEBUG_USART, NULL);
	}
	t_mag.exit_flag = t_mag.wait_cfg = 0;

	if(t_mag.sel_test_i < 0)
	{
		while(1)
		{

			int i = 0;
			
			qst_logi("Select function:\r\n");
			if(t_mag.item)
			{
				while(t_mag.item[i].index != TEST_MAX)
				{
					if(t_mag.item[i].support)
					{
						qst_logi("[%d]: %s\r\n", t_mag.item[i].index, t_mag.item[i].info);
					}
					i++;
				}
			}
			else
			{
				qst_logi("no test item!\r\n");
			}
			scanf("%d", &t_mag.sel_test_i);
			if(t_mag.sel_test_i < 0)
			{
				NVIC_SystemReset();		// reset
			}
			else if((t_mag.sel_test_i >=0)&&(t_mag.sel_test_i <TEST_MAX))
			{
				qst_logi("User select function: %d\r\n\r\n", t_mag.sel_test_i);
				break;
			}
			else
			{
				qst_logi("User select function: %d error!!!\r\n\r\n", t_mag.sel_test_i);
			}
		}
	}

	if((t_mag.sel_test_i == TEST_POLL_DATA_AUTO) || (t_mag.sel_test_i == TEST_POLL_FIFO_AUTO))
	{
		t_mag.sel_auto_test = 1;
	}
	else
	{
		t_mag.sel_auto_test = 0;
	}

	if(t_mag.sel_test_i == TEST_POLL_FIFO_MANUAL)
	{
		qst_mag_entry_sel_fifo_mode_wmk(1, 1);
	}
	if(t_mag.sel_test_i == TEST_POLL_FIFO_AUTO)
	{
		qst_mag_entry_sel_fifo_mode_wmk(1, 1);
	}
#if defined(MAG_TEST_I3C_SUPPORT)
	if(t_mag.sel_test_i == TEST_IBI_TO_FIFO_WMK)
	{
		qst_mag_entry_sel_fifo_mode_wmk(1, 1);
	}	
	if(t_mag.sel_test_i == TEST_IBI_TO_FIFO_FULL)
	{
		t_mag.fifo_wmk = t_mag.fifo_size-1;
		qst_mag_entry_sel_fifo_mode_wmk(1, 0);
	}
	if((t_mag.sel_test_i >= TEST_IBI_TO_DRDY) && (t_mag.sel_test_i <= TEST_IBI_TO_FIFO_WMK))
	{
		qst_mag_entry_sel_manual_auto();
	}
	if((t_mag.sel_test_i >= TEST_IBI_TO_DRDY) && (t_mag.sel_test_i <= TEST_IBI_TO_SELFTEST))
	{
		t_mag.ibi_en = 1;
	}
#endif
}

#if defined(CUSTOMER_CONFIG_REG)
void qst_mag_entry_customer_config_reg(void)
{
	int input_data = 0;
	//unsigned char customer_reg[] = {0x0e, 0x40, 0x45};
	//unsigned char customer_reg[] = {0x11, 0x40};
	unsigned char customer_reg[] = {0x40};

	for(int i=0; i<(sizeof(customer_reg)/sizeof(customer_reg[0])); i++)
	{
		t_mag.reg = 0x00;
		input_data = 0x00;
		if(0xff != customer_reg[i])
		{
			qst_evb_mag_read_reg(customer_reg[i], &t_mag.reg, 1);
			while(1)
			{
				qst_logi("\r\nSet value of 0x%02x(current is: 0x%02x):\r\n", customer_reg[i], t_mag.reg);
				input_data = -1;
				scanf("0x%x", &input_data);
				if(input_data >= 0 && input_data < 256)
				{
					t_mag.reg = (unsigned char)input_data;
					qst_logi("\r\nuser set 0x%02x = 0x%02x\r\n", customer_reg[i], t_mag.reg);
					break;
				}
			}
			qst_evb_mag_write_reg(customer_reg[i], t_mag.reg);
			// store reg value
			t_mag.user_set_reg = 1;
			if(MAG_REG_SR_CTRL == customer_reg[i])
			{
				t_mag.val_40.value = t_mag.reg;
			}
			// store reg value
		}
	}
	qst_logi("\r\n");
	t_mag.reg = 0x00;
}
#endif

void qst_mag_entry_set_counter(void)
{
	if(t_mag.data_max <= 0)
	{
		qst_logi("Set Max test count:");
		while(1)
		{
			scanf("%d", &t_mag.data_max);
			if(t_mag.data_max > 0)
			{
				qst_logi("\r\nUser set counter %d\r\n\r\n", t_mag.data_max);
				break;
			}
			else
			{
				qst_logi("\r\nUser set counter %d error!\r\n\r\n", t_mag.data_max);
			}
		}
	}
}

void qst_mag_entry_set_id(void)
{
	if(t_mag.sel_mag_i < 0)
	{
		if(t_mag.sel_inf == INTERFACE_I2C_SW)
		{
			unsigned char chipid = 0x00;
			for(int i=0; i<t_mag.mag_num; i++)
			{
				i2c_sw_gpio_config(i);
				for(int j=0; (j<sizeof(mag_slave_array)/sizeof(mag_slave_array[0])); j++)
				{
					mag_slave = mag_slave_array[j];
					qst_evb_mag_read_reg(0x00, &chipid, 1);
					if((chipid==0x80)||(chipid==0x90)||(chipid==0x91)||(chipid==0x92)||(chipid==0x10)||(chipid==0x20))
					{
						qst_evb_mag_disable();
						//mag_dump_reg(1);
						break;
					}
				}
				if((chipid==0x80)||(chipid==0x90)||(chipid==0x91)||(chipid==0x92)||(chipid==0x10)||(chipid==0x20))
				{
					qst_logi("slot[%d] = 0x%02x\r\n", i, chipid);
				}
				else
				{
					qst_logi("slot[%d] = error!\r\n", i);
				}
			}

			qst_logi("Set chip index:\r\n");
			while(1)
			{
				scanf("%d", &t_mag.sel_mag_i);
				if((t_mag.sel_mag_i >= 0)&&(t_mag.sel_mag_i < t_mag.mag_num))
				{
					qst_logi("\r\nUser set test chip index[%d]\r\n\r\n", t_mag.sel_mag_i);
					break;
				}
				else
				{
					qst_logi("\r\nUser set set chip index[%d] error!\r\n\r\n", t_mag.sel_mag_i);
				}
			}
		}
		else
		{
			t_mag.sel_mag_i = 0;
		}
	}

	if(t_mag.sel_mag_i < t_mag.mag_num)
	{
		if(t_mag.sel_inf == INTERFACE_I2C_SW)
		{
			i2c_sw_gpio_config(t_mag.sel_mag_i);
		}
	}
}

void qst_mag_entry_set_delay(void)
{
	if(t_mag.sel_delay < 0)
	{
		qst_logi("User set sample delay(uint:ms):");
		while(1)
		{
			scanf("%d", &t_mag.sel_delay);
			if(t_mag.sel_delay >= 0)
			{
				qst_logi("\r\nUser set sample delay %dms\r\n\r\n", t_mag.sel_delay);
				break;
			}
			else
			{
				qst_logi("\r\nUser set sample delay %dms error\r\n\r\n", t_mag.sel_delay);
			}
		}
	}
}

void qst_evb_init_port(int sel_inf)
{
	if((sel_inf >= INTERFACE_I2C_SW)&&(sel_inf <= INTERFACE_I2C_HW_1M))
	{
		bsp_port_i2c_init((evb_interface_e)sel_inf);
	}
	else if((sel_inf >= INTERFACE_I3C_4M)&&(sel_inf <= INTERFACE_I3C_12_5M))
	{
#if defined(MAG_TEST_I3C_SUPPORT)
		bsp_port_i3c_init(sel_inf);
#endif
	}
}

void qst_evb_deinit_port(int sel_inf)
{
	if((sel_inf >= INTERFACE_I2C_SW)&&(sel_inf <= INTERFACE_I2C_HW_1M))
	{
		bsp_port_i2c_deinit();
	}
	else if((sel_inf >= INTERFACE_I3C_4M)&&(sel_inf <= INTERFACE_I3C_12_5M))
	{
#if defined(MAG_TEST_I3C_SUPPORT)
		bsp_port_i3c_deinit();
#endif
	}
}

void qst_evb_mag_para_init(int interface)
{
	memset(g_reg_tbl, 0, sizeof(g_reg_tbl));
	memset(&g_m_out, 0, sizeof(g_m_out));

	if(t_mag.mag_sensor == QST_SENSOR_ID_NONE)
	{	
		//memset(&t_mag, 0, sizeof(t_mag));
#if defined(QST_MAG_USE_SWJ)
		t_mag.wait_cfg = 1;
#else
		t_mag.wait_cfg = 0;
#endif
		t_mag.mag_sensor = QST_SENSOR_ID_NONE;
		t_mag.chipid = 0x00;
	
		t_mag.mag_num = SENSOR_NUM;
		t_mag.sel_mag_i = 0;
		t_mag.sel_inf = interface;
		t_mag.sel_test_i = -1;
		t_mag.sel_range = 0;
		t_mag.sel_odr = -1;
		t_mag.sel_osr1 = -1;
		t_mag.sel_osr2 = -1;
		t_mag.sel_zdbl = 0;	// default 1 : zdbl on 
		t_mag.sel_sr = 0;	// default 0 : set-reset on
		t_mag.sel_delay = 0;

		t_mag.ibi_en = 0;
		t_mag.fifo_en = 0;
		t_mag.st_en = 0;
		t_mag.sel_auto_test = -1;
		t_mag.config_id = 0;	
		t_mag.delay = 20;	
		t_mag.data_i = 0;
		t_mag.data_max = 0x7fffffff;
		t_mag.set_reset_num = 50;

		t_mag.fifo_ctrl = 0x00;
		t_mag.fifo_full = 0;
		t_mag.fifo_mode = MAG_FIFO_MODE_STREAM;
		t_mag.fifo_wmk = 0;

		t_mag.reg_max = 0x4b;
		t_mag.data_i = 0;
		t_mag.data_max = 0x7fffffff;

		g_m_out.mag_lsb = 1000;
		g_m_out.temp_lsb = 1024;


#if !defined(QST_MAG_USE_SWJ) && 0
		t_mag.sel_inf = interface;
		t_mag.sel_test_i = TEST_FACTORY;	//TEST_POLL_DATA_MANUAL;	// TEST_FACTORY
		t_mag.sel_range = 0;
		t_mag.sel_odr = 3;
		t_mag.sel_osr1 = 3;
		t_mag.sel_osr2 = 2;
		t_mag.sel_zdbl = 0;
		t_mag.sel_sr = 0;
		t_mag.sel_auto_test = 0;
		t_mag.data_max = 0x7fffffff;
#endif
	}
}


int qst_evb_mag_get_chipid(void)
{
	int retry = 5;

	while(retry > 0)
	{
		for(int i=0; (i<sizeof(mag_slave_array)/sizeof(mag_slave_array[0])); i++)
		{
			mag_slave = mag_slave_array[i];
			t_mag.chipid = 0x00;
			qst_evb_mag_read_reg(0x00, &t_mag.chipid, 1);
			if((t_mag.sel_inf>=INTERFACE_I2C_SW)&&(t_mag.sel_inf<=INTERFACE_I2C_HW_1M))
			{
				qst_logi("mag i2c slave[0x%02x] Id[0x%02x]\r\n", mag_slave, t_mag.chipid);
			}
			else
			{
				qst_logi("mag i3c Id[0x%02x]\r\n", t_mag.chipid);
			}
			if((t_mag.chipid == 0x90)||(t_mag.chipid == 0x92))
			{
				qst_logi("test qmc6309x\r\n");
				t_mag.mag_sensor = QST_MAG_QMC6309;
				t_mag.set.range = qmc6309_range;
				t_mag.set.range_num = sizeof(qmc6309_range)/sizeof(qmc6309_range[0]);
				t_mag.set.odr = qmc6309_odr;
				t_mag.set.odr_num = sizeof(qmc6309_odr)/sizeof(qmc6309_odr[0]);
				t_mag.set.osr1 = qmc6309_osr1;
				t_mag.set.osr1_num = sizeof(qmc6309_osr1)/sizeof(qmc6309_osr1[0]);
				t_mag.set.osr2 = qmc6309_osr2;
				t_mag.set.osr2_num = sizeof(qmc6309_osr2)/sizeof(qmc6309_osr2[0]);
				t_mag.set.sr = qmc6309_sr;
				t_mag.set.sr_num = sizeof(qmc6309_sr)/sizeof(qmc6309_sr[0]);

				t_mag.fifo_size = 8;
				t_mag.fifo_lv_shift = 4;

				t_mag.reg_max = 0x46;
				t_mag.item = mag_test_item;
				return 1;
			}
			else if(t_mag.chipid == 0x91)
			{
				qst_logi("test qmc6309v\r\n");
				t_mag.mag_sensor = QST_MAG_QMC6309V;
				t_mag.set.range = qmc6309v_range;
				t_mag.set.range_num = sizeof(qmc6309v_range)/sizeof(qmc6309v_range[0]);
				t_mag.set.odr = qmc6309v_odr;
				t_mag.set.odr_num = sizeof(qmc6309v_odr)/sizeof(qmc6309v_odr[0]);
				t_mag.set.osr1 = qmc6309v_osr1;
				t_mag.set.osr1_num = sizeof(qmc6309v_osr1)/sizeof(qmc6309v_osr1[0]);
				t_mag.set.osr2 = qmc6309v_osr2;
				t_mag.set.osr2_num = sizeof(qmc6309v_osr2)/sizeof(qmc6309v_osr2[0]);
				t_mag.set.sr = qmc6309v_sr;
				t_mag.set.sr_num = sizeof(qmc6309v_sr)/sizeof(qmc6309v_sr[0]);

				t_mag.fifo_size = 16;
				t_mag.fifo_lv_shift = 3;
			
				t_mag.reg_max = 0x47;
				t_mag.item = mag_test_item;
				return 1;
			}		
			else if(t_mag.chipid == 0x10)
			{
				qst_logi("test qmc6g00v\r\n");
				t_mag.mag_sensor = QST_MAG_MAESTRO_1V;
				t_mag.set.range = maestro_range;
				t_mag.set.range_num = sizeof(maestro_range)/sizeof(maestro_range[0]);
				t_mag.set.odr = maestro_odr;
				t_mag.set.odr_num = sizeof(maestro_odr)/sizeof(maestro_odr[0]);
				t_mag.set.osr1 = maestro_osr1;
				t_mag.set.osr1_num = sizeof(maestro_osr1)/sizeof(maestro_osr1[0]);
				t_mag.set.osr2 = maestro_osr2;
				t_mag.set.osr2_num = sizeof(maestro_osr2)/sizeof(maestro_osr2[0]);
				t_mag.set.sr = maestro_sr;
				t_mag.set.sr_num = sizeof(maestro_sr)/sizeof(maestro_sr[0]);

				t_mag.fifo_size = 16;
				t_mag.fifo_lv_shift = 3;
			
				t_mag.reg_max = 0x55;				
				t_mag.item = mag_test_item;
				return 1;
			}
			else if(t_mag.chipid == 0x20)
			{
				qst_logi("test qmc6g00h\r\n");
				t_mag.mag_sensor = QST_MAG_MAESTRO_2H;
				t_mag.set.range = maestro_range;
				t_mag.set.range_num = sizeof(maestro_range)/sizeof(maestro_range[0]);
				t_mag.set.odr = maestro_odr;
				t_mag.set.odr_num = sizeof(maestro_odr)/sizeof(maestro_odr[0]);
				t_mag.set.osr1 = maestro_osr1;
				t_mag.set.osr1_num = sizeof(maestro_osr1)/sizeof(maestro_osr1[0]);
				t_mag.set.osr2 = maestro_osr2;
				t_mag.set.osr2_num = sizeof(maestro_osr2)/sizeof(maestro_osr2[0]);
				t_mag.set.sr = maestro_sr;
				t_mag.set.sr_num = sizeof(maestro_sr)/sizeof(maestro_sr[0]);
				t_mag.sel_sr = 2;	// test

				t_mag.fifo_size = 16;
				t_mag.fifo_lv_shift = 3;
			
				t_mag.reg_max = 0x55;				
				t_mag.item = mag_test_item;
				return 1;
			}
#if defined(QMC6308)
			else if(t_mag.chipid == 0x80)
			{
				qst_logi("test qmc6308\r\n");
				t_mag.mag_sensor = QST_MAG_QMC6308;
				t_mag.set.range = qmc6308_range;
				t_mag.set.range_num = sizeof(qmc6308_range)/sizeof(qmc6308_range[0]);
				t_mag.set.odr = qmc6308_odr;
				t_mag.set.odr_num = sizeof(qmc6308_odr)/sizeof(qmc6308_odr[0]);
				t_mag.set.osr1 = qmc6308_osr1;
				t_mag.set.osr1_num = sizeof(qmc6308_osr1)/sizeof(qmc6308_osr1[0]);
				t_mag.set.osr2 = qmc6308_osr2;
				t_mag.set.osr2_num = sizeof(qmc6308_osr2)/sizeof(qmc6308_osr2[0]);
				t_mag.set.sr = qmc6308_sr;
				t_mag.set.sr_num = sizeof(qmc6308_sr)/sizeof(qmc6308_sr[0]);
				t_mag.fifo_size = 0;
				t_mag.fifo_lv_shift = 0;

				t_mag.reg_max = 0x40;				
				t_mag.item = mag_test_item_qmc6308;
				return 1;
			}
#endif
			if(t_mag.set.range)
			{
				t_mag.set.range_num = 1;	// fix range 1
			}

			if((t_mag.sel_odr<0)||(t_mag.sel_odr>=t_mag.set.odr_num))
			{
				t_mag.sel_odr = 0;
			}
			if((t_mag.sel_range<0)||(t_mag.sel_range>=t_mag.set.range_num))
			{
				t_mag.sel_range = 0;
			}
			if((t_mag.sel_osr1<0)||(t_mag.sel_osr1>=t_mag.set.osr1_num))
			{
				t_mag.sel_osr1 = 0;
			}
			if((t_mag.sel_osr2<0)||(t_mag.sel_osr2>=t_mag.set.osr2_num))
			{
				t_mag.sel_osr2 = 0;
			}
			if((t_mag.sel_sr<0)||(t_mag.sel_sr>=t_mag.set.sr_num))
			{
				t_mag.sel_sr = 0;
			}
			if(t_mag.fifo_wmk >= t_mag.fifo_size)
			{
				t_mag.fifo_wmk = t_mag.fifo_size-1;
			}

		}
		retry--;
	}

	qst_logi("mag get chipid FAIL slave 0x%01x id 0%02x\r\n", mag_slave, t_mag.chipid);
#if defined(QST_MAG_USE_SWJ)
	//NVIC_SystemReset();
#endif
	return 0;
}


void qst_evb_mag_test_entry(int interface)
{
	bsp_event_clear();
	qst_evb_mag_para_init(interface);
	qst_mag_entry_sel_interface();
	qst_mag_entry_set_id();

	if(qst_evb_mag_get_chipid())
	{
		qst_evb_mag_soft_reset();
		mag_dump_reg(1);
#if defined(CUSTOMER_CONFIG_REG)
		evb_setup_uart_rx(DEBUG_USART, NULL);
		qst_mag_entry_customer_config_reg();
		mag_dump_reg(1);
		qst_evb_deinit_port(t_mag.sel_inf);
		t_mag.sel_inf = -1;
		qst_mag_entry_sel_interface();
#endif
//re_test:
		evb_setup_uart_rx(DEBUG_USART, NULL);
		qst_mag_entry_sel_test_item();	// select test item
		if(t_mag.sel_auto_test == 0)	// manual select
		{
			switch(t_mag.sel_test_i)
			{
			case TEST_POLL_DATA_MANUAL:
			case TEST_POLL_FIFO_MANUAL:
			case TEST_IBI_TO_DRDY:
#if defined(MAG_TEST_I3C_SUPPORT)
			case TEST_IBI_TO_FIFO_FULL:
			case TEST_IBI_TO_FIFO_WMK:
			case TEST_IBI_TO_DATA_OVL:
#endif
				qst_mag_entry_sel_odr();
				qst_mag_entry_sel_range_osr();
				qst_mag_entry_sel_set_reset();
				break;
			case TEST_WRITE_READ_REGISTER:
#if defined(QST_EVB_REG_WR_PLUS)
				t_mag.sel_odr = 0;
#else
				qst_mag_entry_sel_odr();
#endif
				qst_mag_auto_sel_range_osr(t_mag.sel_odr);
				t_mag.sel_sr = -1;
				qst_mag_entry_sel_set_reset();
				break;
			case TEST_SETRESET_SWITCH:
				qst_mag_entry_sel_odr();
				qst_mag_entry_sel_range_osr();
				qst_mag_entry_sel_sr_switch();
				break;
			case TEST_SELFTEST:
			case TEST_SOFT_RESET:
			case TEST_OTP:
				t_mag.sel_odr = 0;
				//t_mag.sel_sr = 0;
				qst_mag_auto_sel_range_osr(t_mag.sel_odr);
			#if !defined(FPGA)
				qst_mag_entry_set_counter();
			#endif
				break;
			case TEST_FACTORY:
				t_mag.sel_odr = 0;
				//t_mag.sel_sr = 0;
				qst_mag_auto_sel_range_osr(t_mag.sel_odr);
				break;
#if defined(MAG_TEST_I3C_SUPPORT)
			case TEST_I3C_CCC:
				qst_mag_entry_sel_ccc();
				t_mag.sel_range = 0;
				t_mag.sel_odr = 0;
				t_mag.sel_osr1 = 0;
				t_mag.sel_osr2 = 0;
				t_mag.sel_zdbl = 1;
				//t_mag.sel_sr = 0;
				break;
#endif
#if defined(FPGA)
			case TEST_GAIN_OFFSET:
			case TEST_TCO:
			case TEST_TCS:
			case TEST_KMTX:
				t_mag.sel_range = 0;
				t_mag.sel_odr = 0;
				t_mag.sel_osr1 = 0;
				t_mag.sel_osr2 = 0;
				t_mag.sel_zdbl = 1;
				//t_mag.sel_sr = 0;
				break;
#endif
			default:
				t_mag.sel_odr = 0;
				//t_mag.sel_sr = 0;
				qst_mag_auto_sel_range_osr(t_mag.sel_odr);
				break;
			}
		}
		else
		{
			t_mag.sel_range = 0;
			t_mag.sel_odr = 0;
			//t_mag.sel_sr = 0;
			t_mag.sel_osr1 = 0;
			t_mag.sel_osr2 = 0;
		}

		if((t_mag.sel_range<0)||(t_mag.sel_odr<0)||(t_mag.sel_osr1<0)||(t_mag.sel_osr2<0)||(t_mag.sel_zdbl<0)||(t_mag.sel_sr<0)||(t_mag.sel_auto_test<0))
		{
			qst_logi("select test parameter error!\r\n");
			while(1) {}
		}

		t_mag.delay = t_mag.set.odr[t_mag.sel_odr].delay;
	}
	else
	{
#ifdef AK0991X_MMC5603
		qst_evb_mag_entry_other();
#endif
		return;
	}

	evb_setup_uart_rx(DEBUG_USART, qst_evb_mag_uart_rx_hdlr);
	switch(t_mag.sel_test_i)
	{
		case TEST_POLL_DATA_MANUAL:
			qst_mag_entry_set_delay();
			qst_evb_mag_enable();
			t_mag.data_max = 0x7fffffff;
			evb_setup_timer(TIM2, qst_evb_mag_sensor_data, t_mag.delay, ENABLE);
			//evb_setup_user_key(1, qst_evb_mag_key1_exit_hdlr);
			break;
		case TEST_POLL_DATA_AUTO:
			t_mag.config_id = 0;
			if(qst_evb_mag_select_config(t_mag.config_id))
			{
				evb_setup_timer(TIM2, qst_evb_mag_auto_sensor_data, t_mag.delay, ENABLE);
			}
			break;
		case TEST_POLL_FIFO_MANUAL:
			t_mag.fifo_en = 1;
			qst_evb_mag_enable();
			t_mag.data_max = 0x7fffffff;
			evb_setup_timer(TIM2, qst_evb_mag_sensor_fifo_data, t_mag.delay, ENABLE);
			//evb_setup_user_key(1, qst_evb_mag_key1_exit_hdlr);
			break;
		case TEST_POLL_FIFO_AUTO:
			t_mag.sel_auto_test = 1;
			t_mag.fifo_en = 1;
			t_mag.config_id = 0;
			if(qst_evb_mag_select_config(t_mag.config_id))
			{
				evb_setup_timer(TIM2, qst_evb_mag_sensor_fifo_auto_data, t_mag.delay, ENABLE);
			}
			break;
		case TEST_POLL_ODR:
			qst_evb_mag_test_odr(0);
			break;
		case TEST_WRITE_READ_REGISTER:
			qst_mag_entry_set_counter();
			qst_evb_mag_enable();
			t_mag.data_max = 50000;
			evb_setup_timer(TIM3, qst_evb_test_reg_wr_plus, /*t_mag.delay*/5, ENABLE);
			break;
		case TEST_WORK_CURRENT:
			qst_evb_mag_current_test();
			break;
		case TEST_WORK_MODE_SWITCH:
			qst_evb_test_work_mode_switch();
			break;
		case TEST_SETRESET_SWITCH:
#if 0
			qst_evb_mag_setreset_test();
#else
			qst_evb_mag_enable();
			t_mag.data_max = 0x7fffffff;
			evb_setup_timer(TIM2, qst_evb_mag_sr_mode_switch_data, t_mag.delay, ENABLE);
#endif
			break;
		case TEST_SELFTEST:
			t_mag.data_max = 0x7fffffff;
			if(t_mag.sel_delay > 0)
			{
				t_mag.delay = t_mag.sel_delay;
			}
			else
			{
				t_mag.delay = 200;
			}
			if(t_mag.mag_sensor == QST_MAG_QMC6308)
			{
			#if defined(QMC6308)
				evb_setup_timer(TIM2, qst_evb_mag_selftest_qmc6308, t_mag.delay, ENABLE);
			#endif
			}
			else
			{
				evb_setup_timer(TIM2, qst_evb_mag_selftest_hldr, t_mag.delay, ENABLE);
			}
			break;
		case TEST_SOFT_RESET:
			qst_evb_mag_softreset_test();
			break;
		case TEST_OTP:
			qst_evb_mag_otp_test();
			break;
		case TEST_FACTORY:
			qst_evb_mag_factory_test();
			break;
#if defined(MAG_TEST_I3C_SUPPORT)
		case TEST_IBI_TO_DRDY:
			t_mag.config_id = 0;
			if(t_mag.sel_auto_test == 1)
			{
				//qst_evb_mag_select_config(t_mag.config_id);
				unsigned char s_osr1, s_osr2;
				if(t_mag.set.odr == maestro_odr)
				{
					s_osr1 = maestro_osr1[3];
					s_osr2 = maestro_osr2[0];
				}
				else if(t_mag.set.odr == qmc6309_odr)
				{
					s_osr1 = qmc6309_osr1[3];
					s_osr2 = qmc6309_osr2[0];
				}
				else if(t_mag.set.odr == qmc6309v_odr)
				{
					s_osr1 = qmc6309v_osr1[3];
					s_osr2 = qmc6309v_osr2[0];
				}				
				qst_evb_mag_select_config_lite(t_mag.config_id, s_osr1, s_osr2);
			}
			else
			{
				qst_evb_mag_enable();
				t_mag.data_max = 0x7fffffff;
			}
			qst_evb_reg_i3c_ibi_hdlr(qst_evb_i3c_ibi_drdy);
			//qst_evb_reg_i3c_ibi_hdlr(qst_evb_i3c_ibi_drdy_test_odr);
			break;
		case TEST_IBI_TO_FIFO_FULL:
			t_mag.config_id = 0;
			t_mag.fifo_en = 1;
			if(t_mag.sel_auto_test == 1)
			{
				//qst_evb_mag_select_config(t_mag.config_id);
				unsigned char s_osr1, s_osr2;
				if(t_mag.set.odr == maestro_odr)
				{
					s_osr1 = maestro_osr1[3];
					s_osr2 = maestro_osr2[0];
				}
				else if(t_mag.set.odr == qmc6309_odr)
				{
					s_osr1 = qmc6309_osr1[3];
					s_osr2 = qmc6309_osr2[0];
				}
				else if(t_mag.set.odr == qmc6309v_odr)
				{
					s_osr1 = qmc6309v_osr1[3];
					s_osr2 = qmc6309v_osr2[0];
				}
				qst_evb_mag_select_config_lite(t_mag.config_id, s_osr1, s_osr2);
			}
			else
			{
				qst_evb_mag_enable();
				t_mag.data_max = 0x7fffffff;
			}
			qst_evb_reg_i3c_ibi_hdlr(qst_evb_i3c_ibi_fifo);
			break;
		case TEST_IBI_TO_FIFO_WMK:
			t_mag.config_id = 0;
			t_mag.fifo_en = 1;
			if(t_mag.sel_auto_test == 1)
			{
				//qst_evb_mag_select_config(t_mag.config_id);
				qst_evb_mag_select_config_lite(t_mag.config_id, t_mag.set.osr1[3], t_mag.set.osr2[0]);
			}
			else
			{
				qst_evb_mag_enable();
				t_mag.data_max = 0x7fffffff;
			}
			qst_evb_reg_i3c_ibi_hdlr(qst_evb_i3c_ibi_fifo);
			break;
		case TEST_IBI_TO_DATA_OVL:
			qst_evb_mag_enable();
			qst_evb_reg_i3c_ibi_hdlr(qst_evb_i3c_ibi_drdy);
			t_mag.data_max = 0x7fffffff;
			break;
		case TEST_IBI_TO_SELFTEST:
			t_mag.st_en = 1;
			qst_evb_mag_enable();
			//qst_evb_mag_enable_ibi(QMC6309_IBI_ST_RDY);
			//qst_evb_i3c_enable_ibi(1);
			//qst_delay_ms(1);
			qst_evb_reg_i3c_ibi_hdlr(qst_evb_i3c_ibi_mag_selftest);
			qst_evb_mag_selftest_enable();
			//qst_evb_mag_read_reg(MAG_STATUS_REG, &g_buf[0], 1);
			//evb_setup_timer(TIM2, qst_evb_mag_i3c_loop_status, 10, ENABLE);
			break;
		case TEST_I3C_CCC:
			I3C_Send_Direct_CCC(t_mag.reg);
			NVIC_SystemReset();
			break;
#endif
#if defined(FPGA)
		case TEST_GAIN_OFFSET:
			evb_setup_uart_rx(DEBUG_USART, NULL);
			//evb_setup_uart_rx(DEBUG_USART, qst_evb_mag_uart_rx_hdlr);
			int input_data = -1;
			while(1)
			{
				qst_logi("Select test gain or offset:\r\n");
				qst_logi("[0]: test apply offset\r\n");
				qst_logi("[1]: test bypass offset\r\n");
				qst_logi("[2]: test apply gain\r\n");
				qst_logi("[3]: test bypass gain\r\n");
				qst_logi("[4]: test apply coarse gain\r\n");
	
				scanf("%d", &input_data);
				if((input_data >= 0)||(input_data <= 3))
				{
					qst_logi("User select function: %d\r\n\r\n", input_data);
					break;
				}
				else
				{
					qst_logi("User select function: %d error\r\n\r\n", input_data);
				}
			}			
			if((input_data == 1)||(input_data == 3))
			{
				t_mag.otp_bypass = 1;
				qst_evb_mag_set_bypass(0x80, 1);
			}
			else if((input_data == 0)||(input_data == 2))
			{
				t_mag.otp_bypass = 0;
				qst_evb_mag_set_bypass(0x80, 0);
			}
			qst_evb_mag_enable();
			if((input_data == 0)||(input_data == 1))
			{
				mag_offset[0] = mag_offset[1] = mag_offset[2] = -32768;
				evb_setup_timer(TIM2, qst_evb_mag_auto_apply_offset, t_mag.delay, ENABLE);
			}
			else if((input_data == 2)||(input_data == 3))
			{
				mag_gain[0] = mag_gain[1] = mag_gain[2] = 0;
				evb_setup_timer(TIM2, qst_evb_mag_auto_apply_gain, t_mag.delay, ENABLE);
			}
			else if(input_data == 4)
			{
				evb_setup_timer(TIM2, qst_evb_mag_auto_apply_coarse_gain, t_mag.delay, ENABLE);	
			}
			break;
		case TEST_TCO:
			qst_evb_mag_tco_test(0);
			break;		
		case TEST_TCS:
			qst_evb_mag_tcs_test(0);
			break;
		case TEST_KMTX:
	#if defined(MAESTRO1)
			qst_evb_mag_kmtx_test();
	#else
			qst_evb_mag_cross_sensitivity_test();
	#endif
			break;
#endif
		case TEST_MISC:
	#if 0
			qst_evb_mag_write_reg(MAG_CTL_REG_TWO, 0x08);
			qst_delay_ms(1);
			qst_evb_mag_write_reg(MAG_CTL_REG_ONE, 0x03);
			qst_delay_ms(1);
			mag_dump_reg(1);
			evb_setup_timer(TIM2, qst_evb_mag_misc_hdlr, 10, ENABLE);
	#elif defined(MAG_TEST_I3C_SUPPORT)
			t_mag.sel_test_i = TEST_IBI_TO_DRDY;
			t_mag.sel_odr = 4;		// 10 Hz
			t_mag.sel_osr1 = 3;
			t_mag.sel_osr2 = 0;
			t_mag.sel_sr = 0;

			t_mag.fifo_en = 1;
			t_mag.fifo_full = 1;
			t_mag.fifo_mode = MAG_FIFO_MODE_FIFO;
			t_mag.fifo_wmk = t_mag.fifo_size - 1;

			t_mag.ibi_en = 1;

			qst_evb_mag_enable();
			t_mag.data_i = 0;
			t_mag.data_max = 0x7fffffff;
			qst_evb_reg_i3c_ibi_hdlr(qst_evb_i3c_ibi_drdy_test);
	#endif
			break;
		default:
			qst_logi("Select function error!\r\n");
			break;
	}

	qst_evb_mag_enable_single();

#if defined(MAG_TEST_I3C_SUPPORT)
	evb_setup_user_key(1, qst_evb_mag_key1_exit_hdlr);
//	if(t_mag.ibi_en)
//	{
//		qst_evb_i3c_enable_ibi(1);
//	}
#endif
}


