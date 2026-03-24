#ifndef __QMA6101T_H
#define __QMA6101T_H

#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "bsp_hardware.h"

#ifndef QST_BASE_TYPE
#define QST_BASE_TYPE
typedef		signed char					qs8;
typedef		unsigned char				qu8;
typedef		short						qs16;
typedef		unsigned short				qu16;
typedef		int							qs32;
typedef		unsigned int				qu32;
typedef		long long					qs64;
typedef		unsigned long long			qu64;
typedef		float						qf32;
typedef		double						qd64;
#endif


#define M_PI		3.141592653589793f
#define M_G			9.80665f


#define QMA6101T_LOG		printf
#define QMA6101T_ERR		printf

//#define QMA6101T_USE_SPI  
//#define QST_USE_SPI 

//#define QST_USE_SW_I2C  QST_USE_SW_I2C

#define QMA6101T_DATA_READY
//#define QMA6101T_FIFO_FUNC

//#define QMA6101T_ANY_MOTION
//#define QMA6101T_MOTION_LPF
//#define QMA6101T_SIGNIFICANT_MOTION
//#define QMA6101T_NO_MOTION

#define QMA6101T_INT_LATCH
#define QMA6101T_DATA_LPF


//#define QMA6101T_TRIMED

#define QMA6101T_DEVICE_ID    0x0a    //0xa0


#define QMA6101T_I2C_SLAVE_ADDR			0x12
#define QMA6101T_I2C_SLAVE_ADDR2		0x13


#define QMA6101T_FAIL					0
#define QMA6101T_SUCCESS				1

#define GRAVITY_EARTH_1000 				9807	// about (9.80665f)*1000   mm/s2
#define QMA6101T_ABS(X) 				((X) < 0 ? (-1 * (X)) : (X))


/*Register Map*/
#define QMA6101T_CHIP_ID		    0x00
#define QMA6101T_YOUTL				0x01
#define QMA6101T_YOUTH				0x02
#define QMA6101T_ZOUTL				0x03
#define QMA6101T_ZOUTH				0x04
#define QMA6101T_XOUTL				0x05
#define QMA6101T_XOUTH				0x06
#define QMA6101T_INT_STATUS_0		0x09
#define QMA6101T_INT_STATUS_1		0x0a
#define QMA6101T_INT_STATUS_2		0x0b
#define QMA6101T_INT_STATUS_3		0x0c
#define QMA6101T_FIFO_STATE			0x0e
#define QMA6101T_REG_RANGE			0x0f
#define QMA6101T_REG_BW_ODR			0x10
#define QMA6101T_REG_POWER_MANAGE	0x11
#define QMA6101T_INT_EN_0			0x16
#define QMA6101T_INT_EN_1			0x17
#define QMA6101T_INT_EN_2			0x18
#define QMA6101T_INT1_MAP_0			0x19
#define QMA6101T_INT1_MAP_1			0x1a
#define QMA6101T_INT2_MAP_0			0x1b
#define QMA6101T_INT2_MAP_1			0x1c

#define QMA6101T_INTPIN_CFG			0x20
#define QMA6101T_INT_CFG			0x21


#define QMA6101T_REG_NVM			0x33
#define QMA6101T_REG_RESET			0x36


#define QMA6101T_DRDY_BIT			0x10	// enable 1

#define QMA6101T_AMD_X_BIT			0x01
#define QMA6101T_AMD_Y_BIT			0x02
#define QMA6101T_AMD_Z_BIT			0x04


typedef enum
{
	QMA6101T_DISABLE = 0,
	QMA6101T_ENABLE = 1
}qma6101t_enable;


typedef enum
{
	QMA6101T_MAP_INT1,
	QMA6101T_MAP_INT2,
	QMA6101T_MAP_INT_NONE
}qma6101t_int_map;

typedef enum
{
	QMA6101T_BW_1600 = 0,
	QMA6101T_BW_800 = 1,
	QMA6101T_BW_400 = 2,
	QMA6101T_BW_200 = 3,
	QMA6101T_BW_100 = 4,
	QMA6101T_BW_50 = 5,
	QMA6101T_BW_OTHER = 12
}qma6101t_bw;


typedef enum
{
	QMA6101T_ODR_1600 = 0,
	QMA6101T_ODR_800 = 1,
	QMA6101T_ODR_400 = 2,
	QMA6101T_ODR_200 = 3,
	QMA6101T_ODR_100 = 4,
	QMA6101T_ODR_50 = 5,
	QMA6101T_ODR_25 = 6,
	QMA6101T_ODR_12_5 = 7,
	QMA6101T_ODR_6_25 = 8,
	QMA6101T_ODR_3_125 = 9,
	QMA6101T_ODR_1_56 = 10,
	QMA6101T_ODR_0_78 = 11,
	QMA6101T_ODR_OTHER = 12
}qma6101t_odr;

typedef enum
{
	QMA6101T_RANGE_2G = 0x01,
	QMA6101T_RANGE_4G = 0x02,
	QMA6101T_RANGE_8G = 0x03,
	QMA6101T_RANGE_16G = 0x04,
//	QMA6101T_RANGE_32G = 0x0f
}qma6101t_range;


typedef enum
{
	QMA6101T_HPF_0 = (0x00<<4),
	QMA6101T_HPF_1 = (0x01<<4),
	QMA6101T_HPF_2 = (0x02<<4),
	QMA6101T_HPF_3 = (0x03<<4),
	QMA6101T_HPF_4 = (0x04<<4),
	QMA6101T_HPF_5 = (0x05<<4),
	QMA6101T_HPF_6 = (0x06<<4),
	QMA6101T_HPF_7 = (0x07<<4),
	QMA6101T_HPF_RESERVED = 0xff
}qma6101t_nhpf;


typedef enum
{
	QMA6101T_LPF_1 = (0x00<<5),
	QMA6101T_LPF_2 = (0x01<<5),
	QMA6101T_LPF_4 = (0x02<<5),
	QMA6101T_LPF_8 = (0x03<<5),
	QMA6101T_LPF_RESERVED = 0xff
}qma6101t_nlpf;



typedef enum
{
	QMA6101T_MODE_STANDBY = 0,
	QMA6101T_MODE_ACTIVE = 1,
	QMA6101T_MODE_MAX
}qma6101t_mode;


typedef enum
{
	QMA6101T_LPF_SEL = (0x00<<3),
	QMA6101T_HPF_SEL = (0x01<<3),
	QMA6101T_FILTER_MAX
}qma6101t_filter;




typedef enum
{
	QMA6101T_FIFO_MODE_NONE,
	QMA6101T_FIFO_MODE_FIFO,
	QMA6101T_FIFO_MODE_STREAM,
	QMA6101T_FIFO_MODE_BYPASS,
	QMA6101T_FIFO_MODE_MAX
}qma6101t_fifo_mode;



typedef enum
{
	QMA6101T_SLOPE = 0x00,
	QMA6101T_GRAVITY = 0x40,
	QMA6101T_AMD_MAX = 0xff
}qma6101t_amd_type;


typedef enum
{
	QMA6101T_AMD_DISABLE = 0x00,
	QMA6101T_AMD_ENABLE_X  = 0x01,
	QMA6101T_AMD_ENABLE_Y  = 0x02,		 
	QMA6101T_AMD_ENABLE_Z  = 0x04,
	QMA6101T_AMD_ENABLE_X_Y = 0x01 | 0x02,
	QMA6101T_AMD_ENABLE_X_Z = 0x01 | 0x04,
	QMA6101T_AMD_ENABLE_Y_Z = 0x02 | 0x04,
	QMA6101T_AMD_ENABLE_X_Y_Z = 0x01 | 0x02 | 0x04,
	QMA6101T_AMD_ENABLE_MAX = 0xff
}qma6101t_amd_enable;


typedef enum
{
	QMA6101T_NMD_DISABLE = 0x00,
	QMA6101T_NMD_ENABLE_X  = 0x20,
	QMA6101T_NMD_ENABLE_Y  = 0x40,		 
	QMA6101T_NMD_ENABLE_Z  = 0x80,
	QMA6101T_NMD_ENABLE_X_Y = 0x20 | 0x40,
	QMA6101T_NMD_ENABLE_X_Z = 0x20 | 0x80,
	QMA6101T_NMD_ENABLE_Y_Z = 0x40 | 0x80,
	QMA6101T_NMD_ENABLE_X_Y_Z = 0x20 | 0x40 | 0x80,
	QMA6101T_NMD_ENABLE_MAX = 0xff
}qma6101t_nmd_enable;


extern qs32 qma6101t_writereg(qu8 reg_add,qu8 reg_dat);
extern qs32 qma6101t_readreg(qu8 reg_add,qu8 *buf,qu16 num);
extern qu8 qma6101t_chip_id(void);
extern qs32 qma6101t_init(int protocol);
extern qs32 qma6101t_set_range(qs32 range, qs32 hpf, qs32 hpf_lpf_sel);
extern qs32 qma6101t_read_raw_xyz(qs16 *data);
extern qs32 qma6101t_read_acc_xyz(float *accData);
#if defined(QMA6101T_DATA_READY)
extern void qma6101t_drdy_config(qs32 int_map, qs32 enable);
#endif
#if defined(QMA6101T_FIFO_FUNC)
extern void qma6101t_fifo_config(qma6101t_fifo_mode fifo_mode, qu8 wmk, qs32 int_map, qs32 enable);
extern qs32 qma6101t_read_fifo(qu8 *fifo_buf);
#endif


#if defined(QMA6101T_ANY_MOTION)
extern void qma6101t_anymotion_config(qs32 thr, qs32 duration, qs32 slope, qs32 amd_enable, qs32 int_map);
#endif

#if defined(QMA6101T_SIGNIFICANT_MOTION)
extern void qma6101t_sigmotion_config(qs32 thr, qs32 duration, qs32 slope, qs32 skip, qs32 proof, qs32 amd_enable, qs32 enable, qs32 int_map);
#endif


#if defined(QMA6101T_NO_MOTION)
extern void qma6101t_nomotion_config(qs32 thr, qu16 duration, qs32 enable, qs32 int_map);
#endif

extern void qma6101t_irq_hdlr(void);


typedef struct
{
	qu8					protocol;
	qu8					slave;
	qu8					chip_id;
	qs32				lsb_1g;
	qma6101t_fifo_mode	fifo_mode;
	qs32				fifo_len;
	qs16				raw[3];
	float				acc[3];
}qma6101t_data;


#endif  /*QMA6101T*/
