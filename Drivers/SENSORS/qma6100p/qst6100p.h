/*****************************************************************************
written  by rock.fan in 1/21/2022

 *****************************************************************************/
#ifndef QST_QMA6100P_H

#define QST_QMA6100P_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include "bsp_hardware.h"

///----------------------------------------------------------------------------
/// Macros
/// ---------------------------------------------------------------------------
//#define _CFG_BUS_SPI

#ifndef _CFG_BUS_SPI
#define _CFG_BUS_I2C
#endif

/*
#pragma anon_unions
union rtc_status {
    uint32_t status;
    struct {
        unsigned sec :6 ;
        unsigned min :6 ;
        unsigned hour:5 ;
        unsigned day :15;
    };
} rtc, alarm;
*/

#define DIS_AD0

#define STANDBYMODE 0
#define WAKEMODE 1

#define MCLK_100KHZ 3
#define MCLK_51KHZ 4
#define MCLK_25KHZ 5
#define MCLK_12KHZ 6
#define MCLK_6KHZ 7
#define MCLK_Reserved 8

#define HPF 0x40
#define LPF 0x00

typedef enum
{
	HPCF_NONE=0x00,
	HPCF_ODRDIV10=(0x10<<1),
	HPCF_ODRDIV25=0x20<<1,
	HPCF_ODRDIV50=0x30<<1,
	HPCF_ODRDIV100=0x40<<1,
	HPCF_ODRDIV200=0x50<<1,
	HPCF_ODRDIV400=0x60<<1,
	HPCF_ODRDIV800=0x70<<1

} HPF_CF;
typedef enum
{
	LPCF_NONE=0x00<<1,
	LPCF_AVG1=0x40<<1,
	LPCF_AVG2=0x10<<1,
	LPCF_AVG3=0x20<<1,
	LPCF_AVG4=0x30<<1
} LPF_CF;
typedef enum
{
	AM_SLOPE=0x00,
	AM_GRAVITY=0x40
}AM_TYPE;
typedef enum
{
	RANGE_2G=0x1,
	RANGE_4G=0x2,
	RANGE_8G=0x4,
	RANGE_16G=0x8,
	RANGE_32G=0xf
}Range_V;
typedef enum
{
	QMA6100_TAP_SINGLE = 0x80,
	QMA6100_TAP_DOUBLE = 0x20,
	QMA6100_TAP_TRIPLE = 0x10,
	QMA6100_TAP_QUARTER = 0x01,
	QMA6100_TAP_NONE = 0x00
}qma6100P_tap;
typedef enum
{
	AXIS_X=0x01,
	AXIS_Y=0x02,
	AXIS_Z=0x04
}AXIS_SEL;

typedef enum
{
	PORT_1=1,
	PORT_2=2,
	PORT_3=3,	
}Port_V;
#define FIFO_MODE_FIFO 		0x40
#define FIFO_MODE_STREAM	0x80 
#define FIFO_MODE_BYPASS 	0x00
#define FIFO_INT_WATERMARK  0x40
#define FIFO_INT_FULL  		0x20
#define FIFO_INT_NONE	    0xFF

extern int gsensor_init(void);
extern void gsensor_handle_int(uint8_t port);
extern int qma6100P_init(void);
extern int acc_read_acc_xyz(float accData[3]);


#ifdef __cplusplus
}
#endif

#endif
