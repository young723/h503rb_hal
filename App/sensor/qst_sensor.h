
#ifndef __QST_SENSOR_H
#define __QST_SENSOR_H

#ifdef __cplusplus
 extern "C" {
#endif 

#if defined(__CC_ARM)
#pragma anon_unions
#endif

//#define QMAX981_SUPPORT
//#define QMA6101T_SUPPORT
#define QMC5883L_SUPPORT
//#define QMC5883P_SUPPORT
#define ICM4X6XX_SUPPORT
//#define QMP6988_SUPPORT
//#define QMP6990_SUPPORT

#include "qst_log.h"
#include "bsp_hardware.h"
#include "qst_sensor_id.h"

#include "qma6100p.h"
#if defined(QMAX981_SUPPORT)
#include "qmaX981.h"
#endif
#if defined(QMA6101T_SUPPORT)
#include "qma6101t.h"
#endif

#include "qmc6308.h"
#include "qmc6309.h"
#include "qmc6309v.h"
#include "qmc6g00x.h"
#if defined(QMC5883L_SUPPORT)
#include "qmc5883l.h"
#endif
#if defined(QMC5883P_SUPPORT)
#include "qmc5883p.h"
#endif
//#include "ak0991x.h"
//#include "mmc5603.h"
//#include "rtrobot_bmm350.h"

#include "qmi8658.h"
#if defined(ICM4X6XX_SUPPORT)
#include "icm4x6xx.h"
#endif

#include "qmp6989.h"
#if defined(QMP6988_SUPPORT)
#include "qmp6988.h"
#endif
#if defined(QMP6990_SUPPORT)
#include "qmp6990.h"
#endif

#include "magnetic_test.h"

#include "qst_algo_imu_cali.h"
//app filter
#include "MyFilter.h"
#include "butterworth.h"
//app filter
// protocol
#include "qst_uart_wincc.h"
// protocol

#ifndef Pi
#define Pi 3.14159265358979f
#define RadToDeg		(180.0f/Pi)
#define RegToRad		(Pi/180.0f)
#endif

typedef enum
{
	QST_UART_PROTOCOL_NONE = 0x00,
	QST_UART_PROTOCOL_ASCII,
	QST_UART_PROTOCOL_OWN,
	QST_UART_PROTOCOL_ANO,
	QST_UART_PROTOCOL_SERIAL_PLOTTER_1,

	QST_UART_PROTOCOL_TOTAL
} qst_uart_protocol;

typedef enum
{
	QST_SENSOR_NONE = 0x0000,
	QST_SENSOR_ACCEL = 0x0001,
	QST_SENSOR_MAG = 0x0002,
	QST_SENSOR_IMU = 0x0004,
	QST_SENSOR_PRESS = 0x0008,
	QST_SENSOR_MAG_TEST = 0x0010,

	QST_SENSOR_TIMER = 0x0400,

	QST_SENSOR_FUSION = 0x8000,

	//QST_SENSOR_NUM = 5
} qst_sensor_type;


typedef struct
{
	char				*name;
	qst_sensor_type		sensor;
	int					report;
	float				odr;
	evb_interface_e		port;
} qst_sensor_list_t;

typedef struct
{
	int					sensor;
	int					port;
	unsigned char		reset_flag;
	int					intf;
	unsigned int		flash_index;
} qst_evb_t;

extern unsigned char * qst_evb_get_fifo_buf(void);

#ifdef __cplusplus
}
#endif

#endif

