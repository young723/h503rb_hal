/*
 *
 ****************************************************************************
 * Copyright (C) 2021 QST Corporation. <www.qstcorp.com>
 * All rights reserved.
 *
 * File : qmp6989.c
 *
 * Date : 2021/06/05
 *
 * Usage: QMP6989 sensor driver header file
 *
 ****************************************************************************
 *
 */

/*! @file qmp6989.h
 *  @brief  QMP6989 Sensor Driver Header File 
 *  @author 
 */
 
#ifndef __QMP6989_H__
#define __QMP6989_H__

#include "type_support.h"
#include "bsp_hardware.h"

//#define QMP6989_USE_SPI
#define FLOAT_SUPPORT

#define QMP6989_7BIT_I2C_ADDR		0x6C		// 0x6C  0x6D
#define QMP6989_8BIT_I2C_ADDR		(QMP6989_7BIT_I2C_ADDR<<1)

#define QMP6989_TEMPERATURE_SENSITIVITY 256  	//   Celsius = 256 code
#define QMP6989_T_CODE_TO_CELSIUS(tCode) (((float)(tCode)) / QMP6989_TEMPERATURE_SENSITIVITY)

//Registers Address
#define QMP6989_REG_RESET		0x00
#define QMP6989_REG_PID			0x01
#define QMP6989_REG_STATUS		0x02
#define QMP6989_REG_PRESSH		0x06
#define QMP6989_REG_PRESSM		0x07
#define QMP6989_REG_PRESSL		0x08
#define QMP6989_REG_TEMPH		0x09
#define QMP6989_REG_TEMPL		0x0A
#define QMP6989_REG_CMD			0x30
#define QMP6989_REG_CONFIG1		0xA5
#define QMP6989_REG_CONFIG2		0xA6
#define QMP6989_REG_CONFIG3		0xA7
#define QMP6989_REG_CONFIG4		0xA8
#define QMP6989_REG_CONFIG5		0xA9
#define QMP6989_REG_CONFIG6		0xAA
#define QMP6989_REG_CALIB00		0xAA
//Total calibration register count: AAh~BBh total 18
#define QMP6989_CALIBRATION_REGISTER_COUNT 18
//Total calibration parameter count: total 9
#define QMP6989_CALIBRATION_PARAMETER_COUNT (QMP6989_CALIBRATION_REGISTER_COUNT/2)
//Soft reset 
#define QMP6989_SW_RST_SET_VALUE		0x24

#define  SORT_NUM_Q001	(20)
#define  SORT_NUM_Q002	(170)
#define  SORT_NUM_Q003	(54)
#define  SORT_NUM_Q004	(55)
#define  QMP6989_IIC_A8_A9_AA_SUCESS	(3)


/* PID */
#define QMP6989_PID__REG		QMP6989_REG_PID
/* Soft Rest bit */
#define QMP6989_RST__REG		QMP6989_REG_RESET
#define QMP6989_RST__MSK		0x24
#define QMP6989_RST__POS		0
/* DRDY bit */
#define QMP6989_DRDY__REG	QMP6989_REG_STATUS
#define QMP6989_DRDY__MSK	0x01
#define QMP6989_DRDY__POS	0
/* P OSR bits */
#define QMP6989_P_OSR__REG       QMP6989_REG_CONFIG2
#define QMP6989_P_OSR__MSK       0x07
#define QMP6989_P_OSR__POS       0
/* T OSR bits */
#define QMP6989_T_OSR__REG       QMP6989_REG_CONFIG3
#define QMP6989_T_OSR__MSK       0x07
#define QMP6989_T_OSR__POS       0

#define QMP6989_GET_BITSLICE(regvar, bitname)	\
  ((regvar & bitname##__MSK) >> bitname##__POS)

#define QMP6989_SET_BITSLICE(regvar, bitname, val)			\
  ((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

typedef enum {
  QMP6989_P_OSR_256 = 0x04,
  QMP6989_P_OSR_512 = 0x05,
  QMP6989_P_OSR_1024 = 0x00,
  QMP6989_P_OSR_2048 = 0x01,
  QMP6989_P_OSR_4096 = 0x02,
  QMP6989_P_OSR_8192 = 0x03,
  QMP6989_P_OSR_16384 = 0x06,
  QMP6989_P_OSR_32768 = 0x07,	
} QMP6989_P_OSR_Type;

typedef enum {
  QMP6989_T_OSR_256 = 0x04,
  QMP6989_T_OSR_512 = 0x05,
  QMP6989_T_OSR_1024 = 0x00,
  QMP6989_T_OSR_2048 = 0x01,
  QMP6989_T_OSR_4096 = 0x02,
  QMP6989_T_OSR_8192 = 0x03,
  QMP6989_T_OSR_16384 = 0x06,
  QMP6989_T_OSR_32768 = 0x07,	
} QMP6989_T_OSR_Type;

typedef enum
{
	QMP6989_MODE_OFF = 0x00,	
	QMP6989_MODE_T = 0x08,	
	QMP6989_MODE_P = 0x09,
	QMP6989_MODE_TOTAL
} qmp6989_mode;

/*!
 * @brief Read multiple data from the starting regsiter address
 *
 * @param u8Addr Starting register address
 * @param pu8Data The data array of values read
 * @param u8Len Number of bytes to read
 * 
 * @return Result from the burst read function
 * @retval >= 0 Success, number of bytes read
 * @retval -127 Error null bus
 * @retval -1   Bus communication error
 *
 */
s8 qmp6989_burst_read(u8 u8Addr, u8* pu8Data, u8 u8Len);

/*!
 * @brief Write multiple data to the starting regsiter address
 *
 * @param u8Addr Starting register address
 * @param pu8Data The data array of values to write
 * @param u8Len Number of bytes to write
 * 
 * @return Result from the burst write function
 * @retval >= 0 Success, number of bytes write
 * @retval -127 Error null bus
 * @retval -1   Communication error
 *
 */
s8 qmp6989_burst_write(u8 u8Addr, u8* pu8Data, u8 u8Len);


/*!
 * @brief qmp6989 initialize communication bus
 *
 * @param pbus Pointer to the I2C/SPI read/write bus support struct
 * 
 * @return Result from bus communication function
 * @retval 0 Success
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
u8 qmp6989_get_pid(void);

/*!
 * @brief qmp6989 soft reset
 *
 * @param None
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 qmp6989_soft_reset(void);


/*!
 * @brief Get qmp6989 calibration parameters
 *        - Read calibration register AAh~BBh total 18 bytes 
 *        - Compose 9 calibration parameters from the 18 bytes
 *
 * @param fCalibParam: the calibration parameter array returned to caller
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */ 
s8 qmp6989_get_uuid(void);

s8 qmp6989_get_calibration_param(float* fCalibParam);

/*!
 * @brief Get qmp6989 calibration parameters for fixed-point compensation
 *        - Read calibration register AAh~BBh total 18 bytes
 *        - Return 9 calibration parameters with fixed-point value and power parts
 *
 * @param s16Value[]: array of the value part of the calibration parameter
 * @param u8Power[]: array of the power part of the calibration parameter
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 qmp6989_get_calibration_param_fixed_point(s16 s16Value[], u8 u8Power[]);

/*!
 * @brief qmp6989 initialization
 *        Set AAh ~ ADh to 0x00
 *
 * @param None
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 qmp6989_initialization(void);


/*!
 * @brief qmp6989 measure temperature
 *
 * @param *ps16T calibrated temperature code returned to caller
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 qmp6989_measure_T(s16* ps16T);

/*!
 * @brief qmp6989 measure pressure
 *
 * @param *ps32P raw pressure in code returned to caller
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 qmp6989_measure_P(s32* ps32P);

/*!
 * @brief qmp6989 measure pressure and temperature
 *        Read pressure first then commit pressure data conversion for the next call
 *        
 * @param *ps32P raw pressure in code returned to caller
 * @param *ps16T calibrated temperature code returned to caller
 * @param s8WaitPDrdy 1: P wait for DRDY bit set, 0: P no wait
 *
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 qmp6989_measure_P_T(s32* ps32P, s16* ps16T, s8 s8PWaitDrdy);

/*!
 * @brief qmp6989 temperature and pressure compensation
 *
 * @param s16T calibrated temperature in code
 * @param s32P raw pressure in code
 * @param fParam[] pressure calibration parameters
 * @param *pfT_Celsius calibrated temperature in Celsius returned to caller
 * @param *pfP_Pa calibrated pressure in Pa returned to caller
 * 
 * @return None
 *
 */
void qmp6989_compensation(s16 s16T, s32 s32P, float fParam[], float* pfT_Celsius, float* pfP_Pa);

/*!
 * @brief qmp6989 temperature and pressure compensation, s64 fixed point operation
 *
 * @param s16T raw temperature in code
 * @param s32P raw pressure in code
 * @param s16Value[]: array of the value part of the calibration parameter
 * @param u8Power[]: array of the power part of the calibration parameter
 * @param *ps32T_Celsius calibrated temperature in 1/256*Celsius returned to caller
 * @param *ps32P_Pa calibrated pressure in Pa returned to caller
 * 
 * @return None
 *
 */
void qmp6989_compensation_fixed_point_s64(s16 s16T, s32 s32P, s16 s16Value[], u8 u8Power[], s32* ps32T_Celsius, s32* ps32P_Pa);

/*!
 * @brief qmp6989 set pressure OSR
 *
 * @param osrP OSR to set
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 qmp6989_set_P_OSR(QMP6989_P_OSR_Type osrP);

/*!
 * @brief qmp6989 set temperature OSR
 *
 * @param osrT OSR to set
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 qmp6989_set_T_OSR(QMP6989_T_OSR_Type osrT);

void qmp6989_get_data(float *press, float *temp);

void qmp6989_set_mode(qmp6989_mode mode);
void qmp6989_get_data_ext(float *press, float *temp);
int qmp6989_test_process(void);

#endif // __QMP6989_H__
