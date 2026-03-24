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
 
/*! @file qmp6989.c
 *  @brief  QMP6989 Sensor Driver File 
 *  @author 
 */
 
#include "qmp6989.h"
#include <stdio.h>

#define QMP6989_DEBUG	printf

#define QMP6989_CALIBRATION_DATA_CNT	(16) 
#define WAIT_FOR_DRDY_LOOP_DELAY(count) {int i;for(i = 0; i < (count); ++i);}

static const s32 QMP6989_POWER_SCALE[] = {1, 10, 100, 1000};
#ifdef FLOAT_SUPPORT
static const float QMP6989_CALIB_SCALE_FACTOR[] = {
	1.0E+00,
	1.0E-05,
	1.0E-10,
	1.0E-05,
	1.0E-10,
	1.0E-15,
	1.0E-12,
	1.0E-17,
	1.0E-21};

static float fCalibParam[QMP6989_CALIBRATION_PARAMETER_COUNT], fT_Celsius, fP_Pa;//, fAlt_m;
#else
static s32 s32T_Celsius, s32P_Pa;
static s16 s16Value[QMP6989_CALIBRATION_PARAMETER_COUNT];
static u8 u8Power[QMP6989_CALIBRATION_PARAMETER_COUNT];
#endif
static s32 s32P = 0;
static s16 s16T = 25*256;
static u8 qmp6989_slave = QMP6989_7BIT_I2C_ADDR;
//static u16 qmp6989_uuid;
//static qmp6989_mode qmp6989_curr_mode = QMP6989_MODE_OFF;

// QMP6989

/*!
 * @brief Read multiple data from the starting regsiter address
 *
 * @param u8Addr Starting register address
 * @param pu8Data The data array of values read
 * @param u8Len Number of bytes to read
 * 
 * @return Result from the burst read function
 * @retval >= 0 Success
 * @retval -127 Error null bus
 * @retval < 0  Communication error
 *
 */
void qmp6989_delay1ms(int cnt) 
{
	extern void qst_delay_ms(unsigned int n_ms);

	qst_delay_ms(cnt);
}

s8 qmp6989_burst_read(u8 u8Addr, u8* pu8Data, u8 u8Len)
{
	s8 comRslt = -1;
	u8 ret = 0;

#if defined(QMP6989_USE_SPI)
	u8 i=0;
	for(i=0; i<u8Len; i++)
	{
		ret += qmp6989_spi_read(u8Addr+i, &pu8Data[i], 1);
	}
	if(ret >= 1)
		comRslt = 1;
	else
		comRslt = 0;
#else
	ret = bsp_read_reg(qmp6989_slave, u8Addr, pu8Data, u8Len);
	if(ret != 1)
		comRslt = -1;
	else
		comRslt = 1;

#endif
    return comRslt;
}

 

/*!
 * @brief Write multiple data to the starting regsiter address
 *
 * @param u8Addr Starting register address
 * @param pu8Data The data array of values to write
 * @param u8Len Number of bytes to write
 * 
 * @return Result from the burst write function
 * @retval >= 0 Success
 * @retval -127 Error null bus
 * @retval < 0   Communication error
 *
 */
s8 qmp6989_burst_write(u8 u8Addr, u8* pu8Data, u8 u8Len)
{
	s8 comRslt = -1;
	u8 ret = 0;
	u8 i=0;

#if defined(QMP6989_USE_SPI)


	for(i=0; i<u8Len; i++)
	{
		ret = qmp6989_spi_write(u8Addr+i, &pu8Data[i], 1);   
	}	
	if(ret >= 1)
		comRslt = 1;
	else
		comRslt = 0;
#else
	for(i=0; i<u8Len; i++)
	{
		ret = bsp_write_reg(qmp6989_slave, u8Addr+i, pu8Data[i]);
	}
	if(ret != 1)
		comRslt = -1;
	else
		comRslt = 1;
#endif

  	return comRslt;	
}


/*!
 * @brief QMP6989 initialize communication bus
 *
 * @param pbus Pointer to the I2C/SPI read/write bus support struct
 * 
 * @return Result from bus communication function
 * @retval 0 Success
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
u8 qmp6989_get_pid(void){
	s8 comRslt = -1;
	u8 u8Data = 0;
	u8 index = 0;
	u8 slave[2] = {0x6c, 0x6d};

	// 100ms
	//  qmp6989_soft_reset();
	qmp6989_delay1ms(100);
#if defined(QMP6989_USE_SPI)
	u8Data = 0x81;
	comRslt = qmp6989_burst_write(QMP6989_RST__REG, &u8Data, 1);
	qmp6989_delay1ms(50);  // 10ms
#endif

	for(index = 0; index < sizeof(slave)/sizeof(slave[0]); index++)
	{
		u8Data = 0x00;
		qmp6989_slave = slave[index];
		comRslt = qmp6989_burst_read(QMP6989_REG_PID, &u8Data, 1);
		if(u8Data == 0x02)
		{
			QMP6989_DEBUG("qmp6989_get_pid OK: %d slave0x%x id:0x%x\n", comRslt, slave[index], u8Data);
			break;
		}
		else
		{	  	
			QMP6989_DEBUG("qmp6989_get_pid FAIL: %d slave0x%x id:0x%x\n", comRslt, slave[index], u8Data);
		}
	}
//  qmp6989_get_uuid();
	
  return u8Data;
}

 
/*!
 * @brief QMP6989 soft reset
 *
 * @param None
 *
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 qmp6989_soft_reset(void)
{

	s8 comRslt = -1;
	u8 u8Data = QMP6989_SW_RST_SET_VALUE;

	// Set 00h = 0x24
	comRslt = qmp6989_burst_write(QMP6989_RST__REG, &u8Data, 1);

	//默认?��?，�??�断返�?�?
	return comRslt;
}

s8 qmp6989_get_uuid(void)
{

	s8 comRslt = -1;
	u8 u8Data[2];

	// Set 00h = 0x24
	//comRslt = qmp6989_burst_write(QMP6989_REG_CONFIG4, u8Data, 2);
	
	comRslt = qmp6989_burst_read(QMP6989_REG_CONFIG4, u8Data, 2);
	QMP6989_DEBUG("qmp6989_get_uuid 0x%x 0x%x\n", u8Data[0], u8Data[1]);
	qmp6989_uuid = (u16)(u8Data[1] << 8) | u8Data[0];

	//默认?��?，�??�断返�?�?
	return comRslt;
}

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
#ifdef FLOAT_SUPPORT
s8 qmp6989_get_calibration_param(float *fCalibParam)
{

	u8 u8DataBuf[QMP6989_CALIBRATION_REGISTER_COUNT];
	s8 comRslt;
	s32 tmp, shift, i;

	// read the calibration registers
	comRslt = qmp6989_burst_read(QMP6989_REG_CALIB00, u8DataBuf, QMP6989_CALIBRATION_REGISTER_COUNT);

	if (comRslt < 0)
	{
		comRslt = -1;
		goto EXIT;
	}

	// Get the parameters
	shift = sizeof(s32) * 8 - 16;
	for (i = 0; i < QMP6989_CALIBRATION_PARAMETER_COUNT; ++i)
	{
		tmp = ((s32)u8DataBuf[2 * i] << 8) + u8DataBuf[2 * i + 1];
		fCalibParam[i] = ((tmp << shift) >> (shift + 2)) * QMP6989_POWER_SCALE[(u8DataBuf[2 * i + 1] & 0x03)] * QMP6989_CALIB_SCALE_FACTOR[i];
		// QMP6989_DEBUG("fCalibParam[%d]=%f\n",i,fCalibParam[i]);
	}

EXIT:
	return comRslt;
}

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
#else
s8 qmp6989_get_calibration_param_fixed_point(s16 s16Value[], u8 u8Power[])
{

	u8 u8DataBuf[QMP6989_CALIBRATION_REGISTER_COUNT];
	s8 comRslt;
	s16 tmp, i;

	// read the calibration registers
	comRslt = qmp6989_burst_read(QMP6989_REG_CALIB00, u8DataBuf, QMP6989_CALIBRATION_REGISTER_COUNT);

	if (comRslt < 0)
	{
		comRslt = -1;
		goto EXIT;
	}

	for (i = 0; i < QMP6989_CALIBRATION_PARAMETER_COUNT; ++i)
	{
		tmp = ((s16)u8DataBuf[2 * i] << 8) + u8DataBuf[2 * i + 1];
		s16Value[i] = (tmp >> 2);
		u8Power[i] = (tmp & 0x03);
	}

EXIT:
	return comRslt;
}
#endif

s8 qmp6989_get_sensor_uuid(u8 *reg_A8, u8 *reg_A9, u8 *reg_AA)
{

	s8 comRslt = 0, s8Tmp;
	u8 u8Data;

	// Read A8h,��ȡ�ɹ���s8Tmp = 1;ʧ��Ϊ-1;
	s8Tmp = qmp6989_burst_read(QMP6989_REG_CONFIG4, &u8Data, 1);

	if (s8Tmp <= 0)
	{ // communication error
		printf("qmp6989 iic fail, reg = a8\n");
		comRslt = s8Tmp;
		goto EXIT;
	}

	*reg_A8 = u8Data;

	comRslt += s8Tmp;

	// Read A9h.��ȡ�ɹ���s8Tmp = 1;ʧ��Ϊ-1
	s8Tmp = qmp6989_burst_read(QMP6989_REG_CONFIG5, &u8Data, 1);

	if (s8Tmp <= 0)
	{ // communication error

		printf("qmp6989 iic fail, reg =  a9\n");
		comRslt = s8Tmp;
		goto EXIT;
	}

	*reg_A9 = u8Data;

	comRslt += s8Tmp;

	// Read AAh.��ȡ�ɹ���s8Tmp = 1;ʧ��Ϊ-1
	s8Tmp = qmp6989_burst_read(QMP6989_REG_CONFIG6, &u8Data, 1);

	if (s8Tmp <= 0)
	{ // communication error

		printf("qmp6989 iic fail, reg =  a9\n");
		comRslt = s8Tmp;
		goto EXIT;
	}

	*reg_AA = u8Data;

	comRslt += s8Tmp;
EXIT:
	return comRslt;
}

// int qmp6989_test_process(void)
//{
//
//	u8 reg_A8,reg_A9,reg_AA,ret,i,find_a8a9 = 0;
//	//
//	ret = qmp6989_get_sensor_uuid(&reg_A8,&reg_A9,&reg_AA);
//
//	if(QMP6989_IIC_A8_A9_AA_SUCESS == ret)
//	{
//		printf("sensor regA8 = 0x%x\n",reg_A8);
//		printf("sensor regA9 = 0x%x\n",reg_A9);
//		printf("sensor regAA = 0x%x\n",reg_AA);
//
//
//		for(i = 0;i<SORT_NUM_Q001;i++)
//		{
//			if((reg_A8 == uuid_A8_Q001[i])&&(reg_A9 == uuid_A9_Q001[i])&&(reg_AA == uuid_AA_Q001[i]))
//			{
//				printf("���sensor Q9H001 �����⣬��ɸѡ������лл��");
//				find_a8a9++;
//				break;
//			}
//		}
//
//
//		for(i = 0;i<SORT_NUM_Q002;i++)
//		{
//			if((reg_A8 == uuid_A8_Q002[i])&&(reg_A9 == uuid_A9_Q002[i])&&(reg_AA == uuid_AA_Q002[i]))
//			{
//				printf("���sensor Q9H002 �����⣬��ɸѡ������лл��");
//				find_a8a9++;
//				break;
//			}
//		}
//
//
//		for(i = 0;i<SORT_NUM_Q003;i++)
//		{
//			if((reg_A8 == uuid_A8_Q003[i])&&(reg_A9 == uuid_A9_Q003[i])&&(reg_AA == uuid_AA_Q003[i]))
//			{
//				printf("���sensor Q9H003 �����⣬��ɸѡ������лл��");
//				find_a8a9++;
//				break;
//			}
//		}
//
//
//		for(i = 0;i<SORT_NUM_Q004;i++)
//		{
//			if((reg_A8 == uuid_A8_Q004[i])&&(reg_A9 == uuid_A9_Q004[i])&&(reg_A9 == uuid_AA_Q004[i]))
//			{
//				printf("���sensor Q9H004 �����⣬��ɸѡ������лл��");
//				find_a8a9++;
//				break;
//			}
//		}
//
//		if(0 == find_a8a9)
//		{
//			printf("���� Q9H 001-002-003-004 ����?��");
//
//		}
//
//	}
//
//	return ret;
// }

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
s8 qmp6989_initialization(void)
{
	s8 comRslt = 0, s8Tmp;
	u8 u8Data[] = {0, 0, 0, 0};

	qmp6989_soft_reset();
	qmp6989_delay1ms(100); // 100ms
	qmp6989_get_uuid();

#ifdef FLOAT_SUPPORT
	qmp6989_get_calibration_param(fCalibParam);
#else
	qmp6989_get_calibration_param_fixed_point(s16Value, u8Power);
#endif
	/* QMP6989 initialization setup */
	// Set AAh ~ AD to 0x00
	s8Tmp = qmp6989_burst_write(QMP6989_REG_CALIB00, u8Data, 4);

	if (s8Tmp < 0)
	{ // communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;

	qmp6989_set_P_OSR(QMP6989_P_OSR_4096);
	qmp6989_set_T_OSR(QMP6989_T_OSR_4096);
	// after init read sensor data once
	// qmp6989_measure_P(&s32P);
//	qmp6989_measure_T(&s16T);
//	qmp6989_set_mode(QMP6989_MODE_P);
EXIT:
	return comRslt;
}

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
s8 qmp6989_measure_T(s16 *ps16T)
{

	s8 comRslt = 0, s8Tmp, s8timecnt = 0;
	u8 u8Data[2];

	// Set A5h = 0x00, Calibrated data out
	u8Data[0] = 0x00;
	s8Tmp = qmp6989_burst_write(QMP6989_REG_CONFIG1, u8Data, 1);

	if (s8Tmp < 0)
	{ // communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;

	// Set 30h = 0x08, T-Forced mode
	u8Data[0] = 0x08;
	s8Tmp = qmp6989_burst_write(QMP6989_REG_CMD, u8Data, 1);

	if (s8Tmp < 0)
	{ // communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;

	// Wait for 02h[0] DRDY bit set
	do
	{

		// wait a while
		qmp6989_delay1ms(1);

		s8Tmp = qmp6989_burst_read(QMP6989_REG_STATUS, u8Data, 1);

		if (s8Tmp < 0)
		{ // communication error
			comRslt = s8Tmp;
			goto EXIT;
		}
		comRslt += s8Tmp;

	} while(( QMP6989_GET_BITSLICE(u8Data[0], QMP6989_DRDY) != 1)&&(s8timecnt++ < 100));
	//} while (QMP6989_GET_BITSLICE(u8Data[0], QMP6989_DRDY) != 1);

	// Read 09h~0Ah
	s8Tmp = qmp6989_burst_read(QMP6989_REG_TEMPH, u8Data, 2);

	if (s8Tmp < 0)
	{ // communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;

	// Get the calibrated temperature in code
	*ps16T = (s16)(((u16)u8Data[0] << 8) + u8Data[1]);
	// QMP6989_DEBUG("qmp6989_measure_T, u8Data[0]= 0x%x,u8Data[1]= 0x%x\n",u8Data[0],u8Data[1]);
EXIT:
	return comRslt;
}

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
s8 qmp6989_measure_P(s32 *ps32P)
{

	s8 comRslt = 0, s8Tmp, s8timecnt = 0;
	u8 u8Data[3];

	// Set A5h = 0x02, raw data out
	u8Data[0] = 0x02;
	s8Tmp = qmp6989_burst_write(QMP6989_REG_CONFIG1, u8Data, 1);

	if (s8Tmp < 0)
	{ // communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;

	// Set 30h = 0x09, P-Forced mode
	u8Data[0] = 0x09;
	s8Tmp = qmp6989_burst_write(QMP6989_REG_CMD, u8Data, 1);

	if (s8Tmp < 0)
	{ // communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;

	// Wait for 02h[0] DRDY bit set
	do
	{

		// wait a while
		qmp6989_delay1ms(1);

		s8Tmp = qmp6989_burst_read(QMP6989_REG_STATUS, u8Data, 1);

		if (s8Tmp < 0)
		{ // communication error
			comRslt = s8Tmp;
			goto EXIT;
		}
		comRslt += s8Tmp;

	 } while(( QMP6989_GET_BITSLICE(u8Data[0], QMP6989_DRDY) != 1)&&(s8timecnt++ < 100));
	//} while (QMP6989_GET_BITSLICE(u8Data[0], QMP6989_DRDY) != 1);

	// Read 06h~08h
	s8Tmp = qmp6989_burst_read(QMP6989_REG_PRESSH, u8Data, 3);

	if (s8Tmp < 0)
	{ // communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;

	s8Tmp = sizeof(*ps32P) * 8 - 24;
	// Get the raw pressure in code
	*ps32P = (((u32)u8Data[0] << 16) + ((s32)u8Data[1] << 8) + u8Data[2]);
	*ps32P = (*ps32P << s8Tmp) >> s8Tmp; // 24 bit sign extension

EXIT:
	return comRslt;
}

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

#if 0
s8 qmp6989_measure_P_T(s32 *ps32P, s16 *ps16T, s8 s8PWaitDrdy)
{

	s8 comRslt = 0, s8Tmp, s8timecnt = 0;
	u8 u8Data[3];

	/*
	 *
	 * Read raw P code
	 *
	 */
	if (s8PWaitDrdy)
	{
		// Wait for 02h[0] DRDY bit set if s8PWaitDrdy is 1
		do
		{

			// wait a while
			qmp6989_delay1ms(1);

			s8Tmp = qmp6989_burst_read(QMP6989_REG_STATUS, u8Data, 1);

			if (s8Tmp < 0)
			{ // communication error
				comRslt = s8Tmp;
				goto EXIT;
			}
			comRslt += s8Tmp;

		} while ((QMP6989_GET_BITSLICE(u8Data[0], QMP6989_DRDY) != 1) && (s8timecnt++ < 100));
	}

	// Read 06h~08h
	s8Tmp = qmp6989_burst_read(QMP6989_REG_PRESSH, u8Data, 3);

	if (s8Tmp < 0)
	{ // communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;

	s8Tmp = sizeof(*ps32P) * 8 - 24;
	// Get the raw pressure in code
	*ps32P = ((s32)u8Data[0] << 16) + ((s32)u8Data[1] << 8) + u8Data[2];
	*ps32P = (*ps32P << s8Tmp) >> s8Tmp; // 24 bit sign extension

	/*
	 *
	 * Measure calibrated T code
	 *
	 */
	// Set A5h = 0x00, Calibrated data out
	u8Data[0] = 0x00;
	s8Tmp = qmp6989_burst_write(QMP6989_REG_CONFIG1, u8Data, 1);

	if (s8Tmp < 0)
	{ // communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;

	// Set 30h = 0x08, T-Forced mode
	u8Data[0] = 0x08;
	s8Tmp = qmp6989_burst_write(QMP6989_REG_CMD, u8Data, 1);

	if (s8Tmp < 0)
	{ // communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;
	s8timecnt = 0;
	// Wait for 02h[0] DRDY bit set
	do
	{

		// wait a while
		WAIT_FOR_DRDY_LOOP_DELAY(1000)

		s8Tmp = qmp6989_burst_read(QMP6989_REG_STATUS, u8Data, 1);

		if (s8Tmp < 0)
		{ // communication error
			comRslt = s8Tmp;
			goto EXIT;
		}
		comRslt += s8Tmp;

	} while ((QMP6989_GET_BITSLICE(u8Data[0], QMP6989_DRDY) != 1) && (s8timecnt++ < 100));

	// Read 09h~0Ah
	s8Tmp = qmp6989_burst_read(QMP6989_REG_TEMPH, u8Data, 2);

	if (s8Tmp < 0)
	{ // communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;

	// Get the calibrated temperature in code
	*ps16T = (u8Data[0] << 8) + u8Data[1];

	/*
	 *
	 * Commit the next pressure conversion
	 *
	 */
	// Set A5h = 0x02, raw data out
	u8Data[0] = 0x02;
	s8Tmp = qmp6989_burst_write(QMP6989_REG_CONFIG1, u8Data, 1);

	if (s8Tmp < 0)
	{ // communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;

	// Set 30h = 0x09, P-Forced mode
	u8Data[0] = 0x09;
	s8Tmp = qmp6989_burst_write(QMP6989_REG_CMD, u8Data, 1);

	if (s8Tmp < 0)
	{ // communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;

EXIT:
	return comRslt;
}
#endif
float qmp6989_calibration_process(float pfS_Pa, float pfP_Pa)
{
	static float pfo_Pa = 0.0f;
	static u8 u8Cnt = 0;

	if (u8Cnt < QMP6989_CALIBRATION_DATA_CNT)
	{
		pfo_Pa += pfP_Pa;
		u8Cnt++;
	}
	else
	{
		pfo_Pa = (pfo_Pa / u8Cnt - pfS_Pa);
	}

	return pfo_Pa;
}
/*!
 * @brief qmp6989 temperature and pressure compensation
 *
 * @param s16T calibrated temperature in code
 * @param s32P raw pressure in code
 * @param fParam[] pressure calibration parameters
 * @param *pfT_Celsius calibrated temperature in Celsius returned to caller
 * @param *pfP_Pa calibraated pressure in Pa returned to caller
 *
 * @return None
 *
 */
#ifdef FLOAT_SUPPORT
void qmp6989_compensation(s16 s16T, s32 s32P, float fParam[], float *pfT_Celsius, float *pfP_Pa)
{

	*pfT_Celsius = QMP6989_T_CODE_TO_CELSIUS(s16T);

	*pfP_Pa =
		fParam[0] +
		fParam[1] * s16T +
		fParam[2] * s16T * s16T +
		fParam[3] * s32P +
		fParam[4] * s16T * s32P +
		fParam[5] * s16T * s16T * s32P +
		fParam[6] * s32P * s32P +
		fParam[7] * s16T * s32P * s32P +
		fParam[8] * s16T * s16T * s32P * s32P;
}
#else
#define ShiftRight(v, s) (((v) + (1 << ((s)-1))) >> (s))
#define RoundDivide(v, d) (((v) + ((d) / 2)) / (d))

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
void qmp6989_compensation_fixed_point_s64(s16 s16T, s32 s32P, s16 s16Value[], u8 u8Power[], s32 *ps32T_Celsius, s32 *ps32P_Pa)
{

	s64 tmp, val, s64T, s64P;
	s64T = s16T;
	s64P = s32P;

	// Temperature
	*ps32T_Celsius = s16T;

	// Pressure
	val = 0;
	// beta0
	tmp = s16Value[0] * QMP6989_POWER_SCALE[u8Power[0]] * 10;
	val += tmp;
	// beta1*T
	tmp = s64T * s16Value[1];
	tmp = tmp * QMP6989_POWER_SCALE[u8Power[1]];
	tmp = RoundDivide(tmp, 10000);
	val += tmp;
	// beta2*T*T
	tmp = s64T * s16Value[2];
	tmp = tmp * s64T;
	tmp = tmp * QMP6989_POWER_SCALE[u8Power[2]];
	tmp = RoundDivide(tmp, 1000000000);
	val += tmp;
	// beta3*P
	tmp = s64P * s16Value[3];
	tmp = tmp * QMP6989_POWER_SCALE[u8Power[3]];
	tmp = RoundDivide(tmp, 10000);
	val += tmp;
	// beta4*P*T
	tmp = s64P * s16Value[4];
	tmp = tmp * s64T;
	tmp = tmp * QMP6989_POWER_SCALE[u8Power[4]];
	tmp = RoundDivide(tmp, 1000000000);
	val += tmp;
	// beta5*P*T*T
	tmp = s64P * s16Value[5];
	tmp = tmp * s64T;
	tmp = ShiftRight(tmp, 10) * s64T;
	tmp = ShiftRight(tmp, 10) * QMP6989_POWER_SCALE[u8Power[5]];
	tmp = RoundDivide(tmp, 95367432);
	val += tmp;
	// beta6*P*P
	tmp = s64P * s16Value[6];
	tmp = tmp * s64P;
	tmp = ShiftRight(tmp, 7) * QMP6989_POWER_SCALE[u8Power[6]];
	tmp = RoundDivide(tmp, 781250000);
	val += tmp;
	// beta7*P*P*T
	tmp = s64P * s16Value[7];
	tmp = tmp * s64P;
	tmp = ShiftRight(tmp, 10) * s64T;
	tmp = ShiftRight(tmp, 10) * QMP6989_POWER_SCALE[u8Power[7]];
	tmp = RoundDivide(tmp, 9536743164);
	val += tmp;
	// beta8*P*P*T*T
	tmp = s64P * s16Value[8];
	tmp = tmp * s64P;
	tmp = ShiftRight(tmp, 9) * ShiftRight(s64T, 1);
	tmp = ShiftRight(tmp, 12) * ShiftRight(s64T, 3);
	tmp = ShiftRight(tmp, 7) * QMP6989_POWER_SCALE[u8Power[8]];
	tmp = RoundDivide(tmp, 23283064365);
	val += tmp;

	*ps32P_Pa = (s32)RoundDivide(val, 10);

	return;
}
#endif

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
s8 qmp6989_set_P_OSR(QMP6989_P_OSR_Type osrP)
{

	s8 comRslt = 0, s8Tmp;
	u8 u8Data;

	// Read A6h
	s8Tmp = qmp6989_burst_read(QMP6989_REG_CONFIG2, &u8Data, 1);

	if (s8Tmp < 0)
	{ // communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;

	// Set the A6h[2:0] OSR bits
	u8Data = QMP6989_SET_BITSLICE(u8Data, QMP6989_P_OSR, osrP);
	s8Tmp = qmp6989_burst_write(QMP6989_REG_CONFIG2, &u8Data, 1);

	if (s8Tmp < 0)
	{ // communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;

EXIT:
	return comRslt;
}

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
s8 qmp6989_set_T_OSR(QMP6989_T_OSR_Type osrT)
{

	s8 comRslt = 0, s8Tmp;
	u8 u8Data;

	// Read A7h
	s8Tmp = qmp6989_burst_read(QMP6989_REG_CONFIG3, &u8Data, 1);

	if (s8Tmp < 0)
	{ // communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;

	// Set the A7h[2:0] OSR bits
	u8Data = QMP6989_SET_BITSLICE(u8Data, QMP6989_T_OSR, osrT);
	s8Tmp = qmp6989_burst_write(QMP6989_REG_CONFIG3, &u8Data, 1);

	if (s8Tmp < 0)
	{ // communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;

EXIT:
	return comRslt;
}

void qmp6989_get_data(float *press, float *temp)
{
	qmp6989_measure_P(&s32P);
	qmp6989_measure_T(&s16T);

#ifdef FLOAT_SUPPORT
	qmp6989_compensation(s16T, s32P, fCalibParam, &fT_Celsius, &fP_Pa);
	*press = fP_Pa;
	*temp = fT_Celsius;
#else
	qmp6989_compensation_fixed_point_s64(s16T, s32P, s16Value, u8Power, &s32T_Celsius, &s32P_Pa);
	*press = s32P_Pa;
	*temp = s32T_Celsius / 256.0;
#endif

	printf("qmp6989_get_data s32P = %d ,s16T = %d  \n", s32P, s16T);
	printf("qmp6989_get_data press = %d ,temp = %d  \n", *press, *temp);
}

// static qmp6989_sensor qmp6989_curr_sensor = 0;
#if 0
void qmp6989_set_mode(qmp6989_mode mode)
{
	u8 u8Data[3];
	s8 ret = 0;

	printf("qmp6989_set_mode %d \n", mode);

	qmp6989_curr_mode = mode;
	if (mode == QMP6989_MODE_P)
	{
		u8Data[0] = 0x02; // Set A5h = 0x02, raw data out
	}
	else if (mode == QMP6989_MODE_T)
	{
		u8Data[0] = 0x00; // Set A5h = 0x00, Calibrated data out
	}
	qmp6989_burst_write(QMP6989_REG_CONFIG1, u8Data, 1);

	// Set 30h = 0x09, P-Forced mode
	// Set 30h = 0x08, T-Forced mode
	u8Data[0] = (u8)mode;
	qmp6989_burst_write(QMP6989_REG_CMD, u8Data, 1);
}

void qmp6989_get_data_ext(float *press, float *temp)
{
	static u8 qmp6989_data_i = 0, press_temp_com = 0;

	u8 u8Data[3] = {0x00, 0x00, 0x00};
	u8 s8timecnt = 0, s8Tmp;
	s8 ret;

	printf("qmp6989_get_data_ext s32P = %d s16T = %d  \n", s32P, s16T);

	qmp6989_burst_read(QMP6989_REG_STATUS, u8Data, 1);
	while (!(u8Data[0] & 0x01) && (s8timecnt++ < 100))
	{
		qmp6989_delay1ms(1); //确�?>=1ms
		qmp6989_burst_read(QMP6989_REG_STATUS, u8Data, 1);
	}

	if (u8Data[0] & 0x01)
	{
		if (qmp6989_curr_mode == QMP6989_MODE_P)
		{
			qmp6989_burst_read(QMP6989_REG_PRESSH, u8Data, 3);

#if 1
			s8Tmp = sizeof(s32) * 8 - 24;
			// Get the raw pressure in code
			s32P = (u8Data[0] << 16) + (u8Data[1] << 8) + u8Data[2];
			s32P = (s32P << s8Tmp) >> s8Tmp; // 24 bit sign extension
#else
			s32P = (s32)(((s32)u8Data[0] << 24) + ((s32)u8Data[1] << 16) + ((s32)u8Data[2] << 8));
			s32P = s32P >> 8;
#endif

			press_temp_com = 1;
		}
		else if (qmp6989_curr_mode == QMP6989_MODE_T)
		{
			// Read 09h~0Ah
			qmp6989_burst_read(QMP6989_REG_TEMPH, u8Data, 2);

			// Get the calibrated temperature in code
			s16T = (s16)(((u16)u8Data[0] << 8) + u8Data[1]);
			press_temp_com = 2;
		}

		if ((++qmp6989_data_i) >= 5)
		{
			qmp6989_data_i = 0;
			qmp6989_set_mode(QMP6989_MODE_T);
		}
		else
		{
			qmp6989_set_mode(QMP6989_MODE_P);
		}
	}
	else
	{
		qmp6989_set_mode(QMP6989_MODE_P);
	}

	if (2 == press_temp_com)
	{
		printf("s32P = %d s16T = %d  \n", s32P, s16T);
		press_temp_com = 0;
#ifdef FLOAT_SUPPORT
		qmp6989_compensation(s16T, s32P, fCalibParam, &fT_Celsius, &fP_Pa);
		*press = fP_Pa;
		*temp = fT_Celsius;
#else
		qmp6989_compensation_fixed_point_s64(s16T, s32P, s16Value, u8Power, &s32T_Celsius, &s32P_Pa);
		*press = s32P_Pa;
		*temp = s32T_Celsius / 256.0;
#endif
	}
}
#endif

