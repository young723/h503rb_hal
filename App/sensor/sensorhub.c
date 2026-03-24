
#include <math.h>
#include "sensorhub.h"

#define QST_BUF_LEN		768
static unsigned char g_fifo[QST_BUF_LEN];

static evb_sensor_timer_t tim_array[] = {{TIM2, 0}, {TIM3, 0}};
static evb_sensor_irq_t irq_array[] = {{QST_EVB_INT1, 0}, {QST_EVB_INT2, 0}};
static evb_sensor_info_t qhub_info[] = 
{
	{QST_SENSOR_ID_NONE, QST_REPORT_OFF, NULL,NULL, 100, 1, {{0.0f, 0.0f, 0.0f}, 0, 0, 0}},			// QST_SENSOR_ACCEL
	{QST_SENSOR_ID_NONE, QST_REPORT_OFF, NULL,NULL, 100, 1, {{0.0f, 0.0f, 0.0f}, 0, 0, 0}},			// QST_SENSOR_MAG
	{QST_SENSOR_ID_NONE, QST_REPORT_OFF, NULL,NULL, 100, 1, {{0.0f, 0.0f, 0.0f}, 0, 0, 0}},			// QST_SENSOR_IMU
	{QST_SENSOR_ID_NONE, QST_REPORT_OFF, NULL,NULL, 100, 1, {{0.0f, 0.0f, 0.0f}, 0, 0, 0}},			// QST_SENSOR_PRESS	
	{QST_SENSOR_ID_NONE, QST_REPORT_OFF, NULL,NULL, 100, 1, {{0.0f, 0.0f, 0.0f}, 0, 0, 0}}			// QST_SENSOR_OTHER
};
static evb_sensor_cfg_t qsensor_cfg;

unsigned char *qst_evb_get_fifo_buf(void)
{
	return g_fifo;
}

static void qst_evb_show_data2(evb_sensor_data_t *d)
{
#if EVB_SENSOR_LOG == 1
	qst_logi("%.2f,%.2f,D-%d\r\n", d->val[0], d->val[1], d->num);
#elif EVB_SENSOR_LOG == 2
	qst_logi("$%.2f %.2f;", d->val[0], d->val[1]);
#elif EVB_SENSOR_LOG == 3
	uart_ano_send_senosr_data(qhub_info[0].out.val, qhub_info[2].out.val, qhub_info[1].out.val, qhub_info[3].out.val[0]);
#endif
}

static void qst_evb_show_data3(evb_sensor_data_t *d)
{
#if EVB_SENSOR_LOG == 1
	qst_logi("%.2f,%.2f,%.2f,D-%d\r\n", d->val[0], d->val[1], d->val[2], d->num);
#elif EVB_SENSOR_LOG == 2
	qst_logi("$%.2f %.2f %.2f;", d->val[0], d->val[1], d->val[2]);
#elif EVB_SENSOR_LOG == 3
	uart_ano_send_senosr_data(qhub_info[0].out.val, qhub_info[2].out.val, qhub_info[1].out.val, qhub_info[3].out.val[0]);
#endif
}

static void qst_evb_show_data6(evb_sensor_data_t *d1, evb_sensor_data_t *d2, float temp)
{
#if EVB_SENSOR_LOG == 1
	qst_logi("%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.2f,D-%d\r\n", d1->val[0],d1->val[1],d1->val[2],d2->val[0],d2->val[1],d2->val[2],temp,d2->num);
#elif EVB_SENSOR_LOG == 2
	qst_logi("$%.2f %.2f %.2f %.4f %.4f %.4f;", d1->val[0],d1->val[1],d1->val[2],d1->val[0],d1->val[1],d1->val[2]);
#elif EVB_SENSOR_LOG == 3
	uart_ano_send_senosr_data(qhub_info[0].out.val, qhub_info[2].out.val, qhub_info[1].out.val, qhub_info[3].out.val[0]);
#endif
}

#if defined(QST_EVB_NOMOTION)
static void qst_evb_check_nomotion(float acc[3])
{
	static QstStd3	a_std;
	static unsigned short m_static_num = 0;
	float std_out[3];

	getStandardDeviation3(&a_std, acc, std_out);
	if((std_out[0] < 0.1f)&&(std_out[1] < 0.1f)&&(std_out[2] < 0.1f))
	{
		if(m_static_num++ > (500))
		{
			qst_logi("evb_sleep_handle entry\r\n");
			m_static_num = 0;

#if defined(QMI8658_USE_FIFO)
			if(qhub_info[2].report & QST_REPORT_FIFO)
			{
				qmi8658_config_fifo(0, qmi8658_Fifo_64, qmi8658_Fifo_Stream, qmi8658_Int1, QMI8658_ACCGYR_ENABLE);
			}
#endif
#if defined(QMI8658_USE_AMD)
			if(qhub_info[2].id == QST_IMU_QMI8658)
			{
				qmi8658_enable_amd(1, qmi8658_Int1, 0);
			}
#endif

			SysTick_Enable(0);
#if defined(STM32F401xx)
			PWR_EnterSTOPMode(PWR_LowPowerRegulator_ON, PWR_STOPEntry_WFI);
#endif
			SYSCLK_Config_WakeUp();
			SysTick_Init(1);
			qst_logi("evb_sleep_handle exit\r\n");

			if(qhub_info[2].id == QST_IMU_QMI8658)
			{
#if defined(QMI8658_USE_AMD)
				qmi8658_enable_amd(0, qmi8658_Int1, 0);
#endif
				qmi8658_enableSensors(QMI8658_DISABLE_ALL);	
				qst_delay_ms(2);
				qmi8658_config_reg(0);
				qmi8658_enableSensors(QMI8658_ACCGYR_ENABLE);
				qst_delay_ms(10);
#if defined(QMI8658_USE_FIFO)
				if(qhub_info[2].report & QST_REPORT_FIFO)
				{
					qmi8658_config_fifo(qhub_info[2].wmk, qmi8658_Fifo_64, qmi8658_Fifo_Stream, qmi8658_Int1, QMI8658_ACCGYR_ENABLE);
				}
#endif
			}
		}
	}
	else
	{
		m_static_num = 0;
	}
}
#endif

static void qst_evb_accel_data(void)
{
	if(qhub_info[0].id == QST_ACCEL_QMA6100P)
		qma6100p_read_acc_xyz(&qhub_info[0].out.val[0]);
#if defined(QMAX981_SUPPORT)
	else if(qhub_info[0].id == QST_ACCEL_QMAX981)
		qmaX981_read_xyz(&qhub_info[0].out.val[0]);
#endif
#if defined(QMA6101T_SUPPORT)
	else if(qhub_info[0].id == QST_ACCEL_QMA6101T)
		qma6101t_read_acc_xyz(&qhub_info[0].out.val[0]);
#endif

	qhub_info[0].out.num++;
	qhub_info[0].out.timestamp += qhub_info[0].period;

#if defined(QST_FUSION)
	qst_algo_set_data(QST_SENSOR_ACCEL, &qhub_info[0].out);
#else
	qst_evb_show_data3(&qhub_info[0].out);
#endif
#if defined(QST_EVB_NOMOTION)
	qst_evb_check_nomotion(qhub_info[0].out.val);
#endif
}

static void qst_evb_accel_fifo_data(void)
{
	if(qhub_info[0].id == QST_ACCEL_QMA6100P)
	{
#if defined(QMA6100P_FIFO_FUNC)
		int fifo_level = qma6100p_read_fifo(g_fifo);
		short raw[3];
		for(int i = 0; i<fifo_level; i++)
		{
			raw[0] = (short)((g_fifo[i*6+1]<<8) | g_fifo[i*6+0]);
			raw[1] = (short)((g_fifo[i*6+3]<<8) | g_fifo[i*6+2]);
			raw[2] = (short)((g_fifo[i*6+5]<<8) | g_fifo[i*6+4]);
			raw[0] = raw[0]>>2;
			raw[1] = raw[1]>>2;
			raw[2] = raw[2]>>2;
			qhub_info[0].out.val[0] = (raw[0]*9.80665f)/1024;
			qhub_info[0].out.val[1] = (raw[1]*9.80665f)/1024;
			qhub_info[0].out.val[2] = (raw[2]*9.80665f)/1024;
			qst_algo_set_data(QST_SENSOR_ACCEL, &qhub_info[0].out);
			qst_evb_show_data3(&qhub_info[0].out);
		}

#endif
	}
}

static void qst_evb_accel_irq(void)
{
	if(qhub_info[0].id == QST_ACCEL_QMA6100P)
		qma6100p_irq_hdlr();
}

static void qst_evb_mag_data(void)
{
	if(qhub_info[1].id == QST_MAG_QMC6308)
		qmc6308_read_mag_xyz(&qhub_info[1].out.val[0]);
#if defined(QMC5883P_SUPPORT)
	else if(qhub_info[1].id == QST_MAG_QMC5883P)
		qmc5883p_read_mag_xyz(&qhub_info[1].out.val[0]);
#endif
	else if(qhub_info[1].id == QST_MAG_QMC6309)
		qmc6309_read_mag_xyz(&qhub_info[1].out.val[0]);
	else if(qhub_info[1].id == QST_MAG_QMC6309V)
		qmc6309v_read_mag_xyz(&qhub_info[1].out.val[0]);
	else if(qhub_info[1].id == QST_MAG_QMC6G00X)
		qmc6g00x_read_mag_xyz(&qhub_info[1].out.val[0]);
#if defined(QMC5883L_SUPPORT)
	else if(qhub_info[1].id == QST_MAG_QMC5883L)
		qmc5883l_GetData(&qhub_info[1].out.val[0]);
#endif

	qhub_info[1].out.num++;
	qhub_info[1].out.timestamp += qhub_info[1].period;

#if defined(QST_FUSION)
	qst_algo_set_data(QST_SENSOR_MAG, &qhub_info[1].out);
#else
	qst_evb_show_data3(&qhub_info[1].out);
#endif
}

static void qst_evb_mag_fifo_data(void)
{
	int level = 0;
	short			raw[3];

	if(qhub_info[1].id == QST_MAG_QMC6309)
		level = qmc6309_fifo_read(g_fifo);
	else if(qhub_info[1].id == QST_MAG_QMC6309V)
		level = qmc6309v_fifo_read(g_fifo);
	else if(qhub_info[1].id == QST_MAG_QMC6G00X)
		level = qmc6g00x_fifo_read(g_fifo);
		
	qst_logi("fifo level=%d\r\n", level);
	if(level)
	{
		for(int i=0; i<level; i++)
		{
			raw[0] = (short)(((g_fifo[1+i*6]) << 8) | g_fifo[0+i*6]);
			raw[1] = (short)(((g_fifo[3+i*6]) << 8) | g_fifo[2+i*6]);
			raw[2] = (short)(((g_fifo[5+i*6]) << 8) | g_fifo[4+i*6]);
			qhub_info[1].out.val[0] = raw[0]*0.1f;
			qhub_info[1].out.val[1] = raw[1]*0.1f;
			qhub_info[1].out.val[2] = raw[2]*0.1f;
			qhub_info[1].out.num++;
			//qst_logi("%d,%d,%d,M-%d\r\n", raw[0], raw[1], raw[2], qhub_info[1].out.num);
#if defined(QST_FUSION)
			qst_algo_set_data(QST_SENSOR_MAG, &qhub_info[1].out);
#else
			qst_evb_show_data3(&qhub_info[1].out);
#endif
		}
	}
}

static void qst_evb_mag_st_data(void)
{
	if(qhub_info[1].id == QST_MAG_QMC6308)
		qmc6308_self_test();
#if defined(QMC5883P_SUPPORT)
	else if(qhub_info[1].id == QST_MAG_QMC5883P)
		qmc5883p_self_test();
#endif
	else if(qhub_info[1].id == QST_MAG_QMC6309)
		qmc6309_self_test();
	else if(qhub_info[1].id == QST_MAG_QMC6309V)
		qmc6309v_self_test();
	else if(qhub_info[1].id == QST_MAG_QMC6G00X)
		qmc6g00x_self_test();
}

static void qst_evb_imu_data(void)
{
	evb_sensor_data_t *data1, *data2;

	data1 = &qhub_info[0].out;
	data2 = &qhub_info[2].out;

	if(qhub_info[2].id == QST_IMU_QMI8658)
	{
		qmi8658_read_xyz(&data1->val[0], &data2->val[0]);
		if((qhub_info[2].out.num % 200)==0)
			qhub_info[3].out.val[1] = qmi8658_readTemp();
	}
#if defined(ICM4X6XX_SUPPORT)
	else if(qhub_info[2].id == QST_IMU_ICM4X6XX)
	{
		inv_icm4x6xx_read_data(&data1->val[0], &data2->val[0]);
	}
#endif
	qhub_info[0].out.num++;
	qhub_info[2].out.num++;
	data1->timestamp += qhub_info[0].period;
	data2->timestamp += qhub_info[2].period;

#if defined(QST_FUSION)
	qst_algo_set_data(QST_SENSOR_ACCEL, data1);
	qst_algo_set_data(QST_SENSOR_IMU, data2);
#else
	qst_evb_show_data6(data1 ,data2, qhub_info[3].out.val[1]);
#endif
#if defined(QST_EVB_NOMOTION)
	qst_evb_check_nomotion(data1->val);
#endif
}

static void qst_evb_imu_fifo_data(void)
{
#if defined(QMI8658_USE_FIFO)
	if(qhub_info[2].id == QST_IMU_QMI8658)
	{
		unsigned short f_level = 0; 
		float tempearture = 25.0f;
		evb_sensor_data_t *data1, *data2;
		
		data1 = &qhub_info[0].out;
		data2 = &qhub_info[2].out;

		short raw[6];
		f_level = qmi8658_read_fifo(g_fifo);		
		tempearture = qmi8658_readTemp();
		qst_logi("level=%d\r\n", f_level);
		for(int i=0; i<f_level; i++)
		{
			raw[0] = (g_fifo[i*12+1]<<8)|(g_fifo[i*12+0]);
			raw[1] = (g_fifo[i*12+3]<<8)|(g_fifo[i*12+2]);
			raw[2] = (g_fifo[i*12+5]<<8)|(g_fifo[i*12+4]);
			raw[3] = (g_fifo[i*12+7]<<8)|(g_fifo[i*12+6]);
			raw[4] = (g_fifo[i*12+9]<<8)|(g_fifo[i*12+8]);
			raw[5] = (g_fifo[i*12+11]<<8)|(g_fifo[i*12+10]);

			data1->val[0] = (float)(raw[0]*(9.807f))/2048;
			data1->val[1] = (float)(raw[1]*(9.807f))/2048;
			data1->val[2] = (float)(raw[2]*(9.807f))/2048;
			data2->val[0] = (float)(raw[3]*3.1415926f)/(32*180);
			data2->val[1] = (float)(raw[4]*3.1415926f)/(32*180);
			data2->val[2] = (float)(raw[5]*3.1415926f)/(32*180);
			
			qst_algo_set_data(QST_SENSOR_IMU, data2);
			qst_algo_set_data(QST_SENSOR_ACCEL, data1);
			qst_evb_show_data6(data1->val ,data2->val, tempearture);
#if defined(QST_EVB_NOMOTION)
			qst_evb_check_nomotion(data1->val);
#endif
			//qst_logi("%d %d %d %d %d %d\r\n", raw[0],raw[1],raw[2],raw[3],raw[4],raw[5]);
			//((void)(tempearture));

		}
		//qst_logi("f_level = %d\r\n", f_level);
	}
#endif
}

void qst_evb_imu_motion_hdlr(void)
{
	unsigned char int_status = 0x00;
	if(qhub_info[2].id == QST_IMU_QMI8658)
	{
		int_status = qmi8658_readStatus1();
		qst_logi("qst_evb_imu_motion_hdlr	status1:0x%x -- ", int_status);		
		if(int_status & 0x80)
		{
			qst_logi("qst_evb_imu_motion	SIG MOTION\n");
		}
		if(int_status & 0x40)
		{
			qst_logi("qst_evb_imu_motion	NO MOTION\n");
		}
		if(int_status & 0x20)
		{
			qst_logi("qst_evb_imu_motion	AMD\n");
		}
		if(int_status & 0x10)
		{
			qst_logi("qst_evb_imu_motion	pedometer\n");
		}
		if(int_status & 0x04)
		{
			qst_logi("qst_evb_imu_motion	WOM\n");
		}
		if(int_status & 0x02)
		{
#if defined(QMI8658_USE_TAP)
			unsigned char tap_status = qmi8658_readTapStatus();
			qst_logi("qst_evb_imu_motion	TAP--0x%x  %d\n", tap_status, tap_status&0x03);
#endif
		}
	}
}

static void qst_evb_press_data(void)
{
	if(qhub_info[3].id == QST_PRESS_QMP6989)
		qmp6989_get_data(&qhub_info[3].out.val[0], &qhub_info[3].out.val[1]);
#if defined(QMP6988_SUPPORT)
	else if(qhub_info[3].id == QST_PRESS_QMP6988)
		qmp6988_get_data(&qhub_info[3].out.val[0], &qhub_info[3].out.val[1]);
#endif
#if defined(QMP6990_SUPPORT)
	else if(qhub_info[3].id == QST_PRESS_QMP6990)
		qmp6990_measure_data(&qhub_info[3].out.val[0], &qhub_info[3].out.val[1]);
#endif
		
	qhub_info[3].out.num++;
	qhub_info[3].out.timestamp += qhub_info[3].period;

	qst_evb_show_data2(&qhub_info[3].out);
}

static void qst_evb_timer_data(void)
{
	qst_logi("qst_evb_timer_data count=%d\r\n", qsensor_cfg.count++);

	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
}

// timer max delay 20 Seconds
static TIM_TypeDef * qst_evb_get_timer(void)
{
	int index = 0;

	for(index=0; index<(sizeof(tim_array)/sizeof(tim_array[0])); index++)
	{
		if(tim_array[index].userd == 0)
		{
			tim_array[index].userd = 1;
			return tim_array[index].timId;
		}
	}
	qst_logi("qst_evb_get_timer no timer found!");

	return NULL;
}

static int qst_evb_get_irq(void)
{
	int index = 0;

	for(index=0; index<(sizeof(irq_array)/sizeof(irq_array[0])); index++)
	{
		if(irq_array[index].userd == 0)
		{
			irq_array[index].userd = 1;
			return irq_array[index].irq_num;
		}
	}
	qst_logi("qst_evb_get_timer no irq found!");

	return 0;
}

void qst_evb_init_sensor(qst_sensor_type type, int port, int report, float odr)
{
	int protocol = 0;

	if((port>=INTERFACE_I2C_SW)&&(port<=INTERFACE_I2C_HW_1M))
	{
		protocol = 0;
	}
	else if((port>=INTERFACE_I3C_4M)&&(port<=INTERFACE_I3C_12_5M))
	{
		protocol = 1;
	}
	else if((port>=INTERFACE_SPI_HW4)&&(port<=INTERFACE_SPI_SW3))
	{
		protocol = 2;
	}

	if(type & QST_SENSOR_ACCEL)
	{
		if(qma6100p_init(protocol))
		{
			qhub_info[0].id = QST_ACCEL_QMA6100P;
			if(report & QST_REPORT_FIFO)
			{
				qhub_info[0].wmk = 5;
			}
			if(report & QST_REPORT_EXT_INT)
			{

			}
		}
#if defined(QMAX981_SUPPORT)
		else if(qmaX981_init())
		{
			qhub_info[0].id = QST_ACCEL_QMAX981;
		}
#endif
#if defined(QMA6101T_SUPPORT)
		else if(qma6101t_init(protocol))
		{
			qhub_info[0].id = QST_ACCEL_QMA6101T;
		}
#endif
		else
		{
			qhub_info[0].id = QST_SENSOR_ID_NONE;
		}

		qhub_info[0].report = report;
		qhub_info[0].period = (unsigned short)(1000/odr);

		if(report & QST_REPORT_FIFO)
			qhub_info[0].irq_hdlr = qst_evb_accel_fifo_data;
		else
			qhub_info[0].irq_hdlr = qst_evb_accel_data;

		if(report & QST_REPORT_EXT_INT)
			qhub_info[0].motion_irq_hdlr = qst_evb_accel_irq;

	}
	if(type & QST_SENSOR_MAG)
	{
		if(qmc6309_init(protocol))
		{
			qhub_info[1].id = QST_MAG_QMC6309;
			if(report & QST_REPORT_FIFO)
			{
				qhub_info[1].wmk = 5;
				qmc6309_fifo_config(QMC6309_FIFO_MODE_STREAM, qhub_info[1].wmk);
			}
		}
		else if(qmc6309v_init(protocol))
		{
			qhub_info[1].id = QST_MAG_QMC6309V;
			if(report & QST_REPORT_FIFO)
			{
				qhub_info[1].wmk = 5;
				qmc6309v_fifo_config(QMC6309V_FIFO_MODE_STREAM, qhub_info[1].wmk);
			}
		}
		else if(qmc6g00x_init(protocol))
		{
			qhub_info[1].id = QST_MAG_QMC6G00X;
			if(report & QST_REPORT_FIFO)
			{
				qhub_info[1].wmk = 5;
				qmc6g00x_fifo_config(QMC6G00X_FIFO_MODE_STREAM, qhub_info[1].wmk);
			}
		}
#if defined(QMC5883P_SUPPORT)
		else if(qmc5883p_init())
		{
			qhub_info[1].id = QST_MAG_QMC5883P;
		}
#else
		else if(qmc6308_init())
		{
			qhub_info[1].id = QST_MAG_QMC6308;
		}
#endif
#if defined(QMC5883L_SUPPORT)
		else if(qmc5883l_InitConfig())
		{
			qhub_info[1].id = QST_MAG_QMC5883L;
		}
#endif
		else
		{
			qhub_info[1].id = QST_SENSOR_ID_NONE;
		}

		qhub_info[1].report = report;
		qhub_info[1].period = (unsigned short)(1000/odr);
		if(report & QST_REPORT_POLLING)
		{
			if(report & QST_REPORT_FIFO)
				qhub_info[1].irq_hdlr = qst_evb_mag_fifo_data;
			else if(report & QST_REPORT_SELFTEST)
				qhub_info[1].irq_hdlr = qst_evb_mag_st_data;
			else
				qhub_info[1].irq_hdlr = qst_evb_mag_data;
		}
	}
	if(type & QST_SENSOR_IMU)
	{
		if(qmi8658_init(protocol))
		{
			qhub_info[2].id = QST_IMU_QMI8658;
			if(report & QST_REPORT_FIFO)
			{
				qhub_info[2].wmk = 5;
#if defined(QMI8658_USE_FIFO)
				//qmi8658_enableSensors(QMI8658_DISABLE_ALL);
				//qst_delay_ms(2);
				qmi8658_config_fifo(qhub_info[2].wmk, qmi8658_Fifo_64, qmi8658_Fifo_Stream, qmi8658_Int1, QMI8658_ACCGYR_ENABLE);
				//qmi8658_enableSensors(QMI8658_ACCGYR_ENABLE);
#endif
			}
			if(report & QST_REPORT_EXT_INT)
			{
#if !defined(QST_EVB_NOMOTION)
#if defined(QMI8658_USE_AMD)
				qmi8658_enable_amd(1, qmi8658_Int1, 0);
#endif
#if defined(QMI8658_USE_WOM)
				qmi8658_enable_wom(1, qmi8658_Int1);
#endif
#endif
			}
		}
#if defined(ICM4X6XX_SUPPORT)
		else if(inv_icm4x6xx_initialize() > 0)
		{
			qhub_info[2].id = QST_IMU_ICM4X6XX;
			inv_icm4x6xx_acc_set_rate(200, 2);
			inv_icm4x6xx_acc_enable();
			inv_icm4x6xx_gyro_set_rate(200, 2);
			inv_icm4x6xx_gyro_enable();
		}
#endif
		else
		{
			qhub_info[2].id = QST_SENSOR_ID_NONE;
		}

		qhub_info[2].report = report;
		qhub_info[0].period = qhub_info[2].period = (unsigned short)(1000/odr);
		if(report & QST_REPORT_FIFO)
		{
			qhub_info[2].irq_hdlr = qst_evb_imu_fifo_data;
		}
		else
		{
			qhub_info[2].irq_hdlr = qst_evb_imu_data;
		}
		if(report & QST_REPORT_EXT_INT)
		{
			qhub_info[2].motion_irq_hdlr = qst_evb_imu_motion_hdlr;
		}
	}
	if(type & QST_SENSOR_PRESS)
	{
		if(qmp6989_get_pid() == 0x02)
		{
			qmp6989_initialization();
			qhub_info[3].id = QST_PRESS_QMP6989;
		}
#if defined(QMP6988_SUPPORT)
		else if(qmp6988_init())
		{
			qhub_info[3].id = QST_PRESS_QMP6988;
		}
#endif
#if defined(QMP6990_SUPPORT)
		else if(qmp6990_setup())
		{
			qhub_info[3].id = QST_PRESS_QMP6990;
		}
#endif
		else
		{
			qhub_info[3].id = QST_SENSOR_ID_NONE;
		}
		qhub_info[3].report = report;
		qhub_info[3].period = (unsigned short)(1000/odr);
		qhub_info[3].irq_hdlr = qst_evb_press_data;
	}
	if(type & QST_SENSOR_TIMER)
	{
		qhub_info[4].id = QST_MAG_TOTAL;
		qhub_info[4].report = report;
		qhub_info[4].period = (unsigned short)(1000/odr);
		qhub_info[4].irq_hdlr = qst_evb_timer_data;
	}
}

void qst_evb_start_sensor(void)
{
	qsensor_cfg.protocol = QST_UART_PROTOCOL_ASCII;	//QST_UART_PROTOCOL_ASCII;
	qsensor_cfg.count = 0;
	qst_algo_init();

	for(int i=0; i<(sizeof(qhub_info)/sizeof(qhub_info[0])); i++)
	{
		if(qhub_info[i].id != QST_SENSOR_ID_NONE)
		{
			if(qhub_info[i].report & QST_REPORT_POLLING)
			{
				TIM_TypeDef *timid = qst_evb_get_timer();
				if(timid)
				{
					if(qhub_info[i].wmk >= 1)
						evb_setup_timer(timid, qhub_info[i].irq_hdlr, qhub_info[i].period*qhub_info[i].wmk , ENABLE);
					else
						qst_loge("qst_evb_start_sensor timer error: wmk < 1!\r\n");
				}
			}
			if(qhub_info[i].report & QST_REPORT_DRI)
			{
				int irq_num = qst_evb_get_irq();
				if(irq_num)
				{
					evb_setup_irq(irq_num, qhub_info[i].irq_hdlr, ENABLE);
				}
			}
			if(qhub_info[i].report & QST_REPORT_EXT_INT)
			{
				int irq_num = qst_evb_get_irq();
				if(irq_num)
				{
					evb_setup_irq(irq_num, qhub_info[i].motion_irq_hdlr, ENABLE);
				}
			}
		}
	}
}


