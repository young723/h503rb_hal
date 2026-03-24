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
#include <string.h>
#include <math.h>
#include "sensorhub.h"
#include "sensorhub.h"

static qst_evb_t g_evb;
#define QST_SENSOR_ALL	(QST_SENSOR_ACCEL|QST_SENSOR_MAG|QST_SENSOR_IMU|QST_SENSOR_PRESS) 
const qst_sensor_list_t sensor_list[] = 
{
	{"Accelerometer",	QST_SENSOR_ACCEL,		QST_REPORT_POLLING, 50,		INTERFACE_I2C_HW},
	{"Magnetic",		QST_SENSOR_MAG,			QST_REPORT_POLLING, 50,		INTERFACE_I2C_HW},
	{"IMU",				QST_SENSOR_IMU,			QST_REPORT_POLLING, 100,	INTERFACE_I2C_HW},		// INTERFACE_USER_SEL
	{"Barometer",		QST_SENSOR_PRESS,		QST_REPORT_POLLING, 5,		INTERFACE_I2C_HW},
//	{"Magnetic Test",	QST_SENSOR_MAG_TEST,	QST_REPORT_POLLING, 5,	INTERFACE_USER_SEL}
};

//static void qst_evb_key1_hdlr(void)
//{
//#if defined(BSP_FLASH_SUPPORT)
//	g_evb.test_num[0] = 0;
//	g_evb.test_num[1] = 0;
//	flash_write_u32(ADDR_FLASH_SECTOR_7, 0, &(g_evb.test_num[0]), sizeof(g_evb.test_num));
//#endif
//}

void qst_evb_uart_rx_hdlr(unsigned char *rx_buf, unsigned short len)
{
	if(strncmp((const char*)rx_buf, "rst", 3) == 0)
	{
		NVIC_SystemReset();
	}
	else if(strncmp((const char*)rx_buf, "pause", 5) == 0)
	{
		bsp_event_clear();
	}
	else if(strncmp((const char*)rx_buf, "resume", 5) == 0)
	{
		//g_evb.reset_flag = 1;
	}

}


void qst_evb_entry_sel_sensor(int user_sel)
{
	int sel_i = 0;
	int sel_port;


	if(user_sel == QST_SENSOR_ALL)
	{

		for(sel_i=0; sel_i<(sizeof(sensor_list)/sizeof(sensor_list[0])); sel_i++)
		{
			if((sensor_list[sel_i].report > QST_REPORT_OFF) && (sensor_list[sel_i].odr > 0))
			{
				sel_port = sensor_list[sel_i].port;
				bsp_entry_sel_interface(&sel_port);
				if(sensor_list[sel_i].sensor & QST_SENSOR_ALL)
					qst_evb_init_sensor(sensor_list[sel_i].sensor , sel_port, sensor_list[sel_i].report, sensor_list[sel_i].odr);
			}
		}
	}
	else if((user_sel < 0) || (user_sel>=(sizeof(sensor_list)/sizeof(sensor_list[0]))) )
	{
		while(1)
		{
			evb_setup_uart_rx(USART2, NULL);
			qst_logi("\r\nSelect sensor type:\r\n");
			for(int i=0; i<(sizeof(sensor_list)/sizeof(sensor_list[0])); i++)
			{
				if((sensor_list[i].report > QST_REPORT_OFF) && (sensor_list[i].odr > 0))
					qst_logi("[%d]: %s \r\n", i, sensor_list[i].name);
			}
			scanf("%d", &sel_i);
			if( (sel_i>=0) && (sel_i<(sizeof(sensor_list)/sizeof(sensor_list[0]))) )
			{
				if((sensor_list[sel_i].report > QST_REPORT_OFF) && (sensor_list[sel_i].odr > 0))
				{
					qst_logi("\r\nSelect sensor: %s \r\n\r\n", sensor_list[sel_i].name);
					break;
				}
			}
			else
			{
				qst_logi("\r\nSelect sensor: %d error!!!\r\n\r\n", sel_i);
			}
		}

		sel_port = (int)sensor_list[sel_i].port;
		bsp_entry_sel_interface(&sel_port);
		qst_evb_init_sensor(sensor_list[sel_i].sensor, sel_port, sensor_list[sel_i].report, sensor_list[sel_i].odr);
	}
	else
	{
		if((sensor_list[user_sel].report > QST_REPORT_OFF) && (sensor_list[user_sel].odr > 0))
		{
			sel_port = sensor_list[user_sel].port;
			bsp_entry_sel_interface(&sel_port);

			qst_evb_init_sensor(sensor_list[user_sel].sensor , sel_port, sensor_list[user_sel].report, sensor_list[user_sel].odr);
		}
		else
		{
			qst_loge("sensor_list[%d]: parameter error!!!\r\n", user_sel);
		}
	}
}

int main(void)
{ 
	bsp_hardware_init();
	qst_delay_ms(100);
	qst_logi("QST demo start[compile time:%s %s]\r\n", __DATE__, __TIME__);
loop_flag:
	#if 0
	qst_evb_entry_sel_sensor(2);
	qst_evb_start_sensor();
	evb_setup_uart_rx(USART3, qst_evb_uart_rx_hdlr);
	#else
	g_evb.intf = INTERFACE_I2C_HW;	//INTERFACE_I2C_HW;
	bsp_entry_sel_interface(&g_evb.intf);
	qst_evb_mag_test_entry(g_evb.intf);
	#endif
	//evb_setup_user_key(1, qst_evb_key1_hdlr);
	while(1)
	{
		qst_i3c_ibi_handle();
		evb_irq_handle();
		evb_tim_handle();
		evb_key_handle();
		evb_usart_rx_handle();
		if(g_evb.reset_flag)
		{
			bsp_event_clear();
			g_evb.reset_flag = 0;
			goto loop_flag;
		}
	}
}


