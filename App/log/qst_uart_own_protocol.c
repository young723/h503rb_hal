
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "bsp_hardware.h"
#include "qst_sensor.h"
#include "qst_uart_own_protocol.h"

static qst_uart_protocol_t g_own_proc;

int qst_evb_send_own_protocol(int demo_sensor, float *data_p, float misc)
{
	int i;
	int len = 0;
	unsigned char data_len;
	unsigned char sum = 0;

	g_own_proc.pc_len = 0;
	if(demo_sensor == QST_SENSOR_ACCEL)
	{
		g_own_proc.pc_buf[0] = '$';
		g_own_proc.pc_buf[1] = ':';
		g_own_proc.pc_buf[2] = 'a';
		g_own_proc.pc_buf[3] = 0;
		len = 4;

		data_len = sizeof(float)*3;
		memcpy(&g_own_proc.pc_buf[len], data_p, data_len);
		len += data_len;

		unsigned int step = (unsigned int)misc;
		data_len = sizeof(step);
		memcpy(&g_own_proc.pc_buf[len], &step, data_len);
		len += data_len;

		g_own_proc.pc_buf[3] = len - 4;
	}
	else if(demo_sensor == QST_SENSOR_MAG)
	{
		g_own_proc.pc_buf[0] = '$';
		g_own_proc.pc_buf[1] = ':';
		g_own_proc.pc_buf[2] = 'm';
		g_own_proc.pc_buf[3] = 0;
		len = 4;
	
		data_len = sizeof(float)*3;
		memcpy(&g_own_proc.pc_buf[len], data_p, data_len);
		len += data_len;

		data_len = sizeof(float);
		memcpy(&g_own_proc.pc_buf[len], &misc, data_len);
		len += data_len;

		g_own_proc.pc_buf[3] = len - 4;
	}
	else if(demo_sensor == QST_SENSOR_IMU)
	{
		g_own_proc.pc_buf[0] = '$';
		g_own_proc.pc_buf[1] = ':';
		g_own_proc.pc_buf[2] = 'g';
		g_own_proc.pc_buf[3] = 0;
		len = 4;

		data_len = sizeof(float)*6;
		memcpy(&g_own_proc.pc_buf[len], data_p, data_len);
		len += data_len;

		data_len = sizeof(float);
		memcpy(&g_own_proc.pc_buf[len], &misc, data_len);
		len += data_len;

		g_own_proc.pc_buf[3] = len - 4;
	}
#if 0
	else if(demo_sensor == QST_SENSOR_ORIENTATION)
	{
		g_own_proc.pc_buf[0] = '$';
		g_own_proc.pc_buf[1] = ':';
		g_own_proc.pc_buf[2] = 'o';
		g_own_proc.pc_buf[3] = 0;
		len = 4;

		data_len = sizeof(float)*3;
		memcpy(&g_own_proc.pc_buf[len], data_p, data_len);
		len += data_len;

		data_len = sizeof(misc);
		memcpy(&g_own_proc.pc_buf[len], &misc, data_len);
		len += data_len;

		g_own_proc.pc_buf[3] = len - 4;
	}
#endif
	else if(demo_sensor == QST_SENSOR_PRESS)
	{
		g_own_proc.pc_buf[0] = '$';
		g_own_proc.pc_buf[1] = ':';
		g_own_proc.pc_buf[2] = 'p';
		g_own_proc.pc_buf[3] = 0;
		len = 4;

		data_len = sizeof(float)*1;
		memcpy(&g_own_proc.pc_buf[len], data_p, data_len);
		len += data_len;

		data_len = sizeof(float)*1;
		memcpy(&g_own_proc.pc_buf[len], &misc, data_len);
		len += data_len;
		
		g_own_proc.pc_buf[3] = len - 4;
	}
#if 0
	else if(demo_sensor == QST_SENSOR_9AXIS_FUSION)
	{
		g_own_proc.pc_buf[0] = '$';
		g_own_proc.pc_buf[1] = ':';
		g_own_proc.pc_buf[2] = '9';
		g_own_proc.pc_buf[3] = 0;
		len = 4;

		data_len = sizeof(float)*6;
		memcpy(&g_own_proc.pc_buf[len], data_p, data_len);
		len += data_len;

		data_len = sizeof(float)*3;
		data_p += 6;
		memcpy(&g_own_proc.pc_buf[len], data_p, data_len);
		len += data_len;

		data_len = sizeof(float);
		memcpy(&g_own_proc.pc_buf[len], &misc, data_len);
		len += data_len;

		g_own_proc.pc_buf[3] = len - 4;
	}
	else if(demo_sensor == QST_SENSOR_HW_SELFTEST)
	{
		g_own_proc.pc_buf[0] = '$';
		g_own_proc.pc_buf[1] = ':';
		g_own_proc.pc_buf[2] = 's';
		g_own_proc.pc_buf[3] = 0;
		len = 4;

		data_len = sizeof(float)*6;
		memcpy(&g_own_proc.pc_buf[len], data_p, data_len);
		len += data_len;

		g_own_proc.pc_buf[3] = len - 4;
	}
	else if(demo_sensor == QST_SENSOR_REGISTER)
	{
		g_own_proc.pc_buf[0] = '$';
		g_own_proc.pc_buf[1] = ':';
		g_own_proc.pc_buf[2] = 'r';
		g_own_proc.pc_buf[3] = 0;
		len = 4;
		
		data_len = (int)misc;
		memcpy(&g_own_proc.pc_buf[len], data_p, data_len);
		len += data_len;
		
		g_own_proc.pc_buf[3] = len - 4;
	}
#endif

	sum = 0;
	for(i=0; i<len; i++)
	{
		sum += g_own_proc.pc_buf[i];
		usart_send_ch(g_own_proc.pc_buf[i]);
	}
	g_own_proc.pc_buf[len] = sum;
	usart_send_ch((uint8_t)sum);
	len+=1;

	g_own_proc.pc_buf[len] = ';';	
	usart_send_ch((uint8_t)';');
	len+=1;
	
	usart_send_ch((uint8_t)'\n');
	len+=1;
	g_own_proc.pc_len = len;

	return len;
}


int qst_evb_send_own_num(unsigned int num)
{
	int i;
	int len = 0;
	unsigned char data_len;
	unsigned char sum = 0;

	g_own_proc.pc_len = 0;

	g_own_proc.pc_buf[0] = '$';
	g_own_proc.pc_buf[1] = ':';
	g_own_proc.pc_buf[2] = 'c';
	g_own_proc.pc_buf[3] = 0;
	len = 4;

	data_len = sizeof(unsigned int);
	memcpy(&g_own_proc.pc_buf[len], &num, data_len);
	len += data_len;

	g_own_proc.pc_buf[3] = len - 4;

	sum = 0;
	for(i=0; i<len; i++)
	{
		sum += g_own_proc.pc_buf[i];
		usart_send_ch(g_own_proc.pc_buf[i]);
	}
	g_own_proc.pc_buf[len] = sum;
	usart_send_ch((uint8_t)sum);
	len+=1;

	g_own_proc.pc_buf[len] = ';';
	usart_send_ch((uint8_t)';');
	len+=1;
	
	usart_send_ch((uint8_t)'\n');
	len+=1;
	g_own_proc.pc_len = len;

	return len;
}


int qst_evb_send_own_info(unsigned int info[4])
{
	int i;
	int len = 0;
	unsigned char data_len;
	unsigned char sum = 0;

	g_own_proc.pc_len = 0;

	g_own_proc.pc_buf[0] = '$';
	g_own_proc.pc_buf[1] = ':';
	g_own_proc.pc_buf[2] = 'q';
	g_own_proc.pc_buf[3] = 0;
	len = 4;

	data_len = sizeof(unsigned int)*4;
	memcpy(&g_own_proc.pc_buf[len], &info[0], data_len);
	len += data_len;

	g_own_proc.pc_buf[3] = len - 4;

	sum = 0;
	for(i=0; i<len; i++)
	{
		sum += g_own_proc.pc_buf[i];
		usart_send_ch(g_own_proc.pc_buf[i]);
	}
	g_own_proc.pc_buf[len] = sum;
	usart_send_ch((uint8_t)sum);
	len+=1;

	g_own_proc.pc_buf[len] = ';';	
	usart_send_ch((uint8_t)';');
	len+=1;
	
	usart_send_ch((uint8_t)'\n');
	len+=1;
	g_own_proc.pc_len = len;

	return len;
}


#if 0
int qst_evb_send_raw_own_protocol(qst_sensor_type demo_sensor)
{
	int i;
	int len = 0;
	unsigned char data_len;
	unsigned char sum = 0;

	g_own_proc.pc_len = 0;

	if(demo_sensor == QST_SENSOR_IMU)
	{
		g_own_proc.pc_buf[0] = '#';
		g_own_proc.pc_buf[1] = ':';
		g_own_proc.pc_buf[2] = 'g';
		g_own_proc.pc_buf[3] = 0;
		len = 4;
		
		data_len = 12;
		memcpy(&g_own_proc.pc_buf[len], &g_own_proc.reg[0], data_len);
		len += data_len;
		data_len = 2;
		memcpy(&g_own_proc.pc_buf[len], &g_own_proc.reg[12], data_len);
		len += data_len;

		g_own_proc.pc_buf[3] = len - 4;
	}	
	else if(demo_sensor == QST_SENSOR_9AXIS_FUSION)
	{
		g_own_proc.pc_buf[0] = '#';
		g_own_proc.pc_buf[1] = ':';
		g_own_proc.pc_buf[2] = '9';
		g_own_proc.pc_buf[3] = 0;
		len = 4;
		
		data_len = 12;
		memcpy(&g_own_proc.pc_buf[len], &g_own_proc.reg[0], data_len);
		len += data_len;
		
		data_len = 6;
		memcpy(&g_own_proc.pc_buf[len], &g_own_proc.reg[12], data_len);
		len += data_len;

		//data_len = 2;
		//memcpy(&g_own_proc.pc_buf[len], &g_own_proc.reg[18], data_len);
		//len += data_len;

		g_own_proc.pc_buf[3] = len - 4;
	}

	sum = 0;
	for(i=0; i<len; i++)
	{
		sum += g_own_proc.pc_buf[i];
		USART_SendData(DEBUG_USART, (uint8_t)g_own_proc.pc_buf[i]);
		while(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);	
	}
	g_own_proc.pc_buf[len] = sum;
	USART_SendData(DEBUG_USART, (uint8_t)sum);
	while(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);
	len+=1;

	g_own_proc.pc_buf[len] = ';';
	USART_SendData(DEBUG_USART, (uint8_t)';');
	while(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);
	len+=1;

	USART_SendData(DEBUG_USART, (uint8_t)'\n');
	while(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);
	len+=1;
	g_own_proc.pc_len = len;

	return len;
}
#endif

