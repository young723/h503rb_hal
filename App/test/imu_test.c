/* USER CODE BEGIN Header */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "bsp_hardware.h"
#include "qst_uart_own_protocol.h"

unsigned int g_counter = 0;

void qst_evb_imu_test_reg_wr(void)
{
	extern char g_out_str[];

	#define MAX_REG		7
	static unsigned int fail_num[MAX_REG];
	static unsigned char reg_array[MAX_REG][2] = 
	{
		{0x00, 0x05},
		{0x01, 0x7c},
		{0x02, 0x78},
		{0x03, 0x35},
		{0x04, 0x65},
		{0x08, 0x03},
		{0x09, 0xc0}
	};		
	uint8_t	reg_value[MAX_REG];
	int	ret = 0;
	int len = 0;
	char *buf = (char*)g_out_str;

//	bsp_write_reg(0x6a, reg_array[3][0], reg_array[3][1]);
//	bsp_write_reg(0x6a, reg_array[4][0], reg_array[4][1]);
//	bsp_write_reg(0x6a, reg_array[5][0], reg_array[5][1]);
#if 0
	for(int i=0; i<MAX_REG; i++)
	{
		ret = bsp_read_reg(0x6a, reg_array[i][0], &reg_value[i], 1);
		if(ret != 1)
			qst_logi("read reg fail \r\n");
	}
#else
	ret = bsp_read_reg(0x6a, reg_array[0][0], &reg_value[0], 1);
	ret = bsp_read_reg(0x6a, reg_array[1][0], &reg_value[1], 4);
	ret = bsp_read_reg(0x6a, reg_array[5][0], &reg_value[5], 2);
#endif
	len = 0;
	len += snprintf(buf+len, 256-len, "%d ", g_counter++);
	for(int i=0; i<MAX_REG; i++)
	{
		if(reg_value[i] !=	reg_array[i][1])
		{
			fail_num[i]++;
		}
		len += snprintf(buf+len, 256-len, "0x%02x=0x%02x fail:%d, ", reg_array[i][0], reg_value[i], fail_num[i]);
	}
	len += snprintf(buf+len, 256-len, "\r\n");
	buf[len] = '\0';
	qst_logi("%s", buf);
}


void qst_evb_imu_test_entry(void)
{
	evb_setup_timer(TIM2, qst_evb_imu_test_reg_wr, 10, ENABLE);
}


