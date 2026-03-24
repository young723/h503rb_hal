
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "bsp_hardware.h"
#include "qst_uart_sscom.h"

void qst_evb_plot_sscom(int x, int y, int z)
{
	int pktSize;
	plotsscom_t g_plotsim;
	unsigned char *buf_p;

	g_plotsim.head = 0xCDAB;             //SimPlot packet header. Indicates start of data packet
	g_plotsim.len = 4*sizeof(plotsscom_t);      //Size of data in bytes. Does not include the header and size fields
	g_plotsim.buf[0] = (short)x*10;
	g_plotsim.buf[1] = (short)y*10;
	g_plotsim.buf[2] = (short)z*10;
	//g_plotsim.buf[3] = z;

	pktSize = sizeof(plotsscom_t); //Header bytes + size field bytes + data
	//IMPORTANT: Change to serial port that is connected to PC
	//Serial.write((uint8_t * )buffer, pktSize);	
	//qst_send_str((unsigned char *)g_plotsim.buf, pktSize);
	buf_p = (unsigned char *)g_plotsim.buf;
	for(int i=0; i<pktSize; i++)
	{
		usart_send_ch(buf_p[i]);
	}
}

