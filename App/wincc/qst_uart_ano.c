
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "bsp_hardware.h"
#include "qst_uart_ano.h"

static unsigned char	data_to_send[50];

int uart_ano_print_log(int priority, const char* tag, const char* fmt, ...)
{
    va_list args;
    int length, ii, i;
    char buf[BUF_SIZE], out[PACKET_LENGTH], this_length;

    /* This can be modified to exit for unsupported priorities. */
    switch (priority) {
    case QST_LOG_UNKNOWN:
    case QST_LOG_DEFAULT:
    case QST_LOG_VERBOSE:
    case QST_LOG_DEBUG:
    case QST_LOG_INFO:
    case QST_LOG_WARN:
    case QST_LOG_ERROR:
    case QST_LOG_SILENT:
        break;
    default:
        return 0;
    }

    va_start(args, fmt);

    length = vsprintf(buf, fmt, args);
    if (length <= 0) {
        va_end(args);
        return length;
    }

    memset(out, 0, PACKET_LENGTH);
    out[0] = '$';
    out[1] = PACKET_DEBUG;
    out[2] = priority;
    out[21] = '\r';
    out[22] = '\n';
		
		
    for(ii = 0; ii < length; ii += (PACKET_LENGTH-5)) {
        this_length = min(length-ii, PACKET_LENGTH-5);
        memset(out+3, 0, 18);
        memcpy(out+3, buf+ii, this_length);
        
		for (i=0; i<PACKET_LENGTH; i++) {
          usart_send_ch(out[i]);
        }
    }
            
    va_end(args);

    return 0;
}


void uart_ano_send_data(unsigned char type, long *data)
{
    char out[PACKET_LENGTH];
    int i;
    if (!data)
        return;
    memset(out, 0, PACKET_LENGTH);
    out[0] = '$';
    out[1] = PACKET_DATA;
    out[2] = type;
    out[21] = '\r';
    out[22] = '\n';
    switch (type) {
    /* Two bytes per-element. */
    case PACKET_DATA_ROT:
        out[3] = (char)(data[0] >> 24);
        out[4] = (char)(data[0] >> 16);
        out[5] = (char)(data[1] >> 24);
        out[6] = (char)(data[1] >> 16);
        out[7] = (char)(data[2] >> 24);
        out[8] = (char)(data[2] >> 16);
        out[9] = (char)(data[3] >> 24);
        out[10] = (char)(data[3] >> 16);
        out[11] = (char)(data[4] >> 24);
        out[12] = (char)(data[4] >> 16);
        out[13] = (char)(data[5] >> 24);
        out[14] = (char)(data[5] >> 16);
        out[15] = (char)(data[6] >> 24);
        out[16] = (char)(data[6] >> 16);
        out[17] = (char)(data[7] >> 24);
        out[18] = (char)(data[7] >> 16);
        out[19] = (char)(data[8] >> 24);
        out[20] = (char)(data[8] >> 16);
        break;
    /* Four bytes per-element. */
    /* Four elements. */
    case PACKET_DATA_QUAT:
        out[15] = (char)(data[3] >> 24);
        out[16] = (char)(data[3] >> 16);
        out[17] = (char)(data[3] >> 8);
        out[18] = (char)data[3];
    /* Three elements. */
    case PACKET_DATA_ACCEL:
    case PACKET_DATA_GYRO:
    case PACKET_DATA_COMPASS:
    case PACKET_DATA_EULER:
        out[3] = (char)(data[0] >> 24);
        out[4] = (char)(data[0] >> 16);
        out[5] = (char)(data[0] >> 8);
        out[6] = (char)data[0];
        out[7] = (char)(data[1] >> 24);
        out[8] = (char)(data[1] >> 16);
        out[9] = (char)(data[1] >> 8);
        out[10] = (char)data[1];
        out[11] = (char)(data[2] >> 24);
        out[12] = (char)(data[2] >> 16);
        out[13] = (char)(data[2] >> 8);
        out[14] = (char)data[2];
        break;
    case PACKET_DATA_HEADING:
        out[3] = (char)(data[0] >> 24);
        out[4] = (char)(data[0] >> 16);
        out[5] = (char)(data[0] >> 8);
        out[6] = (char)data[0];
        break;
    default:
        return;
    }
    for (i=0; i<PACKET_LENGTH; i++) {
      usart_send_ch(out[i]);
    }
}



void uart_ano_send_quat(long *quat)
{
    char out[PACKET_LENGTH];
    int i;
    if (!quat)
        return;
    memset(out, 0, PACKET_LENGTH);
    out[0] = '$';
    out[1] = PACKET_QUAT;
    out[3] = (char)(quat[0] >> 24);
    out[4] = (char)(quat[0] >> 16);
    out[5] = (char)(quat[0] >> 8);
    out[6] = (char)quat[0];
    out[7] = (char)(quat[1] >> 24);
    out[8] = (char)(quat[1] >> 16);
    out[9] = (char)(quat[1] >> 8);
    out[10] = (char)quat[1];
    out[11] = (char)(quat[2] >> 24);
    out[12] = (char)(quat[2] >> 16);
    out[13] = (char)(quat[2] >> 8);
    out[14] = (char)quat[2];
    out[15] = (char)(quat[3] >> 24);
    out[16] = (char)(quat[3] >> 16);
    out[17] = (char)(quat[3] >> 8);
    out[18] = (char)quat[3];
    out[21] = '\r';
    out[22] = '\n';
		
    for (i=0; i<PACKET_LENGTH; i++) {
      usart_send_ch(out[i]);
    }
}


void uart_ano_send_euler(float angle_pit, float angle_rol, float angle_yaw, int alt, unsigned char fly_model, unsigned char armed)
{
	unsigned char i;
	unsigned char sum = 0;
	unsigned char _cnt=0;
	unsigned short _temp;
	int _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	if(angle_yaw > 180)
	{
		angle_yaw = angle_yaw-360;
	}
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	for(i=0;i<_cnt;i++)
		usart_send_ch(data_to_send[i]);
}



void usart_ano_send_rawdata1(short *Gyro,short *Accel, short *Mag)
{
	unsigned char i=0;
	unsigned char _cnt=0;
	unsigned char sum = 0;
//	unsigned int _temp;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;

	data_to_send[_cnt++]=BYTE1(Accel[0]);
	data_to_send[_cnt++]=BYTE0(Accel[0]);
	data_to_send[_cnt++]=BYTE1(Accel[1]);
	data_to_send[_cnt++]=BYTE0(Accel[1]);
	data_to_send[_cnt++]=BYTE1(Accel[2]);
	data_to_send[_cnt++]=BYTE0(Accel[2]);
	
	data_to_send[_cnt++]=BYTE1(Gyro[0]);
	data_to_send[_cnt++]=BYTE0(Gyro[0]);
	data_to_send[_cnt++]=BYTE1(Gyro[1]);
	data_to_send[_cnt++]=BYTE0(Gyro[1]);
	data_to_send[_cnt++]=BYTE1(Gyro[2]);
	data_to_send[_cnt++]=BYTE0(Gyro[2]);
//	data_to_send[_cnt++]=0;
//	data_to_send[_cnt++]=0;
//	data_to_send[_cnt++]=0;
//	data_to_send[_cnt++]=0;
//	data_to_send[_cnt++]=0;
//	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(Mag[0]);
	data_to_send[_cnt++]=BYTE0(Mag[0]);
	data_to_send[_cnt++]=BYTE1(Mag[1]);
	data_to_send[_cnt++]=BYTE0(Mag[1]);
	data_to_send[_cnt++]=BYTE1(Mag[2]);
	data_to_send[_cnt++]=BYTE0(Mag[2]);
	
	data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum+= data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	for(i=0;i<_cnt;i++)
		usart_send_ch(data_to_send[i]);
}

void usart_ano_send_rawdata2(int alt_bar, unsigned short alt_csb)
{
	unsigned char i=0;
	unsigned char _cnt=0;
	unsigned char sum = 0;
	unsigned short _temp = 0;
	int _temp2 = 0;


	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x07;
	data_to_send[_cnt++]=0;

	_temp2 = alt_bar;
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	_temp = alt_csb;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum+= data_to_send[i];
	data_to_send[_cnt++]=sum;

	for(i=0;i<_cnt;i++)
		usart_send_ch(data_to_send[i]);

}

void uart_ano_send_senosr_data(float *euler, float *Gyro, float *Accel, float *Mag, float Press)
{
	short acc_raw[3];
	short gyr_raw[3];
	short mag_raw[3];

	acc_raw[0] = (short)(Accel[0]*1000.0f);
	acc_raw[1] = (short)(Accel[1]*1000.0f);
	acc_raw[2] = (short)(Accel[2]*1000.0f);
	gyr_raw[0] = (short)(Gyro[0]*57.2957796f);
	gyr_raw[1] = (short)(Gyro[1]*57.2957796f);
	gyr_raw[2] = (short)(Gyro[2]*57.2957796f);
	mag_raw[0] = (short)Mag[0];
	mag_raw[1] = (short)Mag[1];
	mag_raw[2] = (short)Mag[2];

	uart_ano_send_euler(euler[1], euler[2], euler[0], 0, 0x01, 1);
	usart_ano_send_rawdata1(gyr_raw, acc_raw, mag_raw);
	usart_ano_send_rawdata2((int)(Press*100), 0);
}


void uart_ano_send_senosr_data_v6(float *Accel, float *Gyro, float *Mag)
{
	unsigned char i=0;
	unsigned char _cnt=0;
	unsigned char sum = 0;
	short acc_raw[3];
	short gyr_raw[3];
	short mag_raw[3];

	acc_raw[0] = (short)(Accel[0]*1000.0f);
	acc_raw[1] = (short)(Accel[1]*1000.0f);
	acc_raw[2] = (short)(Accel[2]*1000.0f);
	gyr_raw[0] = (short)(Gyro[0]*57.2957796f);
	gyr_raw[1] = (short)(Gyro[1]*57.2957796f);
	gyr_raw[2] = (short)(Gyro[2]*57.2957796f);
	mag_raw[0] = (short)Mag[0];
	mag_raw[1] = (short)Mag[1];
	mag_raw[2] = (short)Mag[2];

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=18;	//sizeof(short)*9;

	data_to_send[_cnt++]=BYTE1(acc_raw[0]);
	data_to_send[_cnt++]=BYTE0(acc_raw[0]);
	data_to_send[_cnt++]=BYTE1(acc_raw[1]);
	data_to_send[_cnt++]=BYTE0(acc_raw[1]);
	data_to_send[_cnt++]=BYTE1(acc_raw[2]);
	data_to_send[_cnt++]=BYTE0(acc_raw[2]);

	data_to_send[_cnt++]=BYTE1(gyr_raw[0]);
	data_to_send[_cnt++]=BYTE0(gyr_raw[0]);
	data_to_send[_cnt++]=BYTE1(gyr_raw[1]);
	data_to_send[_cnt++]=BYTE0(gyr_raw[1]);
	data_to_send[_cnt++]=BYTE1(gyr_raw[2]);
	data_to_send[_cnt++]=BYTE0(gyr_raw[2]);

	data_to_send[_cnt++]=BYTE1(mag_raw[0]);
	data_to_send[_cnt++]=BYTE0(mag_raw[0]);
	data_to_send[_cnt++]=BYTE1(mag_raw[1]);
	data_to_send[_cnt++]=BYTE0(mag_raw[1]);
	data_to_send[_cnt++]=BYTE1(mag_raw[2]);
	data_to_send[_cnt++]=BYTE0(mag_raw[2]);

	for(i=0;i<_cnt;i++)
	{
		sum+= data_to_send[i];
		usart_send_ch(data_to_send[i]);
	}
	usart_send_ch(sum);
}


void uart_ano_send_user_data1_v6(ano_user_data_e index, short data)
{
	unsigned char i=0;
	unsigned char _cnt=0;
	unsigned char sum = 0;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=(unsigned char)index;
	data_to_send[_cnt++]=sizeof(short);

	data_to_send[_cnt++]=BYTE1(data);
	data_to_send[_cnt++]=BYTE0(data);

	data_to_send[4] = _cnt-5;

	for(i=0;i<_cnt;i++)
	{
		sum+= data_to_send[i];
		usart_send_ch(data_to_send[i]);
	}
	usart_send_ch(sum);
}

void uart_ano_send_user_data3_v6(ano_user_data_e index, short data[3])
{
	unsigned char i=0;
	unsigned char _cnt=0;
	unsigned char sum = 0;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=(unsigned char)index;
	data_to_send[_cnt++]=sizeof(short)*3;

	data_to_send[_cnt++]=BYTE1(data[0]);
	data_to_send[_cnt++]=BYTE0(data[0]);
	data_to_send[_cnt++]=BYTE1(data[1]);
	data_to_send[_cnt++]=BYTE0(data[1]);
	data_to_send[_cnt++]=BYTE1(data[2]);
	data_to_send[_cnt++]=BYTE0(data[2]);
	data_to_send[4] = _cnt-5;

	for(i=0;i<_cnt;i++)
	{
		sum+= data_to_send[i];
		usart_send_ch(data_to_send[i]);
	}
	usart_send_ch(sum);

}

void uart_ano_send_status_v6(float *Euler, float Alt)
{
	unsigned char i=0;
	unsigned char _cnt=0;
	unsigned char sum = 0;
	short pitch, roll, yaw;
	int	Altitude;

	if(Euler[0] > 180.0f)
	{
		Euler[0] = Euler[0]-360.0f;
	}

	pitch = (short)Euler[1]*100;
	roll = (short)Euler[2]*100;
	yaw = (short)Euler[0]*100;
	Altitude = Alt*100;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=12;
	
	data_to_send[_cnt++]=BYTE1(pitch);
	data_to_send[_cnt++]=BYTE0(pitch);
	data_to_send[_cnt++]=BYTE1(roll);
	data_to_send[_cnt++]=BYTE0(roll);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);

	data_to_send[_cnt++]=BYTE3(Altitude);
	data_to_send[_cnt++]=BYTE2(Altitude);
	data_to_send[_cnt++]=BYTE1(Altitude);
	data_to_send[_cnt++]=BYTE0(Altitude);
	
	data_to_send[_cnt++]=0;		// FLY_MODEL
	data_to_send[_cnt++]=1;		// ARMED
	
	for(i=0;i<_cnt;i++)
	{
		sum+= data_to_send[i];
		usart_send_ch(data_to_send[i]);
	}
	usart_send_ch(sum);
}


void uart_ano_send_power(unsigned short votage, unsigned short current)
{
    unsigned char _cnt=0;
    unsigned short temp;
	unsigned char i=0;
    
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x05;
    data_to_send[_cnt++]=0;
    
    temp = votage;
    data_to_send[_cnt++]=BYTE1(temp);
    data_to_send[_cnt++]=BYTE0(temp);
    temp = current;
    data_to_send[_cnt++]=BYTE1(temp);
    data_to_send[_cnt++]=BYTE0(temp);
    
    data_to_send[3] = _cnt-4;
    
    uint8_t sum = 0;
    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    
    data_to_send[_cnt++]=sum;
    
	for(i=0;i<_cnt;i++)
		usart_send_ch(data_to_send[i]);
}


void uart_ano_send_version(unsigned char hardware_type, unsigned short hardware_ver,unsigned short software_ver,unsigned short protocol_ver,unsigned short bootloader_ver)
{
	unsigned char i=0;
	unsigned char _cnt=0;
	unsigned char sum = 0;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);
	
	data_to_send[3] = _cnt-4;
	
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	for(i=0;i<_cnt;i++)
		usart_send_ch(data_to_send[i]);
}


