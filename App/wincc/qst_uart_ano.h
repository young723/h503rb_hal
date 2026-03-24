
#ifndef __QST_UART_ANO_H__
#define __QST_UART_ANO_H__

typedef struct
{
	short acc_raw[3];
	short gyr_raw[3];
	short mag_raw[3];
	short user_raw[3];
} uart_ano_tc_t;

typedef enum {
    PACKET_DATA_ACCEL = 0,
    PACKET_DATA_GYRO,
    PACKET_DATA_COMPASS,
    PACKET_DATA_QUAT,
    PACKET_DATA_EULER,
    PACKET_DATA_ROT,
    PACKET_DATA_HEADING,
    PACKET_DATA_LINEAR_ACCEL,
    NUM_DATA_PACKETS
} qst_packet_e;

typedef enum {
    USER_DATA_1 = 0xf1,
	USER_DATA_2 = 0xf2,
	USER_DATA_3 = 0xf3,
	USER_DATA_4 = 0xf4,
	USER_DATA_5 = 0xf5,
	USER_DATA_6 = 0xf6,
	USER_DATA_7 = 0xf7,
	USER_DATA_8 = 0xf8,
	USER_DATA_9 = 0xf9,
	USER_DATA_10 = 0xfa,
	USER_DATA_11 = 0xfb,
	
    USER_DATA_TOTAL
} ano_user_data_e;

#define QST_LOG_UNKNOWN		(0)
#define QST_LOG_DEFAULT		(1)
#define QST_LOG_VERBOSE		(2)
#define QST_LOG_DEBUG		(3)
#define QST_LOG_INFO		(4)
#define QST_LOG_WARN		(5)
#define QST_LOG_ERROR		(6)
#define QST_LOG_SILENT		(8)

#define BUF_SIZE					(256)
#define PACKET_LENGTH				(23)
#define PACKET_DEBUG				(1)
#define PACKET_QUAT					(2)
#define PACKET_DATA					(3)
#define min(a,b)					((a < b) ? a : b)
#define BYTE0(dwTemp)				(*(char *)(&dwTemp))
#define BYTE1(dwTemp)				(*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)				(*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)				(*((char *)(&dwTemp) + 3))

#ifdef __cplusplus
extern "C" {
#endif

void uart_ano_send_data(unsigned char type, long *data);
void uart_ano_send_quat(long *quat);
void uart_ano_send_euler(float angle_pit, float angle_rol, float angle_yaw, int alt, unsigned char fly_model, unsigned char armed);
void usart_ano_send_rawdata1(short *Gyro,short *Accel, short *Mag);
void usart_ano_send_rawdata2(int alt_bar, unsigned short alt_csb);
void uart_ano_send_senosr_data(float *euler, float *Gyro, float *Accel, float *Mag, float Press);
void uart_ano_send_power(unsigned short votage, unsigned short current);
void uart_ano_send_version(unsigned char hardware_type, unsigned short hardware_ver,unsigned short software_ver,unsigned short protocol_ver,unsigned short bootloader_ver);

void uart_ano_send_senosr_data_v6(float *Accel, float *Gyro, float *Mag);
void uart_ano_send_status_v6(float *Euler, float Alt);

void uart_ano_send_user_data1_v6(ano_user_data_e index, short data);
void uart_ano_send_user_data3_v6(ano_user_data_e index, short data[3]);

#ifdef __cplusplus
}
#endif


#endif /* __QST_UART_ANO_H__ */
