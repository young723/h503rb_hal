

#ifndef __QST_UART_OWN_PROTOCOL_H__
#define __QST_UART_OWN_PROTOCOL_H__

// QSTÉÏÎ»»ú
#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	unsigned char	pc_buf[36];
	unsigned short	pc_len;
} qst_uart_protocol_t;

extern int qst_evb_send_own_protocol(int demo_sensor, float *data_p, float misc);
extern int qst_evb_send_own_info(unsigned int info[4]);
extern int qst_evb_send_own_num(unsigned int num);

#ifdef __cplusplus
}
#endif

#endif /* __QST_UART_PROTOCOL_H__ */
