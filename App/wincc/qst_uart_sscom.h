
#ifndef __QST_UART_SSCOM_H__
#define __QST_UART_SSCOM_H__

typedef struct plotsim
{
	short head;
	short len;
	short buf[3];
} plotsscom_t;

#ifdef __cplusplus
extern "C" {
#endif

void qst_evb_plot_sscom(int x, int y, int z);

#ifdef __cplusplus
}
#endif

#endif /* __QST_UART_SSCOM_H__ */
