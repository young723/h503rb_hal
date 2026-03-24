

#ifndef __BSP_HARDWARE_H
#define	__BSP_HARDWARE_H

#include "stdio.h"
#include "stdlib.h"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_nucleo.h"
#include "qst_log.h"

#define QST_ABS(X) 				((X) < 0 ? (-1 * (X)) : (X))
#define QST_FABS(X) 			((X) < 0.0f ? (-1.0f * (X)) : (X))

//#define BSP_UART2_SUPPORT
//#define BSP_TIM6_SUPPORT
//#define BSP_TIM7_SUPPORT

typedef void (*int_callback)(void);
typedef void (*usart_callback)(unsigned char *str, unsigned short len);
typedef void (*write_reg)(unsigned char slave, unsigned char reg, unsigned char value);
typedef void (*read_reg)(unsigned char dummy, unsigned char reg, uint8_t* buff, unsigned short len);


typedef enum
{
	INTERFACE_USER_SEL = -1,
	INTERFACE_I2C_SW = 0,
	INTERFACE_I2C_HW,
	INTERFACE_I2C_HW_1M,
	INTERFACE_I3C_4M,
	INTERFACE_I3C_6_25M,
	INTERFACE_I3C_10M,
	INTERFACE_I3C_12_5M,
	INTERFACE_SPI_HW4,
	INTERFACE_SPI_HW3,
	INTERFACE_SPI_SW4,
	INTERFACE_SPI_SW3,

	INTERFACE_TOTAL
} evb_interface_e;

typedef enum
{
	EVB_SPI_MODE0,
	EVB_SPI_MODE1,
	EVB_SPI_MODE2,
	EVB_SPI_MODE3

} evb_spi_mode_e;

typedef struct
{
	int					i2c_type;
	int					i3c_type;
	int					spi_type;
	evb_spi_mode_e		spi_mode;
} evb_port_t;

enum 
{
	QST_REPORT_OFF = 0x0000,
	QST_REPORT_POLLING = 0x0001,
	QST_REPORT_DRI  = 0x0002,
	QST_REPORT_FIFO = 0x0004,
	QST_REPORT_SELFTEST = 0x0008,
	QST_REPORT_EXT_INT = 0x0010,

	QST_REPORT_END
};

typedef enum
{
	QST_NUCLEO_INT_NONE = 0x00,
	QST_NUCLEOPC7_SENSOR_INT1 = 0x01,
	QST_NUCLEOPA8_SENSOR_INT2 = 0x02,
	QST_NUCLEOPC13_KEY_INT = 0x10,
	QST_NUCLEO_INT_ALL = 0xff
}qst_nucleo_int;

typedef struct bsp_tim_func_t
{
	unsigned char		ingore_count;
	unsigned char		tim1_flag;
	unsigned char		tim2_flag;
	unsigned char		tim3_flag;
//	unsigned char		tim4_flag;
//	unsigned char		tim5_flag;
#ifdef BSP_TIM6_SUPPORT
	unsigned char		tim6_flag;
#endif
#ifdef BSP_TIM7_SUPPORT
	unsigned char		tim7_flag;
#endif
	int_callback		tim1_func;
	int_callback		tim2_func;
	int_callback		tim3_func;
//	int_callback		tim4_func;
//	int_callback		tim5_func;
#ifdef BSP_TIM6_SUPPORT
	int_callback		tim6_func;
#endif
#ifdef BSP_TIM7_SUPPORT
	int_callback		tim7_func;
#endif
}bsp_tim_func_t;

typedef struct bsp_irq_func_t
{
	unsigned char		irq1_flag;
	unsigned char		irq2_flag;
	int_callback		irq1_func;
	int_callback		irq2_func;
}bsp_irq_func_t;

typedef struct bsp_usart_rx_t
{
	unsigned char		rx_it[2];
	unsigned char		rx_buf[32];
	unsigned char		rx_len;
	unsigned int		rx_cplt_count;
	usart_callback		rx_cbk;	
	unsigned int		rx_delay_count;
}bsp_usart_rx_t;

typedef struct bsp_key_func_t
{
	unsigned char		key1_flag;
	int_callback		key1_func;
	unsigned char		key2_flag;
	int_callback		key2_func;
}bsp_key_func_t;

typedef struct bsp_ibi_func_t
{
	unsigned char		ibi_req;
	int_callback		ibi_func;
}bsp_ibi_func_t;

extern void bsp_event_clear(void);
extern void bsp_hardware_init(void);
extern void evb_setup_irq(int int_type, int_callback func, FunctionalState enable);
extern void evb_setup_timer(TIM_TypeDef *tim_id, int_callback func, uint16_t ms, FunctionalState enable);
extern void evb_setup_uart_rx(USART_TypeDef* USARTx, usart_callback func);
extern void evb_setup_user_key(int id, int_callback func);

extern void evb_irq_handle(void);
extern void evb_tim_handle(void);
extern void evb_usart_rx_handle(void);
extern void evb_key_handle(void);

extern void qst_delay_ms(unsigned int delay);
extern void qst_delay_us(unsigned int delay);
extern void SysTick_Enable(unsigned char enable);
extern void usart_send_ch(uint8_t ch);

extern void Error_Handler(void);
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line);
#endif

void MX_USART2_UART_DeInit(void);
void bsp_power_pin_set(int on);

void bsp_port_i2c_init(evb_interface_e type);
void bsp_port_i2c_deinit(void);
void bsp_port_i3c_init(evb_interface_e type);
void bsp_port_i3c_deinit(void);
void bsp_port_spi_init(evb_interface_e type, int mode);
extern int bsp_i2c_write_reg(unsigned char slave, unsigned char reg, unsigned char value);
extern int bsp_i2c_read_reg(unsigned char slave, unsigned char reg, uint8_t* buff, unsigned short len);
extern int bsp_i3c_write_reg(unsigned char reg, unsigned char value);
extern int bsp_i3c_read_reg(unsigned char reg, uint8_t* buff, unsigned short len);
extern int bsp_spi_write_reg(unsigned char reg, unsigned char value);
extern int bsp_spi_read_reg(unsigned char reg, uint8_t* buff, unsigned short len);
extern int bsp_spi_write_ext(unsigned char dummy, unsigned char reg, unsigned char value);
extern int bsp_spi_read_ext(unsigned char dummy, unsigned char reg, uint8_t* buff, unsigned short len);
extern int bsp_i3c_write_ext(unsigned char dummy, unsigned char reg, unsigned char value);
extern int bsp_i3c_read_ext(unsigned char dummy, unsigned char reg, uint8_t* buff, unsigned short len);

extern void bsp_entry_sel_interface(int *intf);

#define QST_EVB_NONE		QST_NUCLEO_INT_NONE
#if 1
#define QST_EVB_INT1		QST_NUCLEOPC7_SENSOR_INT1
#define QST_EVB_INT2		QST_NUCLEOPA8_SENSOR_INT2
#else
#define QST_EVB_INT1		QST_NUCLEOPA9_IMU_INT2
#define QST_EVB_INT2		QST_NUCLEOPC7_IMU_INT1
#endif

#endif
