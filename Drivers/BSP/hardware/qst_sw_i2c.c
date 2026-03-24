/**
  ******************************************************************************
  * @file    qmaX981_sw_i2c.c
  * @author  yzq
  * @version V1.0
  * @date    2018-05-01
  * @brief   ??IIC ??
  ******************************************************************************
  */

#include "qst_sw_i2c.h"
#define QST_MULTIPLE_SW_IIC

typedef struct
{
	GPIO_TypeDef*	port_scl;
	GPIO_TypeDef*	port_sda;
	uint16_t		pin_scl;
	uint16_t		pin_sda;
} qst_sw_gpio_t;

static qst_sw_gpio_t iic = {GPIOB, GPIOB, GPIO_PIN_10, GPIO_PIN_13};

const qst_sw_gpio_t iic_array[] = 
{
//	{GPIOB, GPIOB, GPIO_PIN_10, GPIO_PIN_13},
	{GPIOB, GPIOB, GPIO_PIN_6, GPIO_PIN_7},
#if 1
	{GPIOA, GPIOA, GPIO_PIN_5, GPIO_PIN_6},
	{GPIOA, GPIOC, GPIO_PIN_7, GPIO_PIN_9},
	{GPIOC, GPIOC, GPIO_PIN_6, GPIO_PIN_7},
	{GPIOA, GPIOB, GPIO_PIN_8, GPIO_PIN_10},
#endif
#if 0
	{GPIOA, GPIOA, GPIO_PIN_0, GPIO_PIN_1},
	{GPIOA, GPIOB, GPIO_PIN_2, GPIO_PIN_0},
	{GPIOC, GPIOC, GPIO_PIN_1, GPIO_PIN_0},
#endif
};

#define I2C_SCL_OUTPUT()
#define I2C_SCL_INPUT()
#define I2C_SDA_OUTPUT()
#define I2C_SDA_INPUT()
#define I2C_SCL_1()			HAL_GPIO_WritePin(iic.port_scl, iic.pin_scl, GPIO_PIN_SET)					/* SCL = 1 */
#define I2C_SCL_0()			HAL_GPIO_WritePin(iic.port_scl, iic.pin_scl, GPIO_PIN_RESET)				/* SCL = 0 */
#define I2C_SDA_1()			HAL_GPIO_WritePin(iic.port_sda, iic.pin_sda, GPIO_PIN_SET)					/* SDA = 1 */
#define I2C_SDA_0()			HAL_GPIO_WritePin(iic.port_sda, iic.pin_sda, GPIO_PIN_RESET)				/* SDA = 0 */
#define I2C_SDA_READ()		HAL_GPIO_ReadPin(iic.port_sda, iic.pin_sda)		/* read SDA */

#if 0
//#define SDA_IN()	{GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=8<<4;}				// GPIO8
//#define SDA_OUT() {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=3<<4;}				// GPIO9

//#define SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=((uint32_t)8<<28);}		// GPIO6
//#define SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=((uint32_t)3<<28);}	// GPIO7

#if defined(USE_I2C_1_PB6_7)
#define I2C_SCL_INPUT()			//{GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=((uint32_t)8<<24);}
#define I2C_SCL_OUTPUT()		//{GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=((uint32_t)3<<24);}
#define I2C_SDA_INPUT()			{GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=((uint32_t)8<<28);}	
#define I2C_SDA_OUTPUT() 		{GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=((uint32_t)3<<28);}	
#endif
#if defined(USE_I2C_1_PB8_9)
#define I2C_SCL_INPUT()	
#define I2C_SCL_OUTPUT()		
#define I2C_SDA_INPUT()			{GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=((uint32_t)8<<4);}	
#define I2C_SDA_OUTPUT() 		{GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=((uint32_t)3<<4);}
#endif
#define I2C_SCL_1()				GPIO_PORT_I2C->BSRR = I2C_SCL_PIN				/* SCL = 1 */
#define I2C_SCL_0()				GPIO_PORT_I2C->BRR = I2C_SCL_PIN				/* SCL = 0 */
#define I2C_SDA_1()				GPIO_PORT_I2C->BSRR = I2C_SDA_PIN				/* SDA = 1 */
#define I2C_SDA_0()				GPIO_PORT_I2C->BRR = I2C_SDA_PIN				/* SDA = 0 */
#define I2C_SDA_READ()			((GPIO_PORT_I2C->IDR & I2C_SDA_PIN) != 0)	/* ?SDA???? */
#endif

static void i2c_Ack(void);
static void i2c_NAck(void);

static void i2c_delay_us(int nus)
{
	//volatile uint32_t i;

	while(nus--)
	{
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
#if 0
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();


		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

#endif

	}
}


#define i2c_Delay()		i2c_delay_us(2)


/********************************************************************************************************
*	func: i2c_Start
*	brief: IIC start condition
*	parameter: null
*	
********************************************************************************************************/
static void i2c_Start(void)
{
	/* SCL high,SDA from high to low */
	I2C_SCL_OUTPUT();
	I2C_SDA_OUTPUT();
	
	I2C_SDA_1();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_0();
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
}

/********************************************************************************************************
*	func: i2c_Stop
*	brief: IIC stop condition
*	parameter: null
*	
********************************************************************************************************/
static void i2c_Stop(void)
{
	I2C_SCL_OUTPUT();
	I2C_SDA_OUTPUT();
	I2C_SCL_0(); // Junger
	i2c_Delay(); //Junger

	/* ?SCL????,SDA?????????I2C?????? */
	I2C_SDA_0();
	i2c_Delay();  //Junger
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_1();
}


/********************************************************************************************************
*	func: i2c_SendByte
*	brief: send one byte to IIC bus
*	parameter: byte to send(uint8)
*	
********************************************************************************************************/
static void i2c_SendByte(uint8_t _ucByte)
{
	uint8_t i;

	/* ????????bit7 */
	for (i = 0; i < 8; i++)
	{		
		if (_ucByte & 0x80)
		{
			I2C_SDA_1();
		}
		else
		{
			I2C_SDA_0();
		}
		i2c_Delay();
		I2C_SCL_1();
		i2c_Delay();	
		I2C_SCL_0();
		if (i == 7)
		{
			 I2C_SDA_1();
		}
		_ucByte <<= 1;
		i2c_Delay();
	}
}


/********************************************************************************************************
*	func: i2c_ReadByte
*	brief: read one byte to IIC bus
*	parameter: NULL
*	return:	one byte from IIC
********************************************************************************************************/
static uint8_t i2c_ReadByte(uint8_t ack)
{
	uint8_t i;
	uint8_t value;

	/* MSB form bit7 to bit0 */
	I2C_SDA_INPUT();	// set data input	
	i2c_Delay();
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C_SCL_1();
		i2c_Delay();
		if (I2C_SDA_READ())
		{
			value++;
		}
		//I2C_SCL_1();
		//i2c_Delay();
		I2C_SCL_0();
		i2c_Delay();
	}
	
	I2C_SDA_OUTPUT();	// set data output	
	i2c_Delay();
	if(ack==0)
		i2c_NAck();
	else
		i2c_Ack();
	return value;
}

/********************************************************************************************************
*	func: i2c_WaitAck
*	brief: after send one byte, wait slave to ACK
*	parameter: NULL
*	return:	1:OK	0:FAIL
********************************************************************************************************/
static uint8_t i2c_WaitAck(void)
{
	uint8_t re;

	I2C_SDA_1();	/* SDA = 1 */
	I2C_SDA_INPUT();	//set data input
	i2c_Delay();
	I2C_SCL_1();	/* SCL = 1*/
	i2c_Delay();
	if (I2C_SDA_READ())	/* read SDA */
	{
		re = 1;
		//I2C_SDA_OUTPUT();	//set data output
//		I2C_SCL_1();
//		i2c_Delay();
//		I2C_SDA_1();
	}
	else
	{
		re = 0;
		I2C_SCL_0();
		I2C_SDA_OUTPUT();	//set data output
		i2c_Delay();
	}

	return re;
}

/*
*********************************************************************************************************
*	i2c_Ack
*********************************************************************************************************
*/
static void i2c_Ack(void)
{
	I2C_SDA_0();	/* SDA = 0 */
	i2c_Delay();
	I2C_SCL_1();	/* SCL = 1 */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
	I2C_SDA_1();	/* SDA = 1 */
}

/*
*********************************************************************************************************
*	 i2c_NAck
*********************************************************************************************************
*/
static void i2c_NAck(void)
{
	I2C_SDA_1();	/* SDA = 1 */
	i2c_Delay();
	I2C_SCL_1();	/*  */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();	
}

/*
*********************************************************************************************************
*	i2c_sw_gpio_config
*********************************************************************************************************
*/
void i2c_sw_gpio_config(uint16_t id)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	uint16_t array_len = sizeof(iic_array)/sizeof(iic_array[0]);

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	if(id >= array_len)
	{
		id = 0;
	}
//	for(i=0; i<array_len; i++)
//	{	
		GPIO_InitStruct.Pin = iic_array[id].pin_scl;
#if 0
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
#else
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
#endif
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(iic_array[id].port_scl, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = iic_array[id].pin_sda;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(iic_array[id].port_sda, &GPIO_InitStruct);
//	}

	iic.port_scl = iic_array[id].port_scl;
	iic.port_sda = iic_array[id].port_sda;
	iic.pin_scl = iic_array[id].pin_scl;
	iic.pin_sda = iic_array[id].pin_sda;

	i2c_Stop();
}

uint8_t i2c_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;	

	i2c_Start();
	i2c_SendByte(_Address);
	ucAck = i2c_WaitAck();
	i2c_Stop();

	return ucAck;
}

void qst_sw_sel_i2c(uint16_t id)
{
	uint16_t array_len = sizeof(iic_array)/sizeof(iic_array[0]);

	if(id >= array_len)
	{
		id = 0;
	}
	
	iic.port_scl = iic_array[id].port_scl;
	iic.port_sda = iic_array[id].port_sda;
	iic.pin_scl = iic_array[id].pin_scl;
	iic.pin_sda = iic_array[id].pin_sda;
}

void qst_sw_i2c_reset_slave(void)
{
	uint8_t i;
	i2c_Start();
	/* 先发送字节的高位bit7 */
	for (i = 0; i < 9; i++)
	{		
		I2C_SDA_0();
		i2c_Delay();


		I2C_SCL_1();
		i2c_Delay();

		I2C_SCL_0();
		i2c_Delay();

	}
	i2c_Stop();
}

uint8_t qst_sw_writereg(uint8_t slave, uint8_t reg_add,uint8_t reg_dat)
{
	i2c_Start();
	i2c_SendByte(slave);
	if(i2c_WaitAck())
	{
		I2C_SCL_1();
		i2c_Delay();
		I2C_SDA_1();

		return 0;
	}
	i2c_SendByte(reg_add);	
	if(i2c_WaitAck())
	{
		I2C_SCL_1();
		i2c_Delay();
		I2C_SDA_1();

		return 0;
	}
	i2c_SendByte(reg_dat);	
	if(i2c_WaitAck())
	{
		I2C_SCL_1();
		i2c_Delay();
		I2C_SDA_1();

		return 0;
	}
	i2c_Stop();

	return 1;
}


/********************************************************************************************************
*	func: qst_sw_writeregs
*	brief: write values to registers
*	parameter: slave:slave address 8bit / rea_addr: register address / reg_dat: buf to write / len: buf len
*	return: 1:OK	0:FAIL
********************************************************************************************************/
uint8_t qst_sw_writeregs(uint8_t slave, uint8_t reg_add, uint8_t *reg_dat, uint16_t len)
{
	uint16_t i;

	i2c_Start();
	i2c_SendByte(slave);
	if(i2c_WaitAck())
	{
		I2C_SCL_1();
		i2c_Delay();
		I2C_SDA_1();

		return 0;
	}
	i2c_SendByte(reg_add);	
	if(i2c_WaitAck())
	{
		I2C_SCL_1();
		i2c_Delay();
		I2C_SDA_1();

		return 0;
	}
	for(i=0; i<len; i++)
	{
		i2c_SendByte(reg_dat[i]);
		if(i2c_WaitAck())
		{
			I2C_SCL_1();
			i2c_Delay();
			I2C_SDA_1();

			return 0;
		}
	}
	i2c_Stop();

	return 1;
}


/********************************************************************************************************
*	func: qst_sw_writeregs
*	brief: write values to registers
*	parameter: slave:slave address 8bit / rea_addr: register address / buf: buf to read / len: buf len
*	return: 1:OK	0:FAIL
********************************************************************************************************/
uint8_t qst_sw_readreg(uint8_t slave, uint8_t reg_add, uint8_t *buf, uint16_t num)
{
	uint16_t i;

	i2c_Start();
	i2c_SendByte(slave);
	if(i2c_WaitAck())
	{
		i2c_Stop();
		I2C_SCL_1();
		i2c_Delay();
		I2C_SDA_1();
		return 0;
	}
	i2c_SendByte(reg_add);
	if(i2c_WaitAck())
	{
		i2c_Stop();
		I2C_SCL_1();
		i2c_Delay();
		I2C_SDA_1();
		return 0;
	}

	i2c_Start();
	i2c_SendByte(slave|0x01);
	if(i2c_WaitAck())
	{
		i2c_Stop();
		I2C_SCL_1();
		i2c_Delay();
		I2C_SDA_1();
		return 0;
	}

	for(i=0;i<(num-1);i++){
		*buf=i2c_ReadByte(1);
		buf++;
	}
	*buf=i2c_ReadByte(0);
	i2c_Stop();

	return 1;
}


uint8_t qst_iic_tx(uint8_t slave, uint8_t *buf, uint16_t num)
{
	uint16_t i;

	i2c_Start();
	i2c_SendByte(slave);
	if(i2c_WaitAck())
	{
		return 0;
	}

	for(i=0; i<num; i++)
	{
		i2c_SendByte(buf[i]);
		if(i2c_WaitAck())
		{
			return 0;
		}
	}
	i2c_Stop();

	return 1;

}

uint8_t qst_iic_rx(uint8_t slave, uint8_t *buf, uint16_t num)
{
	uint16_t i;

	i2c_Start();
	i2c_SendByte(slave|0x01);
	if(i2c_WaitAck())
	{
		return 0;
	}

	for(i=0;i<(num-1);i++){
		*buf=i2c_ReadByte(1);
		buf++;
	}
	*buf=i2c_ReadByte(0);
	i2c_Stop();

	return 1;
}


