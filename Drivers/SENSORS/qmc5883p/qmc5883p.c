/**
  ******************************************************************************
  * @file    qmc5883p.c
  * @author  QST
  * @version V1.0
  * @date    2021-xx-xx
  * @brief    qmc5883pÇý¶¯
  *****************************************************************************
  */ 

#include "qmc5883p.h"

#define QMC5883P_LOG	qst_logi

static int qmc5883p_read_block(uint8_t addr, uint8_t *buf, uint32_t len)
{
	return bsp_i2c_read_reg(QMC5883P_IIC_ADDR, addr, buf, len);
}

static int qmc5883p_write_reg(uint8_t addr, uint8_t buf)
{
	return bsp_i2c_write_reg(QMC5883P_IIC_ADDR, addr, buf);
}

void qmc5883p_delay(unsigned int ms)
{
	extern void qst_delay_ms(unsigned int n_ms);

	qst_delay_ms(ms);
}

void qmc5883p_dump(void)
{
	unsigned char ctrl_value;
	
	qmc5883p_read_block(0x0d, &ctrl_value, 1);
	QMC5883P_LOG("QMC5883P 0x%x=0x%x \r\n", 0x0d, ctrl_value);//0x40
	qmc5883p_read_block(0x29, &ctrl_value, 1);
	QMC5883P_LOG("QMC5883P 0x%x=0x%x \r\n", 0x29, ctrl_value);//0x06
	qmc5883p_read_block(QMC5883P_CTL_REG_ONE, &ctrl_value, 1);
	QMC5883P_LOG("QMC5883P 0x%x=0x%x \r\n", QMC5883P_CTL_REG_ONE, ctrl_value);//0xc3
	qmc5883p_read_block(QMC5883P_CTL_REG_TWO, &ctrl_value, 1);
	QMC5883P_LOG("QMC5883P 0x%x=0x%x \r\n", QMC5883P_CTL_REG_TWO, ctrl_value);//0x00
}

int qmc5883p_init(void)
{
	uint8_t chipid = 0;
	int ret = 0;

	ret = qmc5883p_read_block(QMC5883P_CHIP_ID_REG, &chipid , 1);
	if(!ret)
	{
		QMC5883P_LOG("%s: QMC5883P_get_chipid failed\n",__func__);
		return 0;
	}
	QMC5883P_LOG("QMC5883P_get_chipid chipid = 0x%x,i2c_addr = 0x2c\n", chipid);
	
	if(chipid == 0x80)
	{
		qmc5883p_write_reg(0x0d, 0x40);
		qmc5883p_delay(1);
		qmc5883p_write_reg(0x29, 0x06);
		qmc5883p_delay(1);
		qmc5883p_write_reg(QMC5883P_CTL_REG_ONE, 0xC3);//0x0A = 0xC3/0xC7/0xCB/0xCF; 	/*ODR = 10/50/100/200Hz,  MODE = continuous*/
		qmc5883p_delay(1);
		qmc5883p_write_reg(QMC5883P_CTL_REG_TWO, 0x00); //0x0B = 0x00/0x04/0x08/0x0C; 	/*RNG = ¡À 30G / ¡À 12G / ¡À 8G / ¡À 2G */
		qmc5883p_delay(1);

		return 1;
	}
	else
	{
		QMC5883P_LOG("%s: QMC5883P_get_chipid failed\n",__func__);
		return 0;
	}
}

int qmc5883p_read_mag_xyz(float *data)
{
	int res;
	unsigned char mag_data[6];
	short hw_d[3] = {0};
	short raw_c[3];
	int t1 = 0;
	unsigned char rdy = 0;
	t1 = 0;
//	t1_printf = 0;
	/* Check status register for data availability */
	while(!(rdy & 0x01) && (t1 < 5))
	{
		rdy = QMC5883P_STATUS_REG;
		res = qmc5883p_read_block(QMC5883P_STATUS_REG, &rdy, 1);
		t1++;
//		t1_printf = t1;
	}

	mag_data[0] = QMC5883P_DATA_OUT_X_LSB_REG;

	res = qmc5883p_read_block(QMC5883P_DATA_OUT_X_LSB_REG, mag_data, 6);
	if(!res)
	{
		return 0;
	}

	hw_d[0] = (short)(((mag_data[1]) << 8) | mag_data[0]);
	hw_d[1] = (short)(((mag_data[3]) << 8) | mag_data[2]);
	hw_d[2] = (short)(((mag_data[5]) << 8) | mag_data[4]);

	//Unit:mG  1G = 100uT = 1000mG
	//QMC5883P_LOG("Hx=%d, Hy=%d, Hz=%d\n",hw_d[0],hw_d[1],hw_d[2]);
	raw_c[0] = (int)(hw_d[0]);
	raw_c[1] = (int)(hw_d[1]);
	raw_c[2] = (int)(hw_d[2]);
	//If Range = ¡À2G, 15000LSB/G	If Range = ¡À 8G, 3750LSB/G 
	//If Range = ¡À12G, 2500LSB/G	If Range = ¡À 30G, 1000LSB/G
	data[0] = (float)raw_c[0] / 10.0f;
	data[1] = (float)raw_c[1] / 10.0f;
	data[2] = (float)raw_c[2] / 10.0f;

	return 1;
}

int qmc5883p_self_test(void)
{
	int selftest_result = 0;
	int selftest_retry = 0;
	int hdata_a[3];
	int hdata_b[3];
	int hdata[3];
	unsigned char rx_buf[6] = {0};
	unsigned char rdy = 0x00;
	int t1 = 0;
//	int ret = 0;

	while((selftest_result == 0)&&(selftest_retry<3))
	{
		selftest_retry++;

		qmc5883p_write_reg(QMC5883P_CTL_REG_ONE, 0x00);
		qmc5883p_write_reg(QMC5883P_CTL_REG_TWO, 0x00);
		qmc5883p_write_reg(QMC5883P_CTL_REG_ONE, 0x03);
		qmc5883p_delay(1);
	
		while(!(rdy & 0x03))
		{
			rdy = QMC5883P_STATUS_REG;
			qmc5883p_read_block(QMC5883P_STATUS_REG, &rdy, 1);
			qmc5883p_delay(1);
			if(t1++ > 50)
				continue;
		}
	
		qmc5883p_read_block(QMC5883P_DATA_OUT_X_LSB_REG, rx_buf, 6);
		//if(ret == 0)
		//	continue;
	
		hdata_a[0] = (short)(((rx_buf[1]) << 8) | rx_buf[0]);
		hdata_a[1] = (short)(((rx_buf[3]) << 8) | rx_buf[2]);
		hdata_a[2] = (short)(((rx_buf[5]) << 8) | rx_buf[4]);
	
		qmc5883p_write_reg(QMC5883P_CTL_REG_ONE, 0x00);
		qmc5883p_delay(1);
		qmc5883p_write_reg(0x0b, 0x40);
		qmc5883p_write_reg(QMC5883P_CTL_REG_ONE, 0x03);
		qmc5883p_delay(10);
		
		t1 = 0;
		rdy = 0;
		while(!(rdy & 0x03))
		{
			rdy = QMC5883P_STATUS_REG;
			qmc5883p_read_block(QMC5883P_STATUS_REG, &rdy, 1);
			qmc5883p_delay(1);
			if(t1++ > 50)
				continue;
		}
		qmc5883p_read_block(QMC5883P_DATA_OUT_X_LSB_REG, rx_buf, 6);
		//if(ret == 0)
		//	continue;

		hdata_b[0] = (short)(((rx_buf[1]) << 8) | rx_buf[0]);
		hdata_b[1] = (short)(((rx_buf[3]) << 8) | rx_buf[2]);
		hdata_b[2] = (short)(((rx_buf[5]) << 8) | rx_buf[4]);
	
		hdata[0] = QMC5883P_ABS(hdata_a[0]-hdata_b[0]);
		hdata[1] = QMC5883P_ABS(hdata_a[1]-hdata_b[1]);
		hdata[2] = QMC5883P_ABS(hdata_a[2]-hdata_b[2]);
	
		qmc5883p_write_reg(QMC5883P_CTL_REG_ONE, 0x00);
		
		QMC5883P_LOG("qmc5883p st %d	%d	%d\r\n", hdata[0], hdata[1], hdata[2]);

		if(
			((hdata[0] < QMC5883P_SELFTEST_MAX_X) && (hdata[0] > QMC5883P_SELFTEST_MIN_X))
		&&	((hdata[1] < QMC5883P_SELFTEST_MAX_Y) && (hdata[1] > QMC5883P_SELFTEST_MIN_Y))
		&&	((hdata[2] < QMC5883P_SELFTEST_MAX_Z) && (hdata[2] > QMC5883P_SELFTEST_MIN_Z))
			)
	    {
	        selftest_result = 1;
	    }
		else
		{
			selftest_result = 0;
		}
	}

	return selftest_result;
}


