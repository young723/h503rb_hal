
#include "qmc6308.h"

#define QMC6308_LOG		printf
#define QMC6308_CHECK_ERR(ret)	do {\
								if((ret) != QMC6308_OK)	\
								QMC6308_LOG("qmc6308 error:%d line:%d",ret, __LINE__);	\
							}while(0)

static qmc6308_data_t p_mag;

int qmc6308_read_block(unsigned char addr, unsigned char *data, unsigned char len)
{
	int ret = 0;
	int retry = 0;

	while((!ret) && (retry++ < 5))
	{
		ret = bsp_i2c_read_reg(p_mag.slave_addr, addr, data, len);
	}

	return ret;
}

int qmc6308_write_reg(unsigned char addr, unsigned char data)
{
	int ret = 0;
	int retry = 0;

	while((!ret) && (retry++ < 5))
	{
		ret = bsp_i2c_write_reg(p_mag.slave_addr, addr, data);
	}

	return ret;
}

void qmc6308_delay(unsigned int ms)
{
	extern void qst_delay_ms(unsigned int n_ms);

	qst_delay_ms(ms);
}

void qmc6308_axis_convert(float data[3], int layout)
{
	float raw[3];

	raw[0] = data[0];
	raw[1] = data[1];
	//raw[2] = data[2];
	if(layout >=4 && layout <= 7)
	{
		data[2] = -data[2];
	}
	//else
	//{
	//	data[2] = raw[2];
	//}

	if(layout%2)
	{
		data[0] = raw[1];
		data[1] = raw[0];
	}
	else
	{
		data[0] = raw[0];
		data[1] = raw[1];
	}

	if((layout==1)||(layout==2)||(layout==4)||(layout==7))
	{
		data[0] = -data[0];
	}
	if((layout==2)||(layout==3)||(layout==6)||(layout==7))
	{
		data[1] = -data[1];
	}
}


#if defined(QMC6308_MODE_SWITCH)
void qmc6308_setrst_auto_mode(short hw_d[3])
{
	if(p_mag.set_ctl.mode == 0)
	{// set reset on
		if((QMC6308_ABS(hw_d[0]) > 6000)||(QMC6308_ABS(hw_d[1]) > 8000))
		{
			p_mag.set_ctl.count++;
			if(p_mag.set_ctl.count >= 2)
			{
				qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0x00);
				qmc6308_config_mode(QMC6308_MODE_SUSPEND);
				qmc6308_config_setrst(QMC6308_SET_ON);
				qmc6308_write_reg(QMC6308_CTL_REG_ONE, p_mag.ctrl1.value);
				qmc6308_config_mode(QMC6308_MODE_CONTINUOUS);
				p_mag.set_ctl.mode = 1;
				p_mag.set_ctl.count = 0;
				QMC6308_LOG("qmc6308 mode switch to set only\r\n");
			}
		}
		else
		{
			p_mag.set_ctl.count = 0;
		}
	}
	else
	{// set only
		int force_switch = 0;
		p_mag.set_ctl.count++;
		//QMC6308_LOG("qmc6308 set mode(%d)\r\n", p_mag.set_ctl.count);
		if(p_mag.set_ctl.count >= 50)
		{
			p_mag.set_ctl.count = 0;
			force_switch = 1;
		}
		if(((QMC6308_ABS(hw_d[0]) < 4000)&&(QMC6308_ABS(hw_d[1]) < 6000)) || force_switch)
		{	
			qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0x00);
			qmc6308_config_mode(QMC6308_MODE_SUSPEND);
			qmc6308_config_setrst(QMC6308_SET_RESET_ON);			
			qmc6308_write_reg(QMC6308_CTL_REG_ONE, p_mag.ctrl1.value);
			qmc6308_config_mode(QMC6308_MODE_CONTINUOUS);
			p_mag.set_ctl.mode = 0;
			p_mag.set_ctl.count = 0;			
			QMC6308_LOG("qmc6308 mode switch to set reset force:%d\r\n",force_switch);
		}
	}

}
#endif

void qmc6308_get_chip_info(unsigned int *info)
{
	unsigned char verid = 0;
	unsigned char ctrl_value[4];
	
	qmc6308_read_block(0x37, ctrl_value, 3);
	qmc6308_read_block(0x12, &verid, 1);
	info[0] = (unsigned int)(((unsigned int)0x80<<24)|((unsigned int)ctrl_value[0]<<16)|((unsigned int)ctrl_value[2]<<8)|((unsigned int)ctrl_value[1]));		
	info[1] = verid;
}

int qmc6308_get_chipid(void)
{
	int ret = 0;
	int i;
	unsigned char chip_id = 0x00;

	for(i=0; i<10; i++)
	{
		ret = qmc6308_read_block(QMC6308_CHIP_ID_REG, &chip_id, 1);
		if(ret == QMC6308_OK)
		{
			break;
		}
	}
	if((chip_id == 0x80))	// || (chip_id & 0xc0)
	{
		QMC6308_LOG("qmc6308_get_chipid slave:0x%x chipid = 0x%x\r\n", p_mag.slave_addr, chip_id);
		p_mag.chip_type = QMC_6308;		
		return 1;
	}
	else
	{
		QMC6308_LOG("qmc6308_get_chipid fail slave:0x%x chipid = 0x%x\r\n", p_mag.slave_addr, chip_id);
		return 0;
	}
}

#if 0
int qmc6310_get_chipid(void)
{
	int ret = 0;
	int i;
	unsigned char chip_id = 0x00;

	QMC6308_LOG("qmc6310_get_chipid addr=0x%x\n",p_mag.slave_addr);
	chip_id = 0;
	for(i=0; i<10; i++)
	{
		ret = qmc6308_read_block(QMC6308_CHIP_ID_REG, &chip_id , 1);
		if(ret == QMC6308_OK)
		{
			break;
		}
	}
	if(chip_id == 0x80)
	{
		QMC6308_LOG("qmc6310_get_chipid chipid = 0x%x\n", chip_id);
		p_mag.chip_type = QMC_6310;
		return 1;
	}
	else
	{
		QMC6308_LOG("qmc6310_get_chipid fail chipid = 0x%x\n", chip_id);
		return 0;
	}
}
#endif

void qmc6308_init_para(void)
{
	p_mag.slave_addr = QMC6308_IIC_ADDR;
#if 1
	p_mag.ctrl1.bit.mode = QMC6308_MODE_CONTINUOUS;
	p_mag.ctrl1.bit.odr = 0;
	p_mag.ctrl1.bit.osr1 = QMC6308_OSR1_8;
	p_mag.ctrl1.bit.osr2 = QMC6308_OSR2_4;
#else
	p_mag.ctrl1.bit.mode = QMC6308_MODE_NORMAL;
	p_mag.ctrl1.bit.odr = QMC6308_ODR_200HZ;
	p_mag.ctrl1.bit.osr1 = QMC6308_OSR1_8;
	p_mag.ctrl1.bit.osr2 = QMC6308_OSR2_4;
#endif
	p_mag.ctrl2.bit.setrst = QMC6308_SET_RESET_ON;
	p_mag.ctrl2.bit.range = QMC6308_RNG_30G;
	p_mag.ctrl2.bit.selftest = 0;
	p_mag.ctrl2.bit.softrst = 0;

	switch(p_mag.ctrl2.bit.range)
	{
		case QMC6308_RNG_30G:
			p_mag.ssvt = 1000;
			break;
		case QMC6308_RNG_12G:
			p_mag.ssvt = 2500;
			break;
		case QMC6308_RNG_8G:
			p_mag.ssvt = 3750;
			break;
		case QMC6308_RNG_2G:
			p_mag.ssvt = 15000;
			break;
		default:
			p_mag.ssvt = 1000;
			break;			
	}
}

void qmc6308_soft_reset(void)
{
	int ret = 0;

	QMC6308_LOG("qmc6308_soft_reset \r\n");
	ret = qmc6308_write_reg(QMC6308_CTL_REG_TWO, 0x80);
	QMC6308_CHECK_ERR(ret);
	qmc6308_delay(10);
	ret = qmc6308_write_reg(QMC6308_CTL_REG_TWO, 0x00);
	QMC6308_CHECK_ERR(ret);
	qmc6308_delay(2);
}

int qmc6308_read_mag_xyz(float *data)
{
	int res = QMC6308_FAIL;
	unsigned char mag_data[6];
	short hw_d[3] = {0};
	int t1 = 0;
	unsigned char rdy = 0;

	//QMC6308_LOG("qmc6308 slave=0x%x\r\n", p_mag.slave_addr);

	/* Check status register for data availability */
	while(!(rdy & 0x01)&& (t1++ < 5))
	{
		res = qmc6308_read_block(QMC6308_STATUS_REG, &rdy, 1);
		qmc6308_delay(1);
	}
	if(!(rdy & 0x01))
	{
		res = qmc6308_read_block(QMC6308_CTL_REG_ONE, mag_data, 2);
		QMC6308_LOG("qmc6308_read_mag_xyz drdy fail! res=%d rdy=0x%x 0x0a=0x%x 0x0b=0x%x\r\n",res,rdy,mag_data[0],mag_data[1]);
		data[0] = p_mag.last_data[0];		// ut
		data[1] = p_mag.last_data[1];		// ut
		data[2] = p_mag.last_data[2];		// ut
		return QMC6308_FAIL;
	}

	mag_data[0] = QMC6308_DATA_OUT_X_LSB_REG;
	res = qmc6308_read_block(QMC6308_DATA_OUT_X_LSB_REG, mag_data, 6);
	if(res == QMC6308_FAIL)
  	{  	
		data[0] = p_mag.last_data[0];		// ut
		data[1] = p_mag.last_data[1];		// ut
		data[2] = p_mag.last_data[2];		// ut

		return QMC6308_FAIL;
	}

	hw_d[0] = (short)(((mag_data[1]) << 8) | mag_data[0]);
	hw_d[1] = (short)(((mag_data[3]) << 8) | mag_data[2]);
	hw_d[2] = (short)(((mag_data[5]) << 8) | mag_data[4]);
#if defined(QMC6308_MODE_SWITCH)
	qmc6308_setrst_auto_mode(hw_d);
#endif
	data[0] = (float)((float)hw_d[0] / ((float)p_mag.ssvt/100.f));		// ut
	data[1] = (float)((float)hw_d[1] / ((float)p_mag.ssvt/100.f));		// ut
	data[2] = (float)((float)hw_d[2] / ((float)p_mag.ssvt/100.f));		// ut

//	qmc6308_axis_convert(data, 0);
// for qmc6310
	if((p_mag.slave_addr == QMC6310_IIC_ADDR_U)||(p_mag.slave_addr == QMC6310_IIC_ADDR_N))
	{
		data[0] = -data[0];
	}
// for qmc6310	
	p_mag.last_data[0] = data[0];
	p_mag.last_data[1] = data[1];
	p_mag.last_data[2] = data[2];

	if(p_mag.ctrl1.bit.mode == QMC6308_MODE_SINGLE)
	{
		qmc6308_config_mode(QMC6308_MODE_SINGLE);
	}

	return res;
}


// set odr , mode,
int qmc6308_config_mode(unsigned char mode)
{	
	int err = 0;
	
	p_mag.ctrl1.bit.mode = mode;
	err = qmc6308_write_reg(QMC6308_CTL_REG_ONE, p_mag.ctrl1.value);
	QMC6308_CHECK_ERR(err);
	qmc6308_delay(1);

	return err;	
}

int qmc6308_config_odr(unsigned char odr)
{
	int err = 0;
	
	p_mag.ctrl1.bit.odr = odr;
	err = qmc6308_write_reg(QMC6308_CTL_REG_ONE, p_mag.ctrl1.value);
	QMC6308_CHECK_ERR(err);
	qmc6308_delay(1);

	return err;
}

// set range , set/reset
int qmc6308_config_range(unsigned char range)
{
	int err = 0;

	p_mag.ctrl2.bit.range = range;
	err = qmc6308_write_reg(QMC6308_CTL_REG_TWO, p_mag.ctrl2.value);
	QMC6308_CHECK_ERR(err);
	//qmc6308_delay(1);

	return err;	
}

int qmc6308_config_setrst(unsigned char setrst)
{
	int err = 0;

	p_mag.ctrl2.bit.setrst = setrst;
	err = qmc6308_write_reg(QMC6308_CTL_REG_TWO, p_mag.ctrl2.value);
	QMC6308_CHECK_ERR(err);
	//qmc6308_delay(1);

	return err;	
}

int qmc6308_config_selftest(unsigned char selftest)
{
	int err = 0;

	if(selftest)
		p_mag.ctrl2.bit.selftest = 1;
	else
		p_mag.ctrl2.bit.selftest = 0;
	
	err = qmc6308_write_reg(QMC6308_CTL_REG_TWO, p_mag.ctrl2.value);
	QMC6308_CHECK_ERR(err);
	return err;
}


int qmc6308_enable(int en)
{
	int ret = 0;

	if(en)
	{
		ret = qmc6308_write_reg(QMC6308_CTL_REG_THREE, 0x40);
		QMC6308_CHECK_ERR(ret);
		qmc6308_delay(1);

		// reduce vdd drop
//		ret = qmc6308_write_reg(QMC6308_CTL_REG_THREE, 0x04);
//		QMC6308_CHECK_ERR(ret);
//		qmc6308_delay(1);
//		ret = qmc6308_write_reg(0x0f, 0x04);
//		QMC6308_CHECK_ERR(ret);
//		qmc6308_delay(1);
		
		ret = qmc6308_write_reg(QMC6308_CTL_REG_TWO, p_mag.ctrl2.value);
		QMC6308_CHECK_ERR(ret);
		qmc6308_delay(1);

		ret = qmc6308_write_reg(QMC6308_CTL_REG_ONE, p_mag.ctrl1.value);
		QMC6308_CHECK_ERR(ret);
		qmc6308_delay(1);
	}
	else
	{
		ret = qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0x00);
		QMC6308_CHECK_ERR(ret);
	}

	return ret;
}

static void qmc6308_dump_reg(void)
{
#if 0
	unsigned char ctrl_value;
	unsigned char id[3];

	qmc6308_read_block(0x37, &id[0], 3);
	QMC6308_LOG("qmc6308  0x37-0x39[0x%x 0x%x 0x%x] \r\n", id[0],id[1],id[2]);
	QMC6308_LOG("qmc6308  wafer id:0x%x \n", id[0]&0x1f);
	QMC6308_LOG("qmc6308  die id:0x%x \n", (id[2]<<8)|id[1]);

	qmc6308_read_block(QMC6308_CTL_REG_ONE, &ctrl_value, 1);
	QMC6308_LOG("qmc6308  0x%x=0x%x \r\n", QMC6308_CTL_REG_ONE, ctrl_value);
	qmc6308_read_block(QMC6308_CTL_REG_TWO, &ctrl_value, 1);
	QMC6308_LOG("qmc6308  0x%x=0x%x \r\n", QMC6308_CTL_REG_TWO, ctrl_value);
	qmc6308_read_block(0x0d, &ctrl_value, 1);
	QMC6308_LOG("qmc6308  0x%x=0x%x \r\n", 0x0d, ctrl_value);
#else
	unsigned char i2c_0_2 = 0;
	unsigned char version_id = 0;
	unsigned char wafer_id = 0;
	unsigned short d_id = 0;	
	unsigned char g_reg_tbl[16];
	
	QMC6308_LOG("\r\n********qst reg dump(hex)********\r\n");
	QMC6308_LOG("     |  0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
	QMC6308_LOG("______________________________________________________\r\n");
//	qmc6308_read_block(0x00, &g_reg_tbl[0], 72);
	qmc6308_read_block(0x00, &g_reg_tbl[0], 16);
	QMC6308_LOG("0x%xx | %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\r\n", 0,
																		g_reg_tbl[0],g_reg_tbl[1],g_reg_tbl[2],g_reg_tbl[3],
																		g_reg_tbl[4],g_reg_tbl[5],g_reg_tbl[6],g_reg_tbl[7],
																		g_reg_tbl[8],g_reg_tbl[9],g_reg_tbl[10],g_reg_tbl[11],
																		g_reg_tbl[12],g_reg_tbl[13],g_reg_tbl[14],g_reg_tbl[15]);

	qmc6308_read_block(0x10, &g_reg_tbl[0], 16);
//	qmc6309_delay(1);
	QMC6308_LOG("0x%xx | %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\r\n", 1,
																		g_reg_tbl[0],g_reg_tbl[1],g_reg_tbl[2],g_reg_tbl[3],
																		g_reg_tbl[4],g_reg_tbl[5],g_reg_tbl[6],g_reg_tbl[7],
																		g_reg_tbl[8],g_reg_tbl[9],g_reg_tbl[10],g_reg_tbl[11],
																		g_reg_tbl[12],g_reg_tbl[13],g_reg_tbl[14],g_reg_tbl[15]);
	version_id = g_reg_tbl[0x02];

	qmc6308_read_block(0x20, &g_reg_tbl[0], 16);
//	qmc6309_delay(1);
	QMC6308_LOG("0x%xx | %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\r\n", 2,
																		g_reg_tbl[0],g_reg_tbl[1],g_reg_tbl[2],g_reg_tbl[3],
																		g_reg_tbl[4],g_reg_tbl[5],g_reg_tbl[6],g_reg_tbl[7],
																		g_reg_tbl[8],g_reg_tbl[9],g_reg_tbl[10],g_reg_tbl[11],
																		g_reg_tbl[12],g_reg_tbl[13],g_reg_tbl[14],g_reg_tbl[15]);
	qmc6308_read_block(0x30, &g_reg_tbl[0], 16);
//	qmc6309_delay(1);
	QMC6308_LOG("0x%xx | %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\r\n", 3,
																		g_reg_tbl[0],g_reg_tbl[1],g_reg_tbl[2],g_reg_tbl[3],
																		g_reg_tbl[4],g_reg_tbl[5],g_reg_tbl[6],g_reg_tbl[7],
																		g_reg_tbl[8],g_reg_tbl[9],g_reg_tbl[10],g_reg_tbl[11],
																		g_reg_tbl[12],g_reg_tbl[13],g_reg_tbl[14],g_reg_tbl[15]);	
	wafer_id = g_reg_tbl[0x07]&0x1f;
	d_id = (unsigned short)((g_reg_tbl[0x09]<<8)|g_reg_tbl[0x08]);
	i2c_0_2 = ((g_reg_tbl[0x07]&0x20)>>3)|(g_reg_tbl[0x0b]&0x03);

	qmc6308_read_block(0x40, &g_reg_tbl[0], 8);
//	qmc6309_delay(1);
	QMC6308_LOG("0x%xx | %02x %02x %02x %02x %02x %02x %02x %02x \r\n", 4,
																		g_reg_tbl[0],g_reg_tbl[1],g_reg_tbl[2],g_reg_tbl[3],
																		g_reg_tbl[4],g_reg_tbl[5],g_reg_tbl[6],g_reg_tbl[7]);

	QMC6308_LOG("Version-ID:[0x%02x] Wafer-ID:[0x%02x] Di-ID:[0x%04x] I2C[bit0-2]:[%x]\r\n", version_id, wafer_id, d_id, i2c_0_2);
	QMC6308_LOG("********qst reg dump done********\r\n\r\n\r\n");
#endif
}

int qmc6308_init(void)
{
	int ret = 0;
	unsigned char qmc6308_slave[]={0x0c,0x1c,0x3c};
 
	p_mag.slave_addr = 0x2c;
	qmc6308_init_para();

	ret = qmc6308_get_chipid();
	if(!ret)
	{
		for(int i=0; i<(sizeof(qmc6308_slave)/sizeof(qmc6308_slave[0])); i++)
		{
			p_mag.slave_addr = qmc6308_slave[i];		
			ret = qmc6308_get_chipid();
			if(ret)		// read id OK
			{		
				ret = qmc6308_write_reg(QMC6308_CTL_REG_TWO, 0x80);
				qmc6308_delay(5);
				ret = qmc6308_write_reg(QMC6308_CTL_REG_TWO, 0x00);
				qmc6308_delay(5);
				QMC6308_LOG("qmc6308 change slave to 0x2c read id again\n");
				p_mag.slave_addr = 0x2c;	
				ret = qmc6308_get_chipid(); // change to 0x2c read id again! 
				if(ret == 0)
				{
					return 0;
				}
				break;
			}
		}
	}

	if(!ret)
	{
		return 0;
	}

	//qmc6308_soft_reset();
	ret = qmc6308_enable(0);
	ret = qmc6308_enable(1);
	qmc6308_dump_reg();
#if defined(QMC6308_MODE_SWITCH)
	p_mag.set_ctl.mode = 0;
	p_mag.set_ctl.count = 0;
#endif
	if(p_mag.ctrl1.bit.mode == QMC6308_MODE_SINGLE)
	{	
		qmc6308_config_mode(QMC6308_MODE_SINGLE);
	}

	return 1;
}

int qmc6308_self_test(void)
{
	int selftest_result = 0;
	int selftest_retry = 0;
	int hdata_a[3];
	int hdata_b[3];
	int hdata[3];
	unsigned char rx_buf[6] = {0};
	unsigned char rdy = 0x00;
	int t1 = 0;
	int ret = 0;

	while((selftest_result == 0)&&(selftest_retry<3))
	{
		selftest_retry++;

		qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0x00);
		qmc6308_write_reg(QMC6308_CTL_REG_TWO, 0x00);
		qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0x03);
		qmc6308_delay(1);
	
		while(!(rdy & 0x03))
		{
			rdy = QMC6308_STATUS_REG;
			ret = qmc6308_read_block(QMC6308_STATUS_REG, &rdy, 1);
			qmc6308_delay(1);
			if(t1++ > 50)
				continue;
		}
	
		ret = qmc6308_read_block(QMC6308_DATA_OUT_X_LSB_REG, rx_buf, 6);
		if(ret == 0)
			continue;
	
		hdata_a[0] = (short)(((rx_buf[1]) << 8) | rx_buf[0]);
		hdata_a[1] = (short)(((rx_buf[3]) << 8) | rx_buf[2]);
		hdata_a[2] = (short)(((rx_buf[5]) << 8) | rx_buf[4]);
	
		qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0x00);
		qmc6308_delay(1);
		qmc6308_write_reg(0x0b, 0x40);
		qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0x03);
		qmc6308_delay(10);
		
		t1 = 0;
		rdy = 0;
		while(!(rdy & 0x03))
		{
			rdy = QMC6308_STATUS_REG;
			ret = qmc6308_read_block(QMC6308_STATUS_REG, &rdy, 1);
			qmc6308_delay(1);
			if(t1++ > 50)
				continue;
		}
		ret = qmc6308_read_block(QMC6308_DATA_OUT_X_LSB_REG, rx_buf, 6);
		if(ret == 0)
			continue;

		hdata_b[0] = (short)(((rx_buf[1]) << 8) | rx_buf[0]);
		hdata_b[1] = (short)(((rx_buf[3]) << 8) | rx_buf[2]);
		hdata_b[2] = (short)(((rx_buf[5]) << 8) | rx_buf[4]);
	
		hdata[0] = QMC6308_ABS(hdata_a[0]-hdata_b[0]);
		hdata[1] = QMC6308_ABS(hdata_a[1]-hdata_b[1]);
		hdata[2] = QMC6308_ABS(hdata_a[2]-hdata_b[2]);
	
		qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0x00);
		
		QMC6308_LOG("qmc6308_self_test(%d) %d	%d	%d\r\n", selftest_retry,hdata[0], hdata[1], hdata[2]);

		if(
			((hdata[0] < QMC6308_SELFTEST_MAX_X) && (hdata[0] > QMC6308_SELFTEST_MIN_X))
		&&	((hdata[1] < QMC6308_SELFTEST_MAX_Y) && (hdata[1] > QMC6308_SELFTEST_MIN_Y))
		&&	((hdata[2] < QMC6308_SELFTEST_MAX_Z) && (hdata[2] > QMC6308_SELFTEST_MIN_Z))
			)
	    {
	        selftest_result = 1;
	    }
		else
		{
			selftest_result = 0;
		}
	}
	qmc6308_enable(1);

	return selftest_result;
}


