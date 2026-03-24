
#include "qmc6309v.h"

static qmc6309v_data_t g6309v;

#include <string.h>
#include "qmc6309.h"

int qmc6309v_read_reg(unsigned char addr, unsigned char *data, unsigned short len)
{
	int ret = QMC6309V_FAIL;
	int retry = 0;

	while((ret!=QMC6309V_OK) && (retry++ < 5))
	{
		if(g6309v.protocol == 1)
			ret = bsp_i3c_read_reg(addr, data, len);
		else
			ret = bsp_i2c_read_reg(g6309v.slave_addr, addr, data, len);
	}

	return ret;
}

int qmc6309v_write_reg(unsigned char addr, unsigned char data)
{
	int ret = QMC6309V_FAIL;
	int retry = 0;

	while((ret!=QMC6309V_OK) && (retry++ < 5))
	{
		if(g6309v.protocol == 1)
			ret = bsp_i3c_write_reg(addr, data);
		else
			ret = bsp_i2c_write_reg(g6309v.slave_addr, addr, data);
	}

	return ret;
}

void qmc6309v_delay(unsigned int ms)
{
	qst_delay_ms(ms);
}

void qmc6309v_dump_reg(void)
{
	#define QMC6309V_REG_MAX	0x47

	int i = 0;
	unsigned char i2c_0_2 = 0;
	unsigned char version_id = 0;
	unsigned char wafer_id = 0;
	unsigned short d_id = 0;
	unsigned char g_reg_tbl[80];
	
	for(i=0; i<=QMC6309V_REG_MAX; i++)
	{
		qmc6309v_read_reg((unsigned char)i, &g_reg_tbl[i], 1);
		qst_delay_us(100);		
	}

	QMC6309V_LOG("\r\n******************qmc6309v reg dump(hex)**************\r\n");
	QMC6309V_LOG("     |  0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
	QMC6309V_LOG("______________________________________________________\r\n");
	
	for(int i=0; i<5; i++)
	{
		int index = i*16;
		QMC6309V_LOG("0x%xx | %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\r\n", i,
																g_reg_tbl[index+0],g_reg_tbl[index+1],g_reg_tbl[index+2],g_reg_tbl[index+3],
																g_reg_tbl[index+4],g_reg_tbl[index+5],g_reg_tbl[index+6],g_reg_tbl[index+7],
																g_reg_tbl[index+8],g_reg_tbl[index+9],g_reg_tbl[index+10],g_reg_tbl[index+11],
																g_reg_tbl[index+12],g_reg_tbl[index+13],g_reg_tbl[index+14],g_reg_tbl[index+15]);
	}
	QMC6309V_LOG("****************vqmc6309v reg dump done****************\r\n\r\n\r\n");
	version_id = g_reg_tbl[0x12];
	wafer_id = g_reg_tbl[0x37]&0x1f;
	d_id = (unsigned short)((g_reg_tbl[0x39]<<8)|g_reg_tbl[0x38]);
	i2c_0_2 = ((g_reg_tbl[0x37]&0x20)>>3)|(g_reg_tbl[0x3b]&0x03);

	QMC6309V_LOG("Version-ID:[0x%02x] Wafer-ID:[0x%02x] Di-ID:[0x%04x] I2C[bit0-2]:[%x]\r\n", version_id, wafer_id, d_id, i2c_0_2);
	QMC6309V_LOG("********qst reg dump done********\r\n\r\n\r\n");
}

int qmc6309v_get_chipid(void)
{
	int ret = QMC6309V_FAIL;
	int retry=0;
	unsigned char chip_id = 0x00;

	retry = 0;
	while((chip_id != QMC6309V_CHIP_ID) && (retry++<5))
	{
		ret = qmc6309v_read_reg(QMC6309V_CHIP_ID_REG, &chip_id, 1);
		if(ret == QMC6309V_OK)
		{
			break;
		}
	}
	if(chip_id == QMC6309V_CHIP_ID)
	{
		QMC6309V_LOG("qmc6309v_get_chipid-ok slave:0x%x chipid = 0x%x\r\n", g6309v.slave_addr, chip_id);
		return 1;
	}
	else
	{
		QMC6309V_LOG("qmc6309v_get_chipid-fail slave:0x%x chip_id = 0x%x\r\n", g6309v.slave_addr, chip_id);
		return 0;
	}
}

void qmc6309v_init_para(unsigned char mode, unsigned char odr)
{
	qmc6309v_ctrlreg1		ctrl1;
	qmc6309v_ctrlreg2		ctrl2;

	ctrl1.bit.mode = mode;
	if((odr == QMC6309V_ODR_200HZ) || (mode == QMC6309V_MODE_HPFM))
	{
		ctrl1.bit.osr1 = QMC6309V_OSR1_8;
		ctrl1.bit.osr2 = QMC6309V_OSR2_2;
	}
	else
	{
		ctrl1.bit.osr1 = QMC6309V_OSR1_4;
		ctrl1.bit.osr2 = QMC6309V_OSR2_2;
	}

	ctrl1.bit.zdbl_enb = 0;
	ctrl2.bit.set_rst = QMC6309V_SET_RESET_ON;
	ctrl2.bit.range = QMC6309V_RNG_32G;
	ctrl2.bit.odr = odr;
	ctrl2.bit.soft_rst = 0;

	switch(ctrl2.bit.range)
	{
		case QMC6309V_RNG_32G:
			g6309v.ssvt = 1000;
			break;
		case QMC6309V_RNG_16G:
			g6309v.ssvt = 2000;
			break;
		case QMC6309V_RNG_8G:
			g6309v.ssvt = 4000;
			break;
		default:
			g6309v.ssvt = 1000;
			break;
	}

#if defined(QMC6309V_X7)
	qmc6309v_read_reg(QMC6309V_CTL_REG_THREE, &g6309v.ctl3_val, 1);
	g6309v.ctl3_val = g6309v.ctl3_val & 0xe7;
	g6309v.ctl3_val = g6309v.ctl3_val | (ctrl1.bit.osr1 << 3);
#endif
	g6309v.ctl1_val = ctrl1.value;
	g6309v.ctl2_val = ctrl2.value;
#if defined(QMC6309V_MODE_SWITCH)
	g6309v.set_ctl.mode = 0;
	g6309v.set_ctl.count = 0;
#endif
}

int qmc6309v_enable(void)
{
	int ret = 0;

	QMC6309V_LOG("qmc6309v_enable!\r\n");
#if defined(QMC6309V_X7)
	ret = qmc6309v_write_reg(QMC6309V_CTL_REG_THREE, g6309v.ctl3_val);
	QMC6309V_CHECK_ERR(ret);
	qmc6309v_delay(1);
#endif
	ret = qmc6309v_write_reg(QMC6309V_CTL_REG_TWO, g6309v.ctl2_val);
	QMC6309V_CHECK_ERR(ret);
	qmc6309v_delay(1);

	ret = qmc6309v_write_reg(QMC6309V_CTL_REG_ONE, g6309v.ctl1_val);
	QMC6309V_CHECK_ERR(ret);
	qmc6309v_delay(1);

	return ret;
}

int qmc6309v_disable(void)
{
	int ret = 0;

	QMC6309V_LOG("qmc6309v_disable!\r\n");
	ret = qmc6309v_write_reg(QMC6309V_CTL_REG_ONE, 0x00);
	QMC6309V_CHECK_ERR(ret);

	return ret;
}

int qmc6309v_enable_ibi(qmc6309v_fifo_ibi flag)
{
	int ret = QMC6309V_FAIL;
	unsigned char ibi_value = 0x00;

	QMC6309V_LOG("qmc6309v_enable_ibi 0x%x\r\n", flag);
	ibi_value = (unsigned char)flag;
	ret = qmc6309v_write_reg(QMC6309V_CTL_IBI, ibi_value);

	QMC6309V_CHECK_ERR(ret);

	return ret;
}

void qmc6309v_soft_reset(void)
{
	int ret = QMC6309V_FAIL;
	int retry = 0;
	unsigned char status = 0x00;

	QMC6309V_LOG("qmc6309v_soft_reset!\r\n");
	ret = qmc6309v_write_reg(QMC6309V_CTL_REG_TWO, 0x80);
	QMC6309V_CHECK_ERR(ret);
	ret = qmc6309v_write_reg(QMC6309V_CTL_REG_TWO, 0x00);
	QMC6309V_CHECK_ERR(ret);
	qmc6309v_delay(5);

	while(retry++<5)
	{
		ret = qmc6309v_read_reg(QMC6309V_STATUS_REG, &status, 1);
		QMC6309V_CHECK_ERR(ret);
		QMC6309V_LOG("qmc6309 status 0x%x\r\n", status);
		if((status & 0x10)&&(status & 0x08))
		{
			QMC6309V_LOG("qmc6309 NVM load done!\r\n");
			break;
		}
		qmc6309v_delay(1);
	}

#if defined(QMC6309V_0X40_CFG)
	//#define SR_PULSE_250NS				0
	//#define SR_PULSE_125NS				1
	//#define SR_PULSE_62_5NS				2
	//#define SR_PULSE_31_25NS				3
	//#define SR_SLOP_100NS					0
	//#define SR_SLOP_50NS					1
	//#define SR_SLOP_25NS					2
	//#define SR_SLOP_0NS					3
	ret = qmc6309v_read_reg(0x40, &status, 1);
	QMC6309V_LOG("read 0x40=0x%02x ret=%d\r\n", status, ret);
	status = (status&0xe0)|0x0d; //170mv		//status = (status&0xe0)|0x0c;
	//status = status|0x80;					// set vddio=vdd disable 1.2v io auto detect
	ret = qmc6309v_write_reg(0x40, status);
	QMC6309V_LOG("write 0x40=0x%02x ret=%d\r\n", status, ret);
	qmc6309v_delay(1);
#endif
}

int qmc6309v_read_mag_raw(short raw[3])
{
	int res = QMC6309V_FAIL;
	unsigned char mag_data[6];
	int t1 = 0;
	unsigned char rdy = 0;

	/* Check status register for data availability */
	res = qmc6309v_read_reg(QMC6309V_STATUS_REG, &rdy, 1);
	while(!(rdy & (QMC6309V_STATUS_DRDY)) & (t1++ < 5))
	{
		res = qmc6309v_read_reg(QMC6309V_STATUS_REG, &rdy, 1);
		QMC6309V_CHECK_ERR(res);
		qmc6309v_delay(1);
	}
	if((res == QMC6309V_FAIL)||(!(rdy & QMC6309V_STATUS_DRDY)))
	{
		raw[0] = g6309v.last_data[0];
		raw[1] = g6309v.last_data[1];
		raw[2] = g6309v.last_data[2];
		QMC6309V_LOG("qmc6309v_read_mag_raw read drdy fail! res=%d rdy=0x%x\r\n",res,rdy);
		res = QMC6309V_OK;	// QMC6309V_FAIL;
	}
	else if(rdy & QMC6309V_STATUS_OVFL)
	{
		raw[0] = 32767;
		raw[1] = 32767;
		raw[2] = 32767;
		g6309v.fail_num = 0;
		return QMC6309V_OK;
	}
	else
	{
		mag_data[0] = QMC6309V_DATA_OUT_X_LSB_REG;
		res = qmc6309v_read_reg(QMC6309V_DATA_OUT_X_LSB_REG, mag_data, 6);
		if(res == QMC6309V_FAIL)
	  	{
			QMC6309V_LOG("qmc6309v_read_mag_raw read data fail! res=%d\r\n",res);
			raw[0] = g6309v.last_data[0];
			raw[1] = g6309v.last_data[1];
			raw[2] = g6309v.last_data[2];
			res = QMC6309V_OK;	// QMC6309V_FAIL;
		}
		else
		{
			raw[0] = (short)(((mag_data[1]) << 8) | mag_data[0]);
			raw[1] = (short)(((mag_data[3]) << 8) | mag_data[2]);
			raw[2] = (short)(((mag_data[5]) << 8) | mag_data[4]);
		}
	}

	if((g6309v.last_data[0]==raw[0])&&(g6309v.last_data[1]==raw[1])&&(g6309v.last_data[2]==raw[2]))
	{
		g6309v.fail_num++;
	}
	else
	{
		g6309v.fail_num = 0;
	}

	if(g6309v.fail_num > 10)
	{
#if defined(QMC6309V_RECOVER)
		//qmc6309v_recover();
#endif
		qmc6309v_soft_reset();
		qmc6309v_enable();
		g6309v.fail_num = 0;
	}
	
	g6309v.last_data[0] = raw[0];
	g6309v.last_data[1] = raw[1];
	g6309v.last_data[2] = raw[2];	
#if defined(QMC6309V_MODE_SWITCH)
	qmc6309v_setrst_auto_mode(raw);
#endif

	return res;
}

int qmc6309v_read_mag_xyz(float uT[3])
{
	int res = QMC6309V_FAIL;
	short raw[3];

	res = qmc6309v_read_mag_raw(raw);
	if(res == QMC6309V_OK)
	{
		uT[0] = (float)((float)raw[0] / ((float)g6309v.ssvt/100.f));		// ut
		uT[1] = (float)((float)raw[1] / ((float)g6309v.ssvt/100.f));		// ut
		uT[2] = (float)((float)raw[2] / ((float)g6309v.ssvt/100.f));		// ut
	}
	else
	{
		uT[0] = uT[1]= uT[2] = 0.0f;
	}

	if((g6309v.ctl1_val&0x03) == QMC6309V_MODE_SINGLE)
	{
		res = qmc6309v_write_reg(QMC6309V_CTL_REG_ONE, g6309v.ctl1_val);
		QMC6309V_CHECK_ERR(res);
	}

	return res;
}

int qmc6309v_self_test(void)
{
	int st_result = 0;
	int st_retry = 0;
	signed char  st_data[3];
	unsigned char abs_data[3];
	unsigned char rdy = 0x00;
	int t1 = 0;
	int ret = QMC6309V_FAIL;

	while((st_result == 0)&&(st_retry<3))
	{
		st_retry++;
		qmc6309v_write_reg(QMC6309V_CTL_REG_ONE, 0x00);
		qmc6309v_delay(2);
		qmc6309v_write_reg(QMC6309V_CTL_REG_TWO, 0x00);
		qmc6309v_delay(2);
		qmc6309v_write_reg(QMC6309V_CTL_REG_ONE, 0x03);
		qmc6309v_delay(20);
		qmc6309v_write_reg(0x0e, 0x80);
		rdy = 0x00;
		t1 = 0;
		while(!(rdy & 0x04))
		{
			qmc6309v_delay(5);
			ret = qmc6309v_read_reg(QMC6309V_STATUS_REG, &rdy, 1);
			if(t1++ > 50)
			{
				break;
			}
		}
		if(rdy & 0x04)
		{
			ret = qmc6309v_read_reg(QMC6309V_DATA_OUT_ST_X, (unsigned char*)st_data, 3);
			if(ret == QMC6309V_FAIL)
				continue;
		}
		else
		{
			QMC6309V_LOG("qmc6309 selftest drdy fail!\r\n");
			continue;
		}

		abs_data[0] = QMC6309V_ABS(st_data[0]);
		abs_data[1] = QMC6309V_ABS(st_data[1]);
		abs_data[2] = QMC6309V_ABS(st_data[2]);

		if(	((abs_data[0] <= QMC6309V_SELFTEST_MAX_X) && (abs_data[0] >= QMC6309V_SELFTEST_MIN_X))
			&& ((abs_data[1] <= QMC6309V_SELFTEST_MAX_Y) && (abs_data[1] >= QMC6309V_SELFTEST_MIN_Y))
			&& ((abs_data[2] <= QMC6309V_SELFTEST_MAX_Z) && (abs_data[2] >= QMC6309V_SELFTEST_MIN_Z)) )
	    {
	        st_result = 1;
	    }
		else
		{
			st_result = 0;
		}		
		QMC6309V_LOG("status[0x%x] data[%d %d %d] [%s]\r\n",rdy,st_data[0],st_data[1],st_data[2], st_result?"PASS":"FAIL");
	}

	return st_result;
}

#if defined(QMC6309V_MODE_SWITCH)
void qmc6309v_setrst_auto_mode(short hw_d[3])
{
	int ret = QMC6309V_FAIL;
	unsigned char ctl1 = g6309v.ctl1_val;
	unsigned char ctl2 = g6309v.ctl2_val;

	if(g6309v.set_ctl.mode == 0)
	{// set reset on
		if((QMC6309V_ABS(hw_d[0]) > 8000)||(QMC6309V_ABS(hw_d[1]) > 8000))
		{
			g6309v.set_ctl.count++;
			if(g6309v.set_ctl.count >= 10)
			{
				g6309v.set_ctl.mode = 1;
				g6309v.set_ctl.count = 0;

				ret = qmc6309v_write_reg(QMC6309V_CTL_REG_ONE, 0x00);
				ctl2 = ((ctl2&0xfc)|QMC6309V_SET_ON);
				ret = qmc6309v_write_reg(QMC6309V_CTL_REG_TWO, ctl2);				
				ret = qmc6309v_write_reg(QMC6309V_CTL_REG_ONE, ctl1);
				qmc6309v_delay(1);
			}
		}
		else
		{
			g6309v.set_ctl.count = 0;
		}
	}
	else
	{// set only
		int force_switch = 0;
		g6309v.set_ctl.count++;
		if(g6309v.set_ctl.count >= 100)
		{
			g6309v.set_ctl.count = 0;
			force_switch = 1;
		}
		if(((QMC6309V_ABS(hw_d[0]) < 6000)&&(QMC6309V_ABS(hw_d[1]) < 6000)) || force_switch)
		{
			g6309v.set_ctl.mode = 0;
			g6309v.set_ctl.count = 0;

			ret = qmc6309v_write_reg(QMC6309V_CTL_REG_ONE, 0x00);			
			ctl2 = ((ctl2&0xfc)|QMC6309V_SET_RESET_ON);
			ret = qmc6309v_write_reg(QMC6309V_CTL_REG_TWO, ctl2);
			ret = qmc6309v_write_reg(QMC6309V_CTL_REG_ONE, ctl1);
			qmc6309v_delay(1);
		}
	}

}
#endif

int qmc6309v_fifo_config(qmc6309v_fifo_mode mode, unsigned char wmk)
{
	unsigned char fifo_reg = 0x00;
	int ret = QMC6309V_OK;

	fifo_reg = (mode|(wmk&0x0f));
	g6309v.fifo_ctrl = fifo_reg;
	ret = qmc6309v_write_reg(QMC6309V_FIFO_REG_CTRL, fifo_reg);
	QMC6309V_CHECK_ERR(ret);

	return ret;
}

int qmc6309v_fifo_read(unsigned char *f_data)
{
	unsigned char fifo_status = 0;
	unsigned char fifo_level = 0;
	int ret = QMC6309V_FAIL;

	ret = qmc6309v_read_reg(QMC6309V_FIFO_REG_STATUS, &fifo_status, 1);
	QMC6309V_CHECK_ERR(ret);
	fifo_level = (fifo_status >> 3);
	if(fifo_level)
	{
		ret = qmc6309v_read_reg(QMC6309V_FIFO_REG_DATA, (unsigned char*)f_data, 6*fifo_level);
		//for(int index=0; index<fifo_level; index++)
		//{
		//	qmc6309v_read_reg(QMC6309V_FIFO_REG_DATA, &f_data[index*6], g6309v.fifo_frame_len);
		//}
		ret = qmc6309v_write_reg(QMC6309V_FIFO_REG_CTRL, g6309v.fifo_ctrl);
		QMC6309V_CHECK_ERR(ret);		
		g6309v.fail_num = 0;
	}
	else
	{
		g6309v.fail_num++;
		if(g6309v.fail_num >= 2)
		{
			qmc6309v_soft_reset();
			qmc6309v_write_reg(QMC6309V_FIFO_REG_CTRL, g6309v.fifo_ctrl);
			qmc6309v_enable();
			g6309v.fail_num = 0;
		}
	}

	return (int)fifo_level;
}

#if defined(QMC6309V_RECOVER)
int qmc6309v_recover(void)
{
	int ret = QMC6309V_FAIL;
	unsigned char slave_loop[]={0x7c,0x0c,0x1c,0x2c,0x3c,0x4c,0x5c,0x6c};

	for(int i=0; i<sizeof(slave_loop)/sizeof(slave_loop[0]); i++)
	{
		g6309v.slave_addr = slave_loop[i];
		ret = qmc6309v_get_chipid();	// loop slave address
		if(ret)
		{
			QMC6309V_LOG("qmc6309v_recover slave=0x%02x read id OK\r\n", g6309v.slave_addr);
			qmc6309v_soft_reset();	// softreset reload OTP
			g6309v.slave_addr = QMC6309V_IIC_ADDR;
			ret = qmc6309v_get_chipid();
			break;
		}
	}
 
	QMC6309V_LOG("qmc6309v_recover %s\r\n", ret?"OK":"FAIL");
	return ret;
}
#endif

int qmc6309v_init(int protocol)
{
	int ret = 0;

	memset(&g6309v, 0, sizeof(g6309v));
	g6309v.protocol = (char)protocol;
	g6309v.slave_addr = QMC6309V_IIC_ADDR;
	ret = qmc6309v_get_chipid();

#if defined(QMC6309V_RECOVER)
	if(ret == 0)
	{
		qmc6309v_recover();
	}
#endif
	if(ret)
	{
		qmc6309v_soft_reset();
		//qmc6309v_dump_reg();
		qmc6309v_init_para(QMC6309V_MODE_HPFM, QMC6309V_ODR_HPFM);
		QMC6309V_CHECK_ERR(ret);
		ret = qmc6309v_enable();
		QMC6309V_CHECK_ERR(ret);

		return 1;
	}
	else
	{
		return 0;
	}
}



