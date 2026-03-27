
#include "qmc6g00x.h"

static qmc6g00x_data_t gTmr;
#if defined(QMC6G00X_RECOVER)
int qmc6g00x_recover(void);
#endif

int qmc6g00x_read_reg(unsigned char addr, unsigned char *data, unsigned short len)
{
	int ret = QMC6G00X_FAIL;
	int retry = 0;

	while((ret!=QMC6G00X_OK) && (retry++ < 5))
	{
		if(gTmr.protocol == 1)
			ret = bsp_i3c_read_reg(addr, data, len);
		else
			ret = bsp_i2c_read_reg(gTmr.slave_addr, addr, data, len);
	}

	return ret;
}

int qmc6g00x_write_reg(unsigned char addr, unsigned char data)
{
	int ret = QMC6G00X_FAIL;
	int retry = 0;

	while((ret!=QMC6G00X_OK) && (retry++ < 5))
	{
		if(gTmr.protocol == 1)
			ret = bsp_i3c_write_reg(addr, data);
		else
			ret = bsp_i2c_write_reg(gTmr.slave_addr, addr, data);
	}

	return ret;
}

void qmc6g00x_delay(unsigned int ms)
{
	qst_delay_ms(ms);
}


void qmc6g00x_dump_reg(void)
{
	#define QMC6G00X_REG_MAX	0x4f

	int i = 0;
	unsigned char i2c_0_2 = 0;
	unsigned char version_id = 0;
	unsigned char wafer_id = 0;
	unsigned short d_id = 0;
	unsigned char g_reg_tbl[80];
	
	for(i=0; i<=QMC6G00X_REG_MAX; i++)
	{
		qmc6g00x_read_reg((unsigned char)i, &g_reg_tbl[i], 1);
		qst_delay_us(100);		
	}

	QMC6G00X_LOG("\r\n******************maestro reg dump(hex)**************\r\n");
	QMC6G00X_LOG("     |  0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
	QMC6G00X_LOG("______________________________________________________\r\n");
	
	for(int i=0; i<5; i++)
	{
		int index = i*16;
		QMC6G00X_LOG("0x%xx | %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\r\n", i,
																g_reg_tbl[index+0],g_reg_tbl[index+1],g_reg_tbl[index+2],g_reg_tbl[index+3],
																g_reg_tbl[index+4],g_reg_tbl[index+5],g_reg_tbl[index+6],g_reg_tbl[index+7],
																g_reg_tbl[index+8],g_reg_tbl[index+9],g_reg_tbl[index+10],g_reg_tbl[index+11],
																g_reg_tbl[index+12],g_reg_tbl[index+13],g_reg_tbl[index+14],g_reg_tbl[index+15]);
	}
	QMC6G00X_LOG("****************maestro reg dump done****************\r\n");
	version_id = g_reg_tbl[0x12];
	wafer_id = g_reg_tbl[0x37]&0x1f;
	d_id = (unsigned short)((g_reg_tbl[0x39]<<8)|g_reg_tbl[0x38]);
	i2c_0_2 = ((g_reg_tbl[0x37]&0x20)>>3)|(g_reg_tbl[0x3b]&0x03);

	QMC6G00X_LOG("Version-ID:[0x%02x] Wafer-ID:[0x%02x] Di-ID:[0x%04x] I2C[bit0-2]:[%x]\r\n", version_id, wafer_id, d_id, i2c_0_2);
	QMC6G00X_LOG("********qst reg dump done********\r\n\r\n");
}

int qmc6g00x_get_chipid(void)
{
	int ret = QMC6G00X_FAIL;
	int retry=0;
	unsigned char chip_id = 0x00;

	retry = 0;
	while(((chip_id != QMC6G00V_CHIP_ID)&&(chip_id != QMC6G00H_CHIP_ID)) && (retry++<5))
	{
		ret = qmc6g00x_read_reg(QMC6G00X_CHIP_ID_REG, &chip_id, 1);
		if(ret == QMC6G00X_OK)
		{
			break;
		}
	}
	if((chip_id == QMC6G00V_CHIP_ID)||(chip_id == QMC6G00H_CHIP_ID))
	{
		gTmr.chipid = chip_id;
		QMC6G00X_LOG("qmc6g00x_get_chipid-ok slave:0x%x chipid = 0x%x\r\n", gTmr.slave_addr, chip_id);
		return 1;
	}
	else
	{
		gTmr.chipid = 0;
		QMC6G00X_LOG("qmc6g00x_get_chipid-fail slave:0x%x chip_id = 0x%x\r\n", gTmr.slave_addr, chip_id);
		return 0;
	}
}

void qmc6g00x_do_set(int count)
{
	unsigned char			ctrl1;
	unsigned char			ctrl2;

	ctrl1 = gTmr.ctl1_val;
	ctrl2 = gTmr.ctl2_val;

	//if((ctrl1&0xfc) != MAESTRO_MODE_HPFM)
	//{
	//	ctrl1 = (ctrl1&0xfc)|MAESTRO_MODE_HPFM;
	//}

	// do set
	for(int i=0; i<count; i++)
	{
		qmc6g00x_write_reg(QMC6G00X_CTL_REG_ONE, 0x00);
		ctrl2 = (ctrl2 & 0xfc) | QMC6G00X_SET_ON;
		QMC6G00X_LOG("set 0x0a=0x%02x 0x0b=0x%02x |", ctrl1, ctrl2);
		qmc6g00x_write_reg(QMC6G00X_CTL_REG_TWO, ctrl2);
		qmc6g00x_write_reg(QMC6G00X_CTL_REG_ONE, ctrl1);
		qmc6g00x_delay(2);
#if 1
		// do reset
		qmc6g00x_write_reg(QMC6G00X_CTL_REG_ONE, 0x00);
		ctrl2 = (ctrl2 & 0xfc) | QMC6G00X_RESET_ON;
		QMC6G00X_LOG("reset 0x0a=0x%02x 0x0b=0x%02x\r\n", ctrl1, ctrl2);
		qmc6g00x_write_reg(QMC6G00X_CTL_REG_TWO, ctrl2);
		qmc6g00x_write_reg(QMC6G00X_CTL_REG_ONE, ctrl1);
		qmc6g00x_delay(2);
#endif
	}

	qmc6g00x_write_reg(QMC6G00X_CTL_REG_ONE, 0x00);
}

#if 0
void qmc6g00x_calc_kxky(void)
{
	unsigned char preg[2];
#if 1
	signed char kx_calc = 0;
	signed short ky_calc = 0;
	unsigned char kx_code = 0x00;
	unsigned short ky_code = 0x00;

	qmc6g00x_read_reg(0x4e, preg, 2);

	kx_code = preg[0] & 0x7f;
	ky_code = ((preg[0] & 0x80) << 1) + (preg[1]);
	
	if (kx_code > 63){
		kx_calc = kx_code - 128;
	}
	else{
		kx_calc = kx_code;
	}
	kx_calc = QMC6G00X_MAX(-64, QMC6G00X_MIN(kx_calc, 63));
	
	if (ky_code > 255){
		ky_calc = ky_code - 512;
	}
	else{
		ky_calc = ky_code;
	}
	ky_calc = QMC6G00X_MAX(-256, QMC6G00X_MIN(ky_calc, 255));
	QMC6G00X_LOG("1 0x4e-0x4f[0x%02x 0x%02x] kxky[%d %d]\r\n",preg[0], preg[1], kx_calc, ky_calc);
	gTmr.kx = (float)kx_calc / 128.f;
	gTmr.ky = (float)ky_calc / 128.f;
#endif
#if 0
	qmc6g00x_kxky	kxky;

	kxky.bit.kx = preg[0]&0x7f;
	kxky.bit.ky = (short)(((preg[0]&0x80)<<1)|preg[1]);
	QMC6G00X_LOG("2 0x4e-0x4f[0x%02x 0x%02x] kxky[%d %d]\r\n", preg[0], preg[1], kxky.bit.kx, kxky.bit.ky);
	gTmr.kx = (float)kxky.bit.kx / 128.f;
	gTmr.ky = (float)kxky.bit.ky / 128.f;
#endif
	
	QMC6G00X_LOG("qmc6g00x_calc_kxky [%f %f]  \r\n", gTmr.kx, gTmr.ky);
}
#endif

int qmc6g00x_enable(void)
{
	int ret = 0;

//	qmc6g00x_do_set(50);
	ret = qmc6g00x_write_reg(QMC6G00X_CTL_REG_ONE, 0x00);
	qmc6g00x_delay(1);
	QMC6G00X_CHECK_ERR(ret);
	QMC6G00X_LOG("qmc6g00x_enable! 0x0a=0x%02x 0x0b=0x%02x\r\n", gTmr.ctl1_val, gTmr.ctl2_val);
	ret = qmc6g00x_write_reg(QMC6G00X_CTL_REG_TWO, gTmr.ctl2_val);
	QMC6G00X_CHECK_ERR(ret);
	qmc6g00x_delay(1);
	ret = qmc6g00x_write_reg(QMC6G00X_CTL_REG_ONE, gTmr.ctl1_val);
	QMC6G00X_CHECK_ERR(ret);
	qmc6g00x_delay(1);

	return ret;
}

int qmc6g00x_disable(void)
{
	int ret = 0;

	QMC6G00X_LOG("qmc6g00x_disable!\r\n");
	ret = qmc6g00x_write_reg(QMC6G00X_CTL_REG_ONE, 0x00);
	QMC6G00X_CHECK_ERR(ret);

	return ret;
}

void qmc6g00x_enable_ibi(qmc6g00x_fifo_ibi flag)
{
	int ret = QMC6G00X_FAIL;
	unsigned char ibi_value = 0x00;

	ibi_value = (unsigned char)flag;
	QMC6G00X_LOG("qmc6g00x_enable_ibi 0x%02x\r\n", ibi_value);
	ret = qmc6g00x_write_reg(QMC6G00X_CTL_IBI, ibi_value);

	QMC6G00X_CHECK_ERR(ret);
}

void qmc6g00x_init_para(unsigned char mode, unsigned char odr)
{
	qmc6g00x_ctrla			ctrla;
	qmc6g00x_ctrlb			ctrlb;

	ctrla.bit.mode = mode;
	if(mode == QMC6G00X_MODE_HPFM)
	{
		ctrla.bit.osr1 = QMC6G00X_OSR1_8;
		ctrla.bit.osr2 = QMC6G00X_OSR2_2;
	}
	else if(odr == QMC6G00X_ODR_1000HZ)
	{
		ctrla.bit.osr1 = QMC6G00X_OSR1_8;
		ctrla.bit.osr2 = QMC6G00X_OSR2_2;
	}
	else if(odr == QMC6G00X_ODR_400HZ)
	{
		ctrla.bit.osr1 = QMC6G00X_OSR1_8;
		ctrla.bit.osr2 = QMC6G00X_OSR2_1;
	}
	else if(odr == QMC6G00X_ODR_200HZ)
	{
		ctrla.bit.osr1 = QMC6G00X_OSR1_8;
		ctrla.bit.osr2 = QMC6G00X_OSR2_1;
	}
	else
	{
		ctrla.bit.osr1 = QMC6G00X_OSR1_8;
		ctrla.bit.osr2 = QMC6G00X_OSR2_1;
	}

	ctrlb.bit.set_rst = QMC6G00X_SET_RESET_OFF;
	ctrlb.bit.range = QMC6G00X_RNG_20G;
	ctrlb.bit.odr = odr;
	ctrlb.bit.soft_rst = 0;

	switch(ctrlb.bit.range)
	{
		case QMC6G00X_RNG_20G:
			gTmr.ssvt = 1000;
			break;
		default:
			gTmr.ssvt = 1000;
			break;
	}

	gTmr.ctl1_val = ctrla.value;
	gTmr.ctl2_val = ctrlb.value;
#if defined(QMC6G00X_MODE_SWITCH)
	gTmr.set_ctl.mode = 0;
	gTmr.set_ctl.count = 0;
#endif
}

void qmc6g00x_soft_reset(void)
{
	int ret = QMC6G00X_FAIL;
	int retry = 0;
	unsigned char status = 0x00;

	QMC6G00X_LOG("qmc6g00x_soft_reset!\r\n");
	ret = qmc6g00x_write_reg(QMC6G00X_CTL_REG_TWO, 0x80);
	QMC6G00X_CHECK_ERR(ret);
	qmc6g00x_delay(5);

	while(retry++<5)
	{
		ret = qmc6g00x_read_reg(QMC6G00X_STATUS_REG, &status, 1);
		QMC6G00X_CHECK_ERR(ret);
		QMC6G00X_LOG("maestro status 0x%x\r\n", status);
		if((status & 0x10)&&(status & 0x08))
		{
			QMC6G00X_LOG("maestro NVM load done!\r\n");
			break;
		}
		qmc6g00x_delay(1);
	}

	//status = 0x00;
	//ret = qmc6g00x_read_reg(0x40, &status, 1);
	//QMC6G00X_CHECK_ERR(ret);
	//ret = qmc6g00x_write_reg(0x40, status|0x80);
	//QMC6G00X_CHECK_ERR(ret);
}

int qmc6g00x_read_mag_raw(short raw[3])
{
	int res = QMC6G00X_FAIL;
	unsigned char mag_data[6];
	int t1 = 0;
	unsigned char rdy = 0;

	/* Check status register for data availability */
	res = qmc6g00x_read_reg(QMC6G00X_STATUS_REG, &rdy, 1);
	while(!(rdy & (QMC6G00X_STATUS_DRDY)) & (t1++ < 5))
	{
		res = qmc6g00x_read_reg(QMC6G00X_STATUS_REG, &rdy, 1);
		QMC6G00X_CHECK_ERR(res);
		qmc6g00x_delay(1);
	}
	if((res == QMC6G00X_FAIL)||(!(rdy & QMC6G00X_STATUS_DRDY)))
	{
		raw[0] = gTmr.last_data[0];
		raw[1] = gTmr.last_data[1];
		raw[2] = gTmr.last_data[2];
		QMC6G00X_LOG("qmc6g00x_read_mag_raw read drdy fail! res=%d rdy=0x%x\r\n",res,rdy);
		res = QMC6G00X_OK;	// QMC6G00X_FAIL;
	}
	else if(rdy & QMC6G00X_STATUS_OVFL)
	{
		raw[0] = 32767;
		raw[1] = 32767;
		raw[2] = 32767;
		gTmr.fail_num = 0;
		return QMC6G00X_OK;
	}
	else
	{
		mag_data[0] = QMC6G00X_DATA_OUT_X_LSB_REG;
		res = qmc6g00x_read_reg(QMC6G00X_DATA_OUT_X_LSB_REG, mag_data, 6);
		if(res == QMC6G00X_FAIL)
	  	{
			QMC6G00X_LOG("qmc6g00x_read_mag_raw read data fail! res=%d\r\n",res);
			raw[0] = gTmr.last_data[0];
			raw[1] = gTmr.last_data[1];
			raw[2] = gTmr.last_data[2];
			res = QMC6G00X_OK;	// QMC6G00X_FAIL;
		}
		else
		{
			raw[0] = (short)(((mag_data[1]) << 8) | mag_data[0]);
			raw[1] = (short)(((mag_data[3]) << 8) | mag_data[2]);
			raw[2] = (short)(((mag_data[5]) << 8) | mag_data[4]);
		}
	}

	if((gTmr.last_data[0]==raw[0])&&(gTmr.last_data[1]==raw[1])&&(gTmr.last_data[2]==raw[2]))
	{
		gTmr.fail_num++;
	}
	else
	{
		gTmr.fail_num = 0;
	}

	if(gTmr.fail_num > 10)
	{
#if defined(QMC6G00X_RECOVER)
		//qmc6g00x_recover();
#endif
		qmc6g00x_soft_reset();
		qmc6g00x_enable();
		gTmr.fail_num = 0;
	}
	
	gTmr.last_data[0] = raw[0];
	gTmr.last_data[1] = raw[1];
	gTmr.last_data[2] = raw[2];	
#if defined(QMC6G00X_MODE_SWITCH)
	qmc6g00x_setrst_auto_mode(raw);
#endif

	return res;
}

int qmc6g00x_read_mag_xyz(float uT[3])
{
	int res = QMC6G00X_FAIL;
	short raw[3];

	res = qmc6g00x_read_mag_raw(raw);
	if(res == QMC6G00X_OK)
	{
		uT[0] = (float)((float)raw[0] / ((float)gTmr.ssvt/100.f));		// ut
		uT[1] = (float)((float)raw[1] / ((float)gTmr.ssvt/100.f));		// ut
		uT[2] = (float)((float)raw[2] / ((float)gTmr.ssvt/100.f));		// ut
	}
	else
	{
		uT[0] = uT[1]= uT[2] = 0.0f;
	}

	if((gTmr.ctl1_val&0x03) == QMC6G00X_MODE_SINGLE)
	{
		res = qmc6g00x_write_reg(QMC6G00X_CTL_REG_ONE, gTmr.ctl1_val);
		QMC6G00X_CHECK_ERR(res);
	}

	return res;
}


int qmc6g00x_self_test(void)
{
	int st_result = 0;
	int st_retry = 0;
	signed char  st_data[3];
	unsigned char abs_data[3];
	unsigned char rdy = 0x00;
	int t1 = 0;
	int ret = QMC6G00X_FAIL;

	while((st_result == 0)&&(st_retry<3))
	{
		st_retry++;
		qmc6g00x_write_reg(QMC6G00X_CTL_REG_ONE, 0x00);
		qmc6g00x_delay(2);
		qmc6g00x_write_reg(QMC6G00X_CTL_REG_TWO, 0x01);
		qmc6g00x_delay(2);
		qmc6g00x_write_reg(QMC6G00X_CTL_REG_ONE, 0x17);
		qmc6g00x_delay(20);
		qmc6g00x_write_reg(0x0e, 0x80);
		rdy = 0x00;
		t1 = 0;
		while(!(rdy & 0x04))
		{
			qmc6g00x_delay(5);
			ret = qmc6g00x_read_reg(QMC6G00X_STATUS_REG, &rdy, 1);

			if(t1++ > 50)
			{
				break;
			}
		}		

		if(rdy & 0x04)
		{
			ret = qmc6g00x_read_reg(QMC6G00X_DATA_OUT_ST_X, (unsigned char*)st_data, 3);
			if(ret == QMC6G00X_FAIL)
				continue;
		}
		else
		{
			QMC6G00X_LOG("maestro selftest drdy fail!\r\n");
			continue;
		}

		abs_data[0] = QMC6G00X_ABS(st_data[0]);
		abs_data[1] = QMC6G00X_ABS(st_data[1]);
		abs_data[2] = QMC6G00X_ABS(st_data[2]);

		if(	((abs_data[0] < QMC6G00X_SELFTEST_MAX_X) && (abs_data[0] > QMC6G00X_SELFTEST_MIN_X))
			&& ((abs_data[1] < QMC6G00X_SELFTEST_MAX_Y) && (abs_data[1] > QMC6G00X_SELFTEST_MIN_Y))
			&& ((abs_data[2] < QMC6G00X_SELFTEST_MAX_Z) && (abs_data[2] > QMC6G00X_SELFTEST_MIN_Z)) )
	    {
	        st_result = 1;
	    }
		else
		{
			st_result = 0;
		}
		
		QMC6G00X_LOG("status[0x%x] data[%d %d %d] [%s]\r\n",rdy,st_data[0],st_data[1],st_data[2], st_result?"PASS":"FAIL");
	}

	return st_result;
}

#if defined(QMC6G00X_MODE_SWITCH)
void qmc6g00x_setrst_auto_mode(short hw_d[3])
{
	int ret = QMC6G00X_FAIL;

	if(gTmr.set_ctl.mode == 0)
	{
		if((QMC6G00X_ABS(hw_d[0]) > 8000)||(QMC6G00X_ABS(hw_d[1]) > 8000))
		{
			gTmr.set_ctl.count++;
			if(gTmr.set_ctl.count >= 10)
			{
				qmc6g00x_ctrlb ctrlb;
				gTmr.set_ctl.mode = 1;
				gTmr.set_ctl.count = 0;

				ctrlb.value = gTmr.ctl2_val;
				ctrlb.bit.set_rst = QMC6G00X_SET_ON;
				ret = qmc6g00x_write_reg(QMC6G00X_CTL_REG_ONE, 0x00);
				ret = qmc6g00x_write_reg(QMC6G00X_CTL_REG_TWO, ctrlb.value);
				ret = qmc6g00x_write_reg(QMC6G00X_CTL_REG_ONE, gTmr.ctl1_val);
				QMC6G00X_CHECK_ERR(ret);
				qmc6g00x_delay(1);
			}
		}
		else
		{
			gTmr.set_ctl.count = 0;
		}
	}
	else
	{
		int force_switch = 0;
		gTmr.set_ctl.count++;
		if(gTmr.set_ctl.count >= 100)
		{
			gTmr.set_ctl.count = 0;
			force_switch = 1;
		}
		if(((QMC6G00X_ABS(hw_d[0]) < 6000)&&(QMC6G00X_ABS(hw_d[1]) < 6000)) || force_switch)
		{
			qmc6g00x_ctrlb ctrlb;
			gTmr.set_ctl.mode = 0;
			gTmr.set_ctl.count = 0;

			ctrlb.value = gTmr.ctl2_val;
			ctrlb.bit.set_rst = QMC6G00X_RESET_ON;
			ret = qmc6g00x_write_reg(QMC6G00X_CTL_REG_ONE, 0x00);
			ret = qmc6g00x_write_reg(QMC6G00X_CTL_REG_TWO, ctrlb.value);
			ret = qmc6g00x_write_reg(QMC6G00X_CTL_REG_ONE, gTmr.ctl1_val);
			QMC6G00X_CHECK_ERR(ret);
			qmc6g00x_delay(1);
		}
	}

}
#endif

void qmc6g00x_fifo_config(qmc6g00x_fifo_mode mode, unsigned char wmk)
{
	int ret = QMC6G00X_OK;

	gTmr.fifo_ctrl = (mode|(wmk&0x0f));
	ret = qmc6g00x_write_reg(QMC6G00X_FIFO_REG_CTRL, gTmr.fifo_ctrl);
	QMC6G00X_CHECK_ERR(ret);
	qmc6g00x_delay(1);
}

int qmc6g00x_fifo_read(unsigned char *f_data)
{
	unsigned char fifo_status = 0;
	unsigned char fifo_level = 0;
	int ret = QMC6G00X_FAIL;

	ret = qmc6g00x_read_reg(QMC6G00X_FIFO_REG_STATUS, &fifo_status, 1);
	QMC6G00X_CHECK_ERR(ret);
	fifo_level = (fifo_status >> 3);
	if(fifo_level)
	{
		ret = qmc6g00x_read_reg(QMC6G00X_FIFO_REG_DATA, (unsigned char*)f_data, 6*fifo_level);
		ret = qmc6g00x_write_reg(QMC6G00X_FIFO_REG_CTRL, gTmr.fifo_ctrl);
		QMC6G00X_LOG("st:0x%x	%d\r\n", fifo_status, fifo_level);
		QMC6G00X_CHECK_ERR(ret);
	}

	return (int)fifo_level;
}

#if defined(QMC6G00X_RECOVER)
int qmc6g00x_recover(void)
{
	int ret = QMC6G00X_FAIL;
	unsigned char slave_loop[]={0x0c,0x1c,0x2c,0x3c,0x4c,0x5c,0x6c,0x7c};

	for(int i=0; i<sizeof(slave_loop)/sizeof(slave_loop[0]); i++)
	{
		gTmr.slave_addr = slave_loop[i];
		ret = qmc6g00x_get_chipid();	// read id again
		if(ret) 	// read id OK
		{
			QMC6G00X_LOG("qmc6g00x_recover slave=0x%02x read id OK\r\n", gTmr.slave_addr);
			qmc6g00x_soft_reset();	// softreset reload OTP
			gTmr.slave_addr = QMC6G00X_IIC_ADDR;
			ret = qmc6g00x_get_chipid();
			if(ret)
			{
				QMC6G00X_LOG("qmc6g00x_recover slave=0x%02x read id OK\r\n", gTmr.slave_addr);
			}
			else
			{
				QMC6G00X_LOG("qmc6g00x_recover slave=0x%02x read id fail\r\n", gTmr.slave_addr);
			}
			break;
		}
	}
 
	QMC6G00X_LOG("qmc6g00x_recover %s \r\n", ret?"OK":"FAIL");
	return ret;
}
#endif

int qmc6g00x_init(int protocol)
{
	int ret = 0;

	gTmr.protocol = (unsigned char)protocol;
	gTmr.slave_addr = QMC6G00X_IIC_ADDR;
	ret = qmc6g00x_get_chipid();
#if defined(QMC6G00X_RECOVER)
	if(!ret)
	{
		ret = qmc6g00x_recover();
	}
#endif
	if(ret)
	{
		qmc6g00x_soft_reset();
		qmc6g00x_init_para(QMC6G00X_MODE_HPFM, QMC6G00X_ODR_HPF);		//qmc6g00x_init_para(QMC6G00X_MODE_NORMAL, QMC6G00X_ODR_400HZ);
		ret = qmc6g00x_enable();
		QMC6G00X_CHECK_ERR(ret);
		//qmc6g00x_dump_reg();

		return 1;
	}
	else
	{
		return 0;
	}
}


