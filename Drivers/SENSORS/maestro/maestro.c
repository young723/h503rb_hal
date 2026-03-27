
#include "maestro.h"

static maestro_data_t tmr;
#if defined(MAESTRO_RECOVER)
int maestro_recover(void);
#endif

int maestro_read_reg(unsigned char addr, unsigned char *data, unsigned short len)
{
	int ret = MAESTRO_FAIL;
	int retry = 0;

	while((ret!=MAESTRO_OK) && (retry++ < 5))
	{
		if(tmr.protocol == 1)
			ret = bsp_i3c_read_reg(addr, data, len);
		else
			ret = bsp_i2c_read_reg(tmr.slave_addr, addr, data, len);
	}

	return ret;
}

int maestro_write_reg(unsigned char addr, unsigned char data)
{
	int ret = MAESTRO_FAIL;
	int retry = 0;

	while((ret!=MAESTRO_OK) && (retry++ < 5))
	{
		if(tmr.protocol == 1)
			ret = bsp_i3c_write_reg(addr, data);
		else
			ret = bsp_i2c_write_reg(tmr.slave_addr, addr, data);
	}

	return ret;
}

void maestro_delay(unsigned int ms)
{
	extern void qst_delay_ms(unsigned int n_ms);

	qst_delay_ms(ms);
}


void maestro_dump_reg(void)
{
	#define MAESTRO_REG_MAX	0x4f

	int i = 0;
	unsigned char i2c_0_2 = 0;
	unsigned char version_id = 0;
	unsigned char wafer_id = 0;
	unsigned short d_id = 0;
	unsigned char g_reg_tbl[80];
	
	for(i=0; i<=MAESTRO_REG_MAX; i++)
	{
		maestro_read_reg((unsigned char)i, &g_reg_tbl[i], 1);
		qst_delay_us(100);		
	}

	MAESTRO_LOG("\r\n******************maestro reg dump(hex)**************\r\n");
	MAESTRO_LOG("     |  0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
	MAESTRO_LOG("______________________________________________________\r\n");
	
	for(int i=0; i<5; i++)
	{
		int index = i*16;
		MAESTRO_LOG("0x%xx | %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\r\n", i,
																g_reg_tbl[index+0],g_reg_tbl[index+1],g_reg_tbl[index+2],g_reg_tbl[index+3],
																g_reg_tbl[index+4],g_reg_tbl[index+5],g_reg_tbl[index+6],g_reg_tbl[index+7],
																g_reg_tbl[index+8],g_reg_tbl[index+9],g_reg_tbl[index+10],g_reg_tbl[index+11],
																g_reg_tbl[index+12],g_reg_tbl[index+13],g_reg_tbl[index+14],g_reg_tbl[index+15]);
	}
	MAESTRO_LOG("****************maestro reg dump done****************\r\n");
	version_id = g_reg_tbl[0x12];
	wafer_id = g_reg_tbl[0x37]&0x1f;
	d_id = (unsigned short)((g_reg_tbl[0x39]<<8)|g_reg_tbl[0x38]);
	i2c_0_2 = ((g_reg_tbl[0x37]&0x20)>>3)|(g_reg_tbl[0x3b]&0x03);

	MAESTRO_LOG("Version-ID:[0x%02x] Wafer-ID:[0x%02x] Di-ID:[0x%04x] I2C[bit0-2]:[%x]\r\n", version_id, wafer_id, d_id, i2c_0_2);
	MAESTRO_LOG("********qst reg dump done********\r\n\r\n");
}

int maestro_get_chipid(void)
{
	int ret = MAESTRO_FAIL;
	int retry=0;
	unsigned char chip_id = 0x00;

	retry = 0;
	while(((chip_id != MAESTRO1V_CHIP_ID)&&(chip_id != MAESTRO2H_CHIP_ID)) && (retry++<5))
	{
		ret = maestro_read_reg(MAESTRO_CHIP_ID_REG, &chip_id, 1);
		if(ret == MAESTRO_OK)
		{
			break;
		}
	}
	if((chip_id == MAESTRO1V_CHIP_ID)||(chip_id == MAESTRO2H_CHIP_ID))
	{
		MAESTRO_LOG("maestro_get_chipid-ok slave:0x%x chipid = 0x%x\r\n", tmr.slave_addr, chip_id);
		return 1;
	}
	else
	{
		MAESTRO_LOG("maestro_get_chipid-fail slave:0x%x chip_id = 0x%x\r\n", tmr.slave_addr, chip_id);
		return 0;
	}
}

void maestro_set_range(unsigned char range)
{
	MAESTRO_LOG("maestro_set_range 0x%x\r\n", range);
	tmr.ctrlb.bit.range = range;

	switch(tmr.ctrlb.bit.range)
	{
#if 0
		case MAESTRO_RNG_32G:
			tmr.ssvt = 250;
			break;
		case MAESTRO_RNG_16G:
			tmr.ssvt = 500;
			break;
		case MAESTRO_RNG_8G:
			tmr.ssvt = 1000;
			break;
#else
		case MAESTRO1V_RNG_20G:
			tmr.ssvt = 1000;
			break;
#endif
		default:
			tmr.ssvt = 1000;
			break;
	}
}


void maestro_do_set(void)
{
	maestro1_ctrla			ctrl1;
	maestro1_ctrlb			ctrl2;

	ctrl1.value = tmr.ctrla.value;
	ctrl2.value = tmr.ctrla.value;

	if(ctrl1.bit.mode != MAESTRO_MODE_HPFM)
	{
		ctrl2.bit.odr = MAESTRO_ODR_1000HZ;
	}
	for(int i=0; i<t_mag.set_reset_num; i++)
	{
		// do set
		maestro_write_reg(MAESTRO_CTL_REG_ONE, 0x00);
		maestro_delay(1);
		ctrl2.bit.set_rst = MAESTRO_SET_ON;
		MAESTRO_LOG("maestro do set write 0x0a=0x%02x 0x0b=0x%02x\r\n", ctrl1.value, ctrl2.value);
		maestro_write_reg(MAESTRO_CTL_REG_TWO, ctrl2.value);
		maestro_write_reg(MAESTRO_CTL_REG_ONE, ctrl1.value);
		maestro_delay(2);
#if 1// do reset
		maestro_write_reg(MAESTRO_CTL_REG_ONE, 0x00);
		maestro_delay(1);
		ctrl2.bit.set_rst = MAESTRO_RESET_ON;
		MAESTRO_LOG("maestro do reset write 0x0a=0x%02x 0x0b=0x%02x\r\n", ctrl1.value, ctrl2.value);
		maestro_write_reg(MAESTRO_CTL_REG_TWO, ctrl2.value);
		maestro_write_reg(MAESTRO_CTL_REG_ONE, ctrl1.value);
		maestro_delay(2);
#endif
	}
	maestro_write_reg(MAESTRO_CTL_REG_ONE, 0x00);
}

#if defined(MAESTRO_KXKY)
void maestro_calc_kxky(unsigned char *preg)
{
#if 1
	signed char kx_calc = 0;
	signed short ky_calc = 0;
	
	unsigned char kx_code = preg[0] & 0x7f;
	unsigned short ky_code = ((preg[0] & 0x80) << 1) + (preg[1]);
	
	if (kx_code > 63){
		kx_calc = kx_code - 128;
	}
	else{
		kx_calc = kx_code;
	}
	kx_calc = MAESTRO_MAX(-64, MAESTRO_MIN(kx_calc, 63));
	
	if (ky_code > 255){
		ky_calc = ky_code - 512;
	}
	else{
		ky_calc = ky_code;
	}
	ky_calc = MAESTRO_MAX(-256, MAESTRO_MIN(ky_calc, 255));
	MAESTRO_LOG("1 0x4e-0x4f[0x%02x 0x%02x] kxky[%d %d]\r\n",preg[0], preg[1], kx_calc, ky_calc);
	tmr.kx = (float)kx_calc / 128.f;
	tmr.ky = (float)ky_calc / 128.f;
#endif
#if 0
	maestro_kxky	kxky;

	kxky.bit.kx = preg[0]&0x7f;
	kxky.bit.ky = (short)(((preg[0]&0x80)<<1)|preg[1]);
	MAESTRO_LOG("2 0x4e-0x4f[0x%02x 0x%02x] kxky[%d %d]\r\n", preg[0], preg[1], kxky.bit.kx, kxky.bit.ky);
	tmr.kx = (float)kxky.bit.kx / 128.f;
	tmr.ky = (float)kxky.bit.ky / 128.f;
#endif
	
	MAESTRO_LOG("maestro_calc_kxky [%f %f]  \r\n", tmr.kx, tmr.ky);
}
#endif

int maestro_enable(void)
{
	int ret = 0;
	unsigned char reg[2];

	ret = maestro_write_reg(MAESTRO_CTL_REG_ONE, 0x00);
	maestro_delay(1);
	MAESTRO_CHECK_ERR(ret);

#if defined(MAESTRO_KXKY)
	ret = maestro_read_reg(0x4e, reg, 2);
	maestro_calc_kxky(reg);
#endif
	MAESTRO_LOG("maestro_enable! 0x0a=0x%02x 0x0b=0x%02x\r\n", tmr.ctrla.value, tmr.ctrlb.value);
	ret = maestro_write_reg(MAESTRO_CTL_REG_TWO, tmr.ctrlb.value);
	MAESTRO_CHECK_ERR(ret);
	maestro_delay(1);
	ret = maestro_write_reg(MAESTRO_CTL_REG_ONE, tmr.ctrla.value);
	MAESTRO_CHECK_ERR(ret);
	maestro_delay(1);

#if defined(MAESTRO_0)
	unsigned char reg_v = 0;

	ret = maestro_read_reg(0x0e, &reg_v, 1);
	reg_v |= 0x04;	//MAESTRO0_SR_ON2;
	ret = maestro_write_reg(0x0e, reg_v);
#endif

	return ret;
}

int maestro_disable(void)
{
	int ret = 0;

	MAESTRO_LOG("maestro_disable!\r\n");
	ret = maestro_write_reg(MAESTRO_FIFO_REG_CTRL, 0x00);
	maestro_delay(1);
	ret = maestro_write_reg(MAESTRO_CTL_REG_TWO, 0x00);
	maestro_delay(1);
	ret = maestro_write_reg(MAESTRO_CTL_REG_ONE, 0x00);
	MAESTRO_CHECK_ERR(ret);

	return ret;
}

void maestro_enable_ibi(maestro_fifo_ibi flag)
{
	int ret = MAESTRO_FAIL;
	unsigned char ibi_value = 0x00;

	ibi_value = (unsigned char)flag;
	MAESTRO_LOG("maestro_enable_ibi 0x%02x\r\n", ibi_value);
	ret = maestro_write_reg(MAESTRO_CTL_IBI, ibi_value);

	MAESTRO_CHECK_ERR(ret);
}

void maestro_init_para(unsigned char mode, unsigned char odr)
{
	tmr.ctrla.bit.mode = mode;
	if(mode == MAESTRO_MODE_HPFM)
	{
		tmr.ctrla.bit.osr1 = MAESTRO1V_OSR1_8;
		tmr.ctrla.bit.osr2 = MAESTRO_OSR2_2;
	}
	else if(odr == MAESTRO_ODR_1000HZ)
	{
		tmr.ctrla.bit.osr1 = MAESTRO1V_OSR1_8;
		tmr.ctrla.bit.osr2 = MAESTRO_OSR2_2;
	}
	else if(odr == MAESTRO_ODR_400HZ)
	{
		tmr.ctrla.bit.osr1 = MAESTRO1V_OSR1_8;
		tmr.ctrla.bit.osr2 = MAESTRO_OSR2_2;
	}
	else if(odr == MAESTRO_ODR_200HZ)
	{
		tmr.ctrla.bit.osr1 = MAESTRO1V_OSR1_8;
		tmr.ctrla.bit.osr2 = MAESTRO_OSR2_2;
	}
	else
	{
		tmr.ctrla.bit.osr1 = MAESTRO1V_OSR1_8;
		tmr.ctrla.bit.osr2 = MAESTRO_OSR2_1;
	}

	tmr.ctrlb.bit.set_rst = MAESTRO_RESET_ON;
	tmr.ctrlb.bit.range = MAESTRO1V_RNG_20G;
	tmr.ctrlb.bit.odr = odr;
	tmr.ctrlb.bit.soft_rst = 0;

	maestro_set_range(tmr.ctrlb.bit.range);
#if defined(MAESTRO_MODE_SWITCH)
	tmr.set_ctl.mode = 0;
	tmr.set_ctl.count = 0;
#endif
}

#if 0
void maestro_reload_otp(void)
{
	int ret = 0;
	unsigned char status = 0;
	int retry = 0;
	int count = 0;

	MAESTRO_LOG("maestro_reload_otp\r\n");
	while(retry++ < 20)
	{
		ret = maestro_write_reg(0x28, 0x02);
		maestro_delay(2);
		if(ret != MAESTRO_OK)
		{
			MAESTRO_LOG("write 0x28 = 0x02 fail!\r\n");
		}
		count = 0;
		while(count++<100)
		{
			maestro_delay(1);
			status = 0;
			ret = maestro_read_reg(MAESTRO_STATUS_REG, &status, 1);
			if((ret==MAESTRO_OK)&&(status & 0x10))
			{
				MAESTRO_LOG("maestro_reload_otp done slave=0x%x status=0x%x\r\n", tmr.slave_addr, status);
				return;
			}
		}
	}
	
	MAESTRO_LOG("maestro_reload_otp fail\r\n");
}

void maestro_check_otp(void)
{
	int ret = MAESTRO_FAIL;
	int retry = 0;
	unsigned char status = 0x00;
	int count=10;

	while(count > 0)
	{
		count--;
		retry = 0;
		while(retry++<5)
		{
			ret = maestro_read_reg(MAESTRO_STATUS_REG, &status, 1);
			MAESTRO_CHECK_ERR(ret);
			MAESTRO_LOG("maestro status 0x%x\r\n", status);
			if(status & 0x10)
			{
				MAESTRO_LOG("maestro NVM load done!\r\n");
				return;
			}
			maestro_delay(1);
		}

		if(!(status & 0x10))
		{
			maestro_reload_otp();
		}
		else
		{
			return;
		}
	}
}
#endif

void maestro_soft_reset(void)
{
	int ret = MAESTRO_FAIL;
	int retry = 0;
	unsigned char status = 0x00;

	MAESTRO_LOG("maestro_soft_reset!\r\n");
	ret = maestro_write_reg(MAESTRO_CTL_REG_TWO, 0x80);
	MAESTRO_CHECK_ERR(ret);
	maestro_delay(5);

	while(retry++<5)
	{
		ret = maestro_read_reg(MAESTRO_STATUS_REG, &status, 1);
		MAESTRO_CHECK_ERR(ret);
		MAESTRO_LOG("maestro status 0x%x\r\n", status);
		if((status & 0x10)&&(status & 0x08))
		{
			MAESTRO_LOG("maestro NVM load done!\r\n");
			break;
		}
		maestro_delay(1);
	}
#if 0
	status = 0x00;
	ret = maestro_read_reg(0x40, &status, 1);
	MAESTRO_CHECK_ERR(ret);
	ret = maestro_write_reg(0x40, status|0x80);
	MAESTRO_CHECK_ERR(ret);
#endif
}

int maestro_read_mag_raw(short raw[3])
{
	static int maestro_fail_num = 0;
	int res = MAESTRO_FAIL;
	unsigned char mag_data[6];
	int t1 = 0;
	unsigned char rdy = 0;

	/* Check status register for data availability */
	res = maestro_read_reg(MAESTRO_STATUS_REG, &rdy, 1);
	while(!(rdy & (MAESTRO_STATUS_DRDY|MAESTRO_STATUS_OVFL)) & (t1++ < 5))
	{
		res = maestro_read_reg(MAESTRO_STATUS_REG, &rdy, 1);
		MAESTRO_CHECK_ERR(res);
		maestro_delay(1);
	}
	if((res == MAESTRO_FAIL)||(!(rdy & MAESTRO_STATUS_DRDY)))
  	{
		raw[0] = tmr.last_data[0];
		raw[1] = tmr.last_data[1];
		raw[2] = tmr.last_data[2];
		MAESTRO_LOG("maestro_read_mag_raw read drdy fail! res=%d rdy=0x%x\r\n",res,rdy);
		if(maestro_fail_num++ > 10)
		{
#if defined(MAESTRO_RECOVER)
			maestro_recover();
			maestro_enable();
#endif
			maestro_fail_num = 0;
		}
		res = MAESTRO_OK;	// MAESTRO_FAIL;
	}
	else if(rdy & MAESTRO_STATUS_OVFL)
	{
		raw[0] = 32767;
		raw[1] = 32767;
		raw[2] = 32767;
	}
	else
	{
		maestro_fail_num = 0;
		mag_data[0] = MAESTRO_DATA_OUT_X_LSB_REG;
		res = maestro_read_reg(MAESTRO_DATA_OUT_X_LSB_REG, mag_data, 6);
		if(res == MAESTRO_FAIL)
	  	{
			MAESTRO_LOG("maestro_read_mag_raw read data fail! res=%d\r\n",res);
			raw[0] = tmr.last_data[0];
			raw[1] = tmr.last_data[1];
			raw[2] = tmr.last_data[2];
			res = MAESTRO_OK;	// MAESTRO_FAIL;
		}
		else
		{
			raw[0] = (short)(((mag_data[1]) << 8) | mag_data[0]);
			raw[1] = (short)(((mag_data[3]) << 8) | mag_data[2]);
			raw[2] = (short)(((mag_data[5]) << 8) | mag_data[4]);
		}
	}
	
	tmr.last_data[0] = raw[0];
	tmr.last_data[1] = raw[1];
	tmr.last_data[2] = raw[2];	
#if defined(MAESTRO_MODE_SWITCH)
	maestro_setrst_auto_mode(raw);
#endif

	return res;
}

int maestro_read_mag_xyz(float *ut)
{
	int res = MAESTRO_FAIL;
	short raw[3];

	res = maestro_read_mag_raw(raw);
	if(res == MAESTRO_OK)
	{
		ut[0] = (float)((float)raw[0] / ((float)tmr.ssvt/100.f));		// ut
		ut[1] = (float)((float)raw[1] / ((float)tmr.ssvt/100.f));		// ut
		ut[2] = (float)((float)raw[2] / ((float)tmr.ssvt/100.f));		// ut
#if defined(MAESTRO_KXKY)
		ut[2] = ut[2] - (tmr.kx*ut[0]) - (tmr.ky*ut[1]);
#endif
	}
	else
	{
		ut[0] = ut[1]= ut[2] = 0.0f;
	}

	if(tmr.ctrla.bit.mode == MAESTRO_MODE_SINGLE)
	{
		res = maestro_write_reg(MAESTRO_CTL_REG_ONE, tmr.ctrla.value);
		MAESTRO_CHECK_ERR(res);
	}

	return res;
}

int maestro_self_test(void)
{
	int selftest_result = 0;
	int selftest_retry = 0;
	signed char  st_data[3];
	unsigned char abs_data[3];
	unsigned char rdy = 0x00;
	int t1 = 0;
	int ret = MAESTRO_FAIL;

	while((selftest_result == 0)&&(selftest_retry<3))
	{
		selftest_retry++;
		maestro_write_reg(MAESTRO_CTL_REG_ONE, 0x00);
		maestro_delay(2);
		maestro_write_reg(MAESTRO_CTL_REG_TWO, 0x00);
		maestro_delay(2);
		maestro_write_reg(MAESTRO_CTL_REG_ONE, 0x03);
		maestro_delay(20);
		maestro_write_reg(0x0e, 0x80);
		//maestro_delay(150);	// old 150ms
		rdy = 0x00;
		t1 = 0;
		while(!(rdy & 0x04))
		{
			maestro_delay(5);
			ret = maestro_read_reg(MAESTRO_STATUS_REG, &rdy, 1);

			if(t1++ > 50)
			{
				break;
			}
		}		

		if(rdy & 0x04)
		{
			ret = maestro_read_reg(MAESTRO_DATA_OUT_ST_X, (unsigned char*)st_data, 3);
			if(ret == MAESTRO_FAIL)
				continue;
		}
		else
		{
			MAESTRO_LOG("maestro selftest drdy fail!\r\n");
			continue;
		}

		abs_data[0] = MAESTRO_ABS(st_data[0]);
		abs_data[1] = MAESTRO_ABS(st_data[1]);
		abs_data[2] = MAESTRO_ABS(st_data[2]);

		if(	((abs_data[0] < MAESTRO_SELFTEST_MAX_X) && (abs_data[0] > MAESTRO_SELFTEST_MIN_X))
			&& ((abs_data[1] < MAESTRO_SELFTEST_MAX_Y) && (abs_data[1] > MAESTRO_SELFTEST_MIN_Y))
			&& ((abs_data[2] < MAESTRO_SELFTEST_MAX_Z) && (abs_data[2] > MAESTRO_SELFTEST_MIN_Z)) )
	    {
			MAESTRO_LOG("maestro selftest OK! status[0x%x]data[%d	%d	%d]\r\n",rdy,st_data[0],st_data[1],st_data[2]);
	        selftest_result = 1;
	    }
		else
		{
			MAESTRO_LOG("maestro selftest fail! status[0x%x]data[%d	%d	%d]\r\n",rdy,st_data[0],st_data[1],st_data[2]);
			selftest_result = 0;
		}
	}

	return selftest_result;
}

#if defined(MAESTRO_MODE_SWITCH)
void maestro_setrst_auto_mode(short hw_d[3])
{
	int ret = MAESTRO_FAIL;

	if(tmr.set_ctl.mode == 0)
	{// set reset on
		if((MAESTRO_ABS(hw_d[0]) > 8000)||(MAESTRO_ABS(hw_d[1]) > 8000))
		{
			tmr.set_ctl.count++;
			if(tmr.set_ctl.count >= 10)
			{
				tmr.set_ctl.mode = 1;
				tmr.set_ctl.count = 0;

				tmr.ctrlb.bit.set_rst = MAESTRO_SET_ON;
				ret = maestro_write_reg(MAESTRO_CTL_REG_TWO, tmr.ctrlb.value);
				MAESTRO_CHECK_ERR(ret);
				maestro_delay(1);
			}
		}
		else
		{
			tmr.set_ctl.count = 0;
		}
	}
	else
	{// set only
		int force_switch = 0;
		tmr.set_ctl.count++;
		if(tmr.set_ctl.count >= 100)
		{
			tmr.set_ctl.count = 0;
			force_switch = 1;
		}
		if(((MAESTRO_ABS(hw_d[0]) < 6000)&&(MAESTRO_ABS(hw_d[1]) < 6000)) || force_switch)
		{
			tmr.set_ctl.mode = 0;
			tmr.set_ctl.count = 0;

			tmr.ctrlb.bit.set_rst = MAESTRO_SET_RESET_ON;
			ret = maestro_write_reg(MAESTRO_CTL_REG_TWO, tmr.ctrlb.value);
			MAESTRO_CHECK_ERR(ret);
			maestro_delay(1);
		}
	}

}
#endif

void maestro_fifo_config(maestro_fifo_mode mode, unsigned char wmk)
{
	int ret = MAESTRO_OK;

#if defined(MAESTRO_0)
	tmr.fifo_ctrl = (mode|(wmk<<3)|0x07);
#else
	tmr.fifo_ctrl = (mode|(wmk&0x0f));
#endif
	ret = maestro_write_reg(MAESTRO_FIFO_REG_CTRL, tmr.fifo_ctrl);
	MAESTRO_CHECK_ERR(ret);
	maestro_delay(1);
}

int maestro_fifo_read(unsigned char *f_data)
{
	unsigned char fifo_status = 0;
	unsigned char fifo_level = 0;
	int ret = MAESTRO_FAIL;

	ret = maestro_read_reg(MAESTRO_FIFO_REG_STATUS, &fifo_status, 1);
	MAESTRO_CHECK_ERR(ret);
#if defined(MAESTRO_0)
	fifo_level = (fifo_status >> 4);
#else
	fifo_level = (fifo_status >> 3);
#endif
	if(fifo_level)
	{
		ret = maestro_read_reg(MAESTRO_FIFO_REG_DATA, (unsigned char*)f_data, 6*fifo_level);
		ret = maestro_write_reg(MAESTRO_FIFO_REG_CTRL, tmr.fifo_ctrl);
		MAESTRO_LOG("st:0x%x	%d\r\n", fifo_status, fifo_level);
		MAESTRO_CHECK_ERR(ret);
	}

	return (int)fifo_level;
}

#if defined(MAESTRO_RECOVER)
int maestro_recover(void)
{
	int ret = MAESTRO_FAIL;
	unsigned char slave_loop[]={0x0c,0x1c,0x2c,0x3c,0x4c,0x5c,0x6c,0x7c};

	for(int i=0; i<sizeof(slave_loop)/sizeof(slave_loop[0]); i++)
	{
		tmr.slave_addr = slave_loop[i];
		ret = maestro_get_chipid();	// read id again
		if(ret) 	// read id OK
		{
			MAESTRO_LOG("maestro_recover slave=0x%02x read id OK\r\n", tmr.slave_addr);
			maestro_soft_reset();	// softreset reload OTP
			tmr.slave_addr = MAESTRO_IIC_ADDR;
			ret = maestro_get_chipid();
			if(ret)
			{
				MAESTRO_LOG("maestro_recover slave=0x%02x read id OK\r\n", tmr.slave_addr);
			}
			else
			{
				MAESTRO_LOG("maestro_recover slave=0x%02x read id fail\r\n", tmr.slave_addr);
			}
			break;
		}
	}
 
	MAESTRO_LOG("maestro_recover %s \r\n", ret?"OK":"FAIL");
	return ret;
}
#endif

int maestro_init(int protocol)
{
	int ret = 0;

	tmr.protocol = (unsigned char)protocol;
	tmr.slave_addr = MAESTRO_IIC_ADDR;
	ret = maestro_get_chipid();
#if defined(MAESTRO_RECOVER)
	if(!ret)
	{
		ret = maestro_recover();
	}
#endif
	if(ret)
	{
		maestro_soft_reset();
		maestro_init_para(MAESTRO_MODE_HPFM, MAESTRO_ODR_HPF);
		//maestro_init_para(MAESTRO_MODE_NORMAL, MAESTRO_ODR_400HZ);
		maestro_do_set();
		ret = maestro_enable();
		MAESTRO_CHECK_ERR(ret);
		//maestro_dump_reg();

		return 1;
	}
	else
	{
		return 0;
	}
}


