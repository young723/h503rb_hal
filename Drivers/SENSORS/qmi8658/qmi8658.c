

#include "qmi8658.h"

//#define QMI8658_UINT_MG_DPS
#define M_PI			(3.14159265358979323846f)
#define ONE_G			(9.807f)
#define QFABS(x)		(((x)<0.0f)?(-1.0f*(x)):(x))

static qmi8658_state g_imu;
#if defined(QMI8658_FIRMWARE_DOWNLOAD)
const unsigned char fw_xanadu[] = 
{
#include "FW_1_3_13.hex"
};
#endif

int qmi8658_write_reg(unsigned char reg, unsigned char value)
{
	int ret=0;
	unsigned int retry = 0;

	while((!ret) && (retry++ < 5))
	{
		if(g_imu.protocol == 0)
			ret = bsp_i2c_write_reg(g_imu.slave, reg, value);
		else if(g_imu.protocol == 1)
			ret = bsp_i3c_write_reg(reg, value);
		else if(g_imu.protocol == 2)
			ret = bsp_spi_write_reg(reg, value);
	}

	return ret;
}

int qmi8658_write_regs(unsigned char reg, unsigned char *value, unsigned char len)
{
	int i, ret;

	for(i=0; i<len; i++)
	{
		ret = qmi8658_write_reg(reg+i, value[i]);
	}

	return ret;
}

int qmi8658_read_reg(unsigned char reg, unsigned char* buf, unsigned short len)
{
	int ret=0;
	unsigned int retry = 0;

	while((!ret) && (retry++ < 5))
	{
		if(g_imu.protocol == 0)
			ret = bsp_i2c_read_reg(g_imu.slave, reg, buf, len);
		else if(g_imu.protocol == 1)
			ret = bsp_i3c_read_reg(reg, buf, len);
		else if(g_imu.protocol == 2)
			ret = bsp_spi_read_reg(reg, buf, len);
	}

	return ret;
}

void qmi8658_delay(unsigned int ms)
{
	extern void qst_delay_ms(unsigned int n_ms);

	qst_delay_ms(ms);
}

void qmi8658_delay_us(unsigned int us)
{
	extern void qst_delay_us(unsigned int n_us);

	qst_delay_us(us);
}


void qmi8658_axis_convert(float data_a[3], float data_g[3], int layout)
{
	float raw[3],raw_g[3];

	raw[0] = data_a[0];
	raw[1] = data_a[1];
	//raw[2] = data[2];
	raw_g[0] = data_g[0];
	raw_g[1] = data_g[1];
	//raw_g[2] = data_g[2];

	if(layout >=4 && layout <= 7)
	{
		data_a[2] = -data_a[2];
		data_g[2] = -data_g[2];
	}

	if(layout%2)
	{
		data_a[0] = raw[1];
		data_a[1] = raw[0];
		
		data_g[0] = raw_g[1];
		data_g[1] = raw_g[0];
	}
	else
	{
		data_a[0] = raw[0];
		data_a[1] = raw[1];

		data_g[0] = raw_g[0];
		data_g[1] = raw_g[1];
	}

	if((layout==1)||(layout==2)||(layout==4)||(layout==7))
	{
		data_a[0] = -data_a[0];
		data_g[0] = -data_g[0];
	}
	if((layout==2)||(layout==3)||(layout==6)||(layout==7))
	{
		data_a[1] = -data_a[1];
		data_g[1] = -data_g[1];
	}
}


void qmi8658_config_acc(enum qmi8658_AccRange range, enum qmi8658_AccOdr odr, enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable)
{
	unsigned char ctl_dada;

	switch(range)
	{
		case Qmi8658AccRange_2g:
			g_imu.ssvt_a = (1<<14);
			break;
		case Qmi8658AccRange_4g:
			g_imu.ssvt_a = (1<<13);
			break;
		case Qmi8658AccRange_8g:
			g_imu.ssvt_a = (1<<12);
			break;
		case Qmi8658AccRange_16g:
			g_imu.ssvt_a = (1<<11);
			break;
		default: 
			range = Qmi8658AccRange_8g;
			g_imu.ssvt_a = (1<<12);
	}
	if(stEnable == Qmi8658St_Enable)
		ctl_dada = (unsigned char)range|(unsigned char)odr|0x80;
	else
		ctl_dada = (unsigned char)range|(unsigned char)odr;
		
	qmi8658_write_reg(Qmi8658Register_Ctrl2, ctl_dada);
// set LPF & HPF
	qmi8658_read_reg(Qmi8658Register_Ctrl5, &ctl_dada, 1);
	ctl_dada &= 0xf0;
	if(lpfEnable == Qmi8658Lpf_Enable)
	{
		ctl_dada |= A_LSP_MODE_3;
		ctl_dada |= 0x01;
	}
	else
	{
		ctl_dada &= ~0x01;
	}
	//ctl_dada = 0x00;
	qmi8658_write_reg(Qmi8658Register_Ctrl5,ctl_dada);
// set LPF & HPF
}

void qmi8658_config_gyro(enum qmi8658_GyrRange range, enum qmi8658_GyrOdr odr, enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable)
{
	// Set the CTRL3 register to configure dynamic range and ODR
	unsigned char ctl_dada; 

	// Store the scale factor for use when processing raw data
	switch (range)
	{
		case Qmi8658GyrRange_16dps:
			g_imu.ssvt_g = 2048;
			break;			
		case Qmi8658GyrRange_32dps:
			g_imu.ssvt_g = 1024;
			break;
		case Qmi8658GyrRange_64dps:
			g_imu.ssvt_g = 512;
			break;
		case Qmi8658GyrRange_128dps:
			g_imu.ssvt_g = 256;
			break;
		case Qmi8658GyrRange_256dps:
			g_imu.ssvt_g = 128;
			break;
		case Qmi8658GyrRange_512dps:
			g_imu.ssvt_g = 64;
			break;
		case Qmi8658GyrRange_1024dps:
			g_imu.ssvt_g = 32;
			break;
		case Qmi8658GyrRange_2048dps:
			g_imu.ssvt_g = 16;
			break;
//		case Qmi8658GyrRange_4096dps:
//			g_imu.ssvt_g = 8;
//			break;
		default: 
			range = Qmi8658GyrRange_512dps;
			g_imu.ssvt_g = 64;
			break;
	}

	if(stEnable == Qmi8658St_Enable)
		ctl_dada = (unsigned char)range|(unsigned char)odr|0x80;
	else
		ctl_dada = (unsigned char)range | (unsigned char)odr;
	qmi8658_write_reg(Qmi8658Register_Ctrl3, ctl_dada);

// Conversion from degrees/s to rad/s if necessary
// set LPF & HPF
	qmi8658_read_reg(Qmi8658Register_Ctrl5, &ctl_dada,1);
	ctl_dada &= 0x0f;
	if(lpfEnable == Qmi8658Lpf_Enable)
	{
		ctl_dada |= G_LSP_MODE_3;
		ctl_dada |= 0x10;
	}
	else
	{
		ctl_dada &= ~0x10;
	}
	//ctl_dada = 0x00;
	qmi8658_write_reg(Qmi8658Register_Ctrl5,ctl_dada);
// set LPF & HPF
}


int qmi8658_send_ctl9cmd(enum qmi8658_Ctrl9Command cmd, unsigned int d_ms)
{
	unsigned char status1 = 0x00;
	unsigned short count=0;
	unsigned char status_reg = Qmi8658Register_StatusInt;	
	unsigned char cmd_done = 0x80;
	unsigned char retry = 0;
	int ret1 = 0;
	int ret2 = 0;

#if defined(QMI8658_SYNC_SAMPLE_MODE)
	if(g_imu.cfg.syncSample == 1)
	{
		status_reg = Qmi8658Register_Status1;
		cmd_done = 0x01;
	}
#endif
	while(retry++ < 3)
	{
		qmi8658_write_reg(Qmi8658Register_Ctrl9, (unsigned char)cmd);	// write commond to ctrl9

		qmi8658_read_reg(status_reg, &status1, 1);
		while(((status1&cmd_done)!=cmd_done)&&(count++<100))		// read statusINT until bit7 is 1
		{
			if(d_ms > 0)
			{
				qmi8658_delay(d_ms);
			}
			qmi8658_read_reg(status_reg, &status1, 1);
		}
//		qmi8658_log("ctrl9 cmd (%d) done1 count=%d\n", cmd, count);
		if(count < 100)
		{
			ret1 = 1;
		}
		else
		{
			ret1 = 0;
		}

		qmi8658_write_reg(Qmi8658Register_Ctrl9, qmi8658_Ctrl9_Cmd_Ack);	// write commond  0x00 to ctrl9
		count = 0;
		qmi8658_read_reg(status_reg, &status1, 1);
		while(((status1&cmd_done)==cmd_done)&&(count++<100))		// read statusINT until bit7 is 0
		{
			if(d_ms > 0)
			{
				qmi8658_delay(1);
			}
			qmi8658_read_reg(status_reg, &status1, 1);
		}
//		qmi8658_log("ctrl9 cmd (%d) done2 count=%d\n", qmi8658_Ctrl9_Cmd_Ack, count);
		if(count < 100)
		{
			ret2 = 1;
		}
		else
		{
			ret2 = 0;
		}

		if((ret1==0) || (ret2==0))
		{
			continue;
		}
		else
		{
			break;
		}
	}

	if(ret1 && ret2)
	{
		return 1;
	}
	else
	{
		qmi8658_log("qmi8658_send_ctl9cmd fail cmd=%d\n", cmd);
		return 0;
	}
}

unsigned char qmi8658_readStatusInt(void)
{
	unsigned char status_int;

	qmi8658_read_reg(Qmi8658Register_StatusInt, &status_int, 1);

	return status_int;
}

unsigned char qmi8658_readStatus0(void)
{
	unsigned char status0;

	qmi8658_read_reg(Qmi8658Register_Status0, &status0, 1);

	return status0;
}

unsigned char qmi8658_readStatus1(void)
{
	unsigned char status1;
	
	qmi8658_read_reg(Qmi8658Register_Status1, &status1, 1);

	return status1;
}

float qmi8658_readTemp(void)
{
	unsigned char buf[2];
	short temp = 0;
	float temp_f = 0;

	qmi8658_read_reg(Qmi8658Register_Tempearture_L, buf, 2);
	temp = ((short)buf[1]<<8)|buf[0];
	temp_f = (float)temp/256.0f;

	return temp_f;
}

void qmi8658_read_timestamp(unsigned int *tim_count)
{
	unsigned char	buf[3];
	unsigned int timestamp;

	if(tim_count)
	{
		qmi8658_read_reg(Qmi8658Register_Timestamp_L, buf, 3);
		timestamp = (unsigned int)(((unsigned int)buf[2]<<16)|((unsigned int)buf[1]<<8)|buf[0]);
		if(timestamp > g_imu.timestamp)
			g_imu.timestamp = timestamp;
		else
			g_imu.timestamp = (timestamp+0x1000000-g_imu.timestamp);

		*tim_count = g_imu.timestamp;		
	}
}

void qmi8658_read_xyz_raw(short acc[3], short gyro[3])
{
	unsigned char	status = 0;
	unsigned char data_ready = 0;
	int retry = 0;

	while(retry++ < 3)
	{
#if defined(QMI8658_SYNC_SAMPLE_MODE)
		qmi8658_read_reg(Qmi8658Register_StatusInt, &status, 1);
		if((status==0x01)||(status==0x03))
		{
			data_ready = 1;
			qmi8658_delay_us(12);	// delay 12us <=500Hz£¬ 12us 1000Hz, 4us 2000Hz 2us > 2000Hz
			break;
		}
#else
		qmi8658_read_reg(Qmi8658Register_Status0, &status, 1);
		//qmi8658_log("status0 0x%x\n", status);
		if(status&0x03)
		{
			data_ready = 1;
			break;
		}
#endif
	}
	if(data_ready)
	{
		unsigned char	buf_reg[12];
		int ret = 0;

		ret = qmi8658_read_reg(Qmi8658Register_Ax_L, buf_reg, 12);
		if(!ret)
		{
			qmi8658_log("qmi8658_read_xyz_raw fail!\r\n");
		}
		acc[0] = (short)((unsigned short)(buf_reg[1]<<8) |( buf_reg[0]));
		acc[1] = (short)((unsigned short)(buf_reg[3]<<8) |( buf_reg[2]));
		acc[2] = (short)((unsigned short)(buf_reg[5]<<8) |( buf_reg[4]));
		gyro[0] = (short)((unsigned short)(buf_reg[7]<<8) |( buf_reg[6]));
		gyro[1] = (short)((unsigned short)(buf_reg[9]<<8) |( buf_reg[8]));
		gyro[2] = (short)((unsigned short)(buf_reg[11]<<8) |( buf_reg[10]));

		g_imu.imu[0] = acc[0];
		g_imu.imu[1] = acc[1];
		g_imu.imu[2] = acc[2];
		g_imu.imu[3] = gyro[0];
		g_imu.imu[4] = gyro[1];
		g_imu.imu[5] = gyro[2];
	}
	else
	{	
		qmi8658_log("qmi8658_read_xyz_raw drdy fail!\r\n");
		acc[0] = g_imu.imu[0];
		acc[1] = g_imu.imu[1];
		acc[2] = g_imu.imu[2];
		gyro[0] = g_imu.imu[3];
		gyro[1] = g_imu.imu[4];
		gyro[2] = g_imu.imu[5];
	}
}


void qmi8658_read_xyz(float acc[3], float gyro[3])
{
	short 			raw_acc[3];
	short 			raw_gyro[3];

	qmi8658_read_xyz_raw(raw_acc, raw_gyro);

#if defined(QMI8658_UINT_MG_DPS)
	// mg
	acc[0] = (float)(raw_acc[0]*1000.0f)/g_imu.ssvt_a;
	acc[1] = (float)(raw_acc[1]*1000.0f)/g_imu.ssvt_a;
	acc[2] = (float)(raw_acc[2]*1000.0f)/g_imu.ssvt_a;
#else
	// m/s2
	acc[0] = (float)(raw_acc[0]*ONE_G)/g_imu.ssvt_a;
	acc[1] = (float)(raw_acc[1]*ONE_G)/g_imu.ssvt_a;
	acc[2] = (float)(raw_acc[2]*ONE_G)/g_imu.ssvt_a;
#endif
		
#if defined(QMI8658_UINT_MG_DPS)
	// dps
	gyro[0] = (float)(raw_gyro[0]*1.0f)/g_imu.ssvt_g;
	gyro[1] = (float)(raw_gyro[1]*1.0f)/g_imu.ssvt_g;
	gyro[2] = (float)(raw_gyro[2]*1.0f)/g_imu.ssvt_g;
#else
	// rad/s
	gyro[0] = (float)(raw_gyro[0]*M_PI)/(g_imu.ssvt_g*180); 	// *pi/180
	gyro[1] = (float)(raw_gyro[1]*M_PI)/(g_imu.ssvt_g*180);
	gyro[2] = (float)(raw_gyro[2]*M_PI)/(g_imu.ssvt_g*180);
#endif

//	qmi8658_axis_convert(acc, gyro, 0);
}


void qmi8658_read_imu_raw(float imu[6], short raw[6])
{
	qmi8658_read_xyz_raw(&raw[0], &raw[3]);

#if defined(QMI8658_UINT_MG_DPS)
	// mg
	imu[0] = (float)(raw[0]*1000.0f)/g_imu.ssvt_a;
	imu[1] = (float)(raw[1]*1000.0f)/g_imu.ssvt_a;
	imu[2] = (float)(raw[2]*1000.0f)/g_imu.ssvt_a;
#else
	// m/s2
	imu[0] = (float)(raw[0]*ONE_G)/g_imu.ssvt_a;
	imu[1] = (float)(raw[1]*ONE_G)/g_imu.ssvt_a;
	imu[2] = (float)(raw[2]*ONE_G)/g_imu.ssvt_a;
#endif
		
#if defined(QMI8658_UINT_MG_DPS)
	// dps
	imu[3] = (float)(raw[3]*1.0f)/g_imu.ssvt_g;
	imu[4] = (float)(raw[4]*1.0f)/g_imu.ssvt_g;
	imu[5] = (float)(raw[5]*1.0f)/g_imu.ssvt_g;
#else
	// rad/s
	imu[3] = (float)(raw[3]*M_PI)/(g_imu.ssvt_g*180); 	// *pi/180
	imu[4] = (float)(raw[4]*M_PI)/(g_imu.ssvt_g*180);
	imu[5] = (float)(raw[5]*M_PI)/(g_imu.ssvt_g*180);
#endif
}


#if defined(QMI8658_SYNC_SAMPLE_MODE)
void qmi8658_enable_AHB_clock(int enable)
{
	if(enable)
	{
		qmi8658_write_reg(Qmi8658Register_Ctrl7, 0x00);
		qmi8658_write_reg(Qmi8658Register_Cal1_L, 0x00);
		qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_AHB_Clock_Gating, 1);
	}
	else
	{
		qmi8658_write_reg(Qmi8658Register_Ctrl7, 0x00);
		qmi8658_write_reg(Qmi8658Register_Cal1_L, 0x01);
		qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_AHB_Clock_Gating, 1);
	}
}
#endif

void qmi8658_enableSensors(unsigned char enableFlags)
{
#if defined(QMI8658_SYNC_SAMPLE_MODE)
	qmi8658_write_reg(Qmi8658Register_Ctrl7, enableFlags | 0x80);
#else
	qmi8658_write_reg(Qmi8658Register_Ctrl7, enableFlags);		// QMI8658_DRDY_DISABLE
#endif
	g_imu.cfg.enSensors = enableFlags&0x03;

	qmi8658_delay(2);
}

void qmi8658_dump_reg(void)
{
#if 0
    unsigned char read_data[8];
 
    qmi8658_read_reg(Qmi8658Register_Ctrl1, read_data, 8);
    qmi8658_log("Ctrl1[0x%x]\nCtrl2[0x%x]\nCtrl3[0x%x]\nCtrl4[0x%x]\nCtrl5[0x%x]\nCtrl6[0x%x]\nCtrl7[0x%x]\nCtrl8[0x%x]\n",
                    read_data[0],read_data[1],read_data[2],read_data[3],read_data[4],read_data[5],read_data[6],read_data[7]);
    //qmi8658_read_reg(Qmi8658Register_FifoWmkTh, read_data, 4);
    //qmi8658_log("FIFO reg[0x%x 0x%x 0x%x 0x%x]\n", read_data[0],read_data[1],read_data[2],read_data[3]);
    qmi8658_log("\n");
#else
	unsigned char g_reg_tbl[128];

	qmi8658_read_reg(0x00, &g_reg_tbl[0], 16);
	qmi8658_read_reg(0x10, &g_reg_tbl[16], 16);
	qmi8658_read_reg(0x20, &g_reg_tbl[32], 16);
	qmi8658_read_reg(0x30, &g_reg_tbl[48], 16);
	qmi8658_read_reg(0x40, &g_reg_tbl[64], 16);
	qmi8658_read_reg(0x50, &g_reg_tbl[80], 16);
	qst_logi("\r\n********qst reg dump(hex)********\r\n");
	qst_logi("     |  0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
	qst_logi("______________________________________________________\r\n");
	
	for(int i=0; i<6; i++)
	{
		int index = i*16;
		qst_logi("0x%xx | %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\r\n", i,
																g_reg_tbl[index+0],g_reg_tbl[index+1],g_reg_tbl[index+2],g_reg_tbl[index+3],
																g_reg_tbl[index+4],g_reg_tbl[index+5],g_reg_tbl[index+6],g_reg_tbl[index+7],
																g_reg_tbl[index+8],g_reg_tbl[index+9],g_reg_tbl[index+10],g_reg_tbl[index+11],
																g_reg_tbl[index+12],g_reg_tbl[index+13],g_reg_tbl[index+14],g_reg_tbl[index+15]);
	}
	qst_logi("********qst reg dump done********\r\n\r\n");

#endif
}

void qmi8658_read_chip_info(void)
{
	unsigned char revision_id = 0x00;
	unsigned char firmware_id[3];
	unsigned char uuid[6];
//	unsigned int uuid_low, uuid_high;

	qmi8658_read_reg(Qmi8658Register_Revision, &revision_id, 1);
	qmi8658_read_reg(Qmi8658Register_firmware_id, firmware_id, 3);
	qmi8658_read_reg(Qmi8658Register_uuid, uuid, 6);
	g_imu.fwid[0] = firmware_id[2];
	g_imu.fwid[1] = firmware_id[1];
	g_imu.fwid[2] = firmware_id[0];
	g_imu.uuid[0] = (unsigned int)((unsigned int)(uuid[2]<<16)|(unsigned int)(uuid[1]<<8)|(uuid[0]));
	g_imu.uuid[1] = (unsigned int)((unsigned int)(uuid[5]<<16)|(unsigned int)(uuid[4]<<8)|(uuid[3]));
	//qmi8658_log("VS ID[0x%x]\n", revision_id);
	qmi8658_log("**FW ID[%d %d %d] Revision;0x%x\n", firmware_id[2], firmware_id[1],firmware_id[0],revision_id);
	qmi8658_log("**UUID[0x%x %x]\n", g_imu.uuid[1] ,g_imu.uuid[0]);
}

void qmi8658_get_chip_info(unsigned int info[3])
{
	info[0] = (unsigned int)(((unsigned int)0x05<<24)|((unsigned int)g_imu.fwid[0]<<16)|((unsigned int)g_imu.fwid[1]<<8)|((unsigned int)g_imu.fwid[2]));
	info[1] = g_imu.uuid[0];
	info[2] = g_imu.uuid[1];
}

void qmi8658_soft_reset(void)
{
	unsigned char reset_done = 0x00;
	int retry = 0;

	qmi8658_log("qmi8658_soft_reset \n");
	qmi8658_write_reg(Qmi8658Register_Reset, 0xb0);
	qmi8658_delay(15);	// delay
#if (QMI8658_USE_SPI == 3)
	qmi8658_write_reg(Qmi8658Register_Ctrl1, 0x80|0x60);
	qmi8658_delay(5);
#endif
	while(reset_done != 0x80)
	{
		qmi8658_delay(1);
		qmi8658_read_reg(Qmi8658Register_Reset_done, &reset_done, 1);
		if(retry++ > 500)
		{
			break;
		}
	}
	qmi8658_log("qmi8658_soft_reset done retry=%d\n", retry);
}

void qmi8658_get_gyro_gain(unsigned char cod_data[6])
{
	qmi8658_read_reg(Qmi8658Register_Dvx_L, &cod_data[0], 6);
	qmi8658_log("cod data[0x%x 0x%x 0x%x 0x%x 0x%x 0x%x]\n", cod_data[0],cod_data[1],cod_data[2],
															cod_data[3],cod_data[4],cod_data[5]);
}

void qmi8658_apply_gyr_gain(unsigned char cod_data[6])
{
	qmi8658_log("qmi8658_apply_gyr_gain [0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]cod[%d %d %d]\n", 
				cod_data[0],cod_data[1],cod_data[2],cod_data[3],cod_data[4],cod_data[5],
				(unsigned short)(cod_data[1]<<8|cod_data[0]),
				(unsigned short)(cod_data[3]<<8|cod_data[2]),
				(unsigned short)(cod_data[5]<<8|cod_data[4]));

	qmi8658_enableSensors(QMI8658_DISABLE_ALL);
	qmi8658_write_reg(Qmi8658Register_Cal1_L, cod_data[0]);
	qmi8658_write_reg(Qmi8658Register_Cal1_H, cod_data[1]);
	qmi8658_write_reg(Qmi8658Register_Cal2_L, cod_data[2]);
	qmi8658_write_reg(Qmi8658Register_Cal2_H, cod_data[3]);
	qmi8658_write_reg(Qmi8658Register_Cal3_L, cod_data[4]);
	qmi8658_write_reg(Qmi8658Register_Cal3_H, cod_data[5]);

	qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_Apply_Gyro_Gain, 1);
}

void qmi8658_on_demand_cali(void)
{
	unsigned char cod_status = 0x00;

	qmi8658_log("qmi8658_on_demand_cali start\n");
	qmi8658_write_reg(Qmi8658Register_Ctrl9, (unsigned char)qmi8658_Ctrl9_Cmd_On_Demand_Cali);
	qmi8658_delay(2200);	// delay 2000ms above
	qmi8658_write_reg(Qmi8658Register_Ctrl9, (unsigned char)qmi8658_Ctrl9_Cmd_Ack);
	qmi8658_delay(10);		// delay
	qmi8658_read_reg(Qmi8658Register_Cod_Status, &cod_status, 1);
	if(cod_status)
	{
		qmi8658_log("qmi8658_on_demand_cali fail! status=0x%x\n", cod_status);
	}
	else
	{
		qmi8658_get_gyro_gain(g_imu.cod_data);
		qmi8658_log("qmi8658_on_demand_cali done! [0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]cod[%d %d %d]\n", 
			g_imu.cod_data[0],g_imu.cod_data[1],g_imu.cod_data[2],g_imu.cod_data[3],g_imu.cod_data[4],g_imu.cod_data[5],
			(unsigned short)(g_imu.cod_data[1]<<8|g_imu.cod_data[0]),
			(unsigned short)(g_imu.cod_data[3]<<8|g_imu.cod_data[2]),
			(unsigned short)(g_imu.cod_data[5]<<8|g_imu.cod_data[4]));
	}
}

void qmi8658_config_reg(unsigned char low_power)
{
	qmi8658_enableSensors(QMI8658_DISABLE_ALL);
	if(low_power)
	{
		g_imu.cfg.enSensors = QMI8658_ACC_ENABLE;
		g_imu.cfg.accRange = Qmi8658AccRange_8g;
		g_imu.cfg.accOdr = Qmi8658AccOdr_LowPower_21Hz;
		g_imu.cfg.gyrRange = Qmi8658GyrRange_1024dps;
		g_imu.cfg.gyrOdr = Qmi8658GyrOdr_125Hz;
	}
	else
	{
		g_imu.cfg.enSensors = QMI8658_ACCGYR_ENABLE;
		g_imu.cfg.accRange = Qmi8658AccRange_8g;
		g_imu.cfg.accOdr = Qmi8658AccOdr_125Hz;
		g_imu.cfg.gyrRange = Qmi8658GyrRange_1024dps;
		g_imu.cfg.gyrOdr = Qmi8658GyrOdr_125Hz;
	}
	
	if(g_imu.cfg.enSensors & QMI8658_ACC_ENABLE)
	{
		qmi8658_config_acc(g_imu.cfg.accRange, g_imu.cfg.accOdr, Qmi8658Lpf_Disable, Qmi8658St_Disable);
	}
	if(g_imu.cfg.enSensors & QMI8658_GYR_ENABLE)
	{
		qmi8658_config_gyro(g_imu.cfg.gyrRange, g_imu.cfg.gyrOdr, Qmi8658Lpf_Disable, Qmi8658St_Disable);
	}
}


unsigned char qmi8658_get_id(void)
{
	unsigned char qmi8658_chip_id = 0x00;
	unsigned char qmi8658_slave[2] = {QMI8658_SLAVE_ADDR_L, QMI8658_SLAVE_ADDR_H};
	unsigned char ctrl1_value = 0x60;
	int retry = 0;
	unsigned char iCount = 0;

	iCount = 0;
	while(iCount<2)
	{	
		retry = 0;
#if defined(QMI8658_USE_SPI)
		g_imu.slave = 0x00;
	#if (QMI8658_USE_SPI == 3)
		ctrl1_value |= 0x80;
		qmi8658_write_reg(Qmi8658Register_Ctrl1, ctrl1_value);
		qmi8658_delay(5);
	#endif
#else
		g_imu.slave = qmi8658_slave[iCount];
#endif
		while((qmi8658_chip_id != 0x05)&&(retry++ < 5))
		{
			qmi8658_read_reg(Qmi8658Register_WhoAmI, &qmi8658_chip_id, 1);
			qmi8658_delay(2);
		}
		qmi8658_log("qmi8658 slave = 0x%x WhoAmI = 0x%x\n", g_imu.slave, qmi8658_chip_id);
		if(qmi8658_chip_id == 0x05)
		{
			g_imu.cfg.syncSample = 0;
			g_imu.cfg.ctrl8_value = 0xc0;
#if defined(QMI8658_FIRMWARE_DOWNLOAD)
			qmi8658_firmware_download(fw_xanadu, sizeof(fw_xanadu));
#endif
			qmi8658_soft_reset();
			// read chip id again
#if (QMI8658_USE_SPI == 3)
			qmi8658_chip_id = 0x00;
			qmi8658_read_reg(Qmi8658Register_WhoAmI, &qmi8658_chip_id, 1);
			if(qmi8658_chip_id != 0x05)
			{
				qmi8658_log("qmi8658 read chip id fail after soft reset!\r\n");
				return qmi8658_chip_id;
			}
#endif
			qmi8658_write_reg(Qmi8658Register_Ctrl1, ctrl1_value|QMI8658_INT2_ENABLE|QMI8658_INT1_ENABLE);
			qmi8658_read_chip_info();
#if defined(QMI8658_USE_GYRO_STARTUP_TEST)	// check 0x45
			unsigned char opt_status = 0x00;
			qmi8658_write_reg(Qmi8658Register_Ctrl2, 0x25);
			qmi8658_write_reg(Qmi8658Register_Ctrl3, 0x66);
			qmi8658_write_reg(Qmi8658Register_Ctrl7, 0x03);
			qmi8658_delay(300);
			qmi8658_read_reg(0x45, &opt_status, 1);
			//qmi8658_log("opt_status = 0x%x\n", opt_status);
			if(opt_status != 0x80)
			{
				qmi8658_log("**ERROR[0x45=0x%x]\n", opt_status);
				//return 0;
			}
			else
			{
				qmi8658_log("**SUCCESS[0x45=0x%x]\n", opt_status);
			}
			qmi8658_write_reg(Qmi8658Register_Ctrl7, 0x00);
#endif
#if defined(QMI8658_USE_HW_SELFTEST)
			qmi8658_do_hw_selftest(QMI8658_ACCGYR_ENABLE);
			//qmi8658_soft_reset();
			//qmi8658_write_reg(Qmi8658Register_Ctrl1, 0x60|QMI8658_INT2_ENABLE|QMI8658_INT1_ENABLE);
#endif
#if defined(QMI8658_USE_ON_DEMAND_CALI)
			qmi8658_on_demand_cali();
#endif
			qmi8658_write_reg(Qmi8658Register_Ctrl7, 0x00);
			qmi8658_write_reg(Qmi8658Register_Ctrl8, g_imu.cfg.ctrl8_value);
#if defined(QMI8658_EN_CGAIN)
			qmi8658_write_reg(BANK0_INDIRECT_SYS_ADDR_7_0_ADDR, 0x2a);
			qmi8658_write_reg(BANK0_INDIRECT_SYS_ADDR_15_8_ADDR, 0x00);
			qmi8658_write_reg(BANK0_INDIRECT_SYS_ADDR_23_16_ADDR, 0x06);
			qmi8658_write_reg(BANK0_INDIRECT_SYS_ADDR_31_24_ADDR, 0x00);
#endif
			break;
		}
		iCount++;
	}
	if(qmi8658_chip_id != 0x05)
	{		
		qmi8658_log("**ERROR1[id=0x%x]\n", qmi8658_chip_id);
	}

	return qmi8658_chip_id;
}

#if defined(QMI8658_USE_WOM)
void qmi8658_enable_wom(int enable, enum qmi8658_Interrupt int_map)
{
	unsigned char ctrl1 = 0x00;
	unsigned char wom_int = 0x00;

	if(enable)
	{
		qmi8658_log("qmi8658_enable_wom\n");
		wom_int |= 0x0a;	// Blanking Time
		if(int_map == qmi8658_Int2)
		{
			wom_int |= (0x01<<6);		// map to int2
		}
		//wom_int |= (0x01<<7); 	// int iniial state is high
		//qmi8658_soft_reset();
		qmi8658_enableSensors(QMI8658_DISABLE_ALL);
		qmi8658_delay(2);
		qmi8658_config_acc(Qmi8658AccRange_8g, Qmi8658AccOdr_31_25Hz ,Qmi8658Lpf_Disable,Qmi8658St_Disable);
		qmi8658_write_reg(Qmi8658Register_Cal1_L, 255);		// threshold  mg
		qmi8658_write_reg(Qmi8658Register_Cal1_H, wom_int);
		qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_WoM_Setting, 1);
		qmi8658_enableSensors(QMI8658_ACC_ENABLE);

		qmi8658_read_reg(Qmi8658Register_Ctrl1, &ctrl1, 1);
		if(int_map == qmi8658_Int1)
		{
			ctrl1 |= QMI8658_INT1_ENABLE;
		}
		else if(int_map == qmi8658_Int2)
		{
			ctrl1 |= QMI8658_INT2_ENABLE;
		}
		qmi8658_write_reg(Qmi8658Register_Ctrl1, ctrl1);// enable int for dev-E
	}
	else
	{
		qmi8658_enableSensors(QMI8658_DISABLE_ALL);
		qmi8658_write_reg(Qmi8658Register_Cal1_L, 0);
		qmi8658_write_reg(Qmi8658Register_Cal1_H, 0);
		qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_WoM_Setting, 1);
	}
}
#endif

#if defined(QMI8658_USE_AMD)||defined(QMI8658_USE_NO_MOTION)||defined(QMI8658_USE_SIG_MOTION)
void qmi8658_config_motion(void)
{
	g_imu.cfg.ctrl8_value = 0xc0;	// &= (~QMI8658_CTRL8_ANYMOTION_EN);
	qmi8658_write_reg(Qmi8658Register_Ctrl8, g_imu.cfg.ctrl8_value);
	qmi8658_delay(2);
	qmi8658_enableSensors(QMI8658_DISABLE_ALL);

	qmi8658_write_reg(Qmi8658Register_Cal1_L, 0x02);		// any motion X threshold(uint 1/32 g)
	qmi8658_write_reg(Qmi8658Register_Cal1_H, 0x02);		// any motion Y threshold(uint 1/32 g)
	qmi8658_write_reg(Qmi8658Register_Cal2_L, 0x02);		// any motion Z threshold(uint 1/32 g)
	qmi8658_write_reg(Qmi8658Register_Cal2_H, 0x02);		// no motion X threshold(uint 1/32 g)
	qmi8658_write_reg(Qmi8658Register_Cal3_L, 0x02);		// no motion X threshold(uint 1/32 g)
	qmi8658_write_reg(Qmi8658Register_Cal3_H, 0x02);		// no motion X threshold(uint 1/32 g)

	qmi8658_write_reg(Qmi8658Register_Cal4_L, 0xf7);		// MOTION_MODE_CTRL
	qmi8658_write_reg(Qmi8658Register_Cal4_H, 0x01);		// value 0x01

	qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_Motion, 1);

	qmi8658_write_reg(Qmi8658Register_Cal1_L, 0x03);		// AnyMotionWindow. 
	qmi8658_write_reg(Qmi8658Register_Cal1_H, 0xff);		// NoMotionWindow 
	qmi8658_write_reg(Qmi8658Register_Cal2_L, 0x2c);		// SigMotionWaitWindow[7:0]
	qmi8658_write_reg(Qmi8658Register_Cal2_H, 0x01);		// SigMotionWaitWindow [15:8]
	qmi8658_write_reg(Qmi8658Register_Cal3_L, 0x64);		// SigMotionConfirmWindow[7:0]
	qmi8658_write_reg(Qmi8658Register_Cal3_H, 0x00);		// SigMotionConfirmWindow[15:8]
	//qmi8658_write_reg(Qmi8658Register_Cal4_L, 0xf7);
	qmi8658_write_reg(Qmi8658Register_Cal4_H, 0x02);		// value 0x02

	qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_Motion, 1);
}

#if defined(QMI8658_USE_AMD)
void qmi8658_enable_amd(unsigned char enable, enum qmi8658_Interrupt int_map, unsigned char low_power)
{
	if(enable)
	{
		unsigned char ctrl1;

		qmi8658_write_reg(Qmi8658Register_Ctrl8, 0xc0);
		qmi8658_delay(2);
		qmi8658_enableSensors(QMI8658_DISABLE_ALL);
//		qmi8658_config_reg(low_power);
//		qmi8658_delay(100);

		qmi8658_read_reg(Qmi8658Register_Ctrl1, &ctrl1, 1);
		if(int_map == qmi8658_Int1)
		{
			ctrl1 |= QMI8658_INT1_ENABLE;
			g_imu.cfg.ctrl8_value |= QMI8658_CTRL8_INT_SEL;
		}
		else if(int_map == qmi8658_Int2)
		{
			ctrl1 |= QMI8658_INT2_ENABLE;
			g_imu.cfg.ctrl8_value &= (~QMI8658_CTRL8_INT_SEL);
		}
		qmi8658_write_reg(Qmi8658Register_Ctrl1, ctrl1);// enable int for dev-E

		g_imu.cfg.ctrl8_value |= QMI8658_CTRL8_ANYMOTION_EN;
		qmi8658_write_reg(Qmi8658Register_Ctrl8, g_imu.cfg.ctrl8_value);

		g_imu.cfg.enSensors = QMI8658_ACC_ENABLE;	//QMI8658_ACC_ENABLE; QMI8658_ACCGYR_ENABLE
		if(low_power)
		{
			qmi8658_config_acc(Qmi8658AccRange_8g, Qmi8658AccOdr_LowPower_21Hz, Qmi8658Lpf_Disable, Qmi8658St_Disable);
			qmi8658_config_gyro(Qmi8658GyrRange_1024dps, Qmi8658GyrOdr_31_25Hz, Qmi8658Lpf_Disable, Qmi8658St_Disable);
			qmi8658_delay(100);
		}
		qmi8658_enableSensors(g_imu.cfg.enSensors|QMI8658_DRDY_DISABLE);
	}
	else
	{
		g_imu.cfg.ctrl8_value &= (~QMI8658_CTRL8_ANYMOTION_EN);
		qmi8658_write_reg(Qmi8658Register_Ctrl8, g_imu.cfg.ctrl8_value);
	}
}
#endif

#if defined(QMI8658_USE_NO_MOTION)
void qmi8658_enable_no_motion(unsigned char enable, enum qmi8658_Interrupt int_map)
{
	if(enable)
	{
		unsigned char ctrl1;

		qmi8658_enableSensors(QMI8658_DISABLE_ALL);

		qmi8658_read_reg(Qmi8658Register_Ctrl1, &ctrl1, 1);
		if(int_map == qmi8658_Int1)
		{
			ctrl1 |= QMI8658_INT1_ENABLE;
			g_imu.cfg.ctrl8_value |= QMI8658_CTRL8_INT_SEL;
		}
		else if(int_map == qmi8658_Int2)
		{
			ctrl1 |= QMI8658_INT2_ENABLE;
			g_imu.cfg.ctrl8_value &= (~QMI8658_CTRL8_INT_SEL);
		}
		qmi8658_write_reg(Qmi8658Register_Ctrl1, ctrl1);// enable int for dev-E

		// recommend odr		
		qmi8658_config_acc(Qmi8658AccRange_8g, Qmi8658AccOdr_62_5Hz, Qmi8658Lpf_Disable, Qmi8658St_Disable);
		qmi8658_config_gyro(Qmi8658GyrRange_1024dps, Qmi8658GyrOdr_62_5Hz, Qmi8658Lpf_Disable, Qmi8658St_Disable);

		g_imu.cfg.ctrl8_value |= QMI8658_CTRL8_NOMOTION_EN;
		qmi8658_write_reg(Qmi8658Register_Ctrl8, g_imu.cfg.ctrl8_value);
		qmi8658_delay(1);

		qmi8658_enableSensors(QMI8658_ACCGYR_ENABLE);
	}
	else
	{
		g_imu.cfg.ctrl8_value &= (~QMI8658_CTRL8_NOMOTION_EN);
		qmi8658_write_reg(Qmi8658Register_Ctrl8, g_imu.cfg.ctrl8_value);
	}
}
#endif

#if defined(QMI8658_USE_SIG_MOTION)
void qmi8658_enable_sig_motion(unsigned char enable, enum qmi8658_Interrupt int_map)
{
	if(enable)
	{
		unsigned char ctrl1;

		qmi8658_enableSensors(QMI8658_DISABLE_ALL);

		qmi8658_read_reg(Qmi8658Register_Ctrl1, &ctrl1, 1);
		if(int_map == qmi8658_Int1)
		{
			ctrl1 |= QMI8658_INT1_ENABLE;
			g_imu.cfg.ctrl8_value |= QMI8658_CTRL8_INT_SEL;
		}
		else if(int_map == qmi8658_Int2)
		{
			ctrl1 |= QMI8658_INT2_ENABLE;
			g_imu.cfg.ctrl8_value &= (~QMI8658_CTRL8_INT_SEL);
		}		
		qmi8658_write_reg(Qmi8658Register_Ctrl1, ctrl1);// enable int for dev-E

		g_imu.cfg.ctrl8_value |= QMI8658_CTRL8_SIGMOTION_EN|QMI8658_CTRL8_ANYMOTION_EN|QMI8658_CTRL8_NOMOTION_EN;
		qmi8658_write_reg(Qmi8658Register_Ctrl8, g_imu.cfg.ctrl8_value);
		qmi8658_delay(1);

		qmi8658_enableSensors(QMI8658_ACCGYR_ENABLE);
	}
	else
	{
		g_imu.cfg.ctrl8_value &= (~QMI8658_CTRL8_SIGMOTION_EN);
		qmi8658_write_reg(Qmi8658Register_Ctrl8, g_imu.cfg.ctrl8_value);
	}
}
#endif

#endif

#if defined(QMI8658_USE_TAP)
unsigned char qmi8658_readTapStatus(void)
{
	unsigned char status;
	
	qmi8658_read_reg(Qmi8658Register_Tap_Status, &status, 1);

	return status;
}

void qmi8658_config_tap(void)
{
	unsigned char peakWindow = 0x1e;	//0x1e;
	unsigned char priority = 0x04;
	unsigned short TapWindow = 80;		// 80*4=320ms
	unsigned short DTapWindow = 150;	// 150*4=600ms

	unsigned char alpha = 0x08;
	unsigned char gamma = 0x20;
	unsigned short peakMagThr = 1800;		//1.8g
	unsigned short UDMThr = 0x0199;

	qmi8658_write_reg(Qmi8658Register_Cal1_L, peakWindow & 0xFF);
	qmi8658_write_reg(Qmi8658Register_Cal1_H, priority & 0xFF);
	qmi8658_write_reg(Qmi8658Register_Cal2_L, TapWindow & 0x00FF);
	qmi8658_write_reg(Qmi8658Register_Cal2_H, (TapWindow >> 8) & 0x00FF);
	qmi8658_write_reg(Qmi8658Register_Cal3_L, DTapWindow & 0x00FF);
	qmi8658_write_reg(Qmi8658Register_Cal3_H, (DTapWindow >> 8) & 0x00FF);
	qmi8658_write_reg(Qmi8658Register_Cal4_H, 0x01);
	qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_EnableTap, 1);

	qmi8658_write_reg(Qmi8658Register_Cal1_L, alpha & 0xFF);
	qmi8658_write_reg(Qmi8658Register_Cal1_H, gamma & 0xFF);
	qmi8658_write_reg(Qmi8658Register_Cal2_L, peakMagThr & 0x00FF);
	qmi8658_write_reg(Qmi8658Register_Cal2_H, (peakMagThr>>8) & 0x00FF);
	qmi8658_write_reg(Qmi8658Register_Cal3_L, UDMThr & 0x00FF);
	qmi8658_write_reg(Qmi8658Register_Cal3_H, (UDMThr>>8) & 0x00FF);
	qmi8658_write_reg(Qmi8658Register_Cal4_H, 0x02);
	qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_EnableTap, 1);
}

void qmi8658_enable_tap(unsigned char enable, enum qmi8658_Interrupt int_map)
{
	if(enable)
	{
		unsigned char ctrl1;

		qmi8658_enableSensors(QMI8658_DISABLE_ALL);

		qmi8658_read_reg(Qmi8658Register_Ctrl1, &ctrl1, 1);
		if(int_map == qmi8658_Int1)
		{
			ctrl1 |= QMI8658_INT1_ENABLE;
			g_imu.cfg.ctrl8_value |= QMI8658_CTRL8_INT_SEL;
		}
		else if(int_map == qmi8658_Int2)
		{
			ctrl1 |= QMI8658_INT2_ENABLE;
			g_imu.cfg.ctrl8_value &= (~QMI8658_CTRL8_INT_SEL);
		}
		qmi8658_write_reg(Qmi8658Register_Ctrl1, ctrl1);// enable int for dev-E

		g_imu.cfg.ctrl8_value |= QMI8658_CTRL8_TAP_EN;
		qmi8658_write_reg(Qmi8658Register_Ctrl8, g_imu.cfg.ctrl8_value);
		qmi8658_delay(1);

		qmi8658_enableSensors(QMI8658_ACCGYR_ENABLE);
	}
	else
	{
		g_imu.cfg.ctrl8_value &= (~QMI8658_CTRL8_TAP_EN);
		qmi8658_write_reg(Qmi8658Register_Ctrl8, g_imu.cfg.ctrl8_value);
	}

}
#endif

#if defined(QMI8658_USE_PEDOMETER)
void qmi8658_config_pedometer(unsigned short odr)
{
	float Rate = (float)(1000.0f/odr);
	unsigned short ped_sample_cnt = (unsigned short)(0x0032);//6;//(unsigned short)(0x0032 / finalRate) ;
	unsigned short ped_fix_peak2peak = 100;	//0x00AC;//0x0006;//0x00CC;
	unsigned short ped_fix_peak = 116;	//0x00AC;//0x0006;//0x00CC;
	unsigned short ped_time_up = (unsigned short)(2000 / Rate);
	unsigned char ped_time_low = (unsigned char) (300 / Rate);
	unsigned char ped_time_cnt_entry = 24;
	unsigned char ped_fix_precision = 0;
	unsigned char ped_sig_count = 1;//¼Æ²½Æ÷¼Ó1

	qmi8658_write_reg(Qmi8658Register_Cal1_L, ped_sample_cnt & 0xFF);
	qmi8658_write_reg(Qmi8658Register_Cal1_H, (ped_sample_cnt >> 8) & 0xFF);
	qmi8658_write_reg(Qmi8658Register_Cal2_L, ped_fix_peak2peak & 0xFF);
	qmi8658_write_reg(Qmi8658Register_Cal2_H, (ped_fix_peak2peak >> 8) & 0xFF);
	qmi8658_write_reg(Qmi8658Register_Cal3_L, ped_fix_peak & 0xFF);
	qmi8658_write_reg(Qmi8658Register_Cal3_H, (ped_fix_peak >> 8) & 0xFF);
	qmi8658_write_reg(Qmi8658Register_Cal4_H, 0x01);
	qmi8658_write_reg(Qmi8658Register_Cal4_L, 0x02);
	qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_EnablePedometer, 1);

	qmi8658_write_reg(Qmi8658Register_Cal1_L, ped_time_up & 0xFF);
	qmi8658_write_reg(Qmi8658Register_Cal1_H, (ped_time_up >> 8) & 0xFF);
	qmi8658_write_reg(Qmi8658Register_Cal2_L, ped_time_low);
	qmi8658_write_reg(Qmi8658Register_Cal2_H, ped_time_cnt_entry);
	qmi8658_write_reg(Qmi8658Register_Cal3_L, ped_fix_precision);
	qmi8658_write_reg(Qmi8658Register_Cal3_H, ped_sig_count);
	qmi8658_write_reg(Qmi8658Register_Cal4_H, 0x02);
	qmi8658_write_reg(Qmi8658Register_Cal4_L, 0x02);
	qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_EnablePedometer, 1);
}

void qmi8658_enable_pedometer(unsigned char enable)
{
	if(enable)
	{
		g_imu.cfg.ctrl8_value |= QMI8658_CTRL8_PEDOMETER_EN;
	}
	else
	{
		g_imu.cfg.ctrl8_value &= (~QMI8658_CTRL8_PEDOMETER_EN);
	}
	qmi8658_write_reg(Qmi8658Register_Ctrl8, g_imu.cfg.ctrl8_value);
}

unsigned int qmi8658_read_pedometer(void)
{
	unsigned char buf[3];

    qmi8658_read_reg(Qmi8658Register_Pedo_L, buf, 3);	// 0x5a
	g_imu.step = (unsigned int)((buf[2]<<16)|(buf[1]<<8)|(buf[0]));

	return g_imu.step;
}

void qmi8658_reset_pedometer(void)
{
	qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_ResetPedometer, 1);
}
#endif

#if defined(QMI8658_USE_FIFO)
void qmi8658_config_fifo(unsigned char watermark,enum qmi8658_FifoSize size,enum qmi8658_FifoMode mode,enum qmi8658_Interrupt int_map, uint8_t sensor)
{
	unsigned char ctrl1;

	qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_Rst_Fifo, 1);
	qmi8658_write_reg(Qmi8658Register_FifoCtrl, 0x00);
//	qmi8658_enableSensors(QMI8658_DISABLE_ALL);	
//	qmi8658_delay(2);
	if(watermark > 0)
	{
		qmi8658_read_reg(Qmi8658Register_Ctrl1, &ctrl1, 1);
		if(int_map == qmi8658_Int1)
		{
			ctrl1 |= QMI8658_FIFO_MAP_INT1;
		}
		else if(int_map == qmi8658_Int2)
		{
			ctrl1 &= QMI8658_FIFO_MAP_INT2;
		}
		qmi8658_write_reg(Qmi8658Register_Ctrl1, ctrl1);

		g_imu.cfg.fifo_ctrl = (unsigned char)(size | mode);
		qmi8658_write_reg(Qmi8658Register_FifoCtrl, g_imu.cfg.fifo_ctrl);
		qmi8658_write_reg(Qmi8658Register_FifoWmkTh, watermark);

		g_imu.cfg.enSensors = sensor;
	}

//	qmi8658_enableSensors(sensor);
}

unsigned short qmi8658_read_fifo(unsigned char* data)
{
	unsigned char fifo_status[2] = {0,0};
	unsigned char fifo_sensors = 1;
	unsigned short fifo_bytes = 0;
	unsigned short fifo_level = 0;
	
	if((g_imu.cfg.fifo_ctrl&0x03)!=qmi8658_Fifo_Bypass)
	{
		qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_Req_Fifo, 0);
		qmi8658_read_reg(Qmi8658Register_FifoCount, fifo_status, 2);
		fifo_bytes = (unsigned short)(((fifo_status[1]&0x03)<<8)|fifo_status[0]);
		if((g_imu.cfg.enSensors == QMI8658_ACC_ENABLE)||(g_imu.cfg.enSensors == QMI8658_GYR_ENABLE))
		{
			fifo_sensors = 1;
		}
		else if(g_imu.cfg.enSensors == QMI8658_ACCGYR_ENABLE)
		{
			fifo_sensors = 2;
		}
		fifo_level = fifo_bytes/(3*fifo_sensors);
		fifo_bytes = fifo_level*(6*fifo_sensors);
		//qmi8658_log("fifo-level : %d, byte:%d\n", fifo_level, fifo_bytes);
		if(fifo_level > 0)
		{	
			//for(int i=0; i<fifo_level; i++)
			//{
			//	qmi8658_read_reg(Qmi8658Register_FifoData, &data[i*fifo_sensors*6], fifo_sensors*6);
			//}

			qmi8658_read_reg(Qmi8658Register_FifoData, data, fifo_bytes);
			qmi8658_read_reg(Qmi8658Register_FifoCount, fifo_status, 2);
			fifo_bytes = (unsigned short)(((fifo_status[1]&0x03)<<8)|fifo_status[0]);
			if(fifo_bytes > 0)
			{
				qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_Rst_Fifo, 1);
				qmi8658_write_reg(Qmi8658Register_FifoCtrl, g_imu.cfg.fifo_ctrl);
			}
			else
			{
				qmi8658_write_reg(Qmi8658Register_FifoCtrl, g_imu.cfg.fifo_ctrl);
			}
		}
	}

	return fifo_level;
}
#endif

#if defined(QMI8658_USE_HW_SELFTEST)
void qmi8658_do_hw_selftest(int enSensor)
{
	unsigned char	status_int = 0x00;
	unsigned int	retry = 0;
	unsigned char	reg[6];
	short	raw[3];
	float	st_out[3];

	if(enSensor & QMI8658_ACC_ENABLE)
	{
		qmi8658_write_reg(Qmi8658Register_Ctrl7, 0x00);
		qmi8658_delay(2);
		qmi8658_write_reg(Qmi8658Register_Ctrl2, Qmi8658AccRange_8g|Qmi8658AccOdr_250Hz|0x80);
		status_int = 0;
		retry = 0;
		while(!(status_int & 0x01))
		{
			qmi8658_read_reg(Qmi8658Register_StatusInt, &status_int, 1);
			qmi8658_delay(1);
			//qmi8658_log("count=%d ",retry);
			if(retry++ > 5000)
			{
				qmi8658_log("wati int high timeout\n");
				break;
			}
		}
		qmi8658_write_reg(Qmi8658Register_Ctrl2, Qmi8658AccRange_8g|Qmi8658AccOdr_250Hz);
		retry = 0;
		status_int = 0x01;
		while((status_int & 0x01))
		{
			qmi8658_read_reg(Qmi8658Register_StatusInt, &status_int, 1);
			qmi8658_delay(1);
			//qmi8658_log("count=%d ",retry);
			if(retry++ > 5000)
			{
				qmi8658_log("wati int low timeout\n");
				break;
			}
		}
		qmi8658_read_reg(Qmi8658Register_Dvx_L, reg, 6);
		raw[0] = (short)((unsigned short)(reg[1]<<8) |( reg[0]));
		raw[1] = (short)((unsigned short)(reg[3]<<8) |( reg[2]));
		raw[2] = (short)((unsigned short)(reg[5]<<8) |( reg[4]));
		st_out[0] = (float)(raw[0]*1000.0f/2048);	// mg
		st_out[1] = (float)(raw[1]*1000.0f/2048);
		st_out[2] = (float)(raw[2]*1000.0f/2048);
		g_imu.st_out[0] = st_out[0];
		g_imu.st_out[1] = st_out[1];
		g_imu.st_out[2] = st_out[2];
		if((QFABS(st_out[0]) > 200) && (QFABS(st_out[1]) > 200) && (QFABS(st_out[2]) > 200))
		{
			qmi8658_log("acc-selftest raw[%d	%d	%d] out[%f	%f	%f] Pass!\n", raw[0],raw[1],raw[2],st_out[0],st_out[1],st_out[2]);
		}
		else
		{
			qmi8658_log("acc-selftest raw[%d	%d	%d] out[%f	%f	%f] Fail!\n", raw[0],raw[1],raw[2],st_out[0],st_out[1],st_out[2]);
		}
	}

	if(enSensor & QMI8658_GYR_ENABLE)
	{	
		qmi8658_write_reg(Qmi8658Register_Ctrl7, 0x00);
		qmi8658_delay(2);
		qmi8658_write_reg(Qmi8658Register_Ctrl3, Qmi8658GyrRange_1024dps|Qmi8658GyrOdr_250Hz|0x80);
		status_int = 0;
		retry = 0;
		while(!(status_int & 0x01))
		{
			qmi8658_read_reg(Qmi8658Register_StatusInt, &status_int, 1);
			qmi8658_delay(1);
			if(retry++ > 5000)
			{
				qmi8658_log("wati int high timeout\n");
				break;
			}
		}
		qmi8658_write_reg(Qmi8658Register_Ctrl3, Qmi8658GyrRange_1024dps|Qmi8658GyrOdr_250Hz);
		retry = 0;
		status_int = 0x01;
		while((status_int & 0x01))
		{
			qmi8658_read_reg(Qmi8658Register_StatusInt, &status_int, 1);
			qmi8658_delay(1);
			if(retry++ > 5000)
			{
				qmi8658_log("wati int low timeout\n");
				break;
			}
		}
		qmi8658_read_reg(Qmi8658Register_Dvx_L, reg, 6);
		raw[0] = (short)((unsigned short)(reg[1]<<8) |( reg[0]));
		raw[1] = (short)((unsigned short)(reg[3]<<8) |( reg[2]));
		raw[2] = (short)((unsigned short)(reg[5]<<8) |( reg[4]));
		st_out[0] = (float)(raw[0]/16.0f);	// dps
		st_out[1] = (float)(raw[1]/16.0f);
		st_out[2] = (float)(raw[2]/16.0f);	
		g_imu.st_out[3] = st_out[0];
		g_imu.st_out[4] = st_out[1];
		g_imu.st_out[5] = st_out[2];
		if((QFABS(st_out[0]) > 300) && (QFABS(st_out[1]) > 300) && (QFABS(st_out[2]) > 300))
		{
			qmi8658_log("gyr-selftest raw[%d	%d	%d] out[%f	%f	%f] Pass!\n", raw[0],raw[1],raw[2],st_out[0],st_out[1],st_out[2]);
		}
		else
		{
			qmi8658_log("gyr-selftest raw[%d	%d	%d] out[%f	%f	%f] Fail!\n", raw[0],raw[1],raw[2],st_out[0],st_out[1],st_out[2]);
		}
	}
}

void qmi8658_get_hw_selftest_data(float out[6])
{
	out[0] = g_imu.st_out[0];
	out[1] = g_imu.st_out[1];
	out[2] = g_imu.st_out[2];
	out[3] = g_imu.st_out[3];
	out[4] = g_imu.st_out[4];
	out[5] = g_imu.st_out[5];
}
#endif

#if defined(QMI8658_FIRMWARE_DOWNLOAD)||defined(QMI8658_EN_CGAIN)
#define valSplit4(val, out)	\
	do {\
		out[0] = (unsigned char)(val & 0x000000ff);	\
		out[1] = (unsigned char)((val & 0x0000ff00)>>8);	\
		out[2] = (unsigned char)((val & 0x00ff0000)>>16);	\
		out[3] = (unsigned char)((val & 0xff000000)>>24);	\
		}while(0)

#define set_ctrn	1
#define burst_cnt	1	 // not used in write
#define fifo_full	0	 // Check if this should be initilized as "1"
#define wr			1
#define start_busy	0

#define FW_START_ADDR	0x0010000
#define FW_BYTESNUM		16128
#define BRUST_BYTES		1
#endif

#if defined(QMI8658_EN_CGAIN)
static void readIndirectData(unsigned int startAddr, unsigned char *buf, unsigned char bytesNum)
{
	unsigned char addrList[4];
	unsigned char ctrlData = 0;

	qmi8658_delay(1);
#if 0
	valSplit4(startAddr, addrList);
//	qmi8658_write_regs(BANK0_INDIRECT_SYS_ADDR_7_0_ADDR, addrList, 4); //0x70
	qmi8658_write_reg(BANK0_INDIRECT_SYS_ADDR_7_0_ADDR, addrList[0]);
	qmi8658_write_reg(BANK0_INDIRECT_SYS_ADDR_15_8_ADDR, addrList[1]);
	qmi8658_write_reg(BANK0_INDIRECT_SYS_ADDR_23_16_ADDR, addrList[2]);
	qmi8658_write_reg(BANK0_INDIRECT_SYS_ADDR_31_24_ADDR, addrList[3]);
#endif
	qmi8658_write_reg(BANK0_INDIRECT_CTRL_ADDR, 0x7F);
//	set_ctrn = 1;
//	burst_cnt = bytesNum;
//	fifo_full = 0;
//	wr = 0;
//	start_busy = 1;
//	ctrlData = (set_ctrn << 7)+ (burst_cnt << 3) + (fifo_full << 2)+ (wr << 1)+ start_busy;
	ctrlData = (1 << 7)+ (bytesNum << 3) + (0 << 2)+ (0 << 1)+ 1;
	qmi8658_write_reg(BANK0_INDIRECT_CTRL_ADDR, ctrlData);	// apply new settings
	qmi8658_delay(1);
	// Start reading the data
	qmi8658_read_reg(BANK0_INDIRECT_SYS_DATA_ADDR, buf, bytesNum);
}

unsigned char qmi8658_read_cgain(void)
{
	unsigned char data, start_en, cgain;
		
	readIndirectData(QMI8658_EN_CGAIN, &data, 1);
	start_en=(data&0x80)>>7;
	cgain = data & 0x3F;
	qst_logi("qmi8658_read_cgain: %d\r\n", cgain);
	if(cgain >= 63)
	{
		qmi8658_write_reg(Qmi8658Register_Ctrl7, 0x00);
		qmi8658_delay(10);
		qmi8658_enableSensors(QMI8658_ACCGYR_ENABLE);
	}

	return cgain;
}
#endif

#if defined(QMI8658_FIRMWARE_DOWNLOAD)
static void writeIndirectData(unsigned int startAddr, unsigned char* wrData, unsigned int wrLen)
{
	unsigned char addrList[4];
	unsigned char ctrlData = 0;
	int i = 0;

	valSplit4(startAddr, addrList);
//	qmi8658_write_regs(BANK0_INDIRECT_SYS_ADDR_7_0_ADDR, addrList, 4);
	qmi8658_write_reg(BANK0_INDIRECT_SYS_ADDR_7_0_ADDR, addrList[0]);
	qmi8658_write_reg(BANK0_INDIRECT_SYS_ADDR_15_8_ADDR, addrList[1]);
	qmi8658_write_reg(BANK0_INDIRECT_SYS_ADDR_23_16_ADDR, addrList[2]);
	qmi8658_write_reg(BANK0_INDIRECT_SYS_ADDR_31_24_ADDR, addrList[3]);

	qmi8658_write_reg(BANK0_INDIRECT_CTRL_ADDR, 0x7F);    // clear previous settings

	// burst_cnt = bytesNum    ## Not used in write
	ctrlData = (set_ctrn << 7) + (burst_cnt << 3) + (fifo_full << 2) + (wr << 1) + start_busy;
	qmi8658_write_reg(BANK0_INDIRECT_CTRL_ADDR, ctrlData);    // apply new settings

	for(i=0; i<wrLen; i++)
	{
		ctrlData = 0x00;
		qmi8658_read_reg(BANK0_INDIRECT_CTRL_ADDR, &ctrlData, 1);
		while((ctrlData & 0x04) !=0)
		{
			qmi8658_delay_us(1);
			qmi8658_read_reg(BANK0_INDIRECT_CTRL_ADDR, &ctrlData, 1);
		}
		qmi8658_write_reg(BANK0_INDIRECT_SYS_DATA_ADDR, wrData[i]);		
		qmi8658_log("writeIndirectData 0x%x\r\n", wrData[i]);
	}
}

void qmi8658_firmware_download(const unsigned char *pHex, unsigned int len)
{
//	unsigned int startAddr = 0x0010000;
//	unsigned int bytesNum = 16128;	
	unsigned char addrList[4];
	unsigned char buf[5];
	unsigned char ctrlData = 0;
	unsigned char burst_num = 0;
	int ret=0;
	int retry = 0;
	int i = 0;

	qmi8658_write_reg(Qmi8658Register_Ctrl1, 0x60);		// auto increment on
//	qmi8658_write_reg(Qmi8658Register_Ctrl7, QMI8658_ACCGYR_ENABLE);
//	qmi8658_delay(100);
	qmi8658_read_reg(Qmi8658Register_firmware_id, buf, 3);

	while(retry++ < 5)
	{
		if((buf[1]<3) || (buf[0] < 13))
		{
			qmi8658_log("qmi8658_firmware_download start [%d]\r\n", buf[0]);
			buf[0] = 0x00;
			writeIndirectData(SYS_CLK_CFG, buf, 1);
			buf[0] = 0x0C;
			writeIndirectData(OPT_REG_GEN_CTRL, buf, 1);    // make sure ARM put in hold
			buf[0] = 0x00;
			writeIndirectData(OPT_REG_GEN_CTRL_1, buf, 1);    // Disable BDMA manually.

			valSplit4(FW_START_ADDR, addrList);
			qmi8658_write_regs(BANK0_INDIRECT_SYS_ADDR_7_0_ADDR, addrList, 4);
			qmi8658_write_reg(BANK0_INDIRECT_CTRL_ADDR, 0x7F);    // clear previous settings

			ctrlData = (set_ctrn << 7) + (burst_cnt << 3) + (fifo_full << 2) + (wr << 1) + start_busy;
			qmi8658_write_reg(BANK0_INDIRECT_CTRL_ADDR, ctrlData);    // apply new settings

			if(burst_num > 0)
			{
				int n = 0;

				n = FW_BYTESNUM/burst_num;
				//qmi8658_write_reg(Qmi8658Register_Ctrl1, 0x20);
				for(i=0; i<n; i++)
				{
					qmi8658_delay_us(10);
					ret=qmi8658_write_regs(BANK0_INDIRECT_SYS_DATA_ADDR, (unsigned char*)&pHex[i*burst_num], burst_num);
					if(ret != 1)
					{
						qmi8658_log("i2c error\r\n");
					}
				}
				if(FW_BYTESNUM > (burst_num*n))
				{
					qmi8658_delay_us(10);
					ret=qmi8658_write_regs(BANK0_INDIRECT_SYS_DATA_ADDR, (unsigned char*)&pHex[i*burst_num], FW_BYTESNUM-(burst_num*n));
					if(ret != 1)
					{
						qmi8658_log("i2c error\r\n");
					}					
				}
				//qmi8658_write_reg(Qmi8658Register_Ctrl1, 0x60);
			}
			else
			{
				for(i=0; i<FW_BYTESNUM; i++)
				{
					qmi8658_delay_us(20);
					ret=qmi8658_write_reg(BANK0_INDIRECT_SYS_DATA_ADDR, pHex[i]);
					if(ret != 1)
					{
						qmi8658_log("i2c error\r\n");
					}
				}
			}

			buf[0] = 0x0A;
			writeIndirectData(OPT_REG_GEN_CTRL, buf, 1);
			buf[0] = 0x08;
			writeIndirectData(OPT_REG_GEN_CTRL, buf, 1);
			qmi8658_read_reg(Qmi8658Register_firmware_id, buf, 3);
			qmi8658_log("qmi8658_firmware_download done [%d]\r\n", buf[0]);
		}
		else
		{
			return;
		}
	}

	if(retry >= 5)
	{
		qmi8658_log("qmi8658_firmware_download fail\r\n");
	}
}
#endif

unsigned char qmi8658_init(int protocol)
{
	g_imu.protocol = protocol;
	if(qmi8658_get_id() == 0x05)
	{
#if defined(QMI8658_SYNC_SAMPLE_MODE)
		qmi8658_enable_AHB_clock(0);
		g_imu.cfg.syncSample = 1;
#endif
#if defined(QMI8658_USE_AMD)||defined(QMI8658_USE_NO_MOTION)||defined(QMI8658_USE_SIG_MOTION)
		qmi8658_config_motion();
#endif
#if defined(QMI8658_USE_TAP)
		qmi8658_config_tap();
#endif
#if defined(QMI8658_USE_PEDOMETER)
		qmi8658_config_pedometer(125);
		qmi8658_enable_pedometer(1);
#endif
		qmi8658_config_reg(0);
		qmi8658_enableSensors(g_imu.cfg.enSensors);
		qmi8658_dump_reg();
//		qmi8658_delay(300);

		return 1;
	}
	else
	{
		qmi8658_log("qmi8658_init fail\n");
		return 0;
	}
}


