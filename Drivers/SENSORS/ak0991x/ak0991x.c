
#include "ak0991x.h"

#define AK0991X_LOG		printf

static akm_sensor_t g_akm;

int ak0991x_read_reg(unsigned char addr, unsigned char *data, unsigned short len)
{
	int ret = 0;
	int retry = 0;

	while((!ret) && (retry++ < 5))
	{
		ret = bsp_i2c_read_reg(AK0991X_IIC_ADDR, addr, data, len);
	}

	return ret;
}

int ak0991x_write_reg(unsigned char addr, unsigned char data)
{
	int ret = 0;
	int retry = 0;

	while((!ret) && (retry++ < 5))
	{
		ret = bsp_i2c_write_reg(AK0991X_IIC_ADDR, addr, data);
	}

	return ret;
}

void ak0991x_delay(unsigned int ms)
{
	extern void qst_delay_ms(unsigned int n_ms);

	qst_delay_ms(ms);
}


int ak0991x_get_chipid(void)
{
	int ret = 0;
	int i;
	unsigned char chip_id[4];

	AK0991X_LOG("ak0991x_get_chipid addr=0x%x\n",AK0991X_IIC_ADDR);
	for(i=0; i<10; i++)
	{
		ret = ak0991x_read_reg(AK0991X_REG_WHOAMI, chip_id, 4);
		AK0991X_LOG("ak0991x_get_chipid [0x%x 0x%x 0x%x 0x%x]\n", chip_id[0],chip_id[1],chip_id[2],chip_id[3]);
		if(ret)
		{
			break;
		}
		ak0991x_delay(1);
	}
	switch(chip_id[1])
	{
		case AK09917_WHOAMI_DEV_ID:
			g_akm.type = AK09917;
			g_akm.resolution = 0.15f;
			g_akm.fifo_wmk = 7-1;//16-1;
			break;
		case AK09918_WHOAMI_DEV_ID:
			g_akm.type = AK09918;
			g_akm.resolution = 0.15f;
			g_akm.fifo_wmk = 0;
			break;
		case AK09919_WHOAMI_DEV_ID:
			g_akm.type = AK09919;
			g_akm.resolution = 0.15f;
			g_akm.fifo_wmk = 0;
			break;
		default:
			g_akm.type = UNKNOW;
			g_akm.resolution = 0.0f;
			g_akm.fifo_wmk = 0;
			return 0;
	}

	return 1;
}


void ak0991x_soft_reset(void)
{
	ak0991x_write_reg(AK0991X_REG_CNTL3, 0x01);
	ak0991x_delay(2);
	ak0991x_write_reg(AK0991X_REG_CNTL3, 0x00);
	ak0991x_delay(2);
}

void ak0991x_fifo_config(unsigned char fifo_en, unsigned char fifo_wmk)
{
	g_akm.fifo_wmk = fifo_wmk;
	if(g_akm.fifo_wmk <= 0)
	{
		fifo_en = 0;
	}
	AK0991X_LOG("ak0991x_config fifo_wmk=%d\r\n", fifo_wmk);
	ak0991x_write_reg(AK0991X_REG_CNTL1, 0x00|AK0991X_ITS_LOW|g_akm.fifo_wmk);
	ak0991x_delay(1);
	ak0991x_write_reg(AK0991X_REG_CNTL2, (fifo_en<<7)|AK0991X_SDR_ENABLE|AK0991X_MAG_ODR100);
	ak0991x_delay(1);
}

void ak0991x_config(void)
{
	AK0991X_LOG("ak0991x_config\r\n");
	ak0991x_write_reg(AK0991X_REG_CNTL1, 0x00|AK0991X_ITS_LOW);
	ak0991x_delay(1);
	ak0991x_write_reg(AK0991X_REG_CNTL2, (AK0991X_SDR_ENABLE|AK0991X_MAG_ODR100)&0x7f);
	ak0991x_delay(1);
}


int ak0991x_read_mag_xyz(float *data)
{
	int res = 0;
	unsigned char mag_data[8];
	short hw_d[3] = {0};
	int t1 = 0;
	unsigned char rdy = 0;

	res = ak0991x_read_reg(AK0991X_REG_ST1, &rdy, 1);
	while(!(rdy & 0x01)&&(t1++ < 3))
	{
		//AK0991X_LOG("drdy = 0x%x\n", rdy);
		ak0991x_delay(1);
		res = ak0991x_read_reg(AK0991X_REG_ST1, &rdy, 1);
	}
	if(!(rdy & 0x01))
	{
		AK0991X_LOG("drdy fail!\n");
		data[0] = g_akm.l_data[0];
		data[1] = g_akm.l_data[1];
		data[2] = g_akm.l_data[2];
		return 0;
	}
	res = ak0991x_read_reg(AK0991X_REG_HXL, mag_data, 8);
	if((g_akm.type == AK09917)||(g_akm.type == AK09919))
	{
		hw_d[0] = (short)((mag_data[0] << 8) | mag_data[1]);
		hw_d[1] = (short)((mag_data[2] << 8) | mag_data[3]);
		hw_d[2] = (short)((mag_data[4] << 8) | mag_data[5]);
	}
	else
	{
		hw_d[0] = (short)((mag_data[1] << 8) | mag_data[0]);
		hw_d[1] = (short)((mag_data[3] << 8) | mag_data[2]);
		hw_d[2] = (short)((mag_data[5] << 8) | mag_data[4]);
	}

	data[0] = (float) hw_d[0]*g_akm.resolution;
	data[1] = (float) hw_d[1]*g_akm.resolution;
	data[2] = (float) hw_d[2]*g_akm.resolution;
	g_akm.l_data[0] = data[0];
	g_akm.l_data[1] = data[1];
	g_akm.l_data[2] = data[2];

	return res;
}


int ak0991x_read_mag_fifo(unsigned char *data)
{
	int res = 0;
	unsigned char status = 0;
	unsigned char fifo_lvl;
	unsigned char mag_data[8*32];
	short mag_raw[3];

	res = ak0991x_read_reg(AK0991X_REG_ST1, &status, 1);
	if(g_akm.type == AK09917)
	{
		fifo_lvl = status>>2;
	}
	else
	{
		fifo_lvl = ((status&0xfc)>>3);
	}

	AK0991X_LOG("status = 0x%x fifo_level=%d\n", status, fifo_lvl);
	if(status & 0x01)
	{
		res = ak0991x_read_reg(AK0991X_REG_HXL, mag_data, 8*fifo_lvl);
		if(!res)
		{
			AK0991X_LOG("read AK0991X_REG_HXL error!\n");			
		}
		for(int i=0; i<8*fifo_lvl; i+=8)
		{
			mag_raw[0] = (short)((mag_data[i*8]<<8)|mag_data[i*8+1]);
			mag_raw[1] = (short)((mag_data[i*8+2]<<8)|mag_data[i*8+3]);
			mag_raw[2] = (short)((mag_data[i*8+4]<<8)|mag_data[i*8+5]);
			AK0991X_LOG("fifo-%d %d	%d	%d\n", i/8, mag_raw[0],mag_raw[1],mag_raw[2]);
		}
	}

	return 1;
}

void ak0991x_dump_reg(void)
{
	unsigned char reg_data[2];

	ak0991x_read_reg(AK0991X_REG_CNTL1, reg_data, 2);
	AK0991X_LOG("ak0991x_dump_reg 0x30=0x%02x 0x31=0x%02x\n", reg_data[0],reg_data[1]);
}

int ak0991x_init(void)
{
	if(ak0991x_get_chipid())
	{
		ak0991x_soft_reset();
		//ak0991x_fifo_config(1, 8);
		ak0991x_config();
		ak0991x_dump_reg();

		return 1;
	}
	else
	{
		return 0;
	}
}

int ak0991x_self_test(void)
{
	return 0;
}


