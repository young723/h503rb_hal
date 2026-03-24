
#include "mmc5603.h"

#define MMC5603_LOG		printf

//static mmc5603_data_t p_mag;

int mmc5603_read_block(unsigned char addr, unsigned char *data, unsigned char len)
{
	int ret = 0;
	int retry = 0;

	while((ret!=1) && (retry++ < 5))
	{
		ret = bsp_i2c_read_reg(MMC5603_IIC_ADDR, addr, data, len);
	}

	return ret;

}

int mmc5603_write_reg(unsigned char addr, unsigned char data)
{
	int ret = 0;
	int retry = 0;

	while((ret!=1) && (retry++ < 5))
	{
		ret = bsp_i2c_write_reg(MMC5603_IIC_ADDR, addr, data);
	}

	return ret;
}

void mmc5603_delay(unsigned int ms)
{
	extern void qst_delay_ms(unsigned int n_ms);

	qst_delay_ms(ms);
}


int mmc5603_get_chipid(void)
{
	int ret = 0;
	int i;
	unsigned char chip_id = 0x00;

	MMC5603_LOG("mmc5603_get_chipid addr=0x%x\n",MMC5603_IIC_ADDR);
	for(i=0; i<10; i++)
	{
		ret = mmc5603_read_block(MMC56x3NJ_REG_WHO_AM_I, &chip_id, 1);
		MMC5603_LOG("mmc5603_get_chipid chipid = 0x%x\n", chip_id);
		if(ret)
		{
			break;
		}
	}
	if(chip_id == MMC56x3NJ_WHOAMI_VALUE)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


void mmc5603_soft_reset(void)
{
	mmc5603_write_reg(MMC56x3NJ_REG_CTRL1, MMC56x3X_SOFT_RESET);
	mmc5603_delay(20);
	mmc5603_write_reg(MMC56x3NJ_REG_CTRL1, 0x00);
}

int mmc5603_read_mag_xyz(float *data)
{
	int res;
	unsigned char mag_data[6];
	int hw_d[3] = {0};
//	int t1 = 0;
//	unsigned char rdy = 0;

	/* Check status register for data availability */
/*
	res = mmc5603_read_block(MMC56x3NJ_REG_STATUS1, &rdy, 1);
	while(!(rdy & 0x02)&&(t1++ < 20))
	{
		mmc5603_delay(1);
		res = mmc5603_read_block(MMC56x3NJ_REG_STATUS1, &rdy, 1);
	}
*/
	res = mmc5603_read_block(MMC56x3NJ_REG_DATA, mag_data, 6);
	if(res == 0)
  	{
		return 0;
	}

	hw_d[0] = (((int)mag_data[0] << 8) | (int)mag_data[1]);
	hw_d[1] = (((int)mag_data[2] << 8) | (int)mag_data[3]);
	hw_d[2] = (((int)mag_data[4] << 8) | (int)mag_data[5]);

	data[0] = ((float) hw_d[0] -32768.0f) * 100.0f / 1024.0f;
	data[1] = ((float) hw_d[1] -32768.0f) * 100.0f / 1024.0f;
	data[2] = ((float) hw_d[2] -32768.0f) * 100.0f / 1024.0f;

	return res;
}


// set odr , mode,
int mmc5603_config_mode(unsigned char mode)
{	
	int err = 0;
	
	return err;	
}

int mmc5603_config_odr(unsigned char odr)
{
	int err = 0;
	
	return err;
}

// set range , set/reset
int mmc5603_config_range(unsigned char range)
{
	int err = 0;

	return err;	
}

int mmc5603_config_setrst(unsigned char setrst)
{
	int err = 0;

	return err;	
}


void mmc5603_dump_reg(void)
{
	unsigned char reg_data[4];

	mmc5603_read_block(MMC56x3NJ_REG_ODR, reg_data, 4);
	MMC5603_LOG("mmc5603_dump_reg 0x1A~0x1D[0x%02x 0x%02x 0x%02x 0x%02x]\n", reg_data[0],reg_data[1],reg_data[2],reg_data[3]);
}


int mmc5603_enable(int en)
{
	int ret = 0;

	if(en)
	{
		ret = mmc5603_write_reg(MMC56x3NJ_REG_CTRL1, MMC56x3NJ_BW_300HZ);
		mmc5603_delay(1);
		ret = mmc5603_write_reg(MMC56x3NJ_REG_ODR, 150);		
		mmc5603_delay(1);
		ret = mmc5603_write_reg(MMC56x3NJ_REG_CTRL0, MMC56x3NJ_CMM_CONF);
		mmc5603_delay(1);
		ret = mmc5603_write_reg(MMC56x3NJ_REG_CTRL2, MMC56x3NJ_AUTO_SET);
		mmc5603_delay(1);
	}
	else
	{
		ret = mmc5603_write_reg(MMC56x3NJ_REG_CTRL2, 0x00);
	}
	mmc5603_dump_reg();

	return ret;
}

int mmc5603_init(void)
{
	if(mmc5603_get_chipid())
	{
		mmc5603_soft_reset();
		mmc5603_dump_reg();
		mmc5603_enable(1);

		return 1;
	}
	else
	{
		return 0;
	}
}

int mmc5603_self_test(void)
{
	return 0;
}


