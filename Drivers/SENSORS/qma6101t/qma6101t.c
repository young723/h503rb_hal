/**
  ******************************************************************************
  * @file    qma6101t.c
  * @author  Yangzhiqiang@qst
  * @version V1.0
  * @date    2020-5-27
  * @brief    qma6101t
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */ 

#include "qma6101t.h"

int skip_time = 0;
int proof_time = 0;


static qs32 qma6101t_initialize(void);

qma6101t_data g_qma6101t;

void qma6101t_delay(qu32 delay)
{
	qst_delay_ms(delay);
}

qs32 qma6101t_writereg(qu8 reg_add,qu8 reg_dat)
{
	qs32 ret = QMA6101T_FAIL;
	qu32 retry = 0;

	while((ret==QMA6101T_FAIL) && (retry++ < 5))
	{
		if(g_qma6101t.protocol == 0)
			ret = bsp_i2c_write_reg(g_qma6101t.slave, reg_add, reg_dat);
		else if(g_qma6101t.protocol == 1)
			ret = bsp_i3c_write_reg(reg_add, reg_dat);
		else
			ret = bsp_spi_write_reg(reg_add, reg_dat);
	}

	return ret;
}

qs32 qma6101t_readreg(qu8 reg_add,qu8 *buf,qu16 num)
{
	qs32 ret = QMA6101T_FAIL;
	qu32 retry = 0;

	while((ret==QMA6101T_FAIL) && (retry++ < 5))
	{
		if(g_qma6101t.protocol == 0)
			ret = bsp_i2c_read_reg(g_qma6101t.slave, reg_add, buf, num);
		else if(g_qma6101t.protocol == 1)
			ret = bsp_i3c_read_reg(reg_add, buf, num);
		else
			ret = bsp_spi_read_reg(reg_add, buf, num);
	}

	return ret;

}


qs32 qma6101t_set_range(qs32 range, qs32 hpf, qs32 hpf_lpf_sel)
{
	qs32 ret = 0;
	qu8 reg_data = (qu8)(range | hpf | hpf_lpf_sel);

	if(range == QMA6101T_RANGE_4G)
	{
		g_qma6101t.lsb_1g = 8192;
	}
	else if(range == QMA6101T_RANGE_8G)
	{
		g_qma6101t.lsb_1g = 4096;
	}
	else if(range == QMA6101T_RANGE_16G)
	{
		g_qma6101t.lsb_1g = 2048;
	}
//	else if(range == QMA6101T_RANGE_32G)
//	{
//		g_qma6101t.lsb_1g = 1024;
//	}
	else
	{
		g_qma6101t.lsb_1g = 16384;
	}
	
	ret = qma6101t_writereg(QMA6101T_REG_RANGE, reg_data);

	return ret;
}


qs32 qma6101t_set_bw(qs32 bw, qs32 lpf)
{
	qs32 ret = 0;
	qu8 reg_data = (qu8)(bw| lpf);

	ret = qma6101t_writereg(QMA6101T_REG_BW_ODR, reg_data);
	return ret;
}

qs32 qma6101t_set_mode(qs32 mode, qs32 odr)
{
	qs32 ret = 0;
	qu8 reg_data = 0;
	
	reg_data = (qu8)(odr);
	
	if(mode >= QMA6101T_MODE_ACTIVE)
	{ 
		reg_data = (qu8) (reg_data | 0xc0);
	}
	
	ret = qma6101t_writereg(QMA6101T_REG_POWER_MANAGE, reg_data);

	return ret;
}


void qma6101t_dump_reg(void)
{
	uint8_t i = 0, j = 0, k = 0, reg = 0, idxmax = 0;
	QMA6101T_LOG("\r\n   ");
	for(i = 0; i < 0x10; i++)
	{	
		QMA6101T_LOG("%2x  |  ",i);
	}
	QMA6101T_LOG("\n");
	QMA6101T_LOG("-----------------------------------------------------------------------------------------------------------------\n");
	//printf("   0  *  1  *  2  *  3  *  4  *  5 *  6 *  7  *  8  *  9  *  A  *  B *  C  *  D  *  E  *  F\r\n");

	idxmax = 0x80;
	for(i = 0; i < idxmax; i++)
	{        
		qma6101t_readreg(i, &reg, 1);
		qma6101t_delay(2);
		k = i / 16;
		j = i / 15;

		if(i == (16 * k) )
		{
			QMA6101T_LOG("%x| %2x  |", (i / 15), reg);
		}
		else if(i == (16 * j - 1))
		{
			QMA6101T_LOG("  %2x  |\r\n\r\n",reg);
			QMA6101T_LOG("-----------------------------------------------------------------------------------------------------------------\n");
		}
		else
		{
			QMA6101T_LOG("  %2x  |",reg);
		}
	}
	QMA6101T_LOG("\r\n\n");
}	
	



#if defined(QMA6101T_DATA_READY)
void qma6101t_drdy_config(qs32 int_map, qs32 enable)
{
	qu8 reg_int_map = QMA6101T_INT1_MAP_1;
	qu8 data_enable = 0;
	qu8 data_int_map= 0;

	QMA6101T_LOG("qma6101t_drdy_config %d %d\n", enable, int_map);	
	if(int_map == QMA6101T_MAP_INT1)
	{
		reg_int_map = QMA6101T_INT1_MAP_1;
	}
	else if(int_map == QMA6101T_MAP_INT2)
	{
		reg_int_map = QMA6101T_INT2_MAP_1;
	}
	else
	{
		reg_int_map = 0x00;
	}

	qma6101t_readreg(QMA6101T_INT_EN_1, &data_enable, 1);
	qma6101t_readreg(reg_int_map, &data_int_map, 1);

	if(enable)
	{
		data_enable |= QMA6101T_DRDY_BIT;
		data_int_map |= QMA6101T_DRDY_BIT;
	}
	else
	{
		data_enable &= (~QMA6101T_DRDY_BIT);
		data_int_map &= (~QMA6101T_DRDY_BIT);
	}
	qma6101t_writereg(QMA6101T_INT_EN_1, data_enable);
	qma6101t_writereg(reg_int_map, data_int_map);
	
	data_enable = 0;
	qma6101t_readreg(QMA6101T_INT_EN_1, &data_enable, 1);
	QMA6101T_LOG("qma6101t_dataready_config read %x = %x\n", QMA6101T_INT_EN_1, data_enable);
	
	data_int_map = 0;
	qma6101t_readreg(reg_int_map, &data_int_map, 1);
	QMA6101T_LOG("qma6101t_dataready_config read %x = %x\n", reg_int_map, data_int_map);
}
#endif

#if defined(QMA6101T_FIFO_FUNC)
void qma6101t_fifo_config(qma6101t_fifo_mode fifo_mode, qu8 wmk, qs32 int_map, qs32 enable)
{
	qu8	reg_3e=0;
	qu8	reg_21=0;
	qu8 reg_int_map = QMA6101T_INT1_MAP_1;
	qu8 data_enable = 0;
	qu8 data_int_map= 0;

	QMA6101T_LOG("qma6101t_fifo_config mode:%d enable:%d\n", fifo_mode, enable);
	if(int_map == QMA6101T_MAP_INT1)
	{
		reg_int_map = QMA6101T_INT1_MAP_1;
	}
	else if(int_map == QMA6101T_MAP_INT2)
	{
		reg_int_map = QMA6101T_INT2_MAP_1;
	}
	else
	{
		reg_int_map = 0x00;
	}

	qma6101t_readreg(QMA6101T_INT_EN_1, &data_enable, 1);
	qma6101t_readreg(reg_int_map, &data_int_map, 1);
	qma6101t_readreg(0x21, &reg_21, 1);		// fifo use latch int
	qma6101t_readreg(0x3e, &reg_3e, 1);

	qma6101t_writereg(0x21, reg_21|0x01);
	if(enable)
	{
		g_qma6101t.fifo_mode = fifo_mode;
		if(g_qma6101t.fifo_mode == QMA6101T_FIFO_MODE_FIFO)
		{
			reg_3e |= 0x47;
			qma6101t_writereg(0x31, wmk);	// max 0x40
			qma6101t_writereg(0x3E, reg_3e);	//bit[6:7] 0x00:BYPASS 0x40:FIFO 0x80:STREAM
			qma6101t_writereg(QMA6101T_INT_EN_1, data_enable|0x20);
			qma6101t_writereg(reg_int_map, data_int_map|0x20);
		}
		else if(g_qma6101t.fifo_mode == QMA6101T_FIFO_MODE_STREAM)
		{	
			reg_3e |= 0x87;
			qma6101t_writereg(0x31, wmk);	// 0x3f
			qma6101t_writereg(0x3E, reg_3e);	//bit[6:7] 0x00:BYPASS 0x40:FIFO 0x80:STREAM
			qma6101t_writereg(QMA6101T_INT_EN_1, data_enable|0x40);
			qma6101t_writereg(reg_int_map, data_int_map|0x40);
		}
		else if(g_qma6101t.fifo_mode == QMA6101T_FIFO_MODE_BYPASS)
		{
			reg_3e |= 0x07;
			qma6101t_writereg(0x3E, reg_3e);	//bit[6:7] 0x00:BYPASS 0x40:FIFO 0x80:STREAM
			qma6101t_writereg(QMA6101T_INT_EN_1, data_enable|0x20);
			qma6101t_writereg(reg_int_map, data_int_map|0x20);
		}
	}
	else
	{
		g_qma6101t.fifo_mode = QMA6101T_FIFO_MODE_NONE;
		data_enable &= (~0x60);
		data_int_map &= (~0x60);
		qma6101t_writereg(QMA6101T_INT_EN_1, data_enable);
		qma6101t_writereg(reg_int_map, data_int_map);
	}
}

qs32 qma6101t_read_fifo(qu8 *fifo_buf)
{
	qs32 ret = 0;
	qu8 databuf[2];

#if 1//defined(QMA6101T_INT_LATCH)
	ret = qma6101t_readreg(QMA6101T_INT_STATUS_2, databuf, 1);
#endif
	ret = qma6101t_readreg(QMA6101T_FIFO_STATE, databuf, 1);
	if(ret != QMA6101T_SUCCESS)
	{
		QMA6101T_ERR("qma6101t_read_fifo state error\n");
		return 0;	//ret;
	}
	g_qma6101t.fifo_len = databuf[0]&0x7f;
	if(g_qma6101t.fifo_len > 64)
	{
		QMA6101T_ERR("qma6101t_read_fifo depth(%d) error\n",g_qma6101t.fifo_len);
		return 0;//QMA6101T_FAIL;
	}

	if(fifo_buf)
	{
#if 0
		qma6101t_readreg(0x3f, fifo_buf, g_qma6101t.fifo_len*6);
#else
		for(int icount=0; icount<g_qma6101t.fifo_len; icount++)
		{
			qma6101t_readreg(0x3f, &fifo_buf[icount*6], 6);
		}
#endif
	}
	if(g_qma6101t.fifo_mode == QMA6101T_FIFO_MODE_FIFO)
	{
		ret = qma6101t_writereg(0x3e, 0x47);
	}
	else if(g_qma6101t.fifo_mode == QMA6101T_FIFO_MODE_STREAM)
	{
		ret = qma6101t_writereg(0x3e, 0x87);
	}
	else if(g_qma6101t.fifo_mode == QMA6101T_FIFO_MODE_BYPASS)
	{
		ret = qma6101t_writereg(0x3e, 0x07);
	}
	return g_qma6101t.fifo_len;
}
#endif


#if defined(QMA6101T_ANY_MOTION)
void qma6101t_anymotion_config(qs32 thr, qs32 duration, qs32 slope, qs32 amd_enable, qs32 int_map)
{
	qu8 reg_int_map = QMA6101T_INT1_MAP_1;
	qu8 reg = 0;
	qu8 reg_range = 0;

	QMA6101T_LOG("qma6101t_anymotion_config %d\n", amd_enable);

	
	qma6101t_readreg(QMA6101T_REG_RANGE, &reg_range, 1);

	reg_range = reg_range & 0x0F;

	if(reg_range == QMA6101T_RANGE_2G)
	{
		thr = thr * 10 / 39;
	}
	else if(reg_range == QMA6101T_RANGE_4G)
	{
		thr = thr * 10 / 78;
	}
	else if(reg_range == QMA6101T_RANGE_8G)
	{
		thr = thr * 10 / 156;
	}
	else if(reg_range == QMA6101T_RANGE_16G)
	{
		thr = thr * 10 / 312;
	}
//	else if(reg_range == QMA6101T_RANGE_32G)
//	{
//		thr = thr * 10 / 625;
//	}
	
	thr = thr + 1 ; // close to the next value
	qma6101t_readreg(0x2f, &reg, 1);
	if((reg & 0x40) == QMA6101T_GRAVITY)
	{
		thr = thr / 2;
	}
	else
	{
		thr = thr / 1;
	}
	if(thr > 255)
	{
		thr = 255;
	}

	reg = (thr & 0xff); // thr	 1/G * 16 * 1e
	qma6101t_writereg(0x2e, reg);		// 0.488*16*32 = 250mg
	
	reg = 0;
	qma6101t_readreg(0x2c, &reg, 1);
	reg = ((duration & 0x03) | (reg & 0xfc));	// (ANY_MOT_DUR<1:0> + 1)samples
	qma6101t_writereg(0x2c, reg);	// ANY_MOT_DUR[1:0]
	
	reg = 0;
	qma6101t_readreg(QMA6101T_INT_EN_2, &reg, 1);
	reg = (amd_enable | (reg & 0xe0)); //	enable x ,y z
	qma6101t_writereg(QMA6101T_INT_EN_2, reg);	

	reg = 0;
	qma6101t_readreg(0x2f, &reg, 1);	
	reg = ((slope) | (reg & 0xBF)); //	slope or highG
	qma6101t_writereg(0x2f, reg);
	
	reg = 0;
	qma6101t_readreg(0x30, &reg, 1);
	// add by yang, step counter, raise wake, and tap detector,any motion by pass LPF
#if defined(QMA6101T_MOTION_LPF)
	reg |= 0x80;
#endif	
	qma6101t_writereg(0x30, reg);	// default 0x3f
	
	
	if(int_map == QMA6101T_MAP_INT1)
	{
		reg_int_map = QMA6101T_INT1_MAP_1;			
	}
	else if(int_map == QMA6101T_MAP_INT2)
	{
		reg_int_map = QMA6101T_INT2_MAP_1;
	}
	else
	{
		reg_int_map = 0x00;
	}
	
	reg = 0;
	qma6101t_readreg(reg_int_map, &reg, 1);
	reg = (0x01) | (reg & 0xfe);
	qma6101t_writereg(reg_int_map, reg); 
	qma6101t_readreg(reg_int_map, &reg, 1);
	QMA6101T_LOG("qma6101t_anymotion_config read %x = %x\n", reg_int_map, reg);
}
#endif




#if defined(QMA6101T_SIGNIFICANT_MOTION)
void qma6101t_sigmotion_config(qs32 thr, qs32 duration, qs32 slope, qs32 skip, qs32 proof, qs32 amd_enable, qs32 enable, qs32 int_map)
{
	qu8 reg_int_map = QMA6101T_INT1_MAP_1;
	qu8 reg = 0;
	qu8 reg_range = 0;

	QMA6101T_LOG("qma6101t_anymotion_config %d\n", amd_enable);

	
	qma6101t_readreg(QMA6101T_REG_RANGE, &reg_range, 1);

	reg_range = reg_range & 0x0F;
	
	if(reg_range == QMA6101T_RANGE_2G)
	{
		thr = thr * 10 / 39;
	}
	else if(reg_range == QMA6101T_RANGE_4G)
	{
		thr = thr * 10 / 78;
	}
	else if(reg_range == QMA6101T_RANGE_8G)
	{
		thr = thr * 10 / 156;
	}
	else if(reg_range == QMA6101T_RANGE_16G)
	{
		thr = thr * 10 / 312;
	}
//	else if(reg_range == QMA6101T_RANGE_32G)
//	{
//		thr = thr * 10 / 625;
//	}
	
	thr = thr + 1 ; // close to the next value
	qma6101t_readreg(0x2f, &reg, 1);
	if((reg & 0x40) == QMA6101T_GRAVITY)
	{
		thr = thr / 2;
	}
	else
	{
		thr = thr / 1;
	}
	if(thr > 255)
	{
		thr = 255;
	}
	
	reg = (thr & 0xff); // thr	 1/G * 16 * 1e
	qma6101t_writereg(0x2e, reg);		// 0.488*16*32 = 250mg
	
	reg = 0;
	qma6101t_readreg(0x2c, &reg, 1);
	reg = (duration & 0x03) | (reg & 0xfc);	// (ANY_MOT_DUR<1:0> + 1)samples
	qma6101t_writereg(0x2c, reg);	// ANY_MOT_DUR[1:0]
	
	reg = 0;
	qma6101t_readreg(QMA6101T_INT_EN_2, &reg, 1);
	reg = amd_enable | (reg & 0xe0); //	enable x ,y z
	qma6101t_writereg(QMA6101T_INT_EN_2, reg);	

	reg = 0;
	qma6101t_readreg(0x2f, &reg, 1);	
	
	if(enable)
	{
		reg = ((reg & 0xC0) | 0x01 | (skip << 2) | (proof << 4)); //	significant -motion -sel
	}
	else
	{
		reg = ((reg & 0xC0) | 0x00 | (skip << 2) | (proof << 4));	
	}
	qma6101t_writereg(0x2f, reg);
	
	
	reg = 0;
	qma6101t_readreg(0x30, &reg, 1);
	// add by yang, step counter, raise wake, and tap detector,any motion by pass LPF
#if defined(QMA6101T_MOTION_LPF)
	reg |= 0x80;
#endif	
	qma6101t_writereg(0x30, reg);	// default 0x3f
	
	
	if(int_map == QMA6101T_MAP_INT1)
	{
		reg_int_map = QMA6101T_INT1_MAP_0;			
	}
	else if(int_map == QMA6101T_MAP_INT2)
	{
		reg_int_map = QMA6101T_INT2_MAP_0;
	}
	else
	{
		reg_int_map = 0x00;
	}
	
	reg = 0;
	qma6101t_readreg(reg_int_map, &reg, 1);
	reg = (0x01) | (reg & 0x03);
	qma6101t_writereg(reg_int_map, reg); 
}
#endif




#if defined(QMA6101T_NO_MOTION)
void qma6101t_nomotion_config(qs32 thr, qu16 duration, qs32 enable, qs32 int_map)
{
	qu8 reg_int_map = QMA6101T_INT1_MAP_0;
	qu8 data_enable = 0;
	qu8 data_int_map= 0;
	qu8 reg_range = 0;
	qu8 reg = 0;

	QMA6101T_LOG("qma6101t_nomotion_config %d\n", enable);
	
	qma6101t_readreg(QMA6101T_REG_RANGE, &reg_range, 1);

	reg_range = reg_range & 0x0F;
	
	if(reg_range == QMA6101T_RANGE_2G)
	{
		thr = thr * 10 / 39;
	}
	else if(reg_range == QMA6101T_RANGE_4G)
	{
		thr = thr * 10 / 78;
	}
	else if(reg_range == QMA6101T_RANGE_8G)
	{
		thr = thr * 10 / 156;
	}
	else if(reg_range == QMA6101T_RANGE_16G)
	{
		thr = thr * 10 / 312;
	}
//	else if(reg_range == QMA6101T_RANGE_32G)
//	{
//		thr = thr * 10 / 625;
//	}
	
	
	thr = thr + 1 ; // close to the next value
	reg = (thr & 0xff); // thr	 1/G * 16 * 1e
	qma6101t_writereg(0x2d, reg);		// 0.488*16*32 = 250mg
	
	
	if((duration > 16) && (duration < 20))
	{
		duration = 20 ;
	}
	if((duration > 95) && (duration < 100)) 
	{
		duration = 100 ;
	}
	if(duration > 250) 
	{
		duration  = 250;
	}
	
	
	reg = 0;
	qma6101t_readreg(0x2c, &reg, 1);
	
	if(duration <= 16)
	{
		reg = (((duration - 1) << 2) | (reg & 0x03)); //duration bit[7-3] 
	}
	else if(duration <= 95)
	{
		reg = (0x40 | ((duration / 5 - 4) << 2 ) | (reg & 0x03));
	}
	else //if(duration < 250)
	{
		reg = (0x80 | ((duration / 10 - 10) << 2 )|(reg & 0x03));	
	}
	qma6101t_writereg(0x2c, reg);	
	
	
	
	reg = 0;
	qma6101t_readreg(QMA6101T_INT_EN_2, &reg, 1);
	reg = (0xe0 | (reg & 0x07));  
	qma6101t_writereg(QMA6101T_INT_EN_2, reg);	
	
	
	reg = 0;
	qma6101t_readreg(0x30, &reg, 1);
	// add by yang, step counter, raise wake, and tap detector,no motion by pass LPF
#if defined(QMA6101T_MOTION_LPF)
	reg |= 0x80;
#endif	
	qma6101t_writereg(0x30, reg);	// default 0x3f
	
	
	if(int_map == QMA6101T_MAP_INT1)
	{
		reg_int_map = QMA6101T_INT1_MAP_0;			
	}
	else if(int_map == QMA6101T_MAP_INT2)
	{
		reg_int_map = QMA6101T_INT2_MAP_0;
	}
	else
	{
		reg_int_map = 0x00;
	}
	
	reg = 0;
	qma6101t_readreg(reg_int_map, &reg, 1);
	reg = (0x2) | (reg & 0x03);
	qma6101t_writereg(reg_int_map, reg); 
}
#endif



void qma6101t_irq_hdlr(void)
{
#if defined(QMA6101T_DATA_READY)
//		qma6101t_read_raw_xyz(g_qma6101t.raw);
//		QMA6101T_LOG("drdy_int %d %d %d\n",g_qma6101t.raw[0], g_qma6101t.raw[1], g_qma6101t.raw[2]);
		qma6101t_read_acc_xyz(g_qma6101t.acc);
		QMA6101T_LOG("drdy_int ");
#else
	qu8 ret = QMA6101T_FAIL;
	qu8 databuf[4];
	qs32 retry = 0;

	while((ret==QMA6101T_FAIL)&&(retry++<10))
	{
		ret = qma6101t_readreg(QMA6101T_INT_STATUS_0, databuf, 4);
		if(ret == QMA6101T_SUCCESS)
		{
			break;
		}
	}
	if(ret == QMA6101T_FAIL)
	{
		QMA6101T_LOG("qma6101t_irq_hdlr read status fail!\n");
		return;
	}
	else
	{
		QMA6101T_LOG("irq [0x%x 0x%x 0x%x 0x%x]\n", databuf[0],databuf[1],databuf[2],databuf[3]);
	}
	#if defined(QMA6101T_FIFO_FUNC)
	if((databuf[2]&0x40)&&(g_qma6101t.fifo_mode==QMA6101T_FIFO_MODE_STREAM))
	{
		QMA6101T_LOG("FIFO WMK\n");
		//qma6101t_read_fifo(qma6101t_fifo_reg);
		//qma6101t_exe_fifo(qma6101t_fifo_reg);
	}
	else if((databuf[2]&0x20)) 
	{
		QMA6101T_LOG("FIFO FULL\n");
		//qma6101t_read_fifo(qma6101t_fifo_reg);
		//qma6101t_exe_fifo(qma6101t_fifo_reg);
	}
#endif
#if defined(QMA6101T_ANY_MOTION)
	if(databuf[0]&0x07)
	{
		QMA6101T_LOG("any motion!\n");
	}
#endif

#if defined(QMA6101T_SIGNIFICANT_MOTION)
	if(databuf[1]&0x01)
	{
		QMA6101T_LOG("significant motion!\n");
	}
#endif

#if defined(QMA6101T_NO_MOTION)
	if(databuf[0]&0x80)
	{
		QMA6101T_LOG("no motion!\n");
	}
#endif
#endif
}

void qma6101t_axis_convert(short data_a[3], int layout)
{
	short raw[3];

	raw[0] = data_a[0];
	raw[1] = data_a[1];

	if(layout >=4 && layout <= 7)
	{
		data_a[2] = -data_a[2];
	}

	if(layout%2)
	{
		data_a[0] = raw[1];
		data_a[1] = raw[0];
	}
	else
	{
		data_a[0] = raw[0];
		data_a[1] = raw[1];
	}

	if((layout==1)||(layout==2)||(layout==4)||(layout==7))
	{
		data_a[0] = -data_a[0];
	}
	if((layout==2)||(layout==3)||(layout==6)||(layout==7))
	{
		data_a[1] = -data_a[1];
	}
}

qs32 qma6101t_read_raw_xyz(qs16 data[3])
{
	qu8 databuf[6] = {0}; 	
	qs32 ret = 0;
	
	ret = qma6101t_readreg(QMA6101T_YOUTL, databuf, 6);
	if(ret == QMA6101T_FAIL)
	{
		QMA6101T_ERR("read xyz read reg error!!!\n");
		return QMA6101T_FAIL;	
	}
	
	data[1] = (qs16)(((databuf[1]<<8))|(databuf[0]));
	data[2] = (qs16)(((databuf[3]<<8))|(databuf[2]));
	data[0] = (qs16)(((databuf[5]<<8))|(databuf[4]));

//		QMA6101T_LOG("2--%d	%d	%d\n",data[0],data[1],data[2]);
	return QMA6101T_SUCCESS;
}

#if 0
static int cali_count = 0;
float offset_acc[3] = {0.0, 0.0, 0.0};
float accel_calibration_sum[3] = {0.0f, 0.0f, 0.0f};
#define MAX_CALI_COUNT      100
#define TODEG				57.2957796f
float fusion_9axis_init_pitch = 0;
float fusion_9axis_init_roll = 0;
float accel_data[3] = {0, 0, 0};
#endif
qs32 qma6101t_read_acc_xyz(float accData[3])
{
	qs32 ret;
	qs16 rawData[3];

	ret = qma6101t_read_raw_xyz(rawData);
	if(ret == QMA6101T_SUCCESS)
	{
		g_qma6101t.raw[0] = rawData[0];
		g_qma6101t.raw[1] = rawData[1];
		g_qma6101t.raw[2] = rawData[2];
	}
	
	accData[0] = (float)(g_qma6101t.raw[0]*M_G)/(g_qma6101t.lsb_1g);
	accData[1] = (float)(g_qma6101t.raw[1]*M_G)/(g_qma6101t.lsb_1g);
	accData[2] = (float)(g_qma6101t.raw[2]*M_G)/(g_qma6101t.lsb_1g);
	
//	QMA6101T_LOG("%f %f %f\n", accData[0], accData[1], accData[2]);
#if 0	
	uint8_t axis = 0;
	if(cali_count == 0)
	{
		memset((void *)accel_calibration_sum, 0, sizeof(accel_calibration_sum));
		cali_count++;
	}
	else if(cali_count < MAX_CALI_COUNT)
	{
		for(axis = 0; axis < 3; axis++) 
		{
			if(axis == 2)
			{ 
				accel_calibration_sum[axis] += (accData[axis] - M_G);
			}
			else
			{
				accel_calibration_sum[axis] += accData[axis];
			}
		}
		cali_count++;
	}
	else if(cali_count == MAX_CALI_COUNT)
	{
		for(axis = 0; axis < 3; axis++) 
		{
			offset_acc[axis] = (0.0f -(accel_calibration_sum[axis] / (MAX_CALI_COUNT - 1)));
		}
		cali_count++;
	}
	else
	{
		for(axis = 0; axis < 3; axis++) 
		{
			accel_data[axis] = accData[axis] + offset_acc[axis];
		}
//		QMA6101T_LOG("%f %f %f\n", accel_data[0], accel_data[1], accel_data[2]);
		
		float	normalize = sqrtf(accel_data[0] * accel_data[0] + accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2]);
		accel_data[0] = accel_data[0] / normalize;
		accel_data[1] = accel_data[1] / normalize;
		accel_data[2] = accel_data[2] / normalize;

		float pitch = -atan2f(accel_data[1], accel_data[2]);
		fusion_9axis_init_pitch = pitch * TODEG;////¸©Ñö½Ç

		normalize = sqrtf(accel_data[0] * accel_data[0] + accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2]);

		float roll = asinf((accel_data[0] / normalize));//ºá¹ö½Ç
		fusion_9axis_init_roll = roll * TODEG;//ºá¹ö½Ç
	}
	
	//QMA6101T_LOG("%f	%f\n", fusion_9axis_init_pitch, fusion_9axis_init_roll);
	QMA6101T_LOG("%f %f %f %f	%f\n", accel_data[0], accel_data[1], accel_data[2], fusion_9axis_init_pitch, fusion_9axis_init_roll);
#endif
	return QMA6101T_SUCCESS;
}


qs32 qma6101t_soft_reset(void)
{
	qu8 reg_read = 0;
	qu32 retry = 0;

	QMA6101T_LOG("qma6101t_soft_reset\n");
	
	qma6101t_writereg(QMA6101T_REG_POWER_MANAGE, 0x80);
	
	qma6101t_writereg(QMA6101T_REG_RESET, 0xb6);
	qma6101t_delay(1);

	// check otp
	qma6101t_readreg(QMA6101T_REG_NVM, &reg_read, 1);
	QMA6101T_LOG("Reg 0x33 = 0x%x\r\n",reg_read);
	while((reg_read & 0x05) != 0x05)
	{
		qma6101t_readreg(QMA6101T_REG_NVM, &reg_read, 1);
		qma6101t_delay(1);
		retry++;
		
		if(retry >= 100)
		{	
			QMA6101T_LOG("Read 0x33 status timeout 0x%x\r\n", reg_read);
			break;
		}
	}
	

	return QMA6101T_SUCCESS;
}


static qs32 qma6101t_initialize(void)
{
	qs32 wdt_en = 2;
	qu8 reg_data = 0;
	
	qma6101t_soft_reset();
				
	if(g_qma6101t.slave == 0x12)
	{
		qma6101t_writereg(0x22, 0x40);
	}
	
	qma6101t_set_range(QMA6101T_RANGE_16G, QMA6101T_HPF_4, QMA6101T_LPF_SEL);
	
	qma6101t_set_bw(QMA6101T_BW_100, QMA6101T_LPF_8);
	
	qma6101t_set_mode(QMA6101T_MODE_ACTIVE, QMA6101T_ODR_50);
	

#if defined(QMA6101T_DATA_READY)
	qma6101t_drdy_config(QMA6101T_MAP_INT1, QMA6101T_ENABLE);
#endif

#if defined(QMA6101T_FIFO_FUNC)
	qma6101t_fifo_config(QMA6101T_FIFO_MODE_STREAM, 10, QMA6101T_MAP_INT1, QMA6101T_ENABLE);
#endif


#if defined(QMA6101T_ANY_MOTION)
	qma6101t_anymotion_config(500, 1, QMA6101T_SLOPE, QMA6101T_AMD_ENABLE_X_Y_Z, QMA6101T_MAP_INT1);
#endif

#if defined(QMA6101T_SIGNIFICANT_MOTION)
qma6101t_sigmotion_config(200, 2, QMA6101T_SLOPE, skip_time, proof_time, QMA6101T_AMD_ENABLE_X_Y_Z, QMA6101T_ENABLE, QMA6101T_MAP_INT1);
#endif


#if defined(QMA6101T_NO_MOTION)
	qma6101t_nomotion_config(200, 2, QMA6101T_NMD_ENABLE_X_Y_Z, QMA6101T_MAP_INT2);
#endif


#if defined(QMA6101T_INT_LATCH)
	qma6101t_writereg(0x21, 0x01);	// default 0x1c, step latch mode
#endif

#if defined(QMA6101T_DATA_LPF) // add by yang, acceleration data by pass LPF
	reg_data = 0;
	qma6101t_readreg(0x2f, &reg_data, 1);	
	reg_data = (reg_data | 0x80); 
	qma6101t_writereg(0x2f, reg_data);
#endif



	qma6101t_readreg(0x14, &reg_data,1);
	if(wdt_en)
	{	
		if(wdt_en == 1)
		{
			reg_data |= 0x80;
		}
		else
		{
			reg_data |= 0xC0;
		}
	}
	else
	{
		reg_data &= 0x7F;
	}

	qma6101t_writereg(0x14, reg_data);

	reg_data = 2;
	qma6101t_writereg(0x4a, reg_data);
	
#if defined(QMA6101T_TRIMED)
	reg_data = 5;
	qma6101t_writereg(0x60, reg_data);
#endif	
	qma6101t_dump_reg();

	return QMA6101T_SUCCESS;
}


qs32 qma6101t_init(int protocol)
{
	qs32 ret = QMA6101T_FAIL;
	qu8 slave_addr[2] = {QMA6101T_I2C_SLAVE_ADDR, QMA6101T_I2C_SLAVE_ADDR2};
	qu8 index = 0;	
	qu8 chip_id = 0x00;
	qu8 die_id_temp[2] = {0x00};
	qu8 wafer_id = 0x00;
	unsigned short die_id = 0x00;

	g_qma6101t.protocol = (qu8)protocol;
	for(index = 0; index < 2; index++)
	{
		chip_id = 0;
		g_qma6101t.slave = slave_addr[index];
		qma6101t_readreg(QMA6101T_CHIP_ID, &chip_id, 1);
		QMA6101T_LOG("qma6101t chip_id = 0x%x\n", chip_id);
		g_qma6101t.chip_id = (chip_id >> 4);
		if(g_qma6101t.chip_id == QMA6101T_DEVICE_ID)
		{
			QMA6101T_LOG("qma6101t find slave = 0x%x\n", g_qma6101t.slave);

			qma6101t_readreg(0x47, die_id_temp, 2);
			die_id = ((die_id_temp[1] << 8) | die_id_temp[0]);
			QMA6101T_LOG("qma6101t die_id = 0x%x\n", die_id);
			
			qma6101t_readreg(0x5c, &wafer_id, 1);
			QMA6101T_LOG("qma6101t wafer_id = 0x%x \n", wafer_id);
			
			break;
		}
	}

	if(g_qma6101t.chip_id == QMA6101T_DEVICE_ID)
	{
		ret = qma6101t_initialize();
	}
	else
	{
		ret = QMA6101T_FAIL;
	}
	
	return ret;
}


