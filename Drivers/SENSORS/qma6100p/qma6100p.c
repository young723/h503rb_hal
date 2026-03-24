/**
  ******************************************************************************
  * @file    qma6100p.c
  * @author  Yangzhiqiang@qst
  * @version V1.0
  * @date    2020-5-27
  * @brief    qma6100p
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */ 

#include "qma6100p.h"

#define M_PI		3.141592653589793f
#define M_G			9.80665f

typedef struct
{
	qu8					protocol;
	qu8					slave;
	qu8					chip_id;
	qs32				lsb_1g;
	qma6100p_fifo_mode	fifo_mode;
	qs32				fifo_len;
	qs16				raw[3];
	//float				acc[3];
}qma6100p_data;


static qma6100p_data g_qma6100p;

void qma6100p_delay(qu32 delay)
{
	qst_delay_ms(delay);
}

qs32 qma6100p_writereg(qu8 reg_add,qu8 reg_dat)
{
	qs32 ret = QMA6100P_FAIL;
	qu32 retry = 0;

	while((ret==QMA6100P_FAIL) && (retry++ < 5))
	{
		if(g_qma6100p.protocol == 0)
			ret = bsp_i2c_write_reg(g_qma6100p.slave, reg_add, reg_dat);
		else
			ret = bsp_spi_write_reg(reg_add, reg_dat);
	}

	return ret;
}

qs32 qma6100p_readreg(qu8 reg_add,qu8 *buf,qu16 num)
{
	qs32 ret = QMA6100P_FAIL;
	qu32 retry = 0;

	while((ret==QMA6100P_FAIL) && (retry++ < 5))
	{
		if(g_qma6100p.protocol == 0)
			ret = bsp_i2c_read_reg(g_qma6100p.slave, reg_add, buf, num);
		else
			ret = bsp_spi_read_reg(reg_add, buf, num);
	}

	return ret;
}


qs32 qma6100p_set_range(qs32 range)
{
	qs32 ret = 0;
	qu8 reg_data = (qu8)(range);

	if(range == QMA6100P_RANGE_4G)
	{
		g_qma6100p.lsb_1g = 2048;
	}
	else if(range == QMA6100P_RANGE_8G)
	{
		g_qma6100p.lsb_1g = 1024;
	}
	else if(range == QMA6100P_RANGE_16G)
	{
		g_qma6100p.lsb_1g = 512;
	}
	else if(range == QMA6100P_RANGE_32G)
	{
		g_qma6100p.lsb_1g = 256;
	}
	else
	{
		g_qma6100p.lsb_1g = 4096;
	}
	
	ret = qma6100p_writereg(QMA6100P_REG_RANGE, reg_data);

	return ret;
}


qs32 qma6100p_set_bw(qs32 bw)
{
	qs32 ret = 0;
	qu8 reg_data = (qu8)(bw|QMA6100P_LPF_1);

	ret = qma6100p_writereg(QMA6100P_REG_BW_ODR, reg_data);
	return ret;
}

qs32 qma6100p_set_mode(qs32 mode)
{
	qs32 ret = 0;
	qu8 reg_data = 0;

//	qma6100p_readreg(QMA6100P_REG_POWER_MANAGE, &reg_data, 1);
	if(mode >= QMA6100P_MODE_ACTIVE)
	{
		reg_data &= 0x7f;
		reg_data |= (qu8)QMA6100P_MCLK_51_2K|0x80;
		ret = qma6100p_writereg(QMA6100P_REG_POWER_MANAGE, reg_data);
		qma6100p_writereg(0x5f, 0x80);
		qma6100p_delay(2);
		qma6100p_writereg(0x5f, 0x00);
		qma6100p_delay(2);
		QMA6100P_LOG("enter active mode\n");
	}
	else
	{
	#if 1
		ret = qma6100p_writereg(QMA6100P_REG_POWER_MANAGE, 0x00);
	#else
		reg_data &= 0x7f;
		reg_data |= (qu8)QMA6100P_MCLK_6_4K|0x80;
		ret = qma6100p_writereg(QMA6100P_REG_POWER_MANAGE, reg_data);
		ret = qma6100p_writereg(0x46, 0x0f);
		ret = qma6100p_writereg(0x4A, 0x00);
	#endif
		QMA6100P_LOG("enter standby mode\n");
	}

	return ret;
}

void qma6100p_dump_reg(void)
{
	qu8 reg_data[8];
	qu8 wafer_id =0;
	qu16 die_id = 0;
	qs32 i=0;
	qu8 reg_map[]=
	{
	    0x0f,0x10,0x11,0x17,0x18,0x1a,0x1c,0x20,0x21,0x31,0x3e
	};
	QMA6100P_LOG("qma6100p_dump_reg\n");
	for(i=0; i< sizeof(reg_map)/sizeof(reg_map[0]); i++)
	{
		qma6100p_readreg(reg_map[i],&reg_data[0],1);
		QMA6100P_LOG("0x%x = 0x%x	", reg_map[i], reg_data[0]);
	}
	QMA6100P_LOG("\n");
	qma6100p_readreg(0x5a,&reg_data[0],1);
	wafer_id = reg_data[0];
	qma6100p_readreg(0x47,&reg_data[0],2);
	die_id = (qu16)((reg_data[1]<<8)|reg_data[0]);
	QMA6100P_LOG("wafer-id = 0x%x	die-id=0x%x\r\n", wafer_id, die_id);
}


void qma6100p_chip_info(qu32 *info)
{
	qu8 reg_data[8];
	qu8 waferid =0;
	qu16 dieid = 0;

	qma6100p_readreg(0x5a,&reg_data[0],1);
	waferid = reg_data[0];
	qma6100p_readreg(0x47,&reg_data[0],2);
	dieid = (qu16)((reg_data[1]<<8)|reg_data[0]);

	*info = (qu32)(((qu32)0x90<<24)|((qu32)waferid<<16)|(dieid&0xffff));
}


#if defined(QMA6100P_DATA_READY)
void qma6100p_drdy_config(qs32 int_map, qs32 enable)
{
	qu8 reg_int_map = QMA6100P_INT1_MAP_1;
	qu8 data_enable = 0;
	qu8 data_int_map= 0;

	QMA6100P_LOG("qma6100p_drdy_config %d %d\n", enable, int_map);	
	if(int_map == QMA6100P_MAP_INT1)
		reg_int_map = QMA6100P_INT1_MAP_1;
	else if(int_map == QMA6100P_MAP_INT2)
		reg_int_map = QMA6100P_INT2_MAP_1;
	else
		reg_int_map = 0x00;

	qma6100p_readreg(QMA6100P_INT_EN_1, &data_enable, 1);
	qma6100p_readreg(reg_int_map, &data_int_map, 1);

	if(enable)
	{
		data_enable |= QMA6100P_DRDY_BIT;
		data_int_map |= QMA6100P_DRDY_BIT;
	}
	else
	{
		data_enable &= (~QMA6100P_DRDY_BIT);
		data_int_map &= (~QMA6100P_DRDY_BIT);
	}
	qma6100p_writereg(QMA6100P_INT_EN_1, data_enable);
	qma6100p_writereg(reg_int_map, data_int_map);

}
#endif

#if defined(QMA6100P_FIFO_FUNC)
//static qu8 qma6100p_fifo_reg[64*6];
void qma6100p_fifo_config(qma6100p_fifo_mode fifo_mode, qu8 wmk, qs32 int_map, qs32 enable)
{
	qu8	reg_3e=0;
	qu8	reg_21=0;
	qu8 reg_int_map = QMA6100P_INT1_MAP_1;
	qu8 data_enable = 0;
	qu8 data_int_map= 0;

	QMA6100P_LOG("qma6100p_fifo_config mode:%d enable:%d\n", fifo_mode, enable);
	if(int_map == QMA6100P_MAP_INT1)
		reg_int_map = QMA6100P_INT1_MAP_1;
	else if(int_map == QMA6100P_MAP_INT2)
		reg_int_map = QMA6100P_INT2_MAP_1;
	else
		reg_int_map = 0x00;

	qma6100p_readreg(QMA6100P_INT_EN_1, &data_enable, 1);
	qma6100p_readreg(reg_int_map, &data_int_map, 1);
	qma6100p_readreg(0x21, &reg_21, 1);		// fifo use latch int
	qma6100p_readreg(0x3e, &reg_3e, 1);
	reg_3e &= 0x3f;

	qma6100p_writereg(0x21, reg_21|0x01);
	if(enable)
	{
		g_qma6100p.fifo_mode = fifo_mode;
		if(g_qma6100p.fifo_mode == QMA6100P_FIFO_MODE_FIFO)
		{
			reg_3e |= 0x47;
			qma6100p_writereg(0x31, wmk);	// max 0x40
			qma6100p_writereg(0x3E, reg_3e);	//bit[6:7] 0x00:BYPASS 0x40:FIFO 0x80:STREAM
			qma6100p_writereg(QMA6100P_INT_EN_1, data_enable|0x20);
			qma6100p_writereg(reg_int_map, data_int_map|0x20);
			//qma6100p_writereg(QMA6100P_INT_EN_1, data_enable|0x40);
			//qma6100p_writereg(reg_int_map, data_int_map|0x40);
		}
		else if(g_qma6100p.fifo_mode == QMA6100P_FIFO_MODE_STREAM)
		{	
			reg_3e |= 0x87;
			qma6100p_writereg(0x31, wmk);	// 0x3f
			qma6100p_writereg(0x3E, reg_3e);	//bit[6:7] 0x00:BYPASS 0x40:FIFO 0x80:STREAM
			qma6100p_writereg(QMA6100P_INT_EN_1, data_enable|0x40);
			qma6100p_writereg(reg_int_map, data_int_map|0x40);
		}
		else if(g_qma6100p.fifo_mode == QMA6100P_FIFO_MODE_BYPASS)
		{
			reg_3e |= 0x07;
			qma6100p_writereg(0x3E, reg_3e);	//bit[6:7] 0x00:BYPASS 0x40:FIFO 0x80:STREAM
			qma6100p_writereg(QMA6100P_INT_EN_1, data_enable|0x20);
			qma6100p_writereg(reg_int_map, data_int_map|0x20);
		}
	}
	else
	{
		g_qma6100p.fifo_mode = QMA6100P_FIFO_MODE_NONE;
		data_enable &= (~0x60);
		data_int_map &= (~0x60);
		qma6100p_writereg(QMA6100P_INT_EN_1, data_enable);
		qma6100p_writereg(reg_int_map, data_int_map);
	}
}

qs32 qma6100p_read_fifo(qu8 *fifo_buf)
{
	qs32 ret = 0;
	qu8 databuf[2];

#if 1//defined(QMA6100P_INT_LATCH)
	ret = qma6100p_readreg(QMA6100P_INT_STATUS_2, databuf, 1);
#endif
	ret = qma6100p_readreg(QMA6100P_FIFO_STATE, databuf, 1);
	if(ret != QMA6100P_SUCCESS)
	{
		QMA6100P_ERR("qma6100p_read_fifo state error\n");
		return 0;	//ret;
	}
	g_qma6100p.fifo_len = databuf[0]&0x7f;
	if(g_qma6100p.fifo_len > 64)
	{
		QMA6100P_ERR("qma6100p_read_fifo depth(%d) error\n",g_qma6100p.fifo_len);
		return 0;//QMA6100P_FAIL;
	}

	QMA6100P_ERR("qma6100p_read_fifo depth(%d)\n",g_qma6100p.fifo_len);

	if(fifo_buf)
	{
#if 0
		qma6100p_readreg(0x3f, fifo_buf, g_qma6100p.fifo_len*6);
#else
		for(int icount=0; icount<g_qma6100p.fifo_len; icount++)
		{
			qma6100p_readreg(0x3f, &fifo_buf[icount*6], 6);
		}
#endif
	}
	if(g_qma6100p.fifo_mode == QMA6100P_FIFO_MODE_FIFO)
	{
		ret = qma6100p_writereg(0x3e, 0x47);
	}
	else if(g_qma6100p.fifo_mode == QMA6100P_FIFO_MODE_STREAM)
	{
		ret = qma6100p_writereg(0x3e, 0x87);
	}
	else if(g_qma6100p.fifo_mode == QMA6100P_FIFO_MODE_BYPASS)
	{
		ret = qma6100p_writereg(0x3e, 0x07);
	}
	return g_qma6100p.fifo_len;
}

#if 0
void qma6100p_exe_fifo(qu8 *fifo_buf)
{
	qs32 icount;	
	qs16 raw_data[3];
	//float acc_data[3];

	QMA6100P_ERR("fifo_depth=%d\n", g_qma6100p.fifo_len);
// log fifo
	for(icount=0; icount<g_qma6100p.fifo_len; icount++)
	{
		raw_data[0]  = (qs16)(((qs16)(fifo_buf[1+icount*6]<<8)) |(fifo_buf[0+icount*6]));
		raw_data[1]  = (qs16)(((qs16)(fifo_buf[3+icount*6]<<8)) |(fifo_buf[2+icount*6]));
		raw_data[2]  = (qs16)(((qs16)(fifo_buf[5+icount*6]<<8)) |(fifo_buf[4+icount*6]));
		raw_data[0]  = raw_data[0]>>2;
		raw_data[1]  = raw_data[1]>>2;
		raw_data[2]  = raw_data[2]>>2;
		QMA6100P_LOG("%d:%d	%d	%d	",icount,raw_data[0],raw_data[1],raw_data[2]);
		if(icount%4==3)
		{
			QMA6100P_LOG("\r\n");
		}
		//acc_data[0] = (raw_data[0]*9.807f)/(g_qma6100p.lsb_1g);			//GRAVITY_EARTH_1000
		//acc_data[1] = (raw_data[1]*9.807f)/(g_qma6100p.lsb_1g);
		//acc_data[2] = (raw_data[2]*9.807f)/(g_qma6100p.lsb_1g);
	}	
	QMA6100P_LOG("\r\n");
// log fifo
}
#endif

#endif

#if defined(QMA6100P_STEPCOUNTER)
qu32 qma6100p_read_stepcounter(void)
{
	qu8 data[3];
	qu8 ret;
	qu32 step_num;
	qs32 step_dif;
	static qu32 step_last = 0;

	ret = qma6100p_readreg(QMA6100P_STEP_CNT_L, data, 2);	
	if(ret != QMA6100P_SUCCESS)
	{
		step_num = step_last;
		return step_num;
	}
	ret = qma6100p_readreg(QMA6100P_STEP_CNT_H, &data[2], 1);	
	if(ret != QMA6100P_SUCCESS)
	{
		step_num = step_last;
		return step_num;
	}

	step_num = (qu32)(((qu32)data[2]<<16)|((qu32)data[1]<<8)|data[0]);

#if 1//defined(QMA6100P_CHECK_ABNORMAL_DATA)
	step_dif = (qs32)(step_num-step_last);
	if(QMA6100P_ABS(step_dif) > 100)
	{
		qu32 step_num_temp[3];

		ret = qma6100p_readreg(QMA6100P_STEP_CNT_L, data, 2);	
		ret = qma6100p_readreg(QMA6100P_STEP_CNT_H, &data[2], 1);
		step_num_temp[0] = (qu32)(((qu32)data[2]<<16)|((qu32)data[1]<<8)|data[0]);
		qma6100p_delay(2);
		
		ret = qma6100p_readreg(QMA6100P_STEP_CNT_L, data, 2);	
		ret = qma6100p_readreg(QMA6100P_STEP_CNT_H, &data[2], 1);
		step_num_temp[1] = (qu32)(((qu32)data[2]<<16)|((qu32)data[1]<<8)|data[0]);
		qma6100p_delay(2);
		
		ret = qma6100p_readreg(QMA6100P_STEP_CNT_L, data, 2);	
		ret = qma6100p_readreg(QMA6100P_STEP_CNT_H, &data[2], 1);
		step_num_temp[2] = (qu32)(((qu32)data[2]<<16)|((qu32)data[1]<<8)|data[0]);
		qma6100p_delay(2);
		if((step_num_temp[0]==step_num_temp[1])&&(step_num_temp[1]==step_num_temp[2]))
		{
			QMA6100P_LOG("qma6100p check data, confirm!\n");
			step_num = step_num_temp[0];
		}
		else
		{	
			QMA6100P_LOG("qma6100p check data, abnormal!\n");
			return step_last;
		}
	}
#endif
	step_last = step_num;

	return step_num;
}

void qma6100p_stepcounter_clear(void)
{
	qu8 reg_13 = 0x00;
	qu32	retry = 0;
	#if 0
	qma6100p_readreg(0x13, &reg_13, 1);
	qma6100p_writereg(0x13, 0x80);	// clear step
	qma6100p_delay(2);
	qma6100p_writereg(0x13, reg_13);
	#else
	while(reg_13 != 0x80)
	{
		qma6100p_writereg(0x13, 0x80);	// clear step
		qma6100p_delay(2);
		qma6100p_readreg(0x13, &reg_13, 1);
		if(retry++ > 100)
		{
			QMA6100P_LOG("qma6100p write 0x13 timeout 1!\n");
			break;
		}
	}
	reg_13 = 0x00;
	retry = 0;
	while(reg_13 != 0x7f)
	{
		qma6100p_writereg(0x13, 0x7f);	// clear step
		qma6100p_delay(2);
		qma6100p_readreg(0x13, &reg_13, 1);
		if(retry++ > 100)
		{
			QMA6100P_LOG("qma6100p write 0x13 timeout 2!\n");
			break;
		}
	}
	#endif
}

void qma6100p_stepcounter_config(qs32 enable)
{	
	qs32 odr = 100;   //75;
	qu8 reg_14 = 0x00;
	qu8 reg_15 = 0x00;
	qu8 reg_1e = 0x00;
	qu8 reg_32 = 0x00;

	qma6100p_writereg(0x13, 0x80);	// clear step
	qma6100p_delay(1);
	if(enable)
	{
	    qma6100p_writereg(0x12, 0x8f);     // old 0x94
	}
	else
	{
		qma6100p_writereg(0x12, 0x00);
	}
	qma6100p_writereg(0x13, 0x7f);

	// odr 100 Hz, 10ms
	reg_14 = ((280*odr)/(1000));      // about:280 ms
	reg_15 = (((2000/8)*odr)/1000);   // 2000 ms
	QMA6100P_LOG("step time config 0x14=0x%x	0x15=0x%x\n", reg_14, reg_15);

	qma6100p_writereg(0x14, reg_14);
	qma6100p_writereg(0x15, reg_15);

	qma6100p_readreg(0x1e, &reg_1e, 1);
	reg_1e &= 0x3f;
	qma6100p_writereg(0x1e, (qu8)(reg_1e|QMA6100P_STEP_LPF_2));   // default 0x08
	// start count, p2p, fix peak
	qma6100p_writereg(0x1f, (qu8)QMA6100P_STEP_START_24|0x10);
	// select axis
	qma6100p_readreg(0x32, &reg_32, 1);
	reg_32 &= 0xfc;
	reg_32 |= (qu8)QMA6100P_STEP_AXIS_XY;
	qma6100p_writereg(0x32, reg_32);
}

#if defined(QMA6100P_STEP_INT)
void qma6100p_step_int_config(qs32 int_map, qs32 enable)
{
	qu8	reg_16=0;
	qu8	reg_19=0;
	qu8	reg_1b=0;

	qma6100p_readreg(0x16, &reg_16, 1);
	qma6100p_readreg(0x19, &reg_19, 1);
	qma6100p_readreg(0x1b, &reg_1b, 1);
	if(enable)
	{
		reg_16 |= 0x08;
		reg_19 |= 0x08;
		reg_1b |= 0x08;
		qma6100p_writereg(0x16, reg_16);
		if(int_map == QMA6100P_MAP_INT1)
			qma6100p_writereg(0x19, reg_19);
		else if(int_map == QMA6100P_MAP_INT2)
			qma6100p_writereg(0x1b, reg_1b);
	}
	else
	{
		reg_16 &= (~0x08);
		reg_19 &= (~0x08);
		reg_1b &= (~0x08);

		qma6100p_writereg(0x16, reg_16);
		qma6100p_writereg(0x19, reg_19);
		qma6100p_writereg(0x1b, reg_1b);
	}
}
#endif

#if defined(QMA6100P_SIGNIFICANT_STEP_INT)
void qma6100p_sigstep_int_config(qs32 int_map, qs32 enable)
{
	qu8	reg_16=0;
	qu8	reg_19=0;
	qu8	reg_1b=0;

	qma6100p_readreg(0x16, &reg_16, 1);
	qma6100p_readreg(0x19, &reg_19, 1);
	qma6100p_readreg(0x1b, &reg_1b, 1);
	
	qma6100p_writereg(0x1d, 0x1a);
	if(enable)
	{
		reg_16 |= 0x40;
		reg_19 |= 0x40;
		reg_1b |= 0x40;
		qma6100p_writereg(0x16, reg_16);
		if(int_map == QMA6100P_MAP_INT1)
			qma6100p_writereg(0x19, reg_19);
		else if(int_map == QMA6100P_MAP_INT2)
			qma6100p_writereg(0x1b, reg_1b);
	}
	else
	{
		reg_16 &= (~0x40);
		reg_19 &= (~0x40);
		reg_1b &= (~0x40);

		qma6100p_writereg(0x16, reg_16);
		qma6100p_writereg(0x19, reg_19);
		qma6100p_writereg(0x1b, reg_1b);
	}
}
#endif


#endif

#if defined(QMA6100P_ANY_MOTION)
void qma6100p_anymotion_config(qs32 int_map, qs32 enable)
{
	qu8 reg_int_map = QMA6100P_INT1_MAP_1;
	qu8 data_enable = 0;
	qu8 data_int_map= 0;

	qu8 reg_0x2c = 0;
	qu8 reg_0x2f = 0;
	qu8 reg_0x30 = 0;

	QMA6100P_LOG("qma6100p_anymotion_config %d\n", enable);
	if(int_map == QMA6100P_MAP_INT1)
		reg_int_map = QMA6100P_INT1_MAP_1;
	else if(int_map == QMA6100P_MAP_INT2)
		reg_int_map = QMA6100P_INT2_MAP_1;
	else
		reg_int_map = 0x00;

	qma6100p_readreg(QMA6100P_INT_EN_2, &data_enable, 1);
	qma6100p_readreg(reg_int_map, &data_int_map, 1);
	qma6100p_readreg(0x2c, &reg_0x2c, 1);
	qma6100p_readreg(0x2f, &reg_0x2f, 1);
	qma6100p_readreg(0x30, &reg_0x30, 1);

	reg_0x2c |= 0x00;	// (ANY_MOT_DUR<1:0> + 1)samples
	reg_0x2f &= (~0x40);	// bit6 ANY_MOT_IN_SEL, 0 : Any-motion Input is Slope. 1 : Any-motion Input is Acceleration
	reg_0x30 |= 0x80;	// add by yang, tep counter, raise wake, and tap detector,any motion by pass LPF

	qma6100p_writereg(0x2c, reg_0x2c);	// ANY_MOT_DUR[1:0]
	qma6100p_writereg(0x2e, 0x10);		// 0.976*16*16 = 250mg
	qma6100p_writereg(0x2f, reg_0x2f);
	qma6100p_writereg(0x30, reg_0x30); // default 0x3f
	// add by yang, tep counter, raise wake, and tap detector,any motion by pass LPF
	if(enable)
	{
		data_enable |= 0x07;
		data_int_map |= 0x01;
	}
	else
	{
		data_enable &= (~0x07);
		data_int_map &= (~0x01);
	}
	qma6100p_writereg(QMA6100P_INT_EN_2, data_enable);
	qma6100p_writereg(reg_int_map, data_int_map);
	
#if defined(QMA6100P_SIGNIFICANT_MOTION)
	data_int_map = 0;
	reg_int_map = QMA6100P_INT1_MAP_0;

	if(int_map == QMA6100P_MAP_INT1)
		reg_int_map = QMA6100P_INT1_MAP_0;
	else if(int_map == QMA6100P_MAP_INT2)
		reg_int_map = QMA6100P_INT2_MAP_0;
	else
		reg_int_map = 0x00;

	qma6100p_readreg(reg_int_map, &data_int_map, 1);

	if(enable)
	{
		reg_0x2f |= 0x01;		// bit0: selecat significant motion
		data_int_map |= 0x01;
	}
	else
	{
		reg_0x2f |= (~0x01);
		data_int_map &= (~0x01);
	}
	qma6100p_writereg(0x2f, reg_0x2f);
	qma6100p_writereg(reg_int_map, data_int_map);
#endif	
}
#endif

#if defined(QMA6100P_NO_MOTION)
void qma6100p_nomotion_config(qs32 int_map, qu16 second, qs32 enable)
{
	qu8 reg_int_map = QMA6100P_INT1_MAP_1;
	qu8 data_enable = 0;
	qu8 data_int_map= 0;
	qu8 reg_0x2c = 0;

	QMA6100P_LOG("qma6100p_nomotion_config %d\n", enable);
	if(int_map == QMA6100P_MAP_INT1)
		reg_int_map = QMA6100P_INT1_MAP_1;
	else if(int_map == QMA6100P_MAP_INT2)
		reg_int_map = QMA6100P_INT2_MAP_1;
	else
		reg_int_map = 0x00;

	qma6100p_readreg(QMA6100P_INT_EN_2, &data_enable, 1);
	qma6100p_readreg(reg_int_map, &data_int_map, 1);
	qma6100p_readreg(0x2c, &reg_0x2c, 1);
	// calc no-notion duration reg
	if(second <= 16)
	{
		reg_0x2c |=(((second-1)<<2)&0x3f);
	}
	else if(second <= 95)
	{
		if(second < 20)
		{
			second = 20;
		}
		second = second/5;
		reg_0x2c |=(((second-4)<<2)|0x40);
	}
	else
	{
		if(second < 100)
		{
			second = 100;
		}
		second = second/10;
		reg_0x2c |=(((second-10)<<2)|0x80);
	}
	// calc no-notion duration reg
	qma6100p_writereg(0x2c, reg_0x2c);		// 
	qma6100p_writereg(0x2d, 0x14);			// TH= NO_MOT_TH[7:0] * 16 * LSB

	if(enable)
	{
		data_enable |= 0xe0;
		data_int_map |= 0x80;
	}
	else
	{
		data_enable &= (~0xe0);
		data_int_map &= (~0x80);
	}
	qma6100p_writereg(QMA6100P_INT_EN_2, data_enable);
	qma6100p_writereg(reg_int_map, data_int_map);
}
#endif


#if defined(QMA6100P_TAP_FUNC)
void qma6100p_tap_config(qs32 tap_type, qs32 int_map, qs32 enable)
{
	qu8 reg_int_map = QMA6100P_INT1_MAP_0;
	qu8 data_enable = 0;
	qu8 data_int_map= 0;
	qu8 reg_1e, reg_30;
	qu8 tap_reg_en = (qu8)(tap_type);
	qu8 tap_reg_int = (qu8)(tap_type&0xfe);

	if(int_map == QMA6100P_MAP_INT1)
	{
		if(tap_type == QMA6100P_TAP_QUARTER)
		{
			reg_int_map = QMA6100P_INT1_MAP_1;
		}
		else
		{
			reg_int_map = QMA6100P_INT1_MAP_0;
		}
	}
	else if(int_map == QMA6100P_MAP_INT2)
	{
		if(tap_type == QMA6100P_TAP_QUARTER)
		{
			reg_int_map = QMA6100P_INT2_MAP_1;
		}
		else
		{
			reg_int_map = QMA6100P_INT2_MAP_0;
		}
	}	
	else
	{
		reg_int_map = 0x00;
	}

	qma6100p_readreg(QMA6100P_INT_EN_0, &data_enable, 1);
	qma6100p_readreg(reg_int_map, &data_int_map, 1);
	qma6100p_readreg(0x1e, &reg_1e, 1);
	qma6100p_readreg(0x30, &reg_30, 1);

	reg_1e |= 0x06;
	qma6100p_writereg(0x1e, reg_1e);		// TAP_QUIET_TH 31.25*8 = 250mg 
	qma6100p_writereg(0x2a, 0x86);			// 0x85 tap config1
	qma6100p_writereg(0x2b, (0xc0+6));		// tap config2
	// add by yang, step counter, raise wake, and tap detector,any motion by pass LPF
	reg_30 |= 0x80|0x40;
	qma6100p_writereg(0x30, reg_30);	// default 0x3f
	// add by yang, tep counter, raise wake, and tap detector,any motion by pass LPF

	if(enable)
	{
		data_enable |= tap_reg_en;

		if(tap_type == QMA6100P_TAP_QUARTER)
		{
			data_int_map |= 0x02;
		}
		else
		{
			data_int_map |= (tap_reg_int);
		}
	}
	else
	{	
		data_enable &= (~tap_reg_en);
		if(tap_type == QMA6100P_TAP_QUARTER)
		{
			data_int_map &= (~0x02);
		}
		else
		{
			data_int_map &= (~tap_reg_int);
		}
	}
	
	qma6100p_writereg(QMA6100P_INT_EN_0, data_enable);
	qma6100p_writereg(reg_int_map, data_int_map);
}
#endif

#if defined(QMA6100P_HAND_RAISE_DOWN)
void qma6100p_hand_raise_down(qs32 layout, qs32 int_map, qs32 enable)
{
	qu8 reg_16,reg_19,reg_1b;
	//qu8 reg_24,reg_25;
	qu8 reg_0x30, reg_0x34,reg_0x35,reg_0x42;		// swap x y

	qu8 yz_th_sel = 4;
	qs8 y_th = -3; //-2;				// -16 ~ 15
	qu8 x_th = 6; 	// 0--7.5
	qs8 z_th = 6;				// -8--7

	reg_16 = reg_19 = reg_1b = 0;
	//reg_24 = reg_25 = 0;
	reg_0x34 = reg_0x35 = reg_0x42 = 0;
	if(layout%2)
	{
		qma6100p_readreg(0x42, &reg_0x42, 1);
		reg_0x42 |= 0x80;		// 0x42 bit 7 swap x and y
		qma6100p_writereg(0x42, reg_0x42);
	}
	else
	{
		qma6100p_readreg(0x42, &reg_0x42, 1);
		reg_0x42 &= 0x7f;		// 0x42 bit 7 swap x and y
		qma6100p_writereg(0x42, reg_0x42);
	}
	qma6100p_readreg(0x42, &reg_0x42, 1);
	QMA6100P_LOG("qma6100p_hand_raise_down 0x42 = 0x%x\n", reg_0x42);

	if((layout >=0) && (layout<=3))
	{
		z_th = 3;
		if((layout == 2)||(layout == 3))
			y_th = 3; 
		else if((layout == 0)||(layout == 1))	
			y_th = -3;
	}
	else if((layout >=4) && (layout<=7))
	{
		z_th = -3;
		
		if((layout == 6)||(layout == 7))
			y_th = 3; 
		else if((layout == 4)||(layout == 5))	
			y_th = -3;
	}
	// 0x34 YZ_TH_SEL[7:5]	Y_TH[4:0], default 0x9d  (YZ_TH_SEL   4   9.0 m/s2 | Y_TH  -3  -3 m/s2)
	//qmaX981_write_reg(0x34, 0x9d);	//|yz|>8 m/s2, y>-3 m/m2
	if((y_th&0x80))
	{
		reg_0x34 |= yz_th_sel<<5;
		reg_0x34 |= (y_th&0x0f)|0x10;
		qma6100p_writereg(0x34, reg_0x34);
	}
	else
	{	
		reg_0x34 |= yz_th_sel<<5;
		reg_0x34 |= y_th;
		qma6100p_writereg(0x34, reg_0x34);	//|yz|>8m/s2, y<3 m/m2
	}
	//Z_TH<7:4>: -8~7, LSB 1 (unit : m/s2)	X_TH<3:0>: 0~7.5, LSB 0.5 (unit : m/s2) 
	//qmaX981_write_reg(0x1e, 0x68);	//6 m/s2, 4 m/m2
	qma6100p_writereg(0x22, (0x19|(0x03<<6)));			// 12m/s2 , 0.5m/s2
	qma6100p_writereg(0x23, (0x7c|(0x03>>2)));
	//qmaX981_write_reg(0x22, (0x19|(0x02<<6)));			// 12m/s2 , 0.5m/s2
	//qmaX981_write_reg(0x23, (0x7c|(0x02)));

	if((z_th&0x80))
	{
		reg_0x35 |= (x_th&0x0f);
		reg_0x35 |= ((z_th<<4)|0x80);
		qma6100p_writereg(0x35, reg_0x35);
	}
	else
	{
		reg_0x35 |= (x_th&0x0f);
		reg_0x35 |= (z_th<<4);
		qma6100p_writereg(0x35, reg_0x35);
	}

	// RAISE_WAKE_PERIOD*(1/ODR), default 0x81
	qma6100p_writereg(0x25, 0x50);
	// add by yang, tep counter, raise wake, and tap detector,any motion by pass LPF
	qma6100p_readreg(0x30, &reg_0x30, 1);
	qma6100p_writereg(0x30, reg_0x30|0x40|0x3f);	// default 0x3f	
	// add by yang, tep counter, raise wake, and tap detector,any motion by pass LPF

	qma6100p_readreg(0x16, &reg_16, 1);
	qma6100p_readreg(0x19, &reg_19, 1);
	qma6100p_readreg(0x1b, &reg_1b, 1);

	// 0x24: RAISE_WAKE_TIMEOUT_TH<7:0>: Raise_wake_timeout_th[11:0] * ODR period = timeout count
	// 0x25: RAISE_WAKE_PERIOD<7:0>: Raise_wake_period[10:0] * ODR period = wake count
	// 0x26:
	// RAISE_MODE: 0:raise wake function, 1:ear-in function
	// RAISE_WAKE_PERIOD<10:8>: Raise_wake_period[10:0] * ODR period = wake count
	// RAISE_WAKE_TIMEOUT_TH<11:8>: Raise_wake_timeout_th[11:0] * ODR period = timeout count

	if(enable)
	{
		reg_16 |= (0x02|0x04);
		reg_19 |= (0x02|0x04);
		reg_1b |= (0x02|0x04);
		qma6100p_writereg(0x16, reg_16);
		if(int_map == QMA6100P_MAP_INT1)
			qma6100p_writereg(0x19, reg_19);
		else if(int_map == QMA6100P_MAP_INT2)
			qma6100p_writereg(0x1b, reg_1b);
	}
	else
	{
		reg_16 &= ~((0x02|0x04));
		reg_19 &= ~((0x02|0x04));
		reg_1b &= ~((0x02|0x04));
		qma6100p_writereg(0x16, reg_16);
		qma6100p_writereg(0x19, reg_19);
		qma6100p_writereg(0x1b, reg_1b);
	}
}
#endif

void qma6100p_selftest(void)
{
	short gxvalue,gyvalue,gzvalue,xvalue,yvalue,zvalue;
	uint8_t reg,cnt,pn=0;
	uint8_t rawdata[6];
	
	qma6100p_set_mode(QMA6100P_MODE_STANDBY);
	qma6100p_set_bw(QMA6100P_BW_100);
	qma6100p_set_range(QMA6100P_RANGE_8G);
	qma6100p_set_mode(QMA6100P_MODE_ACTIVE);

	qma6100p_delay(80);

	qma6100p_readreg(QMA6100P_XOUTL, rawdata, 6);
	xvalue = (short)(((unsigned short)rawdata[1] << 8) + (unsigned short)rawdata[0])>>2;
	yvalue = (short)(((unsigned short)rawdata[3] << 8) + (unsigned short)rawdata[2])>>2;
	zvalue = (short)(((unsigned short)rawdata[5] << 8) + (unsigned short)rawdata[4])>>2;	
	QMA6100P_LOG("POLL Rawdata:%d, %d, %d\r\n",xvalue,yvalue,zvalue);

	for(pn=0;pn<=1;pn++)
	{
		reg = 0x80;
		if(pn==1)
		{
			reg = 0x84;
		}
		qma6100p_writereg(0x32, reg);
		QMA6100P_LOG("Generate E-force 0x%x,wait 100ms(at least the 4th data) then get data\r\n",reg);
		
		qma6100p_delay(100);

		while(1)
		{
			qma6100p_readreg(QMA6100P_XOUTL, rawdata, 6);
			gxvalue = (short)(((unsigned short)rawdata[1] << 8) + (unsigned short)rawdata[0])>>2;
			gyvalue = (short)(((unsigned short)rawdata[3] << 8) + (unsigned short)rawdata[2])>>2;
			gzvalue = (short)(((unsigned short)rawdata[5] << 8) + (unsigned short)rawdata[4])>>2;
			//QMA6100P_LOG("POLL after E-Force:%d, %d, %d\r\n",gxvalue,gyvalue,gzvalue);

			QMA6100P_LOG("Delta between Force:%d, %d, %d\r\n",gxvalue-xvalue,gyvalue-yvalue,gzvalue-zvalue);
			cnt++;
			qma6100p_delay(50);
			if(cnt==10)
			{
				cnt = 0;
				QMA6100P_LOG("Remove E-force \r\n");
			    reg= 0x00;
				qma6100p_writereg(0x32, reg);
				break;
			}
		}
	}
	
	return;
}

void qma6100p_irq_hdlr(void)
{
	qu8 ret = QMA6100P_FAIL;
	qu8 databuf[4];
	qs32 retry = 0;

	while((ret==QMA6100P_FAIL)&&(retry++<10))
	{
		ret = qma6100p_readreg(QMA6100P_INT_STATUS_0, databuf, 4);
		if(ret == QMA6100P_SUCCESS)
		{
			break;
		}
	}
	if(ret == QMA6100P_FAIL)
	{
		QMA6100P_LOG("qma6100p_irq_hdlr read status fail!\n");
		return;
	}
	else
	{
		//QMA6100P_LOG("irq [0x%x 0x%x 0x%x 0x%x]\n", databuf[0],databuf[1],databuf[2],databuf[3]);
	}

#if defined(QMA6100P_DATA_READY)
	if(databuf[2]&0x10)
	{
		qma6100p_read_raw_xyz(g_qma6100p.raw);
		QMA6100P_LOG("drdy	%d	%d	%d\n",g_qma6100p.raw[0],g_qma6100p.raw[1],g_qma6100p.raw[2]);
		//qma6100p_read_acc_xyz(g_qma6100p.acc);
		//QMA6100P_LOG("drdy	%f	%f	%f\n",g_qma6100p.acc[0],g_qma6100p.acc[1],g_qma6100p.acc[2]);
	}
#endif
#if defined(QMA6100P_FIFO_FUNC)
	if((databuf[2]&0x40)&&(g_qma6100p.fifo_mode==QMA6100P_FIFO_MODE_STREAM))
	{
		QMA6100P_LOG("FIFO WMK\n");
		//qma6100p_read_fifo(qma6100p_fifo_reg);
		//qma6100p_exe_fifo(qma6100p_fifo_reg);
	}
	else if((databuf[2]&0x20)) 
	{
		QMA6100P_LOG("FIFO FULL\n");
		//qma6100p_read_fifo(qma6100p_fifo_reg);
		//qma6100p_exe_fifo(qma6100p_fifo_reg);
	}
#endif
#if defined(QMA6100P_ANY_MOTION)
	if(databuf[0]&0x07)
	{
		QMA6100P_LOG("any motion!\n");
	}
#if defined(QMA6100P_SIGNIFICANT_MOTION)
	if(databuf[1]&0x01)
	{
		QMA6100P_LOG("significant motion!\n");
	}
#endif
#endif
#if defined(QMA6100P_NO_MOTION)
	if(databuf[0]&0x80)
	{
		QMA6100P_LOG("no motion!\n");
	}
#endif
#if defined(QMA6100P_STEP_INT)
	if(databuf[1]&0x08)
	{
		QMA6100P_LOG("step int! step=%d\n",qma6100p_read_stepcounter());
	}
#endif
#if defined(QMA6100P_SIGNIFICANT_STEP_INT)
	if(databuf[1]&0x40)
	{
		QMA6100P_LOG("significant step int! step=%d\n",qma6100p_read_stepcounter());
	}
#endif
#if defined(QMA6100P_TAP_FUNC)
	if(databuf[1]&0x80)
	{
		QMA6100P_LOG("SINGLE tap int!\n");
	}
	if(databuf[1]&0x20)
	{
		QMA6100P_LOG("DOUBLE tap int!\n");
	}
	if(databuf[1]&0x10)
	{
		QMA6100P_LOG("TRIPLE tap int!\n");
	}	
	if(databuf[2]&0x01)
	{
		QMA6100P_LOG("QUARTER tap int!\n");
	}
#endif
#if defined(QMA6100P_HAND_RAISE_DOWN)
	if(databuf[1]&0x02)
	{
		QMA6100P_LOG("hand raise!\n");
	}
	if(databuf[1]&0x04)
	{
		QMA6100P_LOG("hand down!\n");
	}
#endif
}

void qma6100p_axis_convert(short data_a[3], int layout)
{
	short raw[3];

	raw[0] = data_a[0];
	raw[1] = data_a[1];
	//raw[2] = data[2];


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

qs32 qma6100p_read_raw_xyz(qs16 data[3])
{
	qu8 databuf[6] = {0}; 	
	qs16 raw_data[3];	
	qu8 drdy = 0;
	qs32 ret = 0;

#if defined(QMA6100P_DATA_READY)
	qs32 count = 0;
	ret = qma6100p_readreg(QMA6100P_INT_STATUS_2, &drdy, 1);
	while(!(drdy & 0x10)&& count++<3)
	{
		qma6100p_delay(1);
		ret = qma6100p_readreg(QMA6100P_INT_STATUS_2, &drdy, 1);
	}
#else
	//drdy = (databuf[0]&0x01) + (databuf[2]&0x01) + (databuf[4]&0x01);
	drdy = 0x10;
#endif
	
	if(drdy & 0x10)
	{	
		ret = qma6100p_readreg(QMA6100P_XOUTL, databuf, 6);
		if(ret == QMA6100P_FAIL)
		{
			QMA6100P_ERR("read xyz read reg error!!!\n");
			return QMA6100P_FAIL;	
		}
		raw_data[0] = (qs16)(((databuf[1]<<8))|(databuf[0]));
		raw_data[1] = (qs16)(((databuf[3]<<8))|(databuf[2]));
		raw_data[2] = (qs16)(((databuf[5]<<8))|(databuf[4]));
		data[0] = raw_data[0]>>2;
		data[1] = raw_data[1]>>2;
		data[2] = raw_data[2]>>2;
		//QMA6100P_LOG("1--%d	%d	%d\n",raw_data[0],raw_data[1],raw_data[2]);
		//QMA6100P_LOG("2--%d	%d	%d\n",data[0],data[1],data[2]);

		return QMA6100P_SUCCESS;
	}
	else
	{
		//QMA6100P_ERR("read xyz data ready error!!!\n");
		return QMA6100P_FAIL;
	}
}

qs32 qma6100p_read_acc_xyz(float accData[3])
{
	qs32 ret;
	qs16 rawData[3];

	ret = qma6100p_read_raw_xyz(rawData);
	if(ret == QMA6100P_SUCCESS)
	{
		qma6100p_axis_convert(rawData, 0);
		g_qma6100p.raw[0] = rawData[0];
		g_qma6100p.raw[1] = rawData[1];
		g_qma6100p.raw[2] = rawData[2];
	}
	accData[0] = (float)(g_qma6100p.raw[0]*M_G)/(g_qma6100p.lsb_1g);
	accData[1] = (float)(g_qma6100p.raw[1]*M_G)/(g_qma6100p.lsb_1g);
	accData[2] = (float)(g_qma6100p.raw[2]*M_G)/(g_qma6100p.lsb_1g);

	return QMA6100P_SUCCESS;

}


qs32 qma6100p_soft_reset(void)
{
	qu8 reg_0x33 = 0;
	qu32 retry = 0;

	QMA6100P_LOG("qma6100p_soft_reset\n");	
//	qma6100p_writereg(QMA6100P_REG_POWER_MANAGE, 0x84);
//	qma6100p_delay(5);
	qma6100p_writereg(QMA6100P_REG_RESET, 0xb6);
	qma6100p_delay(10);
	qma6100p_writereg(QMA6100P_REG_RESET, 0x00);
	qma6100p_delay(10);
#if defined(QMA6100P_USE_SPI)
	#if (QMA6100P_USE_SPI == 3)
	qma6100p_writereg(QMA6100P_INTPIN_CFG, 0x25);
	#else
	qma6100p_writereg(QMA6100P_INTPIN_CFG, 0x05);
	#endif
#endif

#if defined(QMA6100P_DIS_AD0)
	{
		qu8 slave_addr[2] = {QMA6100P_I2C_SLAVE_ADDR, QMA6100P_I2C_SLAVE_ADDR2};
		qu8 index = 0;
		qu8 chip_id = 0x00;
		qma6100p_writereg(QMA6100P_INTPIN_CFG, 0x45);
		for(index=0; index<2; index++)
		{
			chip_id = 0;
			g_qma6100p.slave = slave_addr[index];
			qma6100p_readreg(QMA6100P_CHIP_ID, &chip_id, 1);
			QMA6100P_LOG("qma6100p chip_id=0x%x\n", chip_id);
			g_qma6100p.chip_id = (chip_id>>4);
			if(g_qma6100p.chip_id == QMA6100P_DEVICE_ID)
			{
				QMA6100P_LOG("qma6100p find slave=0x%x\n",g_qma6100p.slave);
				break;
			}
		}
	}
#endif

	// check otp
	retry = 0;
	while(retry++ < 100)
	{
		qma6100p_readreg(QMA6100P_REG_NVM, &reg_0x33, 1);
		QMA6100P_LOG("confirm-%d read 0x33 = 0x%x\n",retry, reg_0x33);
		if((reg_0x33 & 0x01) && (reg_0x33 & 0x04))
		{
			break;
		}
		qma6100p_delay(5);
	}
	if(retry>=100)
	{	
		QMA6100P_LOG("Read 0x33 status timeout\r\n");
		return QMA6100P_FAIL;
	}

	return QMA6100P_SUCCESS;
}


static qs32 qma6100p_initialize(void)
{
	qu8 reg_0x45 = 0;
	qu32 cnt = 0;

	qma6100p_soft_reset();
	while(1)
	{
		reg_0x45 = 0x00;
		qma6100p_readreg(0x45, &reg_0x45, 1);
		if( (reg_0x45&0xF0) != 0xC0 )
		{	
			qma6100p_soft_reset();
			cnt++;
			if(cnt>=100)
			{
				QMA6100P_LOG("QMA6100P SWRst fail\r\n");
			  	break;
			}
		}
		else
		{
			QMA6100P_LOG("QMA6100P SWRst end\r\n");
			break;
		}
	}

	//qma6100p_writereg(0x11, 0x80);
	qma6100p_writereg(0x11, 0x84);	
	//qma6100p_writereg(0x50, 0x51);
	qma6100p_writereg(0x4a, 0x20);
	qma6100p_writereg(0x56, 0x01);
	qma6100p_writereg(0x5f, 0x80);
	qma6100p_delay(2);
	qma6100p_writereg(0x5f, 0x00);
	qma6100p_delay(2);

	//qma6100p_writereg(QMA6100P_INT_EN_0, 0x00);
	//qma6100p_writereg(QMA6100P_INT_EN_1, 0x00);
	//qma6100p_writereg(QMA6100P_INT_EN_2, 0x00);
	//qma6100p_writereg(QMA6100P_INT1_MAP_0, 0x00);
	//qma6100p_writereg(QMA6100P_INT1_MAP_1, 0x00);
	//qma6100p_writereg(QMA6100P_INT2_MAP_0, 0x00);
	//qma6100p_writereg(QMA6100P_INT2_MAP_1, 0x00);
	//qma6100p_writereg(0x3e, 0x07);

	qma6100p_set_range(QMA6100P_RANGE_8G);
	qma6100p_set_bw(QMA6100P_BW_100);
	qma6100p_set_mode(QMA6100P_MODE_ACTIVE);

#if 0	// MCLK to int1, 52.25K Hz
	qma6100p_writereg(0x49, 0x01);
	qma6100p_writereg(0x56, 0x10);
#endif	// MCLK to int1

#if defined(QMA6100P_DATA_READY)
	qma6100p_drdy_config(QMA6100P_MAP_INT1, QMA6100P_ENABLE);
#endif

#if defined(QMA6100P_FIFO_FUNC)
	qma6100p_fifo_config(QMA6100P_FIFO_MODE_STREAM, 4, QMA6100P_MAP_INT1, QMA6100P_ENABLE);
#endif

#if defined(QMA6100P_STEPCOUNTER)
	qma6100p_stepcounter_config(QMA6100P_ENABLE);
	#if defined(QMA6100P_STEP_INT)
	qma6100p_step_int_config(QMA6100P_MAP_INT1, QMA6100P_ENABLE);
	#endif	
	#if defined(QMA6100P_SIGNIFICANT_STEP_INT)
	qma6100p_sigstep_int_config(QMA6100P_MAP_INT1, QMA6100P_ENABLE);
	#endif
#endif

#if defined(QMA6100P_ANY_MOTION)
	qma6100p_anymotion_config(QMA6100P_MAP_INT1, QMA6100P_ENABLE);
#endif
#if defined(QMA6100P_NO_MOTION)
	qma6100p_nomotion_config(QMA6100P_MAP_INT1, 10, QMA6100P_ENABLE);
#endif

#if defined(QMA6100P_TAP_FUNC)
	qma6100p_tap_config(QMA6100P_TAP_SINGLE, QMA6100P_MAP_INT1, QMA6100P_ENABLE);
#endif

#if defined(QMA6100P_HAND_RAISE_DOWN)
	qma6100p_hand_raise_down(3, QMA6100P_MAP_INT1, QMA6100P_ENABLE);
#endif
	// int config
	qma6100p_writereg(QMA6100P_INTPIN_CFG, QMA6110P_IRQ1_TRIGGER_LEVEL|QMA6110P_IRQ2_TRIGGER_LEVEL);
#if defined(QMA6100P_INT_LATCH)
	qma6100p_writereg(QMA6100P_INT_CFG, 0x03);
#else
	qma6100p_writereg(QMA6100P_INT_CFG, 0x01);
#endif
	// int config
	unsigned char reg_data[4];
	qma6100p_readreg(0x09, reg_data, 4);
	QMA6100P_LOG("read status=[0x%x 0x%x 0x%x 0x%x] \n", reg_data[0],reg_data[1],reg_data[2],reg_data[3]);

	qma6100p_dump_reg();

	return QMA6100P_SUCCESS;
}


qs32 qma6100p_init(int protocol)
{
	qs32 ret = QMA6100P_FAIL;
	qu8 slave_addr[2] = {QMA6100P_I2C_SLAVE_ADDR, QMA6100P_I2C_SLAVE_ADDR2};
	qu8 index = 0;	
	qu8 chip_id = 0x00;

	g_qma6100p.protocol = (qu8)protocol;

#if defined(QMA6100P_USE_SPI)
#if (QMA6100P_USE_SPI == 3)
	qma6100p_writereg(QMA6100P_INTPIN_CFG, 0x25);
#else
	qma6100p_writereg(QMA6100P_INTPIN_CFG, 0x05);
#endif
#endif
	for(index=0; index<2; index++)
	{
		chip_id = 0;
#if defined(QMA6100P_USE_SPI)
		g_qma6100p.slave = 0x00;
#else
		g_qma6100p.slave = slave_addr[index];
#endif
		qma6100p_readreg(QMA6100P_CHIP_ID, &chip_id, 1);
		QMA6100P_LOG("qma6100p chip_id=0x%x\n", chip_id);
		g_qma6100p.chip_id = (chip_id>>4);
		if(g_qma6100p.chip_id == QMA6100P_DEVICE_ID)
		{
			QMA6100P_LOG("qma6100p find slave=0x%x\n",g_qma6100p.slave);
			break;
		}
	}

#if !defined(QMA6100P_USE_SPI)
#if defined(QMA6100P_DIS_AD0)
	qma6100p_writereg(QMA6100P_INTPIN_CFG, 0x45);
	for(index=0; index<2; index++)
	{
		chip_id = 0;
		g_qma6100p.slave = slave_addr[index];
		qma6100p_readreg(QMA6100P_CHIP_ID, &chip_id, 1);
		QMA6100P_LOG("qma6100p chip_id=0x%x\n", chip_id);
		g_qma6100p.chip_id = (chip_id>>4);
		if(g_qma6100p.chip_id == QMA6100P_DEVICE_ID)
		{
			QMA6100P_LOG("qma6100p find slave=0x%x\n",g_qma6100p.slave);
			break;
		}
	}
#endif
#endif

	if(g_qma6100p.chip_id == QMA6100P_DEVICE_ID)
	{
		ret = qma6100p_initialize();
	}
	else
	{
		ret = QMA6100P_FAIL;
	}
	
	return ret;
}

