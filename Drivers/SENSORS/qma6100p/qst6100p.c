/*****************************************************************************
written  by rock.fan in 1/21/2022
platform : 
modify :  set_anymotion move  'write to 0x2E' after 'write to 0x2F'
 *****************************************************************************/


///----------------------------------------------------------------------------
/// Headers
/// ---------------------------------------------------------------------------

#include "qst6100p.h"

///----------------------------------------------------------------------------
/// Local variables
/// ---------------------------------------------------------------------------

static unsigned char qma6100p_slave = 0x12;

extern void qst_delay_ms(unsigned int n_ms);

void set_chip_mode(uint8_t state);
void ANA_setting_1(void);


int qst_read_regs(unsigned char register_address, unsigned char * destination, unsigned char number_of_bytes)
{
	int ret = 0;
	unsigned int retry = 0;

	while((ret==0) && (retry++ < 5))
	{
#if defined(QMA6100P_USE_SPI)
		ret = bsp_spi_read_reg(register_address, destination, number_of_bytes);
#else
		ret = bsp_i2c_read_reg(qma6100p_slave, register_address, destination, number_of_bytes);
#endif
	}

	return ret;
}

int qst_write_regs(unsigned char register_address, unsigned char *value, unsigned char number_of_bytes)
{
	int ret = 0;
	int retry = 0;

	for(int i=0; i<number_of_bytes; i++)
	{
		ret = 0;
		retry = 0;
		while((ret==0) && (retry++ < 5))
		{	
#if defined(QMA6100P_USE_SPI)
			ret = bsp_spi_write_reg(register_address + i, value[i]);
#else
			ret = bsp_i2c_write_reg(qma6100p_slave, register_address + i, value[i]);
#endif
		}
	}

	return ret;
}

#if 0
/*just for reference , some of values may be changed */
const uint8_t reg_default_value[0x60]=
{
0x90,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*0x00~0x0f*/
	
/*Id-DX------------------------DZ,STCNT[~15],INTSTATE 1~~~~~~~4,STCNT[8],FIFCNT,RangeLPF*/

0x00,0x00,0x14,0x7f,0x19,0x16,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0xA9,/*0x10~0x1f*/
/*BW,PMcl,STEPCONFIG 0------3,INT_EN 0-----3,INT_MAP 0---------3,STEP_TAP_CONFIG*/

0x05,0x00,0xD8,0x7C,0x00,0x81,0x02,0x00,0x00,0x00,0x05,0xCD,0x00,0x00,0x00,0x00,/*0x20~0x2f*/

0x3F,0x00,0x00,0x05,0x9D,0x66,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x07,0x00,/*0x30-0x3F*/
								  /*0x37--------------NVM-------------0x3D*/
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*0x40-0x4F*/
/*0x40------------NVM-------------0x48*/
0x00,0x00,0x00,0xFF,0xFF,0xFF,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00/*0x50-0x5F*/
										/*0x58---------NVM---------0x5D*/
};
#endif


/* 
After power up , sensor will reload OTP automatically .
and if send 0xB6 to Reg 0x36 , it will trigger softwarereset ,  
and if you send 0x00 to Reg 0x36 a few cycles later ,  it will trigger OTP_LOADING too . 

POWERUP  -  otpload

SOFTWARERESET (0x36 [0xb6 ,0x00])  - otpload 

OTPLOAD (0x33 bit3 set '1') -otpload 

*/
void softwarereset(void)
{
	uint8_t i,reg,reg_read,cnt=0;
#ifdef DIS_AD0
	uint8_t reg_20 = 0x45;
	uint8_t slave_addr[2] = {0x12, 0x13};
#endif
	printf("SOFTWARE RESET \r\n");
	reg=0xb6;
	qst_write_regs(0x36,&reg,1);
	qst_delay_ms(5); // delay time can be very short  , such as the time between two i2c cmd .	
	reg=0x00;
    qst_write_regs(0x36,&reg,1);

#ifdef DIS_AD0
	reg = 0x00;
	reg_20 = 0x45;
	qst_write_regs(0x20, &reg_20, 1);
	for(i=0; i<2; i++)
	{
		qma6100p_slave = slave_addr[i];
		qst_read_regs(0x00,&reg,1);
		if((reg&0xF0)==0x90)
		{	 
			break;
		}
	}
	if((reg&0xF0)!=0x90)
	{	 
		printf("QMA6100P not found\r\n");
	}
#endif
	
	qst_read_regs(0x33,&reg_read,1);
	printf("tim 0x33 = 0x%x\r\n",reg_read);
	while((reg_read&0x05)!=0x05)
	{
		qst_read_regs(0x33,&reg_read,1);
		qst_delay_ms(5);
		cnt++;
		if((reg_read&0x05)==0x05)
		printf("timeout cnt=%d, 0x33 = 0x%x\r\n",cnt,reg_read);
		if(cnt>=100)
		{	
			printf("Read 0x33 status timeout\r\n");
			break;
		}
	} 
	//qst_delay_ms(10);

	printf("SOFTWARE RESET END\r\n");
}

/*  
	if send '0x08' to 0x33 Reg , it will reload otp content when chip is in wakemode,if you want to issue 'RELOAD OTP' .	
	suggest issuing this CMD after software reset,and wait until 0x33 bit[0] & bit[2] is 1.  
*/

void reloadOTP(void)
{
	uint8_t reg,reg_read,cnt=0;
	printf("Reload OTP \r\n");
	
	qst_read_regs(0x11,&reg_read,1);
	while((reg_read&0x80)!=0x80)
	{
		set_chip_mode(WAKEMODE);
		qst_delay_ms(3); 
		qst_read_regs(0x11,&reg_read,1);
	}
	
	reg = 0x08;
	qst_write_regs(0x33,&reg,1);
	
	qst_read_regs(0x33,&reg_read,1);
	printf("tim 0x33 = 0x%x\r\n",reg_read);
	while((reg_read&0x05)!=0x05)
	{
		qst_read_regs(0x33,&reg_read,1);
		qst_delay_ms(5);
		cnt++;
		if((reg_read&0x05)==0x05)
		{
			printf("timeout cnt=%d, 0x33 = 0x%x\r\n",cnt,reg_read);
		}
		else if(cnt>=100)
		{	
			printf("Read 0x33 status timeout\r\n");
			break;
		}
	} 
	

	printf("Reload OTP END\r\n");
}

/*

 Compare all the resigers value with the default value. 

*/
void ComparetoDefaultRegvalue(void)
{
	uint8_t i,j,k,reg;
    printf("6100P Before WakeMode\r\n");
	//printf("  0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\r\n");
	printf("  0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F\r\n");
	for(i=0;i<=0x60;i++)
		{	
			if(i==0x3f)
				reg = 0x00;
			else
			qst_read_regs(i,&reg,1);
			qst_delay_ms(2);
				k = i/16;
				j = i/15;
			if( i == (16*k) )
				printf("%x %2x  ",(i/15),reg);
			else if( i == (16*j-1) )
				printf("%2x\r\n",reg);
			else
				printf("%2x  ",reg);
		}
	printf("\r\n");
}

void get_dieID_WaferID(void)
{
	uint16_t dieid ;
	uint8_t waferid;
	uint8_t reg[2];

	qst_read_regs(0x47,reg,2);
	dieid = (reg[1]<<8)|reg[0];
	printf("dieID=0x%x,", dieid);
	qst_read_regs(0x5A,reg,1);
	waferid = reg[0]&0x7F;
	printf("waferID=0x%x\r\n", waferid);

}

int acc_read_raw_xyz(int16_t data[3])
{
	uint8_t databuf[6] = {0}; 	
	int16_t raw_data[3];	
//	uint8_t drdy = 0;
	int ret = 0;


	ret = qst_read_regs(0x01, databuf, 6);
	if(ret == 0)
	{
		printf("read xyz read reg error!!!\n");
		return 0;	
	}
	raw_data[0] = (int16_t)(((databuf[1]<<8))|(databuf[0]));
	raw_data[1] = (int16_t)(((databuf[3]<<8))|(databuf[2]));
	raw_data[2] = (int16_t)(((databuf[5]<<8))|(databuf[4]));
	data[0] = raw_data[0]>>2;
	data[1] = raw_data[1]>>2;
	data[2] = raw_data[2]>>2;
	//QMA6100P_LOG("1--%d	%d	%d\n",raw_data[0],raw_data[1],raw_data[2]);
	//QMA6100P_LOG("2--%d	%d	%d\n",data[0],data[1],data[2]);

	return 1;

}

int acc_read_acc_xyz(float accData[3])
{
	int ret;
	int16_t rawData[3];

	ret = acc_read_raw_xyz(rawData);
	if(ret == 0)
	{
		return 0;
	}
	accData[0] = (float)(rawData[0]*9.80665f)/(1024.0f);
	accData[1] = (float)(rawData[1]*9.80665f)/(1024.0f);
	accData[2] = (float)(rawData[2]*9.80665f)/(1024.0f);

	return 1;
}


/*

change the mode of the chip  between STANDMODE and WAKEMODE 
0x11 bit[7]

suggest setting Mclk to 6khz  to instead of  switch_chip_mode to standby mode

*/
void set_chip_mode(uint8_t state)
{
	uint8_t reg;
	qst_read_regs(0x11,&reg,1);
	printf("Last mode state=0x%x, ",reg);
	if(state==1)
		reg= (reg&0x7f)|0x80;
	else if(state == 0)
		reg= (reg&0x7f);
	else
	{
		printf("Incorrect mode input =0x%x,readback state=0x%x\r\n",state,reg);
		return ;
	}
	printf("Set mode to 0x%x\r\n",reg);
	qst_write_regs(0x11,&reg,1);

	if(state == 1)
	{
		ANA_setting_1();
	}
}
/*

change the mainclock frequency of chip 

100k,51k,25k,12k,6k

0x11 bit[3:0]

*/
void set_Mclk(uint8_t mclk)
{
	uint8_t reg;
	qst_read_regs(0x11,&reg,1);
	//printf("Last mainclock is 0x%x,",reg);
	if(mclk<8)
	{
		reg= (reg&0xf0)| mclk;
		qst_write_regs(0x11,&reg,1);
	//	printf("new mainclock is 0x%x\r\n",reg);
	}
	else
	{
		printf("mainclock input =0x%x is out of range!\r\n",mclk);
		return ;
	}
	

}
/*

change the range and filter 
bit[3:0] range setting   2,4,8,16,32G
bit[6]   0 use LPF  ,  1 use HPF 
LPF  : moveavg 1,2,4,8
HPF  : odr/10 /25 /50 /100 /200 /400 /800
*/
void set_range(Range_V range,uint8_t movingavgiir,LPF_CF lpfcf,HPF_CF hpfcf)
{
	uint8_t reg,reg_lpfrange;
	if((range<RANGE_2G) || range>RANGE_32G)
	{
		range =RANGE_2G;
		printf("Range is invalid,use default 2G \r\n");
	}

	reg = range;
	qst_read_regs(0x0F,&reg_lpfrange,1);

	reg = reg|(reg_lpfrange&0x30)|movingavgiir;
	qst_write_regs(0x0F,&reg,1);
	printf("Range is %dG,0x0f=0x%x\r\n",range*2,reg);

	if(movingavgiir==HPF)
	{
		qst_read_regs(0x10,&reg,1);
		reg = (reg&0x1F)|hpfcf;
		qst_write_regs(0x10,&reg,1);
		printf("FILTER 0x10=0x%x\r\n",reg);
	}
	else if(movingavgiir==LPF)
	{
		qst_read_regs(0x10,&reg,1);
		reg = (reg&0x1F)|lpfcf;
		qst_write_regs(0x10,&reg,1);
		printf("FILTER 0x10=0x%x\r\n",reg);
	}
}


/*
0x3f  get fifo data from it
0x3e  fifo-mode ,fifo-enz-eny-enx
0x31 set watermark
0x1a 0x1c map to int1 int2
*/
void set_map_fifo(uint8_t wm,uint8_t mode,uint8_t fifo_int_type,Port_V port,AXIS_SEL axis_sel)
{
	uint8_t reg;

	qst_read_regs(0x3E,&reg,1);
	reg=(mode)|(axis_sel)|(reg&0x38);
	qst_write_regs(0x3E,&reg,1);
	printf("FIFO CFG 0x3E=0x%x,",reg);

	if(mode==0)
		wm=0;
	reg =wm;
	qst_write_regs(0x31,&reg,1);
	printf("FIFO WM 0x31=0x%x\r\n",reg);

	//if(wm!=0)
	//{
		if(FIFO_INT_FULL==fifo_int_type)
		reg=0x20;//fifo wmk int
		else if(FIFO_INT_WATERMARK==fifo_int_type)
		reg=0x40;
		qst_write_regs(0x17,&reg,1);
	//}
	if(port==1)
	{	
		qst_read_regs(0x1a,&reg,1); 
		if(fifo_int_type == FIFO_INT_WATERMARK)
		reg=(0x40)|(reg&0x9f);
		else if(fifo_int_type == FIFO_INT_FULL)
		reg=(0x20)|(reg&0x9f);
		qst_write_regs(0x1a,&reg,1); 
		
		
	}
	else if(port==2)
	{
		qst_read_regs(0x1c,&reg,1); 
		if(fifo_int_type == FIFO_INT_WATERMARK)
		reg=(0x40)|(reg&0x9f);
		else if(fifo_int_type == FIFO_INT_FULL)
		reg=(0x20)|(reg&0x9f);
		qst_write_regs(0x1c,&reg,1); 
	}
	else
	{
		printf("Wrong FIFO port input %d!\r\n",port);
	}
	
}
/*
 2G 244uG/LSB  
 SEL=0 , for anythr 1LSB = 0.244mg*16=3.9mg/LSB
 SEL=1 , for anythr 1LSB = 0.244mg*32=7.8mg/LSB

 4G 488ug/LSB
 SEL=0 , 1LSB =7.8mg/LSB
 SEL=1 , 1LSB =15.6mg/LSB
 
 thr:  N mg   1G=1000mg  
 
 duration : 0x2C bit[0~1]

 0x18 control AM detected or not. If you want to stop detect AM ,set 0x18[2:0] b'000
*/
void set_anymotion(uint32_t thr,uint8_t duration,AM_TYPE slope,uint8_t port)
{
	uint8_t reg,reg_lpfrange;
	
	qst_read_regs(0x0F,&reg_lpfrange,1);
	reg_lpfrange=reg_lpfrange&0x0F;
	if(reg_lpfrange==RANGE_2G)
	{
		thr = thr*10/39;
	}
	else if(reg_lpfrange==RANGE_4G)
	{
		thr = thr*10/78;
	}
	else if(reg_lpfrange==RANGE_8G)
	{
		thr = thr*10/156;
	}
	else if(reg_lpfrange==RANGE_16G)
	{
		thr = thr*10/312;
	}
	else if(reg_lpfrange==RANGE_32G)
	{
		thr = thr*10/625;
	}
	thr = thr +1 ; // close to the next value
	

	qst_read_regs(0x2c,&reg,1);
	reg=(duration&0x03)|(reg&0xfc); 
	qst_write_regs(0x2c,&reg,1);
	
	qst_read_regs(0x18,&reg,1);
	reg = 0x07|(reg&0xe0); //	enable x ,y z             /* 01(x) 02 (y) 04(z)  07(x y z)*/
	qst_write_regs(0x18,&reg,1); 
	printf("Anymotion thr=0x%x,quiet=%d,reg_lpfrange=0x%x\r\n",thr,duration,reg_lpfrange);

	qst_read_regs(0x2f,&reg,1);
	reg = (slope)|(reg&0xBF); //	slope or highG
	qst_write_regs(0x2f,&reg,1); 
	printf("Anymotion thr=0x%x,quiet=%d\r\n",thr,duration);

	qst_read_regs(0x2F,&reg,1);	
	if( (reg&0x40)==AM_GRAVITY)
		thr=thr/2;
	else
		thr=thr/1;
	if(thr>255)
		thr=255;
	reg = (thr&0xff); // thr	 1/G * 16 * 1e
	qst_write_regs(0x2E,&reg,1);	

	if(port==1)
	{	
		qst_read_regs(0x1a,&reg,1); 
		reg=(0x01)|(reg&0xfe);
		qst_write_regs(0x1a,&reg,1); 
		printf(" Anymotion write to 1A =%x",reg); 
		qst_read_regs(0x1a,&reg,1);
		printf(" Anymotion read  1A =%x",reg); 
		
	}
	else if(port==2)
	{
		qst_read_regs(0x1C,&reg,1); 
		reg=(0x01)|(reg&0xfe);
		qst_write_regs(0x1C,&reg,1); 
		printf(" Anymotion write to 1C =%x",reg);
		qst_read_regs(0x1C,&reg,1);
		printf(" Anymotion read  1C =%x",reg); 
	}
	else
	{
		printf("Wrong Anymotion port input %d!\r\n",port);
	}
	

}
/*
time: Reg 0x2c bit[2-7]  
time range : 1second~16seconds  , 20seconds~95seconds,100seconds~250seconds   
LSB : 1 / [2^bit/ (2*Range)]  
THR :  thr = CNT *16 * (LSB)
*/
void set_nomotion(uint8_t thr,uint8_t time,Port_V port)
{
	uint8_t reg;
	reg=thr; //thr  1/G * 16 * 15   2g -3.91mg/cnt  4g 7.82mg/cnt
	qst_write_regs(0x2d,&reg,1);

	if(time>16&&time<20) time =16 ;
	if(time>95&&time<100) time =95 ;
	if(time>250) time =250;
	
	qst_read_regs(0x2c,&reg,1);
	if(time<=16)
		reg=((time-1)<<2)|(reg&0x03); //duration bit[7-3] 
	else if (time <=95)
		reg=0x40|((time/5-4)<<2 )|(reg&0x03);
	else //if(time<250)
		reg=0x80|((time/10-10)<<2 )|(reg&0x03);

	printf("no motion durtime=0x%x\r\n",reg);
	
	qst_write_regs(0x2c,&reg,1);

	qst_read_regs(0x18,&reg,1);
	reg = 0xe0|(reg&0x07);  
	qst_write_regs(0x18,&reg,1); 

	if(port==1)
	{	
		qst_read_regs(0x1a,&reg,1); 
		reg=(0x80)|(reg&0x7f);
		qst_write_regs(0x1a,&reg,1); 
			
		
	}
	else if(port==2)
	{
		qst_read_regs(0x1C,&reg,1); 
		reg=(0x80)|(reg&0x7f);
		qst_write_regs(0x1C,&reg,1); 
		

	}
	else
	{
		printf("Wrong DreadyInt port input %d!\r\n",port);
	}

}
void set_dataReadyInt(uint8_t enable,Port_V port)
{
	uint8_t reg;
	printf("set_dataReadyInt Enable=%d \r\n",enable);
	qst_read_regs(0x17,&reg,1);
	if( ((reg&0x10)>>4) ==enable)
		return ;
	else if(enable==1)
	{
		reg=(reg&0xef)|0x10;
	}
	else if(enable==0)
	{
		reg=(reg&0xef)|0x00;
	}
	else
	{
		 printf("incorrect parameters,it shoule be 0 or 1 \r\n");
		 return;
	}
	qst_write_regs(0x17,&reg,1);

	if(port==1)
	{	
		qst_read_regs(0x1a,&reg,1); 
		reg=(0x10)|(reg&0xef);
		qst_write_regs(0x1a,&reg,1); 
			
		
	}
	else if(port==2)
	{
		qst_read_regs(0x1C,&reg,1); 
		reg=(0x10)|(reg&0xef);
		qst_write_regs(0x1C,&reg,1); 
		

	}
	else
	{
		printf("Wrong DreadyInt port input %d!\r\n",port);
	}
	
}
/*
type  :  N  =  N tap  N <=3

*/


/*
0x1e bit[0-5]  quiet thr,  31.25mg/cnt
0x2A bit7  tap quiet time  0:20ms,1:30ms  
	 bit6  tap shock time  0:75ms,1:50ms   
	 bit5  tap delay  	0: 3tap not wait for 4tap  1: 3tap will wait for 4tap 
	 bit4  tap-ear-in  	0: tap enable by 0x16  1: tap enable by 0x08[bit1]
	 bit0-3 tap duration	0:100ms-150ms-200ms-250ms-300ms-400ms-500ms-700ms
0x2B bit7-6 tap-axis-sel	x,y,z,sqrt(x2+y2+z2)
	 bit5-0 tap-shock-thr	31.25mg*CNT	
	 
ODR : 200Hz is better .
*/
void set_map_tapInt(qma6100P_tap type,uint8_t quietthr,uint8_t shockthr,Port_V port)
{
	uint8_t reg;

	qst_read_regs(0x1e,&reg,1); 
	reg=quietthr|(reg&0xc0);;//quietthr|(reg&0xc0);
	qst_write_regs(0x1e,&reg,1); 

	/*  bit7 bit6 bit5 bit4*/
	reg=0x80|0x00|0x00|0x00|0x06;
	qst_write_regs(0x2a,&reg,1);

	reg=0xC0|shockthr; //  sqrt(xyz^2)|tap  shock  threshold 
	qst_write_regs(0x2b,&reg,1);

	/*enable tap 1-2-3-4*/
	qst_read_regs(0x16,&reg,1); 
	reg=type|(reg&0x4E);
	qst_write_regs(0x16,&reg,1); 

	if(port==1)
	{	
		qst_read_regs(0x1a,&reg,1); 
		reg=((type&0x01)<<1)|(reg&0xfd);
		qst_write_regs(0x1a,&reg,1); 
		qst_read_regs(0x19,&reg,1); 
		reg=(type&0xB0)|(reg&0x4E);
		qst_write_regs(0x19,&reg,1);		
		
	}
	else if(port==2)
	{
		qst_read_regs(0x1C,&reg,1); 
		reg=((type&0x01)<<1)|(reg&0xfd);
		qst_write_regs(0x1C,&reg,1); 
		qst_read_regs(0x1B,&reg,1); 
		reg=(type&0xB0)|(reg&0x4E);
		qst_write_regs(0x1B,&reg,1);	

	}
	else
	{
		printf("Wrong tapInt port input %d!\r\n",port);
	}
	
	
	
}
uint16_t odr[8] ={13,25,50,100,200,400,800,1600};
uint8_t rateregs[8]={0x07,0x06,0x05,0x00,0x01,0x02,0x03,0x04};
/*set odr after set mCLK*/
/*mclk lower , ODR lower
e.g.  
	mclk   regvalue  ODR
	50Khz  	3		800Hz
	25Khz   3		about 400Hz
	12Khz  	3		about 200Hz
*/
/*If you wanna change ODR ,1st set_chip_mode Standby ,2nd change odr, 3rd set_chip_mode Wakemode*/
void set_odr(uint16_t rate,uint8_t mCLK)
{
	uint8_t reg,i;
	qst_read_regs(0x10,&reg,1); 
	reg=(reg&0xE0);
	rate = rate*(1<<(mCLK-MCLK_51KHZ));  //     4 5 6 7  (-4)   +1 |||   0 1 2 3 ||| 1 2 3 4
	for(i=0;i<8;i++)
	{
		if(rate<=odr[i])
		{
			break;
		}
	}
	if(i>7)
		i=7;
	reg = (reg&0xE0) |rateregs[i];
	qst_write_regs(0x10,&reg,1); 
	printf("Set desired ODR = %d ,real ODR = %d,0x10 =0x%x\r\n",rate,odr[i],reg);
}

void map_int_port(uint8_t port)
{
	uint8_t reg;
	if(port!=1&&port!=2)
	{
		printf("Invalid port %d ,Map to Port 1 !!\r\n",port);
		port =1 ;
	}
	if(port==1)
	{
		reg = 0xff; 
		qst_write_regs(0x19,&reg,1); 
		reg = 0xf3; 
		qst_write_regs(0x1A,&reg,1); 
		printf("MAP ALL INT to Port %d\r\n",port);
		
	}
	else if(port==2)
	{
		reg = 0xff; 
		qst_write_regs(0x1B,&reg,1); 
		reg = 0xf3; 
		qst_write_regs(0x1C,&reg,1); 
		printf("MAP ALL INT to Port %d\r\n",port);
	}
	
}

/*

Use  ODR @ 50Hz
*/
void set_map_stepInt(uint8_t enable,Port_V port)
{

	uint8_t reg;

	if(enable>0)
		enable=1;
	qst_read_regs(0x16,&reg,1); 
	reg=(enable<<3)|(reg&0xF7);
	qst_write_regs(0x16,&reg,1); 

	qst_read_regs(0x12,&reg,1); //0x89
	reg=(enable<<7)|(reg&0x7F);
	qst_write_regs(0x12,&reg,1); 

	//qst_read_regs(0x13,&reg,1); 
	reg=0x80;
	qst_write_regs(0x13,&reg,1); 
	qst_delay_ms(1);
	reg=0x7F;
	qst_write_regs(0x13,&reg,1); 
	reg=0x0B;
	qst_write_regs(0x14,&reg,1); 
	reg=0x0E;
	qst_write_regs(0x15,&reg,1); 
	reg=0x00;  // significant step used .no need to cfg
	qst_write_regs(0x1D,&reg,1); 
	reg=0x08;  //  data filter  :  1
	qst_write_regs(0x1E,&reg,1); 
	reg=0x60|0x00; // 12 start step 
	qst_write_regs(0x1F,&reg,1); 
	reg=0x3F;		// step rawdata use filted data
	qst_write_regs(0x30,&reg,1); 
	reg=0x00;      // use  3-axis
	qst_write_regs(0x32,&reg,1);
	reg=0x00;      // use  3-axis
	qst_write_regs(0x5F,&reg,1);
	if(port==1)
	{
		qst_read_regs(0x19,&reg,1); 
		reg=0x08|(reg&0xF7);;//quietthr|(reg&0xc0);
		qst_write_regs(0x19,&reg,1); 
	}
	else if(port==2)
	{
		qst_read_regs(0x1B,&reg,1); 
		reg=0x08|(reg&0xF7);;//quietthr|(reg&0xc0);
		qst_write_regs(0x1B,&reg,1); 
	}
	else
	{
		printf("Wrong StepInt port input %d!\r\n",port);
	}


}
/*
 set qma6100P step count to zero ,step counter will recalculate the step from zero .
*/
void clear_step_cnt(void)
{
	uint8_t reg;
	reg=0x80;
	qst_write_regs(0x13,&reg,1); 
	qst_delay_ms(1);
	reg=0x7F;
	qst_write_regs(0x13,&reg,1); 
	printf("RESET Current Stepcnt \r\n");

}
void read_step_cnt(void)
{
	uint32_t stepcnt;
	uint8_t reg7,reg8;
	qst_read_regs(0x7,&reg7,1);
	qst_read_regs(0x8,&reg8,1);
	//qst_read_regs(0xd,&regd,1);
	stepcnt = reg7|(reg8<<8);//|(regd<<16);
	printf("Current Stepcnt = %d \r\n",stepcnt);

	if(stepcnt>=65535)
	{	
		/*step counter will run to 0 */
		//clear_step_cnt();
	}
}
/*

ODR: 100~200Hz is better.
*/

void set_map_raiseINT(uint8_t upenable,uint8_t downenable,Port_V port)
{
	uint8_t reg;
	
	if(upenable>0)
		upenable=1;
	if(downenable>0)
		downenable=1;
	qst_read_regs(0x16,&reg,1); 
	reg=(upenable<<1)|(downenable<<2)|(reg&0xF7);
	qst_write_regs(0x16,&reg,1); 

	
	if(port==1)
	{
		qst_read_regs(0x19,&reg,1); 
		reg=(upenable<<1)|(downenable<<2)|(reg&0xF9);
		qst_write_regs(0x19,&reg,1); 
	}
	else if(port==2)
	{
		qst_read_regs(0x1B,&reg,1); 
		reg=(upenable<<1)|(downenable<<2)|(reg&0xF9);
		qst_write_regs(0x1B,&reg,1); 
	}
	else
	{
		printf("Wrong StepInt port input %d!\r\n",port);
	}
	

}
/*   
	0x21 : INT latched bit0,spi 0x21 ,iic 0x01
	0x20:  10pin SENB dis or enable pullup resitor,SPI3-4,INT1-2 OD-PP Default Level 


*/
#define ACTIVE_LOW		0x00
#define ACTIVE_HIGH		0x05		
#define PUSH_PULL		0x00	
#define OPEN_DRAIN		0x0A
#define DIS_CS_PULLUP   0x80
#define EN_CS_PULLUP    0x00
void configure_INTPIN(void)
{
	uint8_t reg;
#ifdef _CFG_BUS_I2C
	reg = 0x01;
#else
	reg = 0x21;
#endif	
	qst_write_regs(0x21,&reg,1); 

	reg=ACTIVE_HIGH|PUSH_PULL;	  //  DIS_CS_PULLUP|
	qst_write_regs(0x20,&reg,1);

}

/*Registers must be set during the first initilization*/

void ANA_setting(void)
{
	uint8_t reg ;
	reg=0x51; 
	qst_write_regs(0x50,&reg,1);
	reg=0x20;						
	qst_write_regs(0x4A,&reg,1); 
	reg=0x01;
	qst_write_regs(0x56,&reg,1); 
}

/* It must be called after chip was in wakemode */
void ANA_setting_1(void)
{
	uint8_t reg ;
	reg=0x80;					
	qst_write_regs(0x5f,&reg,1);
	qst_delay_ms(1);
	reg=0x00;
	qst_write_regs(0x5f,&reg,1);
	qst_delay_ms(1);	
}

int qma6100P_init(void)
{
	unsigned char slave_addr[2] = {0x12, 0x13};
//	unsigned char buf[2]={0};
	uint8_t i,reg,reg_20,cnt;
	
	// POR poweronreset in 1ms . 
	
	reg = 0x00;
	for(i=0; i<2; i++)
	{
		qma6100p_slave = slave_addr[i];
		qst_read_regs(0x00,&reg,1);
		if((reg&0xF0)==0x90)
		{	 
			break;
		}
	}

	if((reg&0xF0)!=0x90)
	{    
		printf("QMA6100P not found\r\n");
		return 0;
	}
#ifdef DIS_AD0
	reg = 0x00;
	reg_20 = 0x45;
	qst_write_regs(0x20, &reg_20, 1);
	for(i=0; i<2; i++)
	{
		qma6100p_slave = slave_addr[i];
		qst_read_regs(0x00,&reg,1);
		if((reg&0xF0)==0x90)
		{	 
			break;
		}
	}
	if((reg&0xF0)!=0x90)
	{    
		printf("QMA6100P not found\r\n");
		return 0;
	}
#endif

	printf("QMA6100P begin to init\r\n");

	softwarereset();

	reg = 0x00;
	while( 1 )
	{
		qst_read_regs(0x45,&reg,1);

		if( (reg&0xF0)!=0xC0 )
		{	
			softwarereset();
			cnt++;
			if(cnt>=100)
			{
				printf("QMA6100P SWRst fail\r\n");
			  	break;
			}
		}
		else
		{
			printf("QMA6100P SWRst end\r\n");
			break;
		}
		
	}
	
	
	set_chip_mode(STANDBYMODE);
	ANA_setting();
	
	set_Mclk(MCLK_51KHZ); /*mclk lower , current lower*/
	set_range(RANGE_8G,LPF,LPCF_NONE,HPCF_ODRDIV10);
	set_odr(25,MCLK_51KHZ);	/*choose proper ODR to adapt  different motion interrupt*/
	configure_INTPIN();
	//set_dataReadyInt(1,PORT_1);
	//set_map_fifo(10,FIFO_MODE_FIFO,FIFO_INT_WATERMARK,PORT_1,(AXIS_X|AXIS_Y|AXIS_Z));	
	
	//set_anymotion(500,0,AM_SLOPE,PORT_1);
	//set_nomotion(0x1e,5,PORT_1);
	//set_map_raiseINT(1,1,PORT_2); /* ODR should be about 100-200Hz */
	//set_map_stepInt(1,PORT_2);	/* ODR should be about 50-100Hz */
	//set_map_tapInt(QMA6100_TAP_SINGLE|QMA6100_TAP_DOUBLE,3,5,PORT_2);  /* ODR should be about 200Hz */

	set_chip_mode(WAKEMODE);
	
	/* when ANA_setting_1 is called , chip must be in wakemode ,it's called in set_chip_mode()*/
	//ANA_setting_1();	
	
	printf("finish init \r\n");
	return 1;

}

/*example for INT_HANDLE */

void qst_interrupt_handler(void)
{
	// store fifo data 
	short rdata[64][3];
	unsigned char rawdata[384*2];	//2*32*12
	unsigned char int_State[8];
	uint16_t i,j =0;
	uint8_t reg;
    uint8_t cntforint;
 	uint16_t readbytes;
	uint8_t nbytes=0;

	// check x y z enabled-axis //
	qst_read_regs(0x3E, &reg,1);
	reg = reg&0x07;
	if((reg&0x01)==0x01)
		nbytes+=1;
	if((reg&0x02)==0x02)
		nbytes+=1;
	if((reg&0x04)==0x04)
		nbytes+=1;
	// fifo-read-bytes  0~6 bytes
	
	qst_read_regs(0x09, int_State,6);
    printf("Port 2:Inter2 0x09-B State  0x%x ,0x%x, 0x%x ,framecnt=%d\r\n",int_State[0],int_State[1],int_State[2],int_State[5]);
	/*STEP INT*/
	if((int_State[1]&0x08)==0x08)
	{
		read_step_cnt();
	}
	/*DATA READY INT*/
	else if((int_State[2]&0x10)==0x10)
	{
		qst_read_regs(0x1,rawdata,6);
		rdata[0][0] = (short)(((unsigned short)rawdata[1] << 8) + (unsigned short)rawdata[0])>>2;
		rdata[0][1] = (short)(((unsigned short)rawdata[3] << 8) + (unsigned short)rawdata[2])>>2;
		rdata[0][2] = (short)(((unsigned short)rawdata[5] << 8) + (unsigned short)rawdata[4])>>2;	
		printf("Port2: %5d, %5d, %5d,\r\n",rdata[0][0],rdata[0][1],rdata[0][2]);
	}
	/*FIFO INT*/
	else if((int_State[2]&0x60)!=0x0)
	{
		cntforint = int_State[5];
	
		readbytes=nbytes*2*cntforint;

		/*when using lowest Mclk , strongly recommend read 6 bytes one cycle !!*/
		qst_read_regs(0x3f,rawdata,readbytes);
				
		for(i=0;i<(cntforint);i++)
		{		
			for(j=0;j<nbytes;j++)
			rdata[i][j] = (short)(((unsigned short)rawdata[(1+j*2)+nbytes*2*i] << 8) + (unsigned short)rawdata[(0+j*2)+nbytes*2*i])>>2;		
		}
			
		qst_read_regs(0x0E,&reg,1);
		printf("Port2: data left in fifo: %d\r\n",reg);
		
		/*clear framcnt when using SPI or need to clear fifoframe without read all data in FIFO */	
		/*Rewrite 0x31,0x37 ,fifo framecnt will be cleared*/
		qst_read_regs(0x31,&reg,1);	
		qst_write_regs(0x31,&reg,1);
			
		
	 }
	else
	{
		printf("Port2: clr state\r\n");
	}

	/*
	when map tow kinds of INT on same INT_PIN, e.g. (Data_INT,FIFO_INT) and other Motion_INT . 
	Read 0x09~0x0B after reading data to avoid missing handling Motion_INT .
	*/
	qst_read_regs(0x09, int_State,3);
	
}


int gsensor_init(void)
{
	 return qma6100P_init();
}
/*Port means INT PIN ,QMA6100P have two INT PINs*/
void gsensor_handle_int(uint8_t port)
{
	if(port==PORT_1 || port==PORT_2 )
	qst_interrupt_handler();		
}




