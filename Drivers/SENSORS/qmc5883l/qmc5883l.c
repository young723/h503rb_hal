
#include "qmc5883l.h"

#define QMC5883L_ADDR               0x0d
//#define QMC5883L_OTP_KXKY

typedef struct
{
	uint8_t		slave;
	unsigned short	count;
	int16_t		raw_back[3];
	int16_t		raw[3];
	uint16_t	svvt;
	float		kx;
	float		ky;
} qmc5883l_data_t;

typedef struct
{
	unsigned short		chipno;
	signed char			kx_code;
	signed char			ky_code;
} otp_kxky_t;

const otp_kxky_t kxky_array[] = 
{
	{133 , 		2,-3},
	{146 , 		3,-3},
	{15	,		1,-2},
	{63	, 		2,-2},
	{33	, 		3,0},
	{101 ,		3,7},
	{131 ,		1,8},
	{73	,		4,7},
	{84	,		4,5},
	{110 ,		4,6},
};

static qmc5883l_data_t gMag;

int qmc5883l_ReadReg(uint8_t reg, uint8_t *buf, uint16_t len)
{
	int ret = 0;

	ret = bsp_i2c_read_reg(gMag.slave, reg, buf, len);

	return ret;
}

int qmc5883l_WriteReg(uint8_t val, uint8_t reg)
{
	int ret = 0;

	ret = bsp_i2c_write_reg(gMag.slave, reg, val);

	return ret;
}

#if defined(QMC5883L_OTP_KXKY)
void qmc5883l_calc_kxky(void)
{
	unsigned short	chipno = 0;
	unsigned char	buf[2];

	gMag.kx = 0.0f;
	gMag.ky = 0.0f;

	qmc5883l_WriteReg(0x3d-0x2f, 0x2e);		// read 3d
	qmc5883l_ReadReg(0x2f, &buf[0], 1);
	qmc5883l_WriteReg(0x3f-0x2f, 0x2e);		// read 3f
	qmc5883l_ReadReg(0x2f, &buf[1], 1);

	chipno = (unsigned short)((buf[1]<<8)|buf[0]);
	qst_logi("chip no = %d [0x%02x 0x%02x]\r\n", chipno, buf[0], buf[1]);
	for(int i=0; i<(sizeof(kxky_array)/sizeof(kxky_array[0])); i++)
	{
		if(chipno == kxky_array[i].chipno)
		{
			gMag.kx = kxky_array[i].kx_code * 0.005f;
			gMag.ky = kxky_array[i].ky_code * 0.005f;
			break;
		}
	}
	
	qst_logi("qmc5883l_calc_kxky kxky[%f %f]\r\n", gMag.kx, gMag.ky);
}
#endif

void qmc5883l_enable(unsigned char enable)
{
	if(enable)
	{
		qmc5883l_WriteReg(0x1D, QMC5883L_REG_CONF1);/****OSR=512,RNG=+/-8G,ODR=200Hz,MODE= continuous*******/
	}
	else
	{
		qmc5883l_WriteReg(0x00, QMC5883L_REG_CONF1);
	}
}


int qmc5883l_InitConfig(void)
{
	uint8_t value = 0;
	int retry = 0;

	gMag.slave = QMC5883L_ADDR;
	gMag.svvt = 3000/100;	// uT, +-8Gauss		;12000/100;	// uT, +-2Gauss

	while(retry++ < 5)
	{
		qmc5883l_WriteReg(0x1D, QMC5883L_REG_CONF1);
		qst_delay_ms(1);
		qmc5883l_ReadReg(QMC5883L_REG_CONF1, &value, 1);
		qst_logi("qmc5883l_InitConfig 0x09=0x%02x\r\n", value);
		if(value == 0x1D)
		{
			qmc5883l_WriteReg(0x80, QMC5883L_REG_CONF2);
			qst_delay_ms(5);

			qmc5883l_WriteReg(0xF0, 0x0B);//qmc5883l_WriteReg(0xE0, 0x0B);
			qmc5883l_WriteReg(0x40, 0x20);
			qmc5883l_WriteReg(0x01, 0x21);
#if defined(QMC5883L_OTP_KXKY)
			qmc5883l_calc_kxky();
#endif
			qmc5883l_enable(1);

			//for(unsigned char i=0x30; i<=0x3b; i++)
			//{
			//	qmc5883l_ReadReg(i, &value, 1);
			//	qst_logi("0x%02x=0x%02x\r\n", i, value);
			//}

			return 1;
		}
	}	


	return 0;
}

int qmc5883l_GetData(float *Magnet)
{
	uint8_t Buff[6];
	uint8_t status;

	int16_t MagnetRawAd[3];

	qmc5883l_ReadReg(QMC5883L_REG_STATUS, &status, 1);
	if(status & 0x01)
	{		
		qmc5883l_ReadReg(QMC5883L_REG_DATA_OUTPUT_X, &Buff[0], 6);		
		MagnetRawAd[0] = (int16_t)((Buff[1] << 8) | Buff[0]);
		MagnetRawAd[1] = (int16_t)((Buff[3] << 8) | Buff[2]);
		MagnetRawAd[2] = (int16_t)((Buff[5] << 8) | Buff[4]);
		gMag.raw[0] = MagnetRawAd[0];
		gMag.raw[1] = MagnetRawAd[1];
		gMag.raw[2] = MagnetRawAd[2];
	}
	else
	{
		MagnetRawAd[0] = gMag.raw[0];
		MagnetRawAd[1] = gMag.raw[1];
		MagnetRawAd[2] = gMag.raw[2];
		qst_logi("qmc5883l_GetData drdy fail\r\n");
		if(gMag.count++ > 5)
		{
			qmc5883l_WriteReg(0x80, QMC5883L_REG_CONF2);
			qst_delay_ms(5);

			qmc5883l_WriteReg(0xF0, 0x0B);//qmc5883l_WriteReg(0xE0, 0x0B);
			qmc5883l_WriteReg(0x40, 0x20);
			qmc5883l_WriteReg(0x01, 0x21);
			qmc5883l_enable(1);
		}
	}

	Magnet[0] = (float)MagnetRawAd[0] / gMag.svvt;
	Magnet[1] = (float)MagnetRawAd[1] / gMag.svvt;
	Magnet[2] = (float)MagnetRawAd[2] / gMag.svvt;
#if defined(QMC5883L_OTP_KXKY)
	Magnet[2] = Magnet[2] - Magnet[0]*gMag.kx - Magnet[1]*gMag.ky;
#endif

	return 1;
}

