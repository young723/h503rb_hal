#ifndef _QMC5883P_H_
#define _QMC5883P_H_

#include "bsp_hardware.h"

/* vendor chip id*/
#define QMC5883P_IIC_ADDR				0x2c

#define QMC5883P_CHIP_ID_REG			0x00

/*data output register*/
#define QMC5883P_DATA_OUT_X_LSB_REG		0x01
#define QMC5883P_DATA_OUT_X_MSB_REG		0x02
#define QMC5883P_DATA_OUT_Y_LSB_REG		0x03
#define QMC5883P_DATA_OUT_Y_MSB_REG		0x04
#define QMC5883P_DATA_OUT_Z_LSB_REG		0x05
#define QMC5883P_DATA_OUT_Z_MSB_REG		0x06
/*Status registers */
#define QMC5883P_STATUS_REG				0x09
/* configuration registers */
#define QMC5883P_CTL_REG_ONE			0x0A  /* Contrl register one */
#define QMC5883P_CTL_REG_TWO			0x0B  /* Contrl register two */

/* Magnetic Sensor Operating Mode MODE[1:0]*/
#define QMC5883P_SUSPEND_MODE			0x00
#define QMC5883P_NORMAL_MODE			0x01
#define QMC5883P_SINGLE_MODE			0x02
#define QMC5883P_H_PFM_MODE				0x03

#define QMC5883P_SELFTEST_MAX_X			(1800)
#define QMC5883P_SELFTEST_MIN_X			(120)
#define QMC5883P_SELFTEST_MAX_Y			(1800)
#define QMC5883P_SELFTEST_MIN_Y			(120)
#define QMC5883P_SELFTEST_MAX_Z			(1800)
#define QMC5883P_SELFTEST_MIN_Z			(120)


#define QMC5883P_ABS(X) 				((X) < 0.0f ? (-1 * (X)) : (X))
#define QMC5883P_ABSF(X) 				((X) < 0.0f ? (-1.0 * (X)) : (X))


int qmc5883p_init(void);
int qmc5883p_read_mag_xyz(float *data);
int qmc5883p_self_test(void);

#endif

