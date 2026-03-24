
#ifndef _SENSORHUB_H_
#define _SENSORHUB_H_

#include "qst_sensor.h"

#define EVB_SENSOR_LOG		1
//#define QST_FUSION
//#define QST_EVB_NOMOTION

typedef struct
{
	float				val[3];	
	unsigned int		num;
	unsigned long		timestamp;
	char				status;
} evb_sensor_data_t;

typedef struct
{
	float			acc_oft[3];
	float			gyr_oft[3];
//	float			dummy_gyr[3];
//	float			euler[3];
//	float			quat[4];
//	float			matrix[9];
//	float			mag_cal[3];
//	float			mag_offset[3];
//	float			mag_rr;
//	signed char		mag_accuracy;
//	float			magPara[4];
//	float			altitude;
	struct qst_fusion_interface_t	*fusion;
	bool			a_cali_flag;
	bool			g_cali_flag;
} evb_sensor_algo_t;

typedef struct
{
	TIM_TypeDef *		timId;
	bool				userd;
} evb_sensor_timer_t;

typedef struct
{
	int					irq_num;
	bool				userd;
} evb_sensor_irq_t;

typedef struct
{
	qst_sensor_id		id;
	int					report;
	int_callback		irq_hdlr;
	int_callback		motion_irq_hdlr;
	unsigned short		period;	
	unsigned short		wmk;
//	unsigned int		num;
	evb_sensor_data_t	out;
} evb_sensor_info_t;

typedef struct
{
	qst_uart_protocol	protocol;
	unsigned int		count;
} evb_sensor_cfg_t;

void qst_evb_init_sensor(qst_sensor_type type, int port, int report, float odr);
void qst_evb_start_sensor(void);
void qst_algo_init(void);
void qst_algo_set_data(qst_sensor_type sensor, evb_sensor_data_t *data);


#endif

