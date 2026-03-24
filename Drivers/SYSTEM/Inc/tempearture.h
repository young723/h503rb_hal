
#ifndef _TEMP_SENSOR_H_
#define _TEMP_SENSOR_H_

#include "TSE64.h"

typedef enum
{
	QST_TEMP_NONE,
	QST_TEMP_TSE64,

	QST_TEMP_TOTAL
} qst_temp_model;

typedef struct
{
	unsigned int		count;
	unsigned long long	timestamp;
} qst_temp_t;

extern int qst_evb_demo_temp(int report_mode);

#endif

