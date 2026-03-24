
#include <stdio.h>
#include <string.h>
#include "butterworth.h"

#if defined(BTW_USE_FAST_MATH)
#include "FastMath.h"
#define BTW_SQRT	FastSqrt
#define BTW_SIN		FastSin
#define BTW_COS		FastCos
#define BTW_TAN		FastTan
#define BTW_ASIN	FastAsin
#define BTW_ATAN2	FastAtan2
#else
#include <math.h>
#define BTW_SQRT	sqrtf
#define BTW_SIN		sinf
#define BTW_COS		cosf
#define BTW_TAN		tanf
#define BTW_ASIN	asinf
#define BTW_ATAN2	atan2f
#endif
#define Pi (3.1415926535897932384626433832795f)

// filter
//static butterworth_f btw_1_f = {0.369527459, 0.195815772, 0.391335815, 0.78267163, 0.391335815};
//static butterworth_f btw_2_f = {-0.747789085, 0.272214949, 0.131106466, 0.262212932, 0.131106466};
//static butterworth_buf btw_1_buf[3];
//static butterworth_buf btw_2_buf[3];

/*!
 * \brief set second order butterworth filtering Parameters.
* \input float sample_freq:accel and gyro filter sampling frequency.
* \input float cutoff_freq:accel and gyro filter cut-off frequency.
* \output Butterworth_Filter *filter:filter coefficient.
* \return void 
 */
void btw_set_cutoff_frequency(float sample_freq, float cutoff_freq, butterworth_f *filter)
{	
	float fr = sample_freq / cutoff_freq;
	float ohm = BTW_TAN(Pi / fr);
	float c = 1.0f + 2.0f * BTW_COS(Pi / 4.0f) * ohm + ohm * ohm;
	filter->b0 = ohm * ohm / c;
	filter->b1 = 2.0f * filter->b0;
	filter->b2 = filter->b0;
	filter->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
	filter->a2 = (1.0f - 2.0f * BTW_COS(Pi / 4.0f) * ohm + ohm * ohm) / c;
}

float btw_filter_apply(float sample, butterworth_buf *buffer, butterworth_f *filter) 
{
	//do the filtering
	float delay_element_0 = sample - buffer->_delay_element_1 * filter->a1 - buffer->_delay_element_2 * filter->a2;

	const float output = delay_element_0 * filter->b0 + buffer->_delay_element_1 * filter->b1 + buffer->_delay_element_2 * filter->b2;

	buffer->_delay_element_2 = buffer->_delay_element_1;
	buffer->_delay_element_1 = delay_element_0;

	//return the value. Should be no need to check limits
	return output;
}


