
#include "butterworth.h"

static butterworth_f btw_1_f = {-0.747789085, 0.272214949, 0.131106466, 0.262212932, 0.131106466};
static butterworth_f btw_2_f = {0.369527459, 0.195815772, 0.391335815, 0.78267163, 0.391335815};
static butterworth_buf btw_1_buf[3];
static butterworth_buf btw_2_buf[3];

void qst_btwz_init(void)
{
	btw_set_cutoff_frequency(50, 20, &btw_1_f);
	btw_set_cutoff_frequency(400, 30, &btw_2_f);
}

void qst_btwz_apply_1(float data[3])
{	
	data[0] = btw_filter_apply(data[0], &btw_1_buf[0], &btw_1_f);
	data[1] = btw_filter_apply(data[1], &btw_1_buf[1], &btw_1_f);
	data[2] = btw_filter_apply(data[2], &btw_1_buf[2], &btw_1_f);

}

void qst_btwz_apply_2(float data[3])
{
	data[0] = btw_filter_apply(data[0], &btw_2_buf[0], &btw_2_f);
	data[1] = btw_filter_apply(data[1], &btw_2_buf[1], &btw_2_f);
	data[2] = btw_filter_apply(data[2], &btw_2_buf[2], &btw_2_f);
}

