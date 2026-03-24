#ifndef BUTTERWORTH_H
#define BUTTERWORTH_H


typedef struct
{
	float a1;
	float a2;
	float b0;
	float b1;
	float b2;
} butterworth_f;

typedef struct
{
	float _delay_element_1;
	float _delay_element_2;
} butterworth_buf;

#ifdef __cplusplus
extern "C" {
#endif

void btw_set_cutoff_frequency(float sample_freq, float cutoff_freq, butterworth_f *filter);
float btw_filter_apply(float sample, butterworth_buf *buffer, butterworth_f *filter);

void qst_btwz_init(void);
void qst_btwz_apply_1(float data[3]);
void qst_btwz_apply_2(float data[3]);

#ifdef __cplusplus
}
#endif

#endif
