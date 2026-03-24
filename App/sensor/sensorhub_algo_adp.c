
#include "sensorhub.h"
#include "fusion_interface.h"

static evb_sensor_algo_t mAlg;
//static LpfFilter_t f_lpf;
//static Avg3AxisFilter_t f3_avg;
#if defined(STD_OUTPUT)
static QstStd3 mStd;
#endif
#ifdef QST_SOFT_MAG
static float soft_mag[3][3]={
    {1.0,  0.0,  0.0},
    {0.0,  1.0,  0.0},
    {0.0,  0.0,  1.0},
};
#endif

//float qst_evb_calc_altitude(float press, float tempearture)
//{
//	float altitude;
//	altitude = (pow((101325.0f/press),1/5.257f)-1)*(tempearture+273.15f)/0.0065f;
//
//	return altitude;
//}

#if defined(QST_FUSION)
static void qst_axis_convert(float data[3], int layout)
{
	float raw[3];

	raw[0] = data[0];
	raw[1] = data[1];
	//raw[2] = data[2];
	if(layout >=4 && layout <= 7)
	{
		data[2] = -data[2];
	}
	//else
	//{
	//	data[2] = raw[2];
	//}

	if(layout%2)
	{
		data[0] = raw[1];
		data[1] = raw[0];
	}
	else
	{
		data[0] = raw[0];
		data[1] = raw[1];
	}

	if((layout==1)||(layout==2)||(layout==4)||(layout==7))
	{
		data[0] = -data[0];
	}
	if((layout==2)||(layout==3)||(layout==6)||(layout==7))
	{
		data[1] = -data[1];
	}
}
#endif

void qst_algo_set_data(qst_sensor_type sensor, evb_sensor_data_t *data)
{
	if(sensor == QST_SENSOR_ACCEL)
	{
		if(mAlg.a_cali_flag)
		{
			if(qst_algo_acc_cali(&data->val[0]) != 0)
			{
				mAlg.a_cali_flag = false;
			}
		}
		data->val[0] += mAlg.acc_oft[0];
		data->val[1] += mAlg.acc_oft[1];
		data->val[2] += mAlg.acc_oft[2];
#if defined(QST_FUSION)
		libData 		dIn;

		dIn.x = data->val[0];
		dIn.y = data->val[1];
		dIn.z = data->val[2];
		dIn.status = 3;
		dIn.timeStamp = data->timestamp;
		mAlg.fusion->SetAccData(&dIn);
#endif
	}
	else if(sensor == QST_SENSOR_MAG)
	{
#if defined(QST_FUSION)
		libData 		dIn;
		libData 		dOut;

#ifdef QST_SOFT_MAG
		dIn.x = (float)(data->val[0]*soft_mag[0][0] + data->val[1]*soft_mag[0][1] + data->val[2]*soft_mag[0][2]);
		dIn.y = (float)(data->val[0]*soft_mag[1][0] + data->val[1]*soft_mag[1][1] + data->val[2]*soft_mag[1][2]);
		dIn.z = (float)(data->val[0]*soft_mag[2][0] + data->val[1]*soft_mag[2][1] + data->val[2]*soft_mag[2][2]);
#else
		dIn.x = data->val[0];
		dIn.y = data->val[1];
		dIn.z = data->val[2];
#endif
		dIn.timeStamp = data->timestamp;
		mAlg.fusion->doCali(&dIn, &dOut);
		data->status = dOut.status;
		data->val[0] = dOut.x;
		data->val[1] = dOut.y;
		data->val[2] = dOut.z;		//mAlg.mag_rr = dOut.w;
		//mAlg.fusion->GetMagOffset(mAlg.mag_offset, &mAlg.mag_rr);
		mAlg.fusion->getOrientation(&dOut);
		qst_logi("STATUS[%d]	%f,%f,%f\r\n", data->status, dOut.x, dOut.y, dOut.z);
#endif
//		dIn[0] = data->val[0];
//		dIn[1] = data->val[1];
//		dIn[2] = data->val[2];
//		getStandardDeviation3(&mStd, dIn, stdOut);
//		qst_logi("%f,%f,%f,STD	",stdOut[0],stdOut[1],stdOut[2]);
	}
	else if(sensor == QST_SENSOR_IMU)
	{
		if(mAlg.g_cali_flag)
		{
			if(qst_algo_gyr_cali(&data->val[0]) != 0)
			{
				mAlg.g_cali_flag = false;
			}
		}
		data->val[0] += mAlg.gyr_oft[0];
		data->val[1] += mAlg.gyr_oft[1];
		data->val[2] += mAlg.gyr_oft[2];
#if defined(QST_FUSION)
		libData 		dIn;

		dIn.x = data->val[0];
		dIn.y = data->val[1];
		dIn.z = data->val[2];
		dIn.status = 3;
		dIn.timeStamp = data->timestamp;
		mAlg.fusion->SetGyroData(&dIn);
#endif
	}
	else if(sensor == QST_SENSOR_PRESS)
	{
	}
}

void qst_algo_user_imu_cali(void)
{
	mAlg.a_cali_flag = true;
	mAlg.g_cali_flag = true;
}

void qst_algo_init(void)
{
	evb_setup_user_key(1, qst_algo_user_imu_cali);

	memset(&mAlg, 0, sizeof(mAlg));
	qst_imu_cali_init(&mAlg.acc_oft[0], &mAlg.gyr_oft[0]);

#if defined(STD_OUTPUT)
	memset(&mStd, 0, sizeof(mStd));
#endif

#if defined(QST_FUSION)
	qst_fusion_get_interface(&mAlg.fusion);
	mAlg.fusion->enableLib(FUSION_NOGYRO);
#endif

//	InitLpfFilter(&f_lpf);
//	InitAvg3AxisFilter(&f3_avg, 6);
//	qst_btwz_init();
}

