
#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_



#include <stdint.h>
#include "sensor_board.h"


#define ENC_MAF_SIZE 1024



typedef enum
{
	MODE_CT_CAL,
	MODE_V_UVW,
	MODE_VF,
}MotorControl_Mode_e;


typedef struct
{
	float Ts;
	float V_f_rate;
	float vf_inc_rate; // [Hz / s]
	int ct_cal_samples;
}MotorControl_Init_t;


typedef struct
{
	MotorControl_Init_t init;
	SensorBoard_t sensor;

	float Vu_ref, Vv_ref, Vw_ref;
	float amp_u, amp_v, amp_w;
	int sector;

	float vf_phase;
	float vf_freq_ref;
	float vf_freq;
	float vf_volt;

	MotorControl_Mode_e mode;
	int first_sample;

	uint16_t enc_count;
	uint16_t enc_count_prev;
	int16_t enc_diff;
	int32_t enc_diff_MAF_buf[ENC_MAF_SIZE];
	int32_t enc_diff_MAF_sum;
	int32_t enc_MAF_cursor;
	float omega_m;

	float ct_cal_Iu_sum;
	float ct_cal_Iv_sum;
	float ct_cal_Iw_sum;
	int ct_cal_count;
}MotorControl_t;



void MotorControl_Init(MotorControl_t *h);


void MotorControl_Update(MotorControl_t *h);



#endif /* _MOTOR_CONTROL_H_ */



