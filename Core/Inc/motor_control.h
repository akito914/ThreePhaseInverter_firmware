
#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_



#include <stdint.h>
#include "sensor_board.h"


#define ENC_MAF_SIZE 1024



typedef enum
{
	MODE_V_UVW,
	MODE_VF,
}MotorControl_Mode_e;


typedef struct
{
	float Ts;
	float V_f_rate;
	float vf_inc_rate; // [Hz / s]
}MotorControl_Init_t;


typedef struct
{
	MotorControl_Init_t init;
	SensorBoard_t sensor;
	float Vu_ref, Vv_ref, Vw_ref;
	float amp_u, amp_v, amp_w;
	float vf_phase;
	float vf_freq_ref;
	float vf_freq;
	float vf_volt;
	MotorControl_Mode_e mode;
	uint16_t enc_count;
	uint16_t enc_count_prev;
	int16_t enc_diff;
	int32_t enc_diff_MAF_buf[ENC_MAF_SIZE];
	int32_t enc_diff_MAF_sum;
	int32_t enc_MAF_cursor;
	int first_sample;
	float omega_m;
}MotorControl_t;



void MotorControl_Init(MotorControl_t *h);


void MotorControl_Update(MotorControl_t *h);



#endif /* _MOTOR_CONTROL_H_ */



