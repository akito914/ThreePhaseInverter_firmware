
#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_



#include <stdint.h>
#include "sensor_board.h"


#define ENC_MAF_SIZE 8

#define SIN_TBL_SIZE 1024
#define SIN_TBL_MASK (SIN_TBL_SIZE-1)

typedef enum
{
	MODE_CT_CAL,
	MODE_PWM_TEST,
	MODE_V_UVW,
	MODE_VF,
	MODE_VECTOR_SLIP,
}MotorControl_Mode_e;


typedef struct
{
	float Ts;
	float V_f_rate;
	float vf_inc_rate; // [Hz / s]
	int ct_cal_samples;
	float Idq_lim;
	float pwm_duty_lim;
	float enc_ppr;
	float omega_acr; // current regulator cutoff frequency [rad/s]
	float omega_asr; // speed controller cutoff frequency [rad/s]
	float omega_apr; // position controller cutoff frequency [rad/s]
}MotorControl_Init_t;

typedef struct
{
	int Pn;
	float R1, R2, Rc;
	float L1, L2, M;
	float sigma;
	float R2M_L2;
	float R2_L2;
	float Jm;
}MotorControl_MotorParam_t;

typedef struct
{
	int enable;
	float Kp, Ki;
	float Id_err, Iq_err;
	float Vd_lim_err, Vq_lim_err;
	float Id_err_integ, Iq_err_integ;
	float Vdq_lim;
}MotorControl_ACR_t;

typedef struct
{
	int enable;
	float Kp, Ki;
	float omega_err;
	float omega_err_integ;
}MotorControl_ASR_t;

typedef struct
{
	uint16_t enc_count;
	uint16_t enc_count_prev;
	int32_t enc_integ_count;
	int16_t enc_diff;
	int32_t enc_diff_MAF_buf[ENC_MAF_SIZE];
	int32_t enc_diff_MAF_sum;
	int32_t enc_MAF_cursor;
}MotorControl_Encoder_t;

typedef struct
{
	int clock_counter;
	int prescale;
	int state_counter;
}MotorControl_TestSignalGenerator_t;

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

	MotorControl_Encoder_t encoder;

	float omega_rm;
	float omega_re;
	float theta_rm;

	float ct_cal_Iu_sum;
	float ct_cal_Iv_sum;
	float ct_cal_Iw_sum;
	int ct_cal_count;

	float sin_t[SIN_TBL_SIZE];

	float Id, Iq;
	float Id_ref, Iq_ref;
	float Vd_ref, Vq_ref;
	float theta;
	float cos_theta, sin_theta;

	float omega_ref;
	float theta_ref;

	float tau_ref;
	float phi_2d_ref;
	float phi_2d_est, phi_2q_est;
	float omega_s_ref;

	float winding_theta_ref;
	float winding_omega_max;

	MotorControl_MotorParam_t param;

	MotorControl_ACR_t acr;
	MotorControl_ASR_t asr;

	MotorControl_TestSignalGenerator_t testSig;


}MotorControl_t;



void MotorControl_Init(MotorControl_t *h);

void MotorControl_Setup(MotorControl_t *h);

void MotorControl_Update(MotorControl_t *h);



#endif /* _MOTOR_CONTROL_H_ */



