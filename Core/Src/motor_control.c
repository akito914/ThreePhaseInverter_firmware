

#include "main.h"
#include "motor_control.h"
#include "inverter_board.h"
#include <math.h>
#include <stdio.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;

static void MotorControl_Update_CT_CAL(MotorControl_t *h);
static void MotorControl_Update_Encoder(MotorControl_t *h);
static void MotorControl_Update_VF_Control(MotorControl_t *h);
static int MotorControl_Update_Vuvw(MotorControl_t *h);
static void MotorControl_Set_SinTable(MotorControl_t *h);
static float MotorControl_Get_SinTbl(MotorControl_t *h, float phase);
static void MotorControl_Update_SlipVector(MotorControl_t *h);
static void MotorControl_Update_CurrentCommand(MotorControl_t *h);
static void MotorControl_Update_SlipFreq(MotorControl_t *h);
static void MotorControl_Init_ASR(MotorControl_t *h);
static void MotorControl_Update_ASR(MotorControl_t *h);
static void MotorControl_Init_ACR(MotorControl_t *h);
static void MotorControl_Update_ACR(MotorControl_t *h);
static void MotorControl_Update_TestSig(MotorControl_t *h);
static void MotorControl_Update_PWM_Test(MotorControl_t *h);

void MotorControl_Init(MotorControl_t *h)
{

	h->param.Pn = 2;
	h->param.R1 = 17.95484109;
	h->param.R2 = 29.45890479;
	h->param.Rc = 11560.95715;
	h->param.L1 = 0.750701284;
	h->param.L2 = 0.750701284;
	h->param.M = 0.710517547;
	h->param.sigma = 0.104191255;
	h->param.Jm = 3.03E-4;

	h->param.R2M_L2 = h->param.R2 * h->param.M / h->param.L2;
	h->param.R2_L2 = h->param.R2 / h->param.L2;


//	h->init.V_f_rate = 200 / 60.0f;
	h->init.V_f_rate = 50 / 8.33f;
	h->init.vf_inc_rate = 10.0f;

	h->init.Ts = 100E-6;
	h->init.ct_cal_samples = 1024;
	h->init.Idq_lim = 2;
	h->init.pwm_duty_lim = 0.95;
	h->init.omega_acr = 500;
	h->init.omega_asr = 50;

	h->testSig.clock_counter = 0;
	h->testSig.prescale = 5000;
	h->testSig.state_counter = 0;


	h->mode = MODE_V_UVW;

	h->Vu_ref = 0.0f;
	h->Vv_ref = 0.0f;
	h->Vw_ref = 0.0f;
	h->sector = 0;

	h->vf_freq = 0.0f;
	h->vf_freq_ref = 0.0f;
	h->vf_phase = 0.0f;

	h->encoder.enc_count = 0;
	h->encoder.enc_count_prev = 0;
	h->first_sample = 1;
	h->encoder.enc_MAF_cursor = 0;
	h->encoder.enc_diff_MAF_sum = 0;
	for(int i = 0; i < ENC_MAF_SIZE; i++)
	{
		h->encoder.enc_diff_MAF_buf[i] = 0;
	}

	h->ct_cal_Iu_sum = 0;
	h->ct_cal_Iv_sum = 0;
	h->ct_cal_Iw_sum = 0;
	h->ct_cal_count = 0;

	h->Id = 0;
	h->Iq = 0;
	h->Id_ref = 0;
	h->Iq_ref = 0;
	h->Vd_ref = 0;
	h->Vq_ref = 0;
	h->theta = 0;
	h->cos_theta = 1;
	h->sin_theta = 0;
	h->omega_m = 0;
	h->omega_re = 0;

	h->omega_ref = 0;

	h->tau_ref = 0;
	h->phi_2d_ref = 0;

	h->phi_2d_est = 0;
	h->phi_2q_est = 0;

	MotorControl_Set_SinTable(h);

	MotorControl_Init_ACR(h);

	MotorControl_Init_ASR(h);

}




void MotorControl_Setup(MotorControl_t *h)
{

	SensorBoard_Init(&h->sensor);


	while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != GPIO_PIN_RESET)
	{
		HAL_Delay(200);
		printf("...\r\n");
	}
	printf("Start!\r\n");


	HAL_Delay(1000);

	HAL_GPIO_WritePin(MC_GPIO_Port, MC_Pin, GPIO_PIN_SET);

	HAL_Delay(1000);

	HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_SET);


	// Inverter Start

	InverterBoard_Init();

	HAL_Delay(100);

	h->mode = MODE_CT_CAL;
	while(h->mode == MODE_CT_CAL){}

//	h->mode = MODE_V_UVW;


//	h->mode = MODE_VF;
//	h->vf_freq_ref = 60;
//
//	h->mode = MODE_VF;
//	h->vf_freq_ref = 8.33;

	h->phi_2d_ref = 0.5;
	h->tau_ref = 0.0;
	h->mode = MODE_VECTOR_SLIP;

	HAL_Delay(1000);

	h->tau_ref = 0.1;

//	h->vf_freq_ref = 60;
//	h->mode = MODE_PWM_TEST;


}




void MotorControl_Update(MotorControl_t *h)
{

	static int scale = 0;

	scale++;
	if(scale >= 2000)
	{
		scale = 0;
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}

	MotorControl_Update_Encoder(h);

	switch(h->mode)
	{
	case MODE_CT_CAL:
		SensorBoard_Update(&h->sensor, 0);
		MotorControl_Update_CT_CAL(h);
		MotorControl_Update_Vuvw(h);
		break;
	case MODE_PWM_TEST:
		SensorBoard_Update(&h->sensor, h->sector);
		MotorControl_Update_PWM_Test(h);
		break;
	case MODE_V_UVW:
		SensorBoard_Update(&h->sensor, h->sector);
		MotorControl_Update_Vuvw(h);
		break;
	case MODE_VF:
		SensorBoard_Update(&h->sensor, h->sector);
		MotorControl_Update_VF_Control(h);
		MotorControl_Update_Vuvw(h);
		break;
	case MODE_VECTOR_SLIP:
		SensorBoard_Update(&h->sensor, h->sector);
		MotorControl_Update_TestSig(h);
		MotorControl_Update_ASR(h);
		MotorControl_Update_SlipVector(h);
		MotorControl_Update_Vuvw(h);
		break;
	}

}


static void MotorControl_Update_Encoder(MotorControl_t *h)
{

	h->encoder.enc_count = htim1.Instance->CNT;

	h->encoder.enc_diff = h->encoder.enc_count - h->encoder.enc_count_prev;
	h->encoder.enc_count_prev = h->encoder.enc_count;
	if(h->first_sample == 1)
	{
		h->encoder.enc_diff = 0;
		h->first_sample = 0;
	}
	h->encoder.enc_diff_MAF_sum -= h->encoder.enc_diff_MAF_buf[h->encoder.enc_MAF_cursor];
	h->encoder.enc_diff_MAF_sum += h->encoder.enc_diff;
	h->encoder.enc_diff_MAF_buf[h->encoder.enc_MAF_cursor] = h->encoder.enc_diff;
	h->encoder.enc_MAF_cursor++;
	if(h->encoder.enc_MAF_cursor >= ENC_MAF_SIZE)
	{
		h->encoder.enc_MAF_cursor = 0;
	}
	h->omega_m = h->encoder.enc_diff_MAF_sum * 2 * M_PI / 8192 / h->init.Ts / ENC_MAF_SIZE;
	h->omega_re = h->omega_m * h->param.Pn;

}


static int MotorControl_Update_Vuvw(MotorControl_t *h)
{
	if(h->sensor.Vdc > 10.0f)
	{
		float v2amp = 2.0f / h->sensor.Vdc;
		h->amp_u = v2amp * h->Vu_ref;
		h->amp_v = v2amp * h->Vv_ref;
		h->amp_w = v2amp * h->Vw_ref;

		float amp_max, amp_min;
		int comp = 0;
		comp |= (h->amp_u > h->amp_v) << 0;
		comp |= (h->amp_v > h->amp_w) << 1;
		comp |= (h->amp_w > h->amp_u) << 2;
		switch(comp)
		{
		case 0: case 7:
		case 1: amp_max = h->amp_u; amp_min = h->amp_v; h->sector = 6; break;
		case 2: amp_max = h->amp_v; amp_min = h->amp_w; h->sector = 2; break;
		case 3: amp_max = h->amp_u; amp_min = h->amp_w; h->sector = 1; break;
		case 4: amp_max = h->amp_w; amp_min = h->amp_u; h->sector = 4; break;
		case 5: amp_max = h->amp_w; amp_min = h->amp_v; h->sector = 5; break;
		case 6: amp_max = h->amp_v; amp_min = h->amp_u; h->sector = 3; break;
		}
		// 中間電圧1/2重畳
		h->amp_u -= (amp_max + amp_min) / 2;
		h->amp_v -= (amp_max + amp_min) / 2;
		h->amp_w -= (amp_max + amp_min) / 2;

		if(h->amp_u < -1) h->amp_u = -1;
		if(h->amp_u > 1) h->amp_u = 1;
		if(h->amp_v < -1) h->amp_v = -1;
		if(h->amp_v > 1) h->amp_v = 1;
		if(h->amp_w < -1) h->amp_w = -1;
		if(h->amp_w > 1) h->amp_w = 1;
		InverterBoard_setPWM(h->amp_u, h->amp_v, h->amp_w, 0.0);
		return 0;
	}
	else
	{
		h->amp_u = 0.0;
		h->amp_v = 0.0;
		h->amp_w = 0.0;
		InverterBoard_setPWM(h->amp_u, h->amp_v, h->amp_w, 0.0);
		return -1;
	}


}


static void MotorControl_Update_PWM_Test(MotorControl_t *h)
{

	float f_step = h->init.vf_inc_rate * h->init.Ts;

	if(h->vf_freq + f_step < h->vf_freq_ref)
	{
		h->vf_freq += f_step;
	}
	else if(h->vf_freq - f_step > h->vf_freq_ref)
	{
		h->vf_freq -= f_step;
	}
	else
	{
		h->vf_freq = h->vf_freq_ref;
	}

	h->vf_volt = h->init.V_f_rate * h->vf_freq * sqrt(2.0/3);

	h->vf_phase += 2 * M_PI * h->vf_freq * h->init.Ts;
	if(h->vf_phase > M_PI)
	{
		h->vf_phase -= 2 * M_PI;
	}
	else if(h->vf_phase < -M_PI)
	{
		h->vf_phase += 2 * M_PI;
	}

	h->amp_u = h->init.pwm_duty_lim * cosf(h->vf_phase);
	h->amp_v = h->init.pwm_duty_lim * cosf(h->vf_phase - M_PI*2/3.0f);
	h->amp_w = h->init.pwm_duty_lim * cosf(h->vf_phase + M_PI*2/3.0f);

	if(h->amp_u < -1) h->amp_u = -1;
	if(h->amp_u > 1) h->amp_u = 1;
	if(h->amp_v < -1) h->amp_v = -1;
	if(h->amp_v > 1) h->amp_v = 1;
	if(h->amp_w < -1) h->amp_w = -1;
	if(h->amp_w > 1) h->amp_w = 1;
	InverterBoard_setPWM(h->amp_u, h->amp_v, h->amp_w, 0.0);

}


static void MotorControl_Update_VF_Control(MotorControl_t *h)
{

	float f_step = h->init.vf_inc_rate * h->init.Ts;

	if(h->vf_freq + f_step < h->vf_freq_ref)
	{
		h->vf_freq += f_step;
	}
	else if(h->vf_freq - f_step > h->vf_freq_ref)
	{
		h->vf_freq -= f_step;
	}
	else
	{
		h->vf_freq = h->vf_freq_ref;
	}

	h->vf_volt = h->init.V_f_rate * h->vf_freq * sqrt(2.0/3);

	h->vf_phase += 2 * M_PI * h->vf_freq * h->init.Ts;
	if(h->vf_phase > M_PI)
	{
		h->vf_phase -= 2 * M_PI;
	}
	else if(h->vf_phase < -M_PI)
	{
		h->vf_phase += 2 * M_PI;
	}

	h->Vu_ref = h->vf_volt * cosf(h->vf_phase);
	h->Vv_ref = h->vf_volt * cosf(h->vf_phase - M_PI*2/3.0f);
	h->Vw_ref = h->vf_volt * cosf(h->vf_phase + M_PI*2/3.0f);

}



static void MotorControl_Init_ACR(MotorControl_t *h)
{

	h->acr.Kp = h->init.omega_acr * h->param.sigma * h->param.L1;
	h->acr.Ki = h->init.omega_acr * (h->param.R1 + h->param.L1 / h->param.L2 * h->param.R2 * (1 - h->param.sigma));

	h->acr.Id_err_integ = 0;
	h->acr.Iq_err_integ = 0;
	h->acr.Vd_lim_err = 0;
	h->acr.Vq_lim_err = 0;

}


static void MotorControl_Update_ACR(MotorControl_t *h)
{

	h->acr.Id_err = h->Id_ref - h->Id;
	h->acr.Iq_err = h->Iq_ref - h->Iq;

	float Id_err_integ_in = h->acr.Id_err - h->acr.Vd_lim_err / h->acr.Kp;
	float Iq_err_integ_in = h->acr.Iq_err - h->acr.Vq_lim_err / h->acr.Kp;

	h->acr.Id_err_integ += Id_err_integ_in * h->init.Ts;
	h->acr.Iq_err_integ += Iq_err_integ_in * h->init.Ts;

	float Vd = h->acr.Kp * h->acr.Id_err + h->acr.Ki * h->acr.Id_err_integ;
	float Vq = h->acr.Kp * h->acr.Iq_err + h->acr.Ki * h->acr.Iq_err_integ;

	// Voltage Limitter
	h->acr.Vdq_lim = h->sensor.Vdc * 0.70710678118 * h->init.pwm_duty_lim;
	float norm_rate = h->acr.Vdq_lim / sqrt(Vd*Vd + Vq*Vq);
	if(norm_rate < 1)
	{
		h->Vd_ref = norm_rate * Vd;
		h->Vq_ref = norm_rate * Vq;
		h->acr.Vd_lim_err = Vd - h->Vd_ref;
		h->acr.Vq_lim_err = Vq - h->Vq_ref;
	}
	else
	{
		h->Vd_ref = Vd;
		h->Vq_ref = Vq;
		h->acr.Vd_lim_err = 0;
		h->acr.Vq_lim_err = 0;
	}

}


static void MotorControl_Init_ASR(MotorControl_t *h)
{
	h->asr.Kp = 2 * h->init.omega_asr * h->param.Jm;
	h->asr.Ki = h->init.omega_asr * h->init.omega_asr * h->param.Jm;

	h->asr.omega_err_integ = 0;
}


static void MotorControl_Update_ASR(MotorControl_t *h)
{

	h->asr.omega_err = h->omega_ref - h->omega_m;

	h->asr.omega_err_integ += h->asr.omega_err * h->init.Ts;

	h->tau_ref = -h->asr.Kp * h->omega_m + h->asr.Ki * h->asr.omega_err_integ;

}



static void MotorControl_Update_SlipVector(MotorControl_t *h)
{
	h->cos_theta = MotorControl_Get_SinTbl(h, h->theta + 1.570796326794897f);
	h->sin_theta = MotorControl_Get_SinTbl(h, h->theta);

	// UVW => dq
	float Ia = 0.816496580927726f * (h->sensor.Iu - 0.5f * h->sensor.Iv - 0.5f * h->sensor.Iw); // sqrt(2/3)*(Iu+1/2*Iv+1/2*Iw)
	float Ib = 0.7071067811865475f * (h->sensor.Iv - h->sensor.Iw); // sqrt(2/3)*(sqrt(3)/2*Iv - sqrt(3)/2)
	h->Id = h->cos_theta * Ia + h->sin_theta * Ib;
	h->Iq = -h->sin_theta * Ia + h->cos_theta * Ib;

	MotorControl_Update_CurrentCommand(h);

	MotorControl_Update_ACR(h);

	MotorControl_Update_SlipFreq(h);

	float Va_ref = h->cos_theta * h->Vd_ref - h->sin_theta * h->Vq_ref;
	float Vb_ref = h->sin_theta * h->Vd_ref + h->cos_theta * h->Vq_ref;
	h->Vu_ref = 0.816496580927726f * Va_ref; // sqrt(2/3)*Ia
	h->Vv_ref = 0.816496580927726f * (-0.5 * Va_ref + 0.8660254037844386f * Vb_ref); // sqrt(2/3)*(-1/2*Ia + sqrt(3)/2*Ib)
	h->Vw_ref = 0.816496580927726f * (-0.5 * Va_ref - 0.8660254037844386f * Vb_ref); // sqrt(2/3)*(-1/2*Ia - sqrt(3)/2*Ib)

	// Update d-q coordinate
	h->theta += (h->omega_s_ref + h->omega_re) * h->init.Ts;
	if(h->theta >= 2 * M_PI)
	{
		h->theta -= 2 * M_PI;
	}
	else if(h->theta < 0)
	{
		h->theta += 2 * M_PI;
	}

}


static void MotorControl_Update_CurrentCommand(MotorControl_t *h)
{

	h->Id_ref = h->phi_2d_ref / h->param.M;
	h->Iq_ref = h->tau_ref * h->param.L2 / h->phi_2d_ref / h->param.M / h->param.Pn;

}


static void MotorControl_Update_SlipFreq(MotorControl_t *h)
{

	// 2nd flux estimation
	float p_phi_2d_est, p_phi_2q_est;
	p_phi_2d_est = -h->param.R2_L2 * h->phi_2d_est + h->omega_s_ref * h->phi_2q_est + h->param.R2M_L2 * h->Id_ref;
	p_phi_2q_est = -h->omega_s_ref * h->phi_2d_est - h->param.R2_L2 * h->phi_2q_est + h->param.R2M_L2 * h->Iq_ref;
	h->phi_2d_est += p_phi_2d_est * h->init.Ts;
	h->phi_2q_est += p_phi_2q_est * h->init.Ts;

	// update slip frequency
	if(h->phi_2d_est > 0.1f || h->phi_2d_est < -0.1f)
	{
		h->omega_s_ref = h->Iq_ref / h->phi_2d_est * h->param.R2M_L2;
	}
	else
	{
		h->omega_s_ref = 0;
	}

}



static void MotorControl_Update_TestSig(MotorControl_t *h)
{

	h->testSig.clock_counter++;
	if(h->testSig.clock_counter >= h->testSig.prescale)
	{
		h->testSig.clock_counter = 0;

//		switch(h->testSig.state_counter)
//		{
//		case 0:
//			h->Id_ref = 0;
//			h->Iq_ref = 0;
//			h->testSig.state_counter++;
//			break;
//		case 1:
//			h->Id_ref = 0.2;
//			h->Iq_ref = 0;
//			h->testSig.state_counter++;
//			break;
//		case 2:
//			h->Id_ref = 0.2;
//			h->Iq_ref = 0.2;
//			h->testSig.state_counter++;
//			break;
//		case 3:
//			h->Id_ref = 0;
//			h->Iq_ref = 0.2;
//			h->testSig.state_counter = 0;
//			break;
//		default:
//			h->testSig.state_counter = 0;
//		}

		switch(h->testSig.state_counter)
		{
		case 0:
			h->omega_ref = 0.0f;
			h->testSig.state_counter++;
			break;
		case 1:
			h->omega_ref = 188.5f;
			h->testSig.state_counter = 0;
			break;
		default:
			h->testSig.state_counter = 0;
		}

	}


}


static void MotorControl_Update_CT_CAL(MotorControl_t *h)
{
	if(h->ct_cal_count < h->init.ct_cal_samples)
	{
		h->ct_cal_Iu_sum += h->sensor.Iu;
		h->ct_cal_Iv_sum += h->sensor.Iv;
		h->ct_cal_Iw_sum += h->sensor.Iw;
		h->ct_cal_count++;
	}
	else
	{
		h->sensor.init.Iuvw_offset[0] += h->ct_cal_Iu_sum / h->init.ct_cal_samples;
		h->sensor.init.Iuvw_offset[1] += h->ct_cal_Iv_sum / h->init.ct_cal_samples;
		h->sensor.init.Iuvw_offset[2] += h->ct_cal_Iw_sum / h->init.ct_cal_samples;
		h->mode = MODE_V_UVW;
	}
	h->Vu_ref = 0;
	h->Vu_ref = 0;
	h->Vu_ref = 0;
}

static void MotorControl_Set_SinTable(MotorControl_t *h)
{
	for(int i = 0; i < SIN_TBL_SIZE; i++)
	{
		float phase = i * 2 * M_PI / 1024.0f;
		h->sin_t[i] = sin(phase);
	}
}

static float MotorControl_Get_SinTbl(MotorControl_t *h, float phase)
{
	float idx_f = phase * SIN_TBL_SIZE * 0.15915494309189533576888376337251f; // phase / (2 * pi) * SIZE
	int idx = (int)idx_f;
	if(idx_f > idx)
	{
		return (1 - idx_f + idx) * h->sin_t[idx & SIN_TBL_MASK] + (idx_f - idx) * h->sin_t[(idx + 1) & SIN_TBL_MASK];
	}
	else
	{
		return (1 - idx + idx_f) * h->sin_t[idx & SIN_TBL_MASK] + (idx - idx_f) * h->sin_t[(idx - 1) & SIN_TBL_MASK];
	}
}



