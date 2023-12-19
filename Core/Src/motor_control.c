

#include "main.h"
#include "motor_control.h"
#include "inverter_board.h"
#include <math.h>
#include <stdio.h>

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;


static void MotorControl_Update_VF_Control(MotorControl_t *h);
static int MotorControl_Update_Vuvw(MotorControl_t *h);


void MotorControl_Init(MotorControl_t *h)
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


	h->mode = MODE_V_UVW;

	h->Vu_ref = 0.0f;
	h->Vv_ref = 0.0f;
	h->Vw_ref = 0.0f;

	h->vf_freq = 0.0f;
	h->vf_freq_ref = 0.0f;
	h->vf_phase = 0.0f;
	h->init.V_f_rate = 200 / 60.0f;
	h->init.vf_inc_rate = 10.0f;

	h->init.Ts = 100E-6;

	InverterBoard_Init();


}


void MotorControl_Update(MotorControl_t *h)
{

	static int scale = 0;

	scale++;
	if(scale >= 2000) scale = 0;

	if(scale == 0) HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	SensorBoard_Update(&h->sensor);


//	MotorControl_Update_VF_Control(h);
//	h->amp_u = 0.85 * cosf(h->vf_phase);
//	h->amp_v = 0.85 * cosf(h->vf_phase - M_PI*2/3.0f);
//	h->amp_w = 0.85 * cosf(h->vf_phase + M_PI*2/3.0f);
//	InverterBoard_setPWM(h->amp_u, h->amp_v, h->amp_w, 0.0);

	switch(h->mode)
	{
	case MODE_V_UVW:
		MotorControl_Update_Vuvw(h);
		break;
	case MODE_VF:
		MotorControl_Update_VF_Control(h);
		MotorControl_Update_Vuvw(h);
		break;
	}

}



static int MotorControl_Update_Vuvw(MotorControl_t *h)
{
	if(h->sensor.Vdc > 10.0f)
	{
		float v2amp = 2.0f / h->sensor.Vdc;
		h->amp_u = v2amp * h->Vu_ref;
		h->amp_v = v2amp * h->Vv_ref;
		h->amp_w = v2amp * h->Vw_ref;
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



