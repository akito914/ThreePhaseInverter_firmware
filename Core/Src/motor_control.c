

#include "main.h"
#include "motor_control.h"
#include <math.h>


extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;



void MotorControl_Init(MotorControl_t *h)
{

	SensorBoard_Init(&h->sensor);



	HAL_Delay(1000);

	HAL_GPIO_WritePin(MC_GPIO_Port, MC_Pin, GPIO_PIN_SET);

	HAL_Delay(1000);

	HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_SET);



	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);


	__HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);
	__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);

	HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_UPDATE);

	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, htim8.Init.Period / 2 * (1 + 0));
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, htim8.Init.Period / 2 * (1 + 0));
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, htim8.Init.Period / 2 * (1 - 0));
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, htim8.Init.Period / 2 * (1 - 0.9));

	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_4);

	HAL_TIMEx_PWMN_Start_IT(&htim8, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start_IT(&htim8, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start_IT(&htim8, TIM_CHANNEL_3);



}


void MotorControl_Update(MotorControl_t *h)
{

	SensorBoard_Update(&h->sensor);

	float omega = 377;


	h->phase += omega * 100E-6;
	if(h->phase > M_PI)
	{
		h->phase -= 2*M_PI;
	}

	h->amp_u = 0.85 * cosf(h->phase);
	h->amp_v = 0.85 * cosf(h->phase - M_PI*2/3.0f);
	h->amp_w = 0.85 * cosf(h->phase + M_PI*2/3.0f);


	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, htim8.Init.Period / 2 * (1 + h->amp_u));
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, htim8.Init.Period / 2 * (1 + h->amp_v));
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, htim8.Init.Period / 2 * (1 + h->amp_w));

}




