

#include "main.h"
#include "inverter_board.h"


extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;



void InverterBoard_Init()
{

	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);


	__HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);
	__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);

	HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_UPDATE);

	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, htim8.Init.Period / 2 * (1 + 0));
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, htim8.Init.Period / 2 * (1 + 0));
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, htim8.Init.Period / 2 * (1 - 0));
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, htim8.Init.Period / 2 * (1 - 0.9));

	HAL_TIM_Base_Start(&htim8);

	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_4);

	HAL_TIMEx_PWMN_Start_IT(&htim8, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start_IT(&htim8, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start_IT(&htim8, TIM_CHANNEL_3);


}


void InverterBoard_setPWM(float amp_u, float amp_v, float amp_w, float amp_br)
{

	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, htim8.Init.Period / 2 * (1 + amp_u));
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, htim8.Init.Period / 2 * (1 + amp_v));
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, htim8.Init.Period / 2 * (1 + amp_w));
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, htim8.Init.Period / 2 * (1 + amp_br));

}



