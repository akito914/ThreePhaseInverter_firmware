


#include "sensor_board.h"
#include "main.h"



void SensorBoard_Init(SensorBoard_t *h)
{
	h->init.ad2Iuvw_gain = 5.0f / 4096.0f * 10.0f / 2.0f;
	h->init.ad2Vdc_gain = 0.081985f; /* 実測より */
	h->init.Iuvw_offset[0] = h->init.ad2Iuvw_gain * 2048 + 0.066685823242186484;
	h->init.Iuvw_offset[1] = h->init.ad2Iuvw_gain * 2048 + 0.0051024550781249975;
	h->init.Iuvw_offset[2] = h->init.ad2Iuvw_gain * 2048 + -0.046348672851562474;
	h->init.Vdc_offset = 0;


	// ADC Setting

	// Initialize
  	HAL_GPIO_WritePin(nCS_GPIO_Port, nCS_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(nRD_GPIO_Port, nRD_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(nWR_GPIO_Port, nWR_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(CONVST_GPIO_Port, CONVST_Pin, GPIO_PIN_SET);

  	// Write mode
  	HAL_GPIO_WritePin(nCS_GPIO_Port, nCS_Pin, GPIO_PIN_RESET);
  	HAL_GPIO_WritePin(nWR_GPIO_Port, nWR_Pin, GPIO_PIN_RESET);

  	// Set GPIO as output port
    GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = D0_Pin|D1_Pin|D2_Pin|D3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// channel activation
  	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_SET);

  	// Write
  	HAL_GPIO_WritePin(nWR_GPIO_Port, nWR_Pin, GPIO_PIN_SET);
  	HAL_Delay(1);
  	HAL_GPIO_WritePin(nCS_GPIO_Port, nCS_Pin, GPIO_PIN_SET);

  	// Set GPIO as input port
	GPIO_InitStruct.Pin = D0_Pin|D8_Pin|D9_Pin|D10_Pin
						  |D11_Pin|nEOC_Pin|D1_Pin|D2_Pin
						  |D3_Pin|D4_Pin|D5_Pin|D6_Pin
						  |D7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}



void SensorBoard_GetADC(SensorBoard_t *h, uint16_t* ad_arr)
{

	while(HAL_GPIO_ReadPin(nEOLC_GPIO_Port, nEOLC_Pin) == GPIO_PIN_SET){}

	HAL_GPIO_WritePin(nCS_GPIO_Port, nCS_Pin, GPIO_PIN_RESET);


	for(int ch = 0; ch < 4; ch++)
	{
	  HAL_GPIO_WritePin(nRD_GPIO_Port, nRD_Pin, GPIO_PIN_RESET);

	  for(int w = 0; w < 10; w++){}

	  uint32_t GB;
	  uint16_t D, DH, DL;
	  GB = GPIOB->IDR;
	  DL = (GB >> 2) & 0x000001FF;
	  DH = (GB >> 12) & 0x00000007;
	  D = (DH << 9) | DL;
	  ad_arr[ch] = D;

	  HAL_GPIO_WritePin(nRD_GPIO_Port, nRD_Pin, GPIO_PIN_SET);

	  for(int w = 0; w < 10; w++){}
	}

	HAL_GPIO_WritePin(nCS_GPIO_Port, nCS_Pin, GPIO_PIN_SET);

}


void SensorBoard_Update(SensorBoard_t *h, int sector)
{

	float Iu, Iv, Iw;

	SensorBoard_GetADC(h, h->ad_arr);

	Iu = h->ad_arr[0] * h->init.ad2Iuvw_gain - h->init.Iuvw_offset[0];
	Iv = h->ad_arr[1] * h->init.ad2Iuvw_gain - h->init.Iuvw_offset[1];
	Iw = h->ad_arr[2] * h->init.ad2Iuvw_gain - h->init.Iuvw_offset[2];

	switch(sector)
	{
	case 6: case 1:
		h->Iu = - Iv - Iw;
		h->Iv = Iv;
		h->Iw = Iw;
		break;
	case 2: case 3:
		h->Iu = Iu;
		h->Iv = - Iw - Iu;
		h->Iw = Iw;
		break;
	case 4: case 5:
		h->Iu = Iu;
		h->Iv = Iv;
		h->Iw = - Iu - Iv;
		break;
	default:
		h->Iu = Iu;
		h->Iv = Iv;
		h->Iw = Iw;
	}

	h->Vdc = h->ad_arr[3] * h->init.ad2Vdc_gain - h->init.Vdc_offset;

}

