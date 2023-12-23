

#ifndef _SENSOR_BOARD_H_
#define _SENSOR_BOARD_H_


#include <stdint.h>


typedef struct
{
	uint16_t ch_num;
	float ad2Iuvw_gain;
	float ad2Vdc_gain;
	float Iuvw_offset[3];
	float Vdc_offset;
}SensorBoard_Init_t;


typedef struct
{
	SensorBoard_Init_t init;
	uint16_t ad_arr[4];
	float Vdc;
	float Iu;
	float Iv;
	float Iw;
}SensorBoard_t;


void SensorBoard_Init(SensorBoard_t *h);

void SensorBoard_GetADC(SensorBoard_t *h, uint16_t* ad_arr);

void SensorBoard_Update(SensorBoard_t *h, int sector);


#endif /* _SENSOR_BOARD_H_ */


