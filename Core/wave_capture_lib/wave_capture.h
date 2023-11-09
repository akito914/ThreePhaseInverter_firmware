

#ifndef _WAVE_CAPTURE_H_
#define _WAVE_CAPTURE_H_

#include <stdint.h>

/* return number of read/written bytes */

typedef int (*WaveCapture_SerialWrite_f)(const uint8_t *buf, int length);

typedef enum
{
	WAVECAPTURE_TYPE_FLOAT,
	WAVECAPTURE_TYPE_INT32,
}WaveCapture_Type_e;

typedef enum
{
	TRIG_MODE_AUTO = 0,
	TRIG_MODE_NORMAL = 1,
}WaveCapture_Slope_e;

typedef enum
{
	WAVECAPTURE_TRIG_MODE_AUTO = 0,
	WAVECAPTURE_TRIG_MODE_NORMAL = 1,
}WaveCapture_Mode_e;

typedef struct
{
	WaveCapture_SerialWrite_f func_write;
	uint32_t sampling_length;
	uint32_t channel_num;
	WaveCapture_Type_e* type_array;
}WaveCapture_Init_t;

typedef struct
{
	WaveCapture_Init_t init;
	uint32_t ch_select;
	float trig_level_f;
	int trig_lebel_i;
	uint32_t trig_ch;
	int32_t trig_pos;
	void** wavedata;
}WaveCapture_t;


int WaveCapture_Init(WaveCapture_t *h, WaveCapture_Init_t *init);


void WaveCapture_Sampling(WaveCapture_t *h);


void WaveCapture_Polling(WaveCapture_t *h);

int WaveCapture_Set_Channel(WaveCapture_t *h, uint32_t channel_select);

int WaveCapture_Set_TriggerLevel(WaveCapture_t *h, float trig_level_f, int trig_level_i);

int WaveCapture_Set_TriggerChannel(WaveCapture_t *h, uint32_t trig_channel);

int WaveCapture_Set_TriggerPos(WaveCapture_t *h, int32_t trig_pos);

int WaveCapture_Set_TriggerEdgeSlope(WaveCapture_t *h, WaveCapture_Slope_e slope);

int WaveCapture_Set_TriggerMode(WaveCapture_t *h, WaveCapture_Mode_e mode);

int WaveCapture_Set_Decimate(WaveCapture_t *h, uint32_t decimate);

int WaveCapture_Get_WaveForm(WaveCapture_t *h, uint8_t *buf, uint32_t *length);


void WaveCapture_Dispose(WaveCapture_t *h);



#endif /* _WAVE_CAPTURE_H_ */
