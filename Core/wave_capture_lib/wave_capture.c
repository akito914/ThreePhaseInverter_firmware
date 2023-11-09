

#include "wave_capture.h"



void WaveCapture_Init(WaveCapture_t *h, WaveCapture_Init_t *init)
{
}


void WaveCapture_Sampling(WaveCapture_t *h);


void WaveCapture_Polling(WaveCapture_t *h);


int WaveCapture_Set_TriggerLevel(WaveCapture_t *h, float trig_level);

int WaveCapture_Set_TriggerPos(WaveCapture_t *h, int32_t trig_pos);

int WaveCapture_Set_TriggerEdgeSlope(WaveCapture_t *h, WaveCapture_Slope_e slope);

int WaveCapture_Set_TriggerMode(WaveCapture_t *h, WaveCapture_Mode_e mode);

int WaveCapture_Set_Decimate(WaveCapture_t *h, uint32_t decimate);

int WaveCapture_Get_WaveForm(WaveCapture_t *h, uint8_t *buf, uint32_t *length);
