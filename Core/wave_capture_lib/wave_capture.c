

#include "wave_capture.h"

#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>


static int get_bytes(WaveCapture_Type_e type)
{
	switch(type)
	{
	case WAVECAPTURE_TYPE_FLOAT:
		return 4;
	case WAVECAPTURE_TYPE_INT32:
		return 4;
	}
	return 0;
}


int WaveCapture_Init(WaveCapture_t *h, WaveCapture_Init_t *init)
{
	// Initialize
	h->ch_select = 0x00000000;
	h->trig_level_f = 0.0f;
	h->trig_lebel_i = 0;
	h->trig_ch = 0;
	h->trig_pos = 0;

	// Set configuration data
	h->init.func_write = init->func_write;
	h->init.sampling_length = init->sampling_length;
	h->init.channel_num = init->channel_num;

	// Allocate for type array
	WaveCapture_Type_e* type_arr = malloc(sizeof(WaveCapture_Type_e) * h->init.channel_num);
	if(type_arr == NULL)
	{
		return -1;
	}
	for(int ch = 0; ch < h->init.channel_num; ch++)
	{
		type_arr[ch] = init->type_array[ch];
	}
	h->init.type_array = type_arr;

	// Allocate for wave memory
	void** pp_wavedata = malloc(16);
	printf("pp_wavedata allocate length = %d\n", sizeof(void*) * h->init.channel_num);
	printf("pp_wavedata allocated length = %d\n", malloc_usable_size(pp_wavedata));
	if(pp_wavedata == NULL)
	{
		free(h->init.type_array);
		return -1;
	}
	for(int ch = 0; ch < h->init.channel_num; ch++)
	{
		// [ch] data allocation
		int ch_data_size = get_bytes(h->init.type_array[ch]);
		void* p_ch_data = malloc(ch_data_size * h->init.sampling_length);
		if(p_ch_data == NULL)
		{
			// Free all allocated memory
			for(int i = 0; i < ch - 1; i++)
			{
				free(pp_wavedata[i]);
			}
			free(pp_wavedata);
			free(h->init.type_array);
			return -1;
		}
		pp_wavedata[ch] = p_ch_data;
	}
	h->wavedata = pp_wavedata;

	return 0;
}


void WaveCapture_Sampling(WaveCapture_t *h)
{

}


void WaveCapture_Polling(WaveCapture_t *h)
{

}

int WaveCapture_Set_Channel(WaveCapture_t *h, uint32_t channel_select)
{
	h->ch_select = channel_select;
	return 0;
}

int WaveCapture_Set_TriggerLevel(WaveCapture_t *h, float trig_level_f, int trig_level_i)
{
	h->trig_level_f = trig_level_f;
	h->trig_lebel_i = trig_level_i;
	return 0;
}

int WaveCapture_Set_TriggerChannel(WaveCapture_t *h, uint32_t trig_channel)
{
	if(trig_channel >= h->init.channel_num)
	{
		return -1;
	}
	h->trig_ch = trig_channel;
	return 0;
}

int WaveCapture_Set_TriggerPos(WaveCapture_t *h, int32_t trig_pos)
{
	int pos_min = -h->init.sampling_length / 2 + 1;
	int pos_max = (h->init.sampling_length + 1) / 2 - 1;
	if(trig_pos < pos_min)
	{
		return -1;
	}
	if(trig_pos > pos_max)
	{
		return -1;
	}
	h->trig_pos = trig_pos;
	return 0;
}


void WaveCapture_Dispose(WaveCapture_t *h)
{
	free(h->init.type_array);
	for(int ch = 0; ch < h->init.channel_num; ch++)
	{
		free(h->wavedata[ch]);
	}
	free(h->wavedata);
}
