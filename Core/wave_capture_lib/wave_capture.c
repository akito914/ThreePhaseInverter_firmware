

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
	h->trig_level_i = 0;
	h->trig_ch = 0;
	h->trig_pos = 0;
	h->trig_slope = WAVECAPTURE_TRIG_SLOPE_RISE;
	h->trig_mode = WAVECAPTURE_MODE_SINGLE;
	h->decimate = 1;
	h->cursor = 0;
	h->cursor_prev = 0;
	h->status = WAVECAPTURE_SAMPLE_FREERUN;
	h->decimate_counter = 0;
	h->cursor_trig = 0;
	h->cursor_end = 0;
	h->timeout = 65536;
	h->timeout_count = 0;

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

	// Allocate for variable pointer array
	void** pp_variable = malloc(sizeof(void*) * h->init.channel_num);
	if(pp_variable == NULL)
	{
		return -1;
	}
	for(int ch = 0; ch < h->init.channel_num; ch++)
	{
		pp_variable[ch] = init->var_ptr_array[ch];
	}
	h->init.var_ptr_array = pp_variable;

	// Allocate for wave memory
	void** pp_wavedata = malloc(sizeof(void*) * h->init.channel_num);
	if(pp_wavedata == NULL)
	{
		free(h->init.type_array);
		free(h->init.var_ptr_array);
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
			free(h->init.var_ptr_array);
			return -1;
		}
		pp_wavedata[ch] = p_ch_data;
	}
	h->wavedata = pp_wavedata;

	return 0;
}


static int detect_trigger(WaveCapture_t *h)
{
	void* ptr_now = h->wavedata[h->trig_ch] + h->cursor * get_bytes(h->init.type_array[h->trig_ch]);
	void* ptr_prev = h->wavedata[h->trig_ch] + h->cursor_prev * get_bytes(h->init.type_array[h->trig_ch]);

	int pole_now, pole_prev;
	switch(h->init.type_array[h->trig_ch])
	{
	case WAVECAPTURE_TYPE_FLOAT:
		pole_now = *(float*)ptr_now > h->trig_level_f;
		pole_prev = *(float*)ptr_prev > h->trig_level_f;
		break;
	case WAVECAPTURE_TYPE_INT32:
		pole_now = *(int32_t*)ptr_now > h->trig_level_i;
		pole_prev = *(int32_t*)ptr_prev > h->trig_level_i;
		break;
	}

	if(h->trig_slope == WAVECAPTURE_TRIG_SLOPE_RISE)
	{
		if(pole_prev == 0 && pole_now == 1)
		{
			return 1;
		}
	}
	else if(h->trig_slope == WAVECAPTURE_TRIG_SLOPE_FALL)
	{
		if(pole_prev == 1 && pole_now == 0)
		{
			return 1;
		}
	}
	return 0;
}

static int timeout_check(WaveCapture_t *h)
{
	if(h->trig_mode == WAVECAPTURE_MODE_AUTO && h->timeout_count >= h->timeout)
	{
		return 1;
	}
	return 0;
}

void WaveCapture_Sampling(WaveCapture_t *h)
{
	if(h->status == WAVECAPTURE_SAMPLE_STOPPED)
	{
		return;
	}

	h->decimate_counter++;
	if(h->decimate_counter >= h->decimate)
	{
		h->decimate_counter = 0;
	}
	if(h->decimate_counter != 0)
	{
		return;
	}

	for(int ch = 0; ch < h->init.channel_num; ch++)
	{
		void* dest_ptr = h->wavedata[ch] + h->cursor * get_bytes(h->init.type_array[ch]);
		switch(h->init.type_array[ch])
		{
		case WAVECAPTURE_TYPE_FLOAT:
			*((float*)dest_ptr) = *(float*)(h->init.var_ptr_array[ch]);
			break;
		case WAVECAPTURE_TYPE_INT32:
			*((int32_t*)dest_ptr) = *(int32_t*)(h->init.var_ptr_array[ch]);
			break;
		}
	}

	// Trigger detection
	switch(h->status)
	{
	case WAVECAPTURE_SAMPLE_FREERUN:
		break;
	case WAVECAPTURE_SAMPLE_TOTRIG:
		if(detect_trigger(h) || timeout_check(h))
		{
			h->cursor_trig = h->cursor;
			uint32_t end = h->cursor_trig + h->init.sampling_length / 2 - h->trig_pos;
			if(end >= h->init.sampling_length)
			{
				end -= h->init.sampling_length;
			}
			h->cursor_end = end;
			h->timeout_count = 0;
			h->status = WAVECAPTURE_SAMPLE_TOEND;
		}
		else
		{
			if(h->trig_mode == WAVECAPTURE_MODE_AUTO)
			{
				h->timeout_count++;
			}
		}
		break;
	case WAVECAPTURE_SAMPLE_TOEND:
		if(h->cursor == h->cursor_end)
		{
			h->status = WAVECAPTURE_SAMPLE_STOPPED;
		}
		break;
	default:
		break;
	}

	h->cursor_prev = h->cursor;
	h->cursor += 1;
	if(h->cursor >= h->init.sampling_length)
	{
		h->cursor = 0;
	}


}


int WaveCapture_Set_Channel(WaveCapture_t *h, uint32_t channel_select)
{
	h->ch_select = channel_select;
	return 0;
}

int WaveCapture_Set_TriggerLevel(WaveCapture_t *h, float trig_level_f, int trig_level_i)
{
	h->trig_level_f = trig_level_f;
	h->trig_level_i = trig_level_i;
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
	int pos_min = -(int32_t)(h->init.sampling_length / 2) + 1;
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

int WaveCapture_Set_TriggerEdgeSlope(WaveCapture_t *h, WaveCaptureTrigSlope_e slope)
{
	h->trig_slope = slope;
	return 0;
}

int WaveCapture_Set_TriggerMode(WaveCapture_t *h, WaveCapture_Mode_e mode)
{
	h->trig_mode = mode;
	return 0;
}

int WaveCapture_Set_Decimate(WaveCapture_t *h, uint32_t decimate)
{
	if(decimate <= 0)
	{
		return -1;
	}
	h->decimate = decimate;
	return 0;
}

int WaveCapture_Start_Sampling(WaveCapture_t *h)
{
	h->status = WAVECAPTURE_SAMPLE_TOTRIG;
	return 0;
}

int WaveCapture_Get_WaveForm(WaveCapture_t *h)
{
	if(h->status != WAVECAPTURE_SAMPLE_STOPPED)
	{
		return -1;
	}

	for(int ch = 0; ch < h->init.channel_num; ch++)
	{
		switch(h->init.type_array[ch])
		{
		case WAVECAPTURE_TYPE_FLOAT:
			for(int i = h->cursor_end+1; i < h->init.sampling_length; i++)
			{
				char buf[100];
				float val = ((float*)(h->wavedata[ch]))[i];
				int len = sprintf(buf, "%f, ", val);
				h->init.func_write((uint8_t*)buf, len);
			}
			for(int i = 0; i <= h->cursor_end; i++)
			{
				char buf[100];
				float val = ((float*)(h->wavedata[ch]))[i];
				int len = sprintf(buf, "%f, ", val);
				h->init.func_write((uint8_t*)buf, len);
			}
			break;
		case WAVECAPTURE_TYPE_INT32:
			for(int i = h->cursor_end+1; i < h->init.sampling_length; i++)
			{
				char buf[100];
				int32_t val = ((int32_t*)(h->wavedata[ch]))[i];
				int len = sprintf(buf, "%ld, ", val);
				h->init.func_write((uint8_t*)buf, len);
			}
			for(int i = 0; i <= h->cursor_end; i++)
			{
				char buf[100];
				int32_t val = ((int32_t*)(h->wavedata[ch]))[i];
				int len = sprintf(buf, "%ld, ", val);
				h->init.func_write((uint8_t*)buf, len);
			}
			break;
		}
		h->init.func_write("\r\n", 2);
	}

	h->status = WAVECAPTURE_SAMPLE_FREERUN;

	return 0;
}

int WaveCapture_Set_Timeout(WaveCapture_t *h, uint32_t timeout)
{
	h->timeout = timeout;
	return 0;
}


void WaveCapture_Dispose(WaveCapture_t *h)
{
	free(h->init.type_array);
	free(h->init.var_ptr_array);
	for(int ch = 0; ch < h->init.channel_num; ch++)
	{
		free(h->wavedata[ch]);
	}
	free(h->wavedata);
}
