
#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_



#include <stdint.h>
#include "sensor_board.h"


typedef struct
{
	float V_f_rate;
}MotorControl_Init_t;


typedef struct
{
	MotorControl_Init_t init;
	SensorBoard_t sensor;
	float Vu_ref, Vv_ref, Vw_ref;
	float amp_u, amp_v, amp_w;
	float phase;
}MotorControl_t;



void MotorControl_Init(MotorControl_t *h);


void MotorControl_Update(MotorControl_t *h);



#endif /* _MOTOR_CONTROL_H_ */



