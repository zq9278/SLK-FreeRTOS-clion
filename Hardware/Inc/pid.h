
#include "main.h"
#ifndef _PID_H
#define _PID_H
	  


#define Limit(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

typedef struct
{
    float target_val;
    float actual_val;
    float err;
    float err_last; 
    float Kp,Ki,Kd;
    float integral;
    float maxIntegral;
    float maxOutput;
}PID_typedef;

void HeatPIDInit(float Temp_set);
uint8_t PID_realize(PID_typedef *pid,float temp_val);
void start_Heat(osMessageQueueId_t Temperature_QueueHandle);


#endif

