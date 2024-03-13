#include "pid.h"

uint8_t HeatPWMVal=0;

PID_typedef HeatPID;
extern uint8_t EyeTmpRaw[2];
extern float EyeTmp;
extern TIM_HandleTypeDef htim14;

void HeatPIDInit(float Temp_set)
{
	HeatPID.target_val=Temp_set;
	HeatPID.actual_val=0; 
	HeatPID.err=0;
	HeatPID.err_last=0;
	HeatPID.integral=0;
	
	HeatPID.maxIntegral=100;
	HeatPID.maxOutput=255;
	
	HeatPID.Kp=50;
	HeatPID.Ki=0.5;
	HeatPID.Kd=500;
	
}

uint8_t PID_realize(PID_typedef *pid,float temp_val) 
{
    pid->err=pid->target_val-temp_val;
    pid->integral+=pid->err;
    Limit(pid->integral,-pid->maxIntegral,pid->maxIntegral);
    pid->actual_val=pid->Kp*pid->err+pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);
    Limit(pid->actual_val,0,pid->maxOutput);
    pid->err_last=pid->err;
    return pid->actual_val;
}

void start_Heat(osMessageQueueId_t Temperature_QueueHandle){
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
      TMP114_Read(0x00, EyeTmpRaw);    // obtain original value of the current temperature sensor by reading the iic
      EyeTmp = TmpRaw2Ture(EyeTmpRaw); // convert raw temperature data
      xQueueSend(Temperature_QueueHandle, &EyeTmp, 0);
      // printf("Temperature:%f\n", EyeTmp);
      HeatPWMVal = PID_realize(&HeatPID, EyeTmp); // Obtain PWM value through PID algorithm
      // snprintf(HeatPWMVal_str, sizeof(HeatPWMVal_str), "%02X", HeatPWMVal);
      //  printf("PWM:%s\n", HeatPWMVal_str);
      __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, HeatPWMVal); // enable timer comparison to generate PWM
}






















