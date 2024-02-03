#include "main.h"
//#define LED_TIM_HANDLE TIM16
#define LED_TIM_CHANNEL TIM_CHANNEL_1




// void sendBit(uint8_t bit);

 void UCS1903Show(void);
void PWM_WS2812B_Init(void);
void PWM_WS2812B_Write_24Bits(uint16_t num,uint32_t GRB_Data);
void loopBreatheEffect(void);
