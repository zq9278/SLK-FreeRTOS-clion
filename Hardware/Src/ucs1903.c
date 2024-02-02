
#include "ucs1903.h"


extern TIM_HandleTypeDef htim16;
#define DATA_SIZE 24  // 每个 LED 24 位数据
//#define T1H    __HAL_TIM_SET_COMPARE(&htim16, LED_TIM_CHANNEL, 80)   // 定义表示数据位 1 的时间间隔
#define T1H    128
//#define T0H    __HAL_TIM_SET_COMPARE(&htim16, LED_TIM_CHANNEL, 20)   // 定义表示数据位 0 的时间间隔
#define T0H    32  // 定义表示数据位 0 的时间间隔
char cmd_head_str[9];
uint32_t Single_LED_Buffer[4 * DATA_SIZE+50];
void PWM_WS2812B_Init(void)
{
    __HAL_TIM_ENABLE_DMA(&htim16, TIM_DMA_CC1);
}


void PWM_WS2812B_Write_24Bits(uint16_t num,uint32_t GRB_Data)
{
    uint8_t i,j;
//	while(hdma_tim16_ch1.State != HAL_DMA_STATE_READY){};
    for(j = 0; j < num; j++)
    {
        for(i = 0; i < DATA_SIZE; i++)
        {
            Single_LED_Buffer[j*DATA_SIZE+i] = ((GRB_Data << i) & 0x800000) ? T1H : T0H;
        }
    }
}

 void UCS1903Show(void){

HAL_TIM_PWM_Start_DMA(&htim16, TIM_CHANNEL_1, (const uint32_t *)Single_LED_Buffer, 4 * DATA_SIZE+50);
 }
