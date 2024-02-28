
#include "ucs1903.h"
#define BREATHE_STEPS 256 // 定义呼吸步骤的总数
uint8_t breatheLUT[BREATHE_STEPS]; // 呼吸灯查找表

extern TIM_HandleTypeDef htim16;
#define DATA_SIZE 24  // 每个 LED 24 位数据
//#define T1H    __HAL_TIM_SET_COMPARE(&htim16, LED_TIM_CHANNEL, 80)   // 定义表示数据位 1 的时间间隔
#define T1H    128
//#define T0H    __HAL_TIM_SET_COMPARE(&htim16, LED_TIM_CHANNEL, 20)   // 定义表示数据位 0 的时间间隔
#define T0H    32  // 定义表示数据位 0 的时间间隔
char cmd_head_str[9];
uint32_t Single_LED_Buffer[4 * DATA_SIZE+200];
void PWM_WS2812B_Init(void)
{
    __HAL_TIM_ENABLE_DMA(&htim16, TIM_DMA_CC1);
    for (int i = 0; i < BREATHE_STEPS; ++i) {
        // 使用正弦函数生成平滑的亮度变化
        //breatheLUT[i] = (uint8_t)((1 + sin(i * 2 * M_PI / BREATHE_STEPS - M_PI / 2)) / 2 * 255);
        breatheLUT[i] = (uint8_t) (((1 + sin(i * 2 * M_PI / BREATHE_STEPS - M_PI / 2)) / 2) * (200 - 10) + 10);
    }

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

HAL_TIM_PWM_Start_DMA(&htim16, TIM_CHANNEL_1, (uint32_t *)Single_LED_Buffer, 4 * DATA_SIZE+200);
 }
void loopBreatheEffect(void){
    for (int i = 0; i < BREATHE_STEPS;/**/ ++i) {
        uint32_t blueColor = breatheLUT[i]; // 仅蓝色通道
        //blueColor <<= 8; // 将红色值移动到GRB格式的正确位置
        PWM_WS2812B_Write_24Bits(4, blueColor); // 假设控制一个LED
        //UCS1903Show(); // 发送更新的颜色数据
        vTaskDelay(2); // 简单的延时来控制呼吸速度
    }
}
