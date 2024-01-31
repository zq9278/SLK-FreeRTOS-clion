
#include "ucs1903.h"


extern TIM_HandleTypeDef htim16;
#define DATA_SIZE 24  // 每个 LED 24 位数据
#define T1H    __HAL_TIM_SET_COMPARE(&htim16, LED_TIM_CHANNEL, 80)   // 定义表示数据位 1 的时间间隔
#define T0H    __HAL_TIM_SET_COMPARE(&htim16, LED_TIM_CHANNEL, 20)   // 定义表示数据位 0 的时间间隔
char cmd_head_str[9];  
uint32_t Single_LED_Buffer[6 * DATA_SIZE];


void sendColor(uint8_t red, uint8_t green, uint8_t blue) {
    
    uint32_t GRB_Data = ((uint32_t)green << 16) | ((uint32_t)red << 8) | blue;

    for (int j = 0; j < 6; j++) {
        for (int i = 0; i < DATA_SIZE; i++) {
            Single_LED_Buffer[j * DATA_SIZE + i] = (GRB_Data & (1 << (23 - i))) ? 128 : 32;
            // snprintf(cmd_head_str, sizeof(cmd_head_str), "%08X", Single_LED_Buffer[j * DATA_SIZE + i]);
            //   printf("cmd_head: %s\n",cmd_head_str);
        }
    }

    
    //    for(int i = 0; i < 160; i++){
    //     __NOP();
    // }
   // vTaskDelay(1000);
}
 void UCS1903Show(void){

HAL_TIM_PWM_Start_DMA(&htim16, TIM_CHANNEL_1, (uint32_t *)Single_LED_Buffer, 6 * DATA_SIZE);
 }
// int main(void){
//     HAL_Init();
//     MX_TIM3_Init(); // 初始化定时器和PWM

//     while(1){
//         sendColor(0, 0, 255); // 发送蓝色
//         HAL_Delay(1000); // 等待一段时间
//     }
// }