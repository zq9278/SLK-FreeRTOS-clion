#include "hx711.h"

int32_t ForceRawOffset;

/* HX711 初始化 */
void HX711_Init(void) {
    ForceRawOffset = HX711_Read();//读取压力传感器初始偏执值。
}

/* 读取HX711数据 */
int32_t HX711_Read(void) {
    int32_t data = 0;
    uint8_t i;
    /* 等待数据引脚为低 ,即A/D转换器准备好*/
    while (HAL_GPIO_ReadPin(HX711_DOUT_GPIO_Port, HX711_DOUT_Pin) == GPIO_PIN_SET)//当数据位是高电平时，一直进入循环，直到数据为变成低电平
    { ; }
    /* 读取24位数据 */
    for (i = 0; i < 24; i++) {
        /* 时钟引脚置高 */
        HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_SET);
        data <<= 1;
        /* 延迟 */
//        delay_us(1);
        __NOP();
        __NOP();
        __NOP();
        /* 时钟引脚置低 */
        HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET);
        /* 检查数据引脚的值 */
        if (HAL_GPIO_ReadPin(HX711_DOUT_GPIO_Port, HX711_DOUT_Pin) == GPIO_PIN_SET) {
            data++;
        }

        /* 延迟 */
//        delay_us(1);
        __NOP();
        __NOP();
        __NOP();
    }

    /* 时钟引脚置高 */
    HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_SET);
//    delay_us(1);
    __NOP();
    __NOP();
    __NOP();
    /* 时钟引脚置低 */
    HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET);
    //delay_us(1);    /* 符号位扩展 */
    if (data & 0x00800000) {
        data |= 0xFF000000;
    }
    //data=data^ 0x800000;
    return data;
}

int32_t HX711_AVG(uint8_t times) {
    int32_t tempdata[times];
    int64_t tempSum = 0;
    int32_t AVGData;
    for (uint16_t i = 0; i < times; i++) {
        tempdata[i] = HX711_Read();
        tempSum += tempdata[i];
    }
    AVGData = tempSum / times;
    return AVGData;
}

/* 获取力（牛顿） */
float HX711_GetForce(void) {
    int32_t rawData = HX711_Read();
    //  float force = (float)rawData;
    //float force = (float)rawData / HX711_SCALE_FACTOR;
    //float force = ((0-(float)rawData) / HX711_SCALE_FACTOR)/10+2.538;
    float force = (float) rawData / HX711_SCALE_FACTOR;
    return force;
}

int32_t Force2Raw(uint8_t Force) {
    return (int32_t) (Force * HX711_SCALE_FACTOR);
}
