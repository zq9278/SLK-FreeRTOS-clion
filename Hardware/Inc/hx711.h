#ifndef __HX711_H__
#define __HX711_H__

#include "main.h"

/* HX711 数据和时钟引脚定义 */
#define HX711_DATA_GPIO_Port    GPIOA
#define HX711_DATA_Pin          GPIO_PIN_9
#define HX711_CLK_GPIO_Port     GPIOA
#define HX711_CLK_Pin           GPIO_PIN_10

/* 数据缩放因子，根据负载传感器的参数和电压参考调整
 * 变大范围变小*/

//#define HX711_SCALE_FACTOR     84534.474f
//#define HX711_SCALE_FACTOR_10     8453.4474f
//#define HX711_SCALE_FACTOR_100     845.34474f

#define HX711_SCALE_FACTOR     80942.474f
#define HX711_SCALE_FACTOR_10     8094.4474f
#define HX711_SCALE_FACTOR_100     809.4224f

/* 初始化HX711 */
void HX711_Init(void);

/* 读取HX711数据 */
int32_t HX711_Read(void);

int32_t HX711_AVG(uint8_t times);

/* 获取力（牛顿） */
float HX711_GetForce(void);
/*获得毛皮*/
float get_maopi(int32_t maopi);

#endif // __HX711_H__
