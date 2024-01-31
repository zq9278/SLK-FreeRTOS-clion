#ifndef screen // 如果 MY_HEADER_H 没有被定义，则执行下面的代码
#define screen

#include "main.h"

// #define NOTIFY_TOUCH_PRESS         0X01  //触摸屏按下通知
// #define NOTIFY_TOUCH_RELEASE       0X03  //触摸屏松开通知
// #define NOTIFY_WRITE_FLASH_OK      0X0C  //写FLASH成功
// #define NOTIFY_WRITE_FLASH_FAILD   0X0D  //写FLASH失败
// #define NOTIFY_READ_FLASH_OK       0X0B  //读FLASH成功
// #define NOTIFY_READ_FLASH_FAILD    0X0F  //读FLASH失败
// #define NOTIFY_MENU                0X14  //菜单事件通知
// #define NOTIFY_TIMER               0X43  //定时器超时通知
// #define NOTIFY_CONTROL             0XB1  //控件更新通知
// #define NOTIFY_READ_RTC            0XF7  //读取RTC时间
// #define MSG_GET_CURRENT_SCREEN     0X01  //画面ID变化通知
// #define MSG_GET_DATA               0X11  //控件数据通知
// #define NOTIFY_HandShake           0X55  //握手通知

// enum CtrlType
// {
//     kCtrlUnknown=0x0,
//     kCtrlButton=0x10,                     //按钮
//     kCtrlText,                            //文本
//     kCtrlProgress,                        //进度条
//     kCtrlSlider,                          //滑动条
//     kCtrlMeter,                            //仪表
//     kCtrlDropList,                        //下拉列表
//     kCtrlAnimation,                       //动画
//     kCtrlRTC,                             //时间显示
//     kCtrlGraph,                           //曲线图控件
//     kCtrlTable,                           //表格控件
//     kCtrlMenu,                            //菜单控件
//     kCtrlSelector,                        //选择控件
//     kCtrlQRCode,                          //二维码
// };

typedef struct __attribute__((packed))
{
    uint32_t cmd_head; // 帧头
    uint16_t cmd_type; // 命令类型(UPDATE_CONTROL)
    uint8_t control_type;  // 控件类型
    uint16_t data;                                                                                                                              

} CTRL_MSG, *PCTRL_MSG;
#define FILTER_SIZE 3//滤波长度
#define FILTER_SIZE_TEMP 3//滤波长度
float processFilter(float *buffer);
uint32_t processFilter_force(uint32_t *buffer);

void processData(PCTRL_MSG msg);
void ScreenUpdateTemperature(float value,uint16_t work_mode);

void SendDataToScreen(uint8_t* Data,uint8_t len);
void ScreenUpdateForce(uint32_t value,uint16_t work_mode);
void ScreenUpdateSOC(uint16_t value,uint8_t state);
void ScreenWorkModeQuit(uint8_t workmodenumber);
void ScreenTimerStart(uint8_t workmodenumber);

void ProcessForceData(uint16_t work_mode);
void ProcessTemperatureData(uint16_t work_mode); 
// 在这里放置你的函数声明、类型定义等

#endif // 结束 #ifndef 块