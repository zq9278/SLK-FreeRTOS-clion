#include "Screen.h"
#define CMD_HEAD 0XEE			  // 帧头
#define CMD_TAIL 0XFFFCFFFF		  // 帧尾
#define CMD_MAX_SIZE 64			  // 帧尾
uint8_t cmd_buffer[CMD_MAX_SIZE]; // 指令缓存

extern EventGroupHandle_t All_EventHandle;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim7;
extern uint8_t MotorCompareState;

float temperature_buffer[FILTER_SIZE] = {0};
int buffer_index = 0;
uint32_t Force_buffer[FILTER_SIZE] = {0};
int buffer_index_Force = 0;
float temperature;
int Force;
extern osMessageQueueId_t Temperature_QueueHandle;
extern QueueHandle_t Force_QueueHandle;
extern DMA_HandleTypeDef hdma_usart1_tx;
const EventBits_t xBitsToSet = Heat_BIT_0;
const EventBits_t xBitsToSet1 = Motor_BIT_2;
uint8_t SendBuff[13];
uint8_t SendBuff2[14];
int32_t ForceRawSet = 422672; // 屏幕设定的值

void processData(PCTRL_MSG msg)
{									   // HAL_UART_Transmit(&huart1, (uint8_t *)msg, sizeof(CTRL_MSG), 0xFFFF);
									   //  解决大端小端问题。stm32是大端处理
	uint32_t cmd_head = msg->cmd_head; // 指令类型
	cmd_head = ((cmd_head & 0x000000FF) << 24) |
			   ((cmd_head & 0x0000FF00) << 8) |
			   ((cmd_head & 0x00FF0000) >> 8) |
			   ((cmd_head & 0xFF000000) >> 24);
	uint16_t cmd_type = msg->cmd_type; // 指令类型
	cmd_type = ((cmd_type & 0x00FF) << 8) |
			   ((cmd_type & 0xFF00) >> 8);
	uint8_t control_type = msg->control_type; // 指令类型
	uint16_t data = msg->data;				  // 指令类型
	data = ((data & 0x00FF) << 8) |
		   ((data & 0xFF00) >> 8);
	// HAL_UART_Transmit(&huart1, (uint8_t *)&(data), sizeof(data), 0xFFFF);
	//  //为每个变量创建字符串缓冲区
	//  char cmd_head_str[9];     // uint32_t 转换为16进制字符串需要8个字符 + '\0'
	//  char cmd_type_str[5];     // uint16_t 转换为16进制字符串需要4个字符 + '\0'
	//  char control_type_str[3]; // uint8_t 转换为16进制字符串需要2个字符 + '\0'
	//  char data2_str[3]; // uint8_t 转换为16进制字符串需要2个字符 + '\0'
	//  char data_str[3]; // uint8_t 转换为16进制字符串需要2个字符 + '\0'
	//  //char data_str[5];         // uint16_t 转换为16进制字符串需要4个字符 + '\0'

	// // 将数值转换为16进制字符串
	// snprintf(cmd_head_str, sizeof(cmd_head_str), "%08X", cmd_head);
	// snprintf(cmd_type_str, sizeof(cmd_type_str), "%04X", cmd_type);
	// snprintf(control_type_str, sizeof(control_type_str), "%02X", control_type);
	// snprintf(data2_str, sizeof(data2_str), "%02X", data2);
	// snprintf(data_str, sizeof(data_str), "%02X", data);
	// //snprintf(data_str, sizeof(data_str), "%04X", data);

	// // 打印转换后的字符串
	// printf("cmd_head: %s, cmd_type: %s, control_type: %s,data2: %s, data: %s\n",
	//        cmd_head_str, cmd_type_str, control_type_str,data2_str, data_str);

	// uint8_t hexValue = 0xAB;
	// printf("Hex value: %x\n", hexValue);
	// uint8_t character = 'A';
	// printf("Character: %c\n", character);
	// uint8_t value = 123;
	// printf("Value: %u\n", (unsigned int)value);

	switch (cmd_type)
	{
		/*热敷开始*/

	case 0x1041:

		HeatPIDInit(37.0);
		// TMP114_Init();
		xEventGroupSetBits(All_EventHandle, xBitsToSet); // 设定热敷任务开启标志位
		HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);		 // enable pwm for heating film
		break;

	/*热敷结束*/
	case 0x1030:                                      
		HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);		   // enable pwm for heating film
		xEventGroupClearBits(All_EventHandle, xBitsToSet|SW_BIT_1); // 清除热敷任务开启标志位
		//xQueueReset(Temperature_QueueHandle);
		ScreenWorkModeQuit(0x03);
		break;

	/*脉动开始*/
	case 0x1005:
		data = data / 80; // 设定压力
						  // HAL_UART_Transmit(&huart1, (uint8_t *)&data, sizeof(uint16_t), 0xFFFF);
		ForceRawSet = data * HX711_SCALE_FACTOR;
		MotorCompareState = 0;
		xEventGroupSetBits(All_EventHandle, xBitsToSet1); // 设定脉动任务开启标志位
		break;

	/*脉动结束*/
	case 0x1034:
		ScreenWorkModeQuit(0x07);
		xEventGroupClearBits(All_EventHandle, xBitsToSet1); // 清除脉动任务开启标志位
		xEventGroupClearBits(All_EventHandle, SW_BIT_1);
		HAL_TIM_Base_Stop_IT(&htim7);
		xEventGroupSetBits(All_EventHandle, Reset_Motor_BIT_4); 
		//xQueueReset(Force_QueueHandle);
		break;

	/*自动模式开始*/
	case 0x1037:
		HeatPIDInit(37.0);
		data = data / 80; // 设定压力
		 // HAL_UART_Transmit(&huart1, (uint8_t *)&data, sizeof(uint16_t), 0xFFFF);
		ForceRawSet = data * HX711_SCALE_FACTOR;
		// WorkMode = 0x02;
		MotorCompareState = 0;
		// HeatPIDInit();
		// TMP114_Init();
		xEventGroupSetBits(All_EventHandle, Auto_BIT_3); // 设定自动任务开启标志位
		HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);		 // enable pwm for heating film
		break;

	/*自动模式结束*/
	case 0x1038:
		ScreenWorkModeQuit(0x0C);
		xEventGroupClearBits(All_EventHandle, Auto_BIT_3|SW_BIT_1); // 清除脉动任务开启标志位
		xEventGroupSetBits(All_EventHandle, Reset_Motor_BIT_4); 
		HAL_TIM_Base_Stop_IT(&htim7);
		HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1); // disable pwm for heating film
		//xQueueReset(Temperature_QueueHandle);
		break;
	default:
		break;
	}
}

void ProcessTemperatureData(uint16_t work_mode)
{
	if (xQueueReceive(Temperature_QueueHandle, &temperature, 10))
	{temperature=temperature;
		ScreenUpdateTemperature(temperature, work_mode);
		temperature_buffer[buffer_index] = temperature;
		buffer_index = (buffer_index + 1) % FILTER_SIZE_TEMP;
		if (buffer_index == 0)
		{
			float filtered_temperature = processFilter(temperature_buffer);
			//printf("Temperature:%f,%f\n",filtered_temperature,42.5);
			//ScreenUpdateTemperature(filtered_temperature, work_mode);                                                  
			//ScreenUpdateTemperature(filtered_temperature, 0x0302);
		}
	}
}
void ProcessForceData(uint16_t work_mode)
{
	if (xQueueReceive(Force_QueueHandle, &Force, 10))
	{
        ScreenUpdateForce(Force, work_mode);
//		Force_buffer[buffer_index_Force] = Force;
//		buffer_index_Force = (buffer_index_Force + 1) % FILTER_SIZE;
//		if (buffer_index_Force == 0)
//		{
//			uint32_t filtered_Force = processFilter_force(Force_buffer);
//			ScreenUpdateForce(filtered_Force, work_mode);
//			//ScreenUpdateForce(filtered_Force, 0x0702);
//		}
	}
}

float processFilter(float *buffer)
{
	float sum = 0;
	for (int i = 0; i < FILTER_SIZE_TEMP; i++)
	{
		sum += buffer[i];
	}
	return sum / FILTER_SIZE_TEMP; // 这里使用简单的平均值滤波
}
uint32_t processFilter_force(uint32_t *buffer)
{
	// 对数组进行排序
	for (int i = 0; i < FILTER_SIZE - 1; i++)
	{
		for (int j = 0; j < FILTER_SIZE - i - 1; j++)
		{
			if (buffer[j] > buffer[j + 1])
			{
				// 交换 buffer[j] 和 buffer[j + 1]
				uint32_t temp = buffer[j];
				buffer[j] = buffer[j + 1];
				buffer[j + 1] = temp;
			}
		}
	}

	// 如果 FILTER_SIZE 是奇数，返回中间的值
	// 如果是偶数，返回中间两个值的平均值
	if (FILTER_SIZE % 2 != 0)
	{
		//Limit(buffer[FILTER_SIZE / 2],min,max) 
		return buffer[FILTER_SIZE / 2];
	}
	else
	{
		return (buffer[(FILTER_SIZE - 1) / 2] + buffer[FILTER_SIZE / 2]) / 2;
	}
}

void ScreenUpdateTemperature(float value, uint16_t work_mode)
{
	uint16_t Tmpvalue;

	Tmpvalue = value + 0.5f;
	while (hdma_usart1_tx.State != HAL_DMA_STATE_READY)
		;
	SendBuff[4] = (work_mode >> 8) & 0xFF; // 高字节
	SendBuff[6] = work_mode & 0xFF;		   // 低字节

	SendBuff[0] = 0xEE;
	SendBuff[1] = 0xB1;
	SendBuff[2] = 0x10;
	SendBuff[3] = 0x00;

	SendBuff[5] = 0x00;

	SendBuff[7] = Tmpvalue / 10 + '0';
	SendBuff[8] = Tmpvalue % 10 + '0';
	SendBuff[9] = 0xFF;
	SendBuff[10] = 0xFC;
	SendBuff[11] = 0xFF;
	SendBuff[12] = 0xFF;

	HAL_UART_Transmit_DMA(&huart1, SendBuff, 13);
}

void ScreenUpdateForce(uint32_t value, uint16_t work_mode)
{
	// printf("%u",value);

	uint16_t Forcevalue = (uint16_t)(value / HX711_SCALE_FACTOR * 75);

	while (hdma_usart1_tx.State != HAL_DMA_STATE_READY)
		;
	SendBuff[4] = (work_mode >> 8) & 0xFF; // 高字节
	SendBuff[6] = work_mode & 0xFF;		   // 低字节
	SendBuff[0] = 0xEE;
	SendBuff[1] = 0xB1;
	SendBuff[2] = 0x10;
	SendBuff[3] = 0x00;

	SendBuff[5] = 0x00;

	SendBuff[7] = Forcevalue / 100 + '0';
	SendBuff[8] = Forcevalue / 10 % 10 + '0';
	SendBuff[9] = Forcevalue % 10 + '0';
	SendBuff[10] = 0xFF;
	SendBuff[11] = 0xFC;
	SendBuff[12] = 0xFF;
	SendBuff[13] = 0xFF;
	HAL_UART_Transmit_DMA(&huart1, SendBuff, 14);
}

void ScreenUpdateSOC(uint16_t value, uint8_t state)
{
	uint8_t SOCvalue;
	SOCvalue = value / 20;
	if (SOCvalue == 5)
		SOCvalue = 4;
	if (state == 1 || state == 2)
		SOCvalue += 5;
	while (hdma_usart1_tx.State != HAL_DMA_STATE_READY)
		;
	SendBuff[0] = 0xEE;
	SendBuff[1] = 0xB1;
	SendBuff[2] = 0x23;
	SendBuff[3] = 0x00;
	SendBuff[4] = 0x1E;
	SendBuff[5] = 0x27;
	SendBuff[6] = 0x15;
	SendBuff[7] = SOCvalue;
	SendBuff[8] = 0xFF;
	SendBuff[9] = 0xFC;
	SendBuff[10] = 0xFF;
	SendBuff[11] = 0xFF;
HAL_UART_Transmit_DMA(&huart1, SendBuff, 12);
    vTaskDelay(20 / portTICK_PERIOD_MS);
//	if (value == 100)
//	{
//		SendBuff2[0] = 0xEE;
//		SendBuff2[1] = 0xB1;
//		SendBuff2[2] = 0x10;
//		SendBuff2[3] = 0x00;
//		SendBuff2[4] = 0x01;
//		SendBuff2[5] = 0x00;
//		SendBuff2[6] = 0x04;
//		SendBuff2[7] = value / 100 + '0';
//		SendBuff2[8] = value / 10 % 10 + '0';
//		SendBuff2[9] = value % 10 + '0';
//
//		SendBuff2[10] = 0xFF;
//		SendBuff2[11] = 0xFC;
//		SendBuff2[12] = 0xFF;
//		SendBuff2[13] = 0xFF;
//		HAL_UART_Transmit_DMA(&huart1, SendBuff2, 14);
//	}
//	if (value < 100 && value > 9)
//	{
//		SendBuff2[0] = 0xEE;
//		SendBuff2[1] = 0xB1;
//		SendBuff2[2] = 0x10;
//		SendBuff2[3] = 0x00;
//		SendBuff2[4] = 0x01;
//		SendBuff2[5] = 0x00;
//		SendBuff2[6] = 0x04;
//		SendBuff2[7] = value / 10 % 10 + '0';
//		SendBuff2[8] = value % 10 + '0';
//
//		SendBuff2[9] = 0xFF;
//		SendBuff2[10] = 0xFC;
//		SendBuff2[11] = 0xFF;
//		SendBuff2[12] = 0xFF;
//		HAL_UART_Transmit_DMA(&huart1, SendBuff2, 13);
//	}
//	if (value < 10)
//	{
//		SendBuff2[0] = 0xEE;
//		SendBuff2[1] = 0xB1;
//		SendBuff2[2] = 0x10;
//		SendBuff2[3] = 0x00;
//		SendBuff2[4] = 0x01;
//		SendBuff2[5] = 0x00;
//		SendBuff2[6] = 0x04;
//		SendBuff2[7] = value % 10 + '0';
//
//		SendBuff2[8] = 0xFF;
//		SendBuff2[9] = 0xFC;
//		SendBuff2[10] = 0xFF;
//		SendBuff2[11] = 0xFF;
//		HAL_UART_Transmit_DMA(&huart1, SendBuff2, 12);
//	}

	
}

void ScreenWorkModeQuit(uint8_t workmodenumber)
{
	while (hdma_usart1_tx.State != HAL_DMA_STATE_READY)
		;
	SendBuff[0] = 0xEE;
	SendBuff[1] = 0xB1;
	SendBuff[2] = 0x10;
	SendBuff[3] = 0x00;
	SendBuff[4] = workmodenumber;
	SendBuff[5] = 0x00;
	SendBuff[6] = 0x07;
	SendBuff[7] = 0x32;
	SendBuff[8] = 0xFF;
	SendBuff[9] = 0xFC;
	SendBuff[10] = 0xFF;
	SendBuff[11] = 0xFF;

	HAL_UART_Transmit_DMA(&huart1, SendBuff, 12);
}

void ScreenTimerStart(uint8_t workmodenumber)
{
	while (hdma_usart1_tx.State != HAL_DMA_STATE_READY)
		;
	SendBuff[0] = 0xEE;
	SendBuff[1] = 0xB1;
	SendBuff[2] = 0x10;
	SendBuff[3] = 0x00;
	SendBuff[4] = workmodenumber;
	SendBuff[5] = 0x00;
	SendBuff[6] = 0x07;
	SendBuff[7] = 0x31;
	SendBuff[8] = 0xFF;
	SendBuff[9] = 0xFC;
	SendBuff[10] = 0xFF;
	SendBuff[11] = 0xFF;

	HAL_UART_Transmit_DMA(&huart1, SendBuff, 12);
}
