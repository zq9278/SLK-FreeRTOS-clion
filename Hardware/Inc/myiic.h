#ifndef _MYIIC_H
#define _MYIIC_H
#include "main.h"
	
//IO方向设置
#define SDA_IN()  {GPIOA->MODER&=~(GPIO_MODER_MODE0<<(8*2));GPIOA->MODER|=0<<8*2;}	//PA8输入模式
#define SDA_OUT() {GPIOA->MODER&=~(GPIO_MODER_MODE0<<(8*2));GPIOA->MODER|=1<<8*2;} //PA8输出模式
//IO操作
#define IIC_SCL(n)  (n?HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET)) //SCL
#define IIC_SDA(n)  (n?HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET)) //SDA
#define WP(n)  (n?HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET)) //SDA
#define READ_SDA    HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)  //输入SDA

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	 
void delay_us(uint32_t nus);
#endif

