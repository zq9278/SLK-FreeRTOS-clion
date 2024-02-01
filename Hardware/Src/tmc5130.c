
#include "tmc5130.h"



/*motor*/
uint32_t MotorSpeed=0x4000;

/*motor*/

extern SPI_HandleTypeDef hspi1;
uint8_t tempa[4];



void TMC5130_Init(void)
{ 
//	TMC_ENN(0);//���ʹ��
	//delay_ms(200);
    vTaskDelay(200/ portTICK_PERIOD_MS);

	TMC5130_Write(0x81,0x00000001);//reset
	TMC5130_Write(0xec,0x000300c3);// CHOPCONF: vsense=1,TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadCycle)
	TMC5130_Write(0x90,0x00001000);// IHOLD_IRUN: IHOLD=10, IRUN=31 (max. current), IHOLDDELAY=6
	TMC5130_Write(0x91,0x0000000a);// TPOWERDOWN=10: Delay before power down in stand still
	TMC5130_Write(0x80,0x00000004);// EN_PWM_MODE=1 enables stealthChop (with default PWM_CONF)
	TMC5130_Write(0x93,0x000001f4);// TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM
	TMC5130_Write(0xf0,0x000701c8);// PWM_CONF: AUTO=1, 2/1024 Fclk, Switch amplitude limit=200, Grad=1
	TMC5130_Write(0xa4,0x00011000);// A1 = 1 000 First acceleration
	TMC5130_Write(0xa5,0x00015000);// V1 = 50 000 Acceleration threshold velocity V1
	TMC5130_Write(0xa6,0x00018fff);// AMAX = 500 Acceleration above V1
	TMC5130_Write(0xa7,MotorSpeed);// VMAX = 200 000
	TMC5130_Write(0xa8,0x00001fff);// DMAX = 700 Deceleration above V1
	TMC5130_Write(0xaa,0x00008000);// D1 = 1400 Deceleration below V1
	TMC5130_Write(0xab,0x0000000a);// VSTOP = 10 Stop velocity (Near to zero)
	TMC5130_Write(0xac,0x00000000);
	TMC5130_Write(0xb4,0x0000075f);
	TMC5130_Write(0xa0,0x00000000);//λ��ģʽ
	
	
}  


void TMC5130_Read(uint8_t ReadAddr,uint8_t* pBuffer)   
{ 	
	uint8_t Data[5];
	Data[0]=ReadAddr;
	//TMC_CSN(0);
    HAL_GPIO_WritePin(TMC_CSN_GPIO_Port,TMC_CSN_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, Data, 5, 100);
	//HAL_GPIO_WritePin(TMC_CSN_GPIO_Port,TMC_CSN_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(TMC_CSN_GPIO_Port,TMC_CSN_Pin,GPIO_PIN_SET);
	//__nop();
	//__nop();
    HAL_GPIO_WritePin(TMC_CSN_GPIO_Port,TMC_CSN_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, Data, 1, 100);
	HAL_SPI_Receive(&hspi1, pBuffer,4, 100);
	HAL_GPIO_WritePin(TMC_CSN_GPIO_Port,TMC_CSN_Pin,GPIO_PIN_SET);
}  
 
void TMC5130_Write(uint8_t WriteAddr,uint32_t WriteData)
{
	uint8_t Data[5];
	Data[0]=WriteAddr;
	Data[1]=(uint8_t)((WriteData)>>24);
	Data[2]=(uint8_t)((WriteData)>>16);
	Data[3]=(uint8_t)((WriteData)>>8);
	Data[4]=(uint8_t)WriteData;
	HAL_GPIO_WritePin(TMC_CSN_GPIO_Port,TMC_CSN_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, Data, 5, 100);
	HAL_GPIO_WritePin(TMC_CSN_GPIO_Port,TMC_CSN_Pin,GPIO_PIN_SET);
} 

void MotorSetHome(void)
{
	TMC5130_Write(0xa1,0);
	TMC5130_Write(0xad,0);
}

void MotorCtrl(int32_t Step)
{
	uint8_t Data[5];
	Data[0]=0xad;
	Data[1]=(uint8_t)((Step)>>24);
	Data[2]=(uint8_t)((Step)>>16);
	Data[3]=(uint8_t)((Step)>>8);
	Data[4]=(uint8_t)Step;
	HAL_GPIO_WritePin(TMC_CSN_GPIO_Port,TMC_CSN_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, Data, 5, 100);
	HAL_GPIO_WritePin(TMC_CSN_GPIO_Port,TMC_CSN_Pin,GPIO_PIN_SET);
}

void VelocityModeMove(uint8_t direction)
{
	uint8_t Data[5];
	Data[0]=0xa0;
	Data[1]=0x00;
	Data[2]=0x00;
	Data[3]=0x00;
	Data[4]=direction;
	HAL_GPIO_WritePin(TMC_CSN_GPIO_Port,TMC_CSN_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, Data, 5, 100);
	HAL_GPIO_WritePin(TMC_CSN_GPIO_Port,TMC_CSN_Pin,GPIO_PIN_SET);
}

void StepMinMax(int32_t *Step,int32_t MinValue,int32_t MaxValue)
{
	if(*Step<MinValue)
	{
		*Step=MinValue;
	}
	if(*Step>MaxValue)
	{
		*Step=MaxValue;
	}
}

uint8_t MotorChecking()
{
	uint32_t ReadCount=0;
	uint8_t ReadData[4];
	
    HAL_GPIO_WritePin(TMC_ENN_GPIO_Port,TMC_ENN_Pin,GPIO_PIN_RESET);
	TMC5130_Write(0xa7,0x10000);
	VelocityModeMove(Negative);
	do
	{
		TMC5130_Read(0x04,ReadData);//读取位置寄存器
		if((ReadData[3]&0x02)==0x00)//如果位置寄存器被设置
		{
			TMC5130_Read(0x04,ReadData);//继续读位置寄存器
		}
		 vTaskDelay(1 / portTICK_PERIOD_MS);
		ReadCount++;
		if(ReadCount==65535)
		{
			TMC5130_Write(0xa7,0);
			return 0;
		}
	}while((ReadData[3]&0x02)==0x02);//如果位置寄存器没被设置

	MotorSetHome();
	TMC5130_Write(0xa0,0x00000000);//λ��ģʽ
    HAL_GPIO_WritePin(TMC_ENN_GPIO_Port,TMC_ENN_Pin,GPIO_PIN_SET);
	return 1;
}

//	else
//	{
//		VelocityModeMove(DriverID,Positive);
//		do
//		{
//			TMC5130_Read(DriverID,0x04,MotorStatus);
//			if((MotorStatus[3]&0x01)==0x00)
//			{
//				TMC5130_Read(DriverID,0x04,MotorStatus);
//			}
//			delay_us(400);
//			ReadCount++;
//			if(ReadCount==65535)
//			{
//				TMC5130_Write(DriverID,0xa7,0);
//				return 0;
//			}
//		}while((MotorStatus[3]&0x01)==0x01);
//	}

uint8_t MotorCompare(int32_t SetData,int32_t CompareData)
{
	int32_t SubData;
	SubData=CompareData-SetData;
	if(SubData>ForceSen)
	{
		TMC5130_Write(0xa7,MotorSpeed);
		TMC5130_Write(0xa0,2);
		//vTaskDelay(200);
		return 2;
	}
	else if(SubData<-ForceSen)
	{
		TMC5130_Write(0xa7,MotorSpeed);
		TMC5130_Write(0xa0,1);
		
		return 1;
	}
	else
	{
		TMC5130_Write(0xa7,0);
		TMC5130_Write(0xa0,0);
		return 0;
	}
}

// void MotorCompare(int32_t SetData,int32_t CompareData)
// {
// 	int32_t SubData;
// 	SubData=CompareData-SetData;
// 	if(SubData>ForceSen)
// 	{
// 		TMC5130_Write(0xa7,MotorSpeed);
// 		TMC5130_Write(0xa0,2);
// 		vTaskDelay(100);
		
// 	}
// 	else if(SubData<-ForceSen)
// 	{
// 		TMC5130_Write(0xa7,MotorSpeed);
// 		TMC5130_Write(0xa0,1);
// 		vTaskDelay(100);
		
// 	}
// 	else
// 	{
// 		TMC5130_Write(0xa7,0);
// 		TMC5130_Write(0xa0,0);
		
// 	}
// }
