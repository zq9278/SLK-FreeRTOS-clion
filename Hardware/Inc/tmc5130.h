//#ifndef __TMC5130_H
//#define __TMC5130_H
#include "main.h"






#define Positive					1
#define Negative					2
#define ForceSen	16907


void TMC5130_Init(void);
void TMC5130_Read(uint8_t ReadAddr,uint8_t* pBuffer);
void TMC5130_Write(uint8_t WriteAddr,uint32_t WriteData);
void MotorCtrl(int32_t Step);
void StepMinMax(int32_t *Step,int32_t MinValue,int32_t MaxValue);
void MotorSetHome(void);
uint8_t MotorChecking(void);
void VelocityModeMove(uint8_t direction);
void MotorMove(uint8_t DriverID,int16_t ADCValue,uint16_t absADCValue,uint16_t ADCThreshold,uint16_t MotorSpeed);
uint8_t MotorCompare(int32_t SetData,int32_t CompareData);
		 
//#endif

