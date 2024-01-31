#ifndef __BQ27441_H
#define __BQ27441_H
#include "main.h"




typedef struct
{
  uint16_t Voltage;
	uint16_t Temperature;
	int16_t AvgCurrent;
	uint16_t SOC;
	uint16_t FullChargeCapacity;
}BQ27441_typedef;


uint8_t BQ27441_Init(void);
void BQ27441_Read(uint8_t ReadAddr,uint8_t* pBuffer);
void BQ27441_MultiRead(BQ27441_typedef *BQ_State);
void BQ27441_WriteByte(uint8_t WriteAddr,uint8_t WriteData);
void BQ27441_WriteWord(uint8_t WriteAddr,uint16_t WriteData);


		 
#endif

