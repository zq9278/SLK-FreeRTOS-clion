#ifndef __BQ25895_H
#define __BQ25895_H
#include "main.h"




void BQ25895_Init(void);
void BQ25895_Read(uint8_t ReadAddr,uint8_t* pBuffer);
void BQ25895_MultiRead(uint8_t* pBuffer);
void BQ25895_Write(uint8_t WriteAddr,uint8_t WriteData);
void PowerStateUpdate(void);


		 
#endif

