#include "main.h"
#ifndef __TMP114_H
#define __TMP114_H

void TMP114_Init(void);
void TMP114_Read(uint8_t ReadAddr,uint8_t* pBuffer);
void TMP114_MultiRead(uint8_t* pBuffer);
void TMP114_WriteByte(uint8_t WriteAddr,uint8_t WriteData);
void TMP114_WriteWord(uint8_t WriteAddr,uint16_t WriteData);
float TmpRaw2Ture(uint8_t* TmpRawData);


		 
#endif

