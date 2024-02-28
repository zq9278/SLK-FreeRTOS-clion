#include "tmp114.h"





uint8_t EyeTmpRaw[2];
int16_t TmpData;
float EyeTmp;

extern I2C_HandleTypeDef hi2c2;

//#define TMP114Address		0x92

void TMP114_Init(void)
{ 
	
}  

void TMP114_Read(uint8_t ReadAddr,uint8_t* pBuffer)   
{ 	//HAL_I2C_Master_Transmit(&hi2c2, TMP114Address,&ReadAddr, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read_DMA(&hi2c2, TMP114Address, ReadAddr,I2C_MEMADD_SIZE_8BIT, pBuffer,2);
	//HAL_I2C_Master_Receive(&hi2c2, TMP114Address, pBuffer, 2, HAL_MAX_DELAY);
}  

void TMP114_MultiRead(uint8_t* pBuffer)   
{ 	
	HAL_I2C_Mem_Read_DMA(&hi2c2, TMP114Address, 0x00,I2C_MEMADD_SIZE_8BIT, pBuffer,0x14);
}  
 
void TMP114_WriteByte(uint8_t WriteAddr,uint8_t WriteData)
{
	uint8_t Data[1];
	Data[0]=WriteData;
	HAL_I2C_Mem_Write_DMA(&hi2c2, TMP114Address, WriteAddr,I2C_MEMADD_SIZE_8BIT, Data, 1);
} 

void TMP114_WriteWord(uint8_t WriteAddr,uint16_t WriteData)
{
	uint8_t Data[2];
	Data[0]=WriteData;
	Data[1]=WriteData>>8;
	HAL_I2C_Mem_Write_DMA(&hi2c2, TMP114Address, WriteAddr,I2C_MEMADD_SIZE_8BIT, Data, 2);
	vTaskDelay(2);
} 

float TmpRaw2Ture(uint8_t* TmpRawData)
{
	TmpData=(TmpRawData[0]<<8) | TmpRawData[1];
	TmpData = TmpData >> 4;
	//return TmpData*0.0625;
	return (int)(TmpData * 0.0625);

//	 TmpData=(TmpRawData[0]<<8) | TmpRawData[1];
//	 return TmpData*0.0078125;//lao



	
}
