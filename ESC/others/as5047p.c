#include "as5047p.h"

extern SPI_HandleTypeDef hspi1;
float angle_1=0,angle_2=0;
#define SPI_CS_L  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,GPIO_PIN_RESET)      //CS_L
#define SPI_CS_H  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,GPIO_PIN_SET)         //CS_H
#define AS5407P_CS_L SPI_CS_L
#define AS5407P_CS_H SPI_CS_H

// 计算奇偶函数
uint16_t Parity_bit_Calculate(uint16_t data_2_cal)
{
	uint16_t parity_bit_value=0;
	while(data_2_cal != 0)
	{
		parity_bit_value ^= data_2_cal; 
		data_2_cal >>=1;
	}
	return (parity_bit_value & 0x1); 
}
// SPI发送读取函数
uint16_t SPI_ReadWrite_OneByte(uint16_t _txdata)
{
	 AS5407P_CS_L;	//cs拉低
	uint16_t rxdata;
	HAL_SPI_TransmitReceive(&hspi1,(uint8_t*)&_txdata,(uint8_t*)&rxdata,1,0xffff);
  	 AS5407P_CS_H;	//cs拉高
	return rxdata;
}
uint16_t AS5047_read(uint16_t add)
{
	uint16_t data;
	add |= 0x4000;	//读指令 bit14 置1
	if(Parity_bit_Calculate(add)==1) add=add|0x8000; //如果前15位 1的个数位偶数，则Bit15 置1
	// AS5407P_CS_L;
	SPI_ReadWrite_OneByte(add);		//发送一条指令，不管读回的数据
	// HAL_SPI_Transmit(&hspi1,(uint8_t*)&add,1,0xffff);
	data=SPI_ReadWrite_OneByte(0x0000|0x4000); //发送一条空指令，读取上一次指令返回的数据。
	// HAL_SPI_Receive(&hspi1,(uint8_t*)&data,1,0xffff);
	// AS5407P_CS_H;
	data &=0x3fff;
	
	return data;
}

float GetAngle(void)
{ 
	float angle;
	angle = (float)AS5047_read(0x3FFE)/16383.0f*2.0f*3.1415926f;

	return angle;
}
 
uint32_t AS5047P_GetAngle(void)
{
	return AS5047_read(0x3FFE);
}
