#ifndef _AS5047P_H
#define _AS5047P_H

#include "main.h"
#include "adc.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

extern float angle_1,angle_2;
uint16_t Parity_bit_Calculate(uint16_t data_2_cal);
uint16_t SPI_ReadWrite_OneByte(uint16_t _txdata);
uint16_t AS5047_read(uint16_t add);
float GetAngle(void);

#endif
