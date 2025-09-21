#ifndef  ADRC_H
#define ADRC_H

#include "main.h"
#include "adc.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <math.h>
#include <stdio.h>
#include "FOC.h"


typedef struct 
{
    float w0;
    float beta[3];//三阶观测器有三个增益,可改

    float b ;
    float u;//控制律
    float z[3]; //三阶ESO观测结果,从0到2依次为z1z2z3,z3为扰动

}ESO_t;

typedef struct 
{
    float r; //TD逼近速度
    float h; //带宽

    float v[2];//对v0进行过渡和微分以后得到的输出
}TD_t;

typedef struct 
{
    float a1;
    float a2;
    float r1;
    float r2;
    float k1;
    float k2;

    float output;//输出
}NLSEF_t;

typedef struct 
{
    ESO_t ESO;
    TD_t TD;
    NLSEF_t NLSEF;
}ADRC_t;

int factorial(int n);
void ADRC_Init(ADRC_t* ADRC);
void ESO(ADRC_t* ADRC,float y);
float fhan1(float e1 , float* e2,float r ,float h,int order);
void TD(ADRC_t* ADRC,float v0);
float fal(float e,float a,float r);
float NLSEF(ADRC_t* ADRC);
float NLSEF_I(ADRC_t* ADRC);
#endif // ! 
