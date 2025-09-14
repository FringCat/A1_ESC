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
    float beta[3];//���׹۲�������������,�ɸ�

    float b ;
    float u;//������
    float z[3]; //����ESO�۲���,��0��2����Ϊz1z2z3,z3Ϊ�Ŷ�

}ESO_t;

typedef struct 
{
    float r; //TD�ƽ��ٶ�
    float h; //����

    float v[2];//��v0���й��ɺ�΢���Ժ�õ������
}TD_t;

typedef struct 
{
    float a1;
    float a2;
    float r1;
    float r2;
    float k1;
    float k2;

    float output;//���
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
