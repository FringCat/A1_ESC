#ifndef __FOC_H
#define __FOC_H

#include "main.h"
#include "adc.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <math.h>
#include <stdio.h>
typedef struct
{
	uint32_t ticks;
    uint32_t ThisTime;
    uint32_t PastTime;
    double dt ;
}Time_t;
typedef struct
{
	float KP,KI,This_I,Last_I,KD;
	float error,Output,last_error;
}PID_t;

typedef void (*Function_SetPWM)(uint16_t,uint16_t,uint16_t);
typedef void (*Function_Delayms)(uint16_t);
typedef double (*Function_Getdt)(Time_t*);
typedef float (*Function_GetAngle)(void);
typedef void (*Function_GetI)(void);

typedef struct
{
	Time_t time ;
	uint16_t IA_raw,IB_raw,IC_raw;
	int motor_number;
	double IA,IB,IC;
	double Ialpha,Ibeta;
	double Iq,Id;
	double IA_offset,IB_offset,IC_offset;
	float angle,angle_el,angle_all,angle_el_zero;
	float LastAngle,LastVelocity,Velocity,Velocity_raw,T;
	float UA,UB,UC;
	float Ualpha,Ubeta;
	float Uq,Ub;

	float DIR;
	float Pole_pairs;
	PID_t PID_param[5];//前两组为电机速度环，电流环参数，后三组为三环PID

	Function_SetPWM SetPWM;
	Function_Delayms Delayms;
	Function_Getdt Getdt;
	Function_GetAngle GetAngle;
	Function_GetI GetIaIbIc;
}Motor_t;
void FOCparamInit(Motor_t *motor);
void FOC_init(void);
double Getdt(Time_t* time);
void FOC_SetPWM_1(uint16_t a,uint16_t b,uint16_t c);
void FOC_SetPWM_2(uint16_t a,uint16_t b,uint16_t c);
void FOC_Delayms(uint16_t ms);
float FOC_GetAngle_Sensor_2(void);
float FOC_GetAngle_Sensor_1(void);
void SetMotor_Velocity(float Velocity_motor1);
void MotorStart_Mult(void);
void MotorStart(Motor_t *motor);
void SVPWM__(Motor_t* motor,int pwm_duty,float angle);

float GetVelocity_(Motor_t* motor);
float PID_Iq(Motor_t* motor,float TargetIq);
float sgn(float x);
float Sat(float s, float delta);
float myabs(float val);
float mymap( float Data ,float formLOW,float formHIGH, float toLOW,float toHIGH);
float Limit_ElectricalAngle(float ElectricalAngle);
float GetElectricalAngle(Motor_t *motor);
float GetElectricalAngle_(float DIR,float Pole_pairs,float angle);
float Limit(float value , float high , float low);
float *Park_N(float Uq , float ElectricalAngle);
float *Park_N2(float Uq , float Ud , float ElectricalAngle);
float *Clark_N(float Ualpha ,float Ubeta);
void SPWM(Motor_t *motor,float Ua , float Ub ,float Uc);
void FOC_SPWM(Motor_t *motor,float Uq,float angle);
int GetSector( float Ualpha , float Ubeta );
float LowPassFilter(float Last_Output , float Input , float alpha);
float GetVelocity(Motor_t *motor);
float  PID_velocity(Motor_t *motor,float TargetVelocity);
void SVPWM(Motor_t *motor,float Uq,float angle);
void SVPWM2(Motor_t *motor,float Uq,float Ud,float angle);
void VF_Start(Motor_t *motor,float TargetVelocity,float acc ,float VF_uq_delta);
float* PLL(float Valpha, float Vbeta ,float Ts);
void FOC_SPWM_(Motor_t* motor,int pwm_duty,float angle);
#endif
