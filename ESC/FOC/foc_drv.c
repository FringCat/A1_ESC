#include "main.h"
#include "adc.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "as5047p.h"  
#include "foc_alg.h"
#include "foc_drv.h"

void stm32_set_pwm_A(float Ua)
{
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(uint16_t)(Ua*A_PWM_Period));
}

void stm32_set_pwm_B(float Ub)
{
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(uint16_t)(Ub*B_PWM_Period));
}

void stm32_set_pwm_C(float Uc)
{
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,(uint16_t)(Uc*C_PWM_Period));
}

uint32_t stm32_update_Ia_raw(void)
{
    return HAL_ADC_GetValue(&hadc1);
}

uint32_t stm32_update_Ib_raw(void)
{
    return HAL_ADC_GetValue(&hadc1);
}

uint32_t stm32_update_Ic_raw(void)
{
    return HAL_ADC_GetValue(&hadc1);
}

float stm32_cal_Ia(uint32_t raw,float offset)
{
    return raw*I_ADC_CONV-offset; 
}

float stm32_cal_Ib(uint32_t raw,float offset)
{
    return raw*I_ADC_CONV-offset; 
}

float stm32_cal_Ic(uint32_t raw,float offset)
{
    return raw*I_ADC_CONV-offset; 
}

uint32_t stm32_update_angle_raw(void)
{
    return AS5047P_GetAngle();
}

float stm32_cal_angle(uint32_t raw)
{
    return (float)(raw)/16383.0f*2.0f*3.1415926f; // 0~2PI
}

void stm32_delayms(uint32_t ms)
{
    HAL_Delay(ms);
}

float stm32_update_dt(Time_t* time)
{
    time->PastTime = time->ThisTime;
    time->ThisTime =__HAL_TIM_GET_COUNTER(&htim3);
    if(time->ThisTime > time->PastTime)
    {
        time->dt = (float)(time->ThisTime - time->PastTime)/(170000000.0f/300.0f);
    }
    else
    {
        time->dt = (float)(65535 - time->PastTime + time->ThisTime)/(170000000.0f/300.0f);
    }
    // if(time->dt == 0)
    // {
    //     time->dt = 0.0001f;
    // }
	return  time->dt ;
}

void foc_init(Motor_HandleTypeDef *motor)
{
    // 初始化FOC算法相关参数和结构体
    memset(motor, 0, sizeof(Motor_HandleTypeDef));
    motor->motor_number = 1; // 设置电机编号

    // 初始化时间管理
    motor->time.ThisTime = 0.0f;
    motor->time.PastTime = 0.0f;
    motor->time.dt = 0.01f;

    //初始化电机配置
    motor->MotorConfig.Pole_pairs = 7; // 设置电机极对数
    motor->MotorConfig.DIR = 1;        // 设置电机转向
    motor->MotorConfig.IMAX = 10.0f;   // 设置电流限幅
    motor->MotorConfig.UMAX = 24.0f;   // 设置电压限幅
    // motor->MotorConfig.Ls = 0.001f;    // 设置定子电感
    // motor->MotorConfig.Rs = 0.5f;      // 设置定子电阻
    motor->MotorConfig.angle_zero = 0.0f; // 设置机械零点
    motor->MotorConfig.angle_el_zero = 0.0f; // 设置电角度零点

    // 初始化PID参数(要用哪个初始化哪个)
    motor->MotorAlg.position_pid.KP = 0.1f;
    motor->MotorAlg.position_pid.KI = 0.01f;
    motor->MotorAlg.position_pid.KD = 0.0f;
    motor->MotorAlg.position_pid.integral_limit = 10.0f;
    motor->MotorAlg.position_pid.output_limit = 100.0f;

    motor->MotorAlg.velocity_pid.KP = 0.1f;
    motor->MotorAlg.velocity_pid.KI = 0.01f;
    motor->MotorAlg.velocity_pid.KD = 0.0f;
    motor->MotorAlg.velocity_pid.integral_limit = 10.0f;
    motor->MotorAlg.velocity_pid.output_limit = 100.0f;

    motor->MotorAlg.id_pid.KP = 0.2f;
    motor->MotorAlg.id_pid.KI = 0.02f;
    motor->MotorAlg.id_pid.KD = 0.0f;
    motor->MotorAlg.id_pid.integral_limit = 10.0f;
    motor->MotorAlg.id_pid.output_limit = motor->MotorConfig.UMAX;

    motor->MotorAlg.iq_pid.KP = 0.2f;
    motor->MotorAlg.iq_pid.KI = 0.02f;
    motor->MotorAlg.iq_pid.KD = 0.0f;
    motor->MotorAlg.iq_pid.integral_limit = 10.0f;
    motor->MotorAlg.iq_pid.output_limit = motor->MotorConfig.UMAX;

    motor->MotorAlg.mixed_pid.KP = 0.1f;
    motor->MotorAlg.mixed_pid.KI = 0.01f;
    motor->MotorAlg.mixed_pid.KD = 0.0f;
    motor->MotorAlg.mixed_pid.integral_limit = 10.0f;
    motor->MotorAlg.mixed_pid.output_limit = 100.0f;

    // motor->MotorData.IA_offset = 0.0f;
    // motor->MotorData.IB_offset = 0.0f;
    // motor->MotorData.IC_offset = 0.0f;
    // motor->MotorData.LastAngle = 0.0f;
    // motor->MotorData.LastVelocity = 0.0f;
    // motor->MotorData.Velocity_raw = 0.0f;
    motor->MotorData.Velocity_LPF.last_output = 0.0f;
    motor->MotorData.Velocity_LPF.alpha = 0.13f; // 速度滤波系数

    // 初始化驱动接口函数指针
    motor->MotorDrv.Set_PWM_A = stm32_set_pwm_A;      // 设置PWM函数指针
    motor->MotorDrv.Set_PWM_B = stm32_set_pwm_B;      // 设置PWM函数指针
    motor->MotorDrv.Set_PWM_C = stm32_set_pwm_C;      // 设置PWM函数指针

    motor->MotorDrv.Update_Ia_raw = stm32_update_Ia_raw;       // 获取IA电流函数指针
    motor->MotorDrv.Update_Ib_raw = stm32_update_Ib_raw;       // 获取IB电流函数指针
    motor->MotorDrv.Update_Ic_raw = stm32_update_Ic_raw;       // 获取IC电流函数指针
    
    motor->MotorDrv.Cal_Ia = stm32_cal_Ia;      // 电流转换函数指针
    motor->MotorDrv.Cal_Ib = stm32_cal_Ib;      // 电流转换函数指针
    motor->MotorDrv.Cal_Ic = stm32_cal_Ic;      // 电流转换函数指针

    motor->MotorDrv.Delayms = stm32_delayms;     // 延时函数指针
    motor->MotorDrv.Update_dt = stm32_update_dt;       // 获取时间差函数指针

    motor->MotorDrv.Update_Angle_raw = stm32_update_angle_raw;    // 获取角度函数指针
    motor->MotorDrv.Cal_Angle = stm32_cal_angle;   // 角度转换函数指针
    
}

