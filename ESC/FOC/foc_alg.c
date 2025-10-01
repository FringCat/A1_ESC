#include "foc_alg.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
float sgn(float x)
{
	if(x>0)
	{
		return 1 ;
	}
	else if(x<0)
	{
		return -1 ;
	}
	else
	{
		return 0 ;
	}
}

float Sat(float e, float r) 
{
    if (e > r) 
    {
        return r;
    } else if (e < -r) 
    {
        return -r;
    } else 
    {
        return e;
    }
}

float myabs(float val)
{
	if(val>=0)
	{
		return val ;
	}
	else 
	{
		return -val;
	}
}

float mymap( float Data ,float formLOW,float formHIGH, float toLOW,float toHIGH)
{
	return ((Data-formLOW)*((float)((toHIGH-toLOW)/(float)(formHIGH-formLOW))))+toLOW;
}

float Limit_angle_el(float angle_el) 
{
    float a = fmod(angle_el, 2*PI);  

    return (a >= 0 ? a : (a + 2*PI));
}

float update_angle(Motor_HandleTypeDef *motor)
{
    motor->MotorAlg.last_angle = motor->MotorAlg.angle;
    motor->MotorAlg.angle = motor->MotorDrv.Cal_Angle(motor->MotorDrv.Update_Angle_raw());
    return motor->MotorAlg.angle;
}

float Get_angle_el(Motor_HandleTypeDef *motor) 
{
    return motor->MotorAlg.angle_el;
}

float Calculate_angle_el(float Pole_pairs,float angle,float angle_el_zero) 
{
    return Limit_angle_el(angle * Pole_pairs - angle_el_zero);
}

float update_angle_el(Motor_HandleTypeDef *motor) 
{
    motor->MotorAlg.last_angle = motor->MotorAlg.angle;
    motor->MotorAlg.angle = update_angle(motor);
    motor->MotorAlg.angle_el = Calculate_angle_el(motor->MotorConfig.Pole_pairs,motor->MotorAlg.angle, motor->MotorConfig.angle_el_zero);
    return motor->MotorAlg.angle_el;
}

float Limit(float value , float high , float low)
{
    return (value)<(low)?(low):((value)>(high)?(high):(value));//如果目标参数超出最大/最小值的范围，就把这个值锁死在最大/最小值
}

float *Calculate_Park_N(float Uq , float Ud , float angle_el)
{

	static float Upark_N[2];
	
	Upark_N[0] = Ud*cos(angle_el) - Uq*sin(angle_el); //Park逆变换 ①Ualpha = Ud * cosθ - Uq * sinθ
    Upark_N[1] = Uq*cos(angle_el) + Ud*sin(angle_el); //           ②Ubeta  = Uq * cosθ + Ud * sinθ
	return Upark_N;
	
}

float *update_Park_N(Motor_HandleTypeDef *motor)
{
    static float* Upark_N;
    Upark_N = Calculate_Park_N(motor->MotorAlg.Uq , motor->MotorAlg.Ud , motor->MotorAlg.angle_el);
    motor->MotorAlg.Ualpha = Upark_N[0];
    motor->MotorAlg.Ubeta  = Upark_N[1];
    return Upark_N;
}

float *Calculate_Clark_N(float Ualpha ,float Ubeta,float Upower)
{	
    static float Uclark_N[3];
    
    //Clark逆变换:
    Uclark_N[0] = Ualpha + Upower/2;                 // ①Ua = Ualpha ;
    Uclark_N[1] = (sqrt_3*Ubeta-Ualpha)/2 + Upower/2; // ②Ub = (√3 * Ubeta - Ualpha)/2 ;
    Uclark_N[2] = -(Ualpha + sqrt_3*Ubeta)/2 + Upower/2;// ③Uc = ( -Ualpha - √3 * Ubeta )/2;
    
    return Uclark_N;
}

float *update_Clark_N(Motor_HandleTypeDef *motor)
{	
    static float* Uclark_N;
    Uclark_N = Calculate_Clark_N(motor->MotorAlg.Ualpha , motor->MotorAlg.Ubeta , motor->MotorConfig.UMAX);
    motor->MotorAlg.UA = Uclark_N[0];
    motor->MotorAlg.UB = Uclark_N[1];
    motor->MotorAlg.UC = Uclark_N[2];
    return Uclark_N;
}

float *Calculate_Clark(float IA ,float IB ,float IC)
{	
    static float Iclark[2];
    
    //Clark变换:
    Iclark[0] = IA ;                         // ①Ialpha = IA ;
    Iclark[1] = (sqrt_3*IB + sqrt_3*IC)/3 ; // ②Ibeta  = (√3 * IB + √3 * IC)/3 ;
    
    return Iclark;
}

float *update_Clark(Motor_HandleTypeDef *motor)
{	
    static float* Iclark;
    Iclark = Calculate_Clark(motor->MotorAlg.IA , motor->MotorAlg.IB , motor->MotorAlg.IC);
    motor->MotorAlg.Ialpha = Iclark[0];
    motor->MotorAlg.Ibeta  = Iclark[1];
    return Iclark;
}

float *Calculate_Park(float Ialpha ,float Ibeta ,float angle_el)
{
    static float Ipark[2];
    
    //Park变换:
    Ipark[0] = Ialpha*cos(angle_el) + Ibeta*sin(angle_el); // ①Id = Ialpha * cosθ + Ibeta * sinθ
    Ipark[1] = -Ialpha*sin(angle_el) + Ibeta*cos(angle_el);// ②Iq = -Ialpha * sinθ + Ibeta * cosθ
    
    return Ipark;
}

float *update_Park(Motor_HandleTypeDef *motor)
{
    static float* Ipark;
    Ipark = Calculate_Park(motor->MotorAlg.Ialpha , motor->MotorAlg.Ibeta , motor->MotorAlg.angle_el);
    motor->MotorAlg.Id = Ipark[0];
    motor->MotorAlg.Iq = Ipark[1];
    return Ipark;
}

void update_pwm(Motor_HandleTypeDef *motor)
{
    float _Ua = Limit(motor->MotorAlg.UA/motor->MotorConfig.UMAX , 1 , 0 );//计算并限制ABC相所需的占空比
	float _Ub = Limit(motor->MotorAlg.UB/motor->MotorConfig.UMAX , 1 , 0 );
	float _Uc = Limit(motor->MotorAlg.UC/motor->MotorConfig.UMAX , 1 , 0 );

    if(motor->MotorDrv.Set_PWM_A!=NULL | motor->MotorDrv.Set_PWM_B!=NULL | motor->MotorDrv.Set_PWM_C!=NULL)
    {
        switch (motor->MotorConfig.DIR)
        {
            case 1:
            {
                motor->MotorDrv.Set_PWM_A(_Ua);
                motor->MotorDrv.Set_PWM_B(_Ub);
                motor->MotorDrv.Set_PWM_C(_Uc);
            }break;
            case 2:
            {
                motor->MotorDrv.Set_PWM_A(_Ub);
                motor->MotorDrv.Set_PWM_B(_Ua);
                motor->MotorDrv.Set_PWM_C(_Uc);
            }break;
            case 3:
            {
                motor->MotorDrv.Set_PWM_A(_Uc);
                motor->MotorDrv.Set_PWM_B(_Ub);
                motor->MotorDrv.Set_PWM_C(_Ua);
            }break;
            case 4:
            {
                motor->MotorDrv.Set_PWM_A(_Uc);
                motor->MotorDrv.Set_PWM_B(_Ua);
                motor->MotorDrv.Set_PWM_C(_Ub);
            }break;
            case 5:
            {
                motor->MotorDrv.Set_PWM_A(_Ub);
                motor->MotorDrv.Set_PWM_B(_Uc);
                motor->MotorDrv.Set_PWM_C(_Ua);
            }break;
            case 6:
            {
                motor->MotorDrv.Set_PWM_A(_Ua);
                motor->MotorDrv.Set_PWM_B(_Uc);
                motor->MotorDrv.Set_PWM_C(_Ub);
            }break;
            default:
            {
                motor->MotorDrv.Set_PWM_A(_Ua);
                motor->MotorDrv.Set_PWM_B(_Ub);
                motor->MotorDrv.Set_PWM_C(_Uc);
            }break;
        }
    }
    else
    {
        /*打印报错信息*/
    }
}

void set_pwm(Motor_HandleTypeDef *motor,float Ta , float Tb ,float Tc)
{
    float _Ua = Limit(Ta , 1 , 0 );//计算并限制ABC相所需的占空比
	float _Ub = Limit(Tb , 1 , 0 );
	float _Uc = Limit(Tc , 1 , 0 );

    if(motor->MotorDrv.Set_PWM_A!=NULL | motor->MotorDrv.Set_PWM_B!=NULL | motor->MotorDrv.Set_PWM_C!=NULL)
    {
        switch (motor->MotorConfig.DIR)
        {
            case 1:
            {
                motor->MotorDrv.Set_PWM_A(_Ua);
                motor->MotorDrv.Set_PWM_B(_Ub);
                motor->MotorDrv.Set_PWM_C(_Uc);
            }break;
            case 2:
            {
                motor->MotorDrv.Set_PWM_A(_Ub);
                motor->MotorDrv.Set_PWM_B(_Ua);
                motor->MotorDrv.Set_PWM_C(_Uc);
            }break;
            case 3:
            {
                motor->MotorDrv.Set_PWM_A(_Uc);
                motor->MotorDrv.Set_PWM_B(_Ub);
                motor->MotorDrv.Set_PWM_C(_Ua);
            }break;
            case 4:
            {
                motor->MotorDrv.Set_PWM_A(_Uc);
                motor->MotorDrv.Set_PWM_B(_Ua);
                motor->MotorDrv.Set_PWM_C(_Ub);
            }break;
            case 5:
            {
                motor->MotorDrv.Set_PWM_A(_Ub);
                motor->MotorDrv.Set_PWM_B(_Uc);
                motor->MotorDrv.Set_PWM_C(_Ua);
            }break;
            case 6:
            {
                motor->MotorDrv.Set_PWM_A(_Ua);
                motor->MotorDrv.Set_PWM_B(_Uc);
                motor->MotorDrv.Set_PWM_C(_Ub);
            }break;
            default:
            {
                motor->MotorDrv.Set_PWM_A(_Ua);
                motor->MotorDrv.Set_PWM_B(_Ub);
                motor->MotorDrv.Set_PWM_C(_Uc);
            }break;
        }
    }
    else
    {
        /*打印报错信息*/
    }
}

void set_pwm_nodir(Motor_HandleTypeDef *motor,float Ta , float Tb ,float Tc)
{
    float _Ua = Limit(Ta , 1 , 0 );//计算并限制ABC相所需的占空比
    float _Ub = Limit(Tb , 1 , 0 );
    float _Uc = Limit(Tc , 1 , 0 );

    if(motor->MotorDrv.Set_PWM_A!=NULL | motor->MotorDrv.Set_PWM_B!=NULL | motor->MotorDrv.Set_PWM_C!=NULL)
    {
        motor->MotorDrv.Set_PWM_A(_Ua);
        motor->MotorDrv.Set_PWM_B(_Ub);
        motor->MotorDrv.Set_PWM_C(_Uc);
    }
    else
    {
        /*打印报错信息*/
    }
}

int Calculate_Sector( float Ualpha , float Ubeta )
{
	if((Ualpha>0.0f) && (Ubeta>0.0f) && (Ubeta/Ualpha < sqrt_3)){return 1 ;}
	else if((Ubeta>0.0f) && (Ubeta/myabs(Ualpha)>sqrt_3)){return 2 ;}
	else if((Ualpha<0.0f) && (Ubeta>0.0f) && (-Ubeta/Ualpha < sqrt_3)){return 3 ;}
	else if((Ualpha<0.0f) && (Ubeta<0.0f) && (Ubeta/Ualpha < sqrt_3)){return 4 ;}
	else if((Ubeta<0.0f) && (-Ubeta/myabs(Ualpha)>sqrt_3)){return 5 ;}
	else if((Ualpha>0.0f) && (Ubeta<0.0f) && (-Ubeta/Ualpha < sqrt_3)){return 6 ;}
	else {return 0;}
}

int update_Sector(Motor_HandleTypeDef *motor)
{
    motor->MotorAlg.Sector = Calculate_Sector(motor->MotorAlg.Ualpha , motor->MotorAlg.Ubeta);
    return motor->MotorAlg.Sector;
}

int get_Sector(Motor_HandleTypeDef *motor)
{
    return motor->MotorAlg.Sector;
}

float Calculate_LPF(float input, float last_output, float alpha)
{
    return alpha * input + (1 - alpha) * last_output;
}

float update_velocity_LPF(Motor_HandleTypeDef *motor)  
{
    motor->MotorData.Velocity_raw = Calculate_velocity_raw(motor->MotorAlg.angle, motor->MotorAlg.last_angle, motor->time.dt);
    motor->MotorAlg.Velocity =  Calculate_LPF(motor->MotorData.Velocity_raw, motor->MotorData.Velocity_LPF.last_output, motor->MotorData.Velocity_LPF.alpha);
    motor->MotorData.Velocity_LPF.last_output = motor->MotorAlg.Velocity; // 更新滤波器的上次输出值
    // motor->MotorAlg.last_angle = motor->MotorAlg.angle; // （已注释）更新上一时刻角度,,update_angle_el函数中已进行此操作
    return motor->MotorAlg.Velocity;
}

float update_velocity_raw(Motor_HandleTypeDef *motor)
{
    motor->MotorData.Velocity_raw = Calculate_velocity_raw(motor->MotorAlg.angle, motor->MotorAlg.last_angle, motor->time.dt);
    // motor->MotorAlg.last_angle = motor->MotorAlg.angle; // （已注释）更新上一时刻角度,,update_angle_el函数中已进行此操作
    return motor->MotorData.Velocity_raw;
}

float Calculate_velocity_raw(float angle, float last_angle, float dt)
{
    if (dt <= 0) 
    {
        return 0; // 避免除以零
    }

    float velocity_raw;
    // 计算原始速度
    if(fabs(angle - last_angle) > (0.8f*2*PI))
    {
        if((angle - last_angle)<0){velocity_raw = (2*PI - last_angle + angle)/dt ;}//正转
        else if((angle - last_angle)>=0){velocity_raw = -(2*PI - angle + last_angle)/dt ;}//反转
    }
    else 
    {
        velocity_raw = (angle - last_angle)/dt ;
    }

    return velocity_raw;
}

float Calculate_velocity_LPF(float angle, float last_angle, float dt, float last_velocity, float alpha)
{
    float velocity_raw = Calculate_velocity_raw(angle, last_angle, dt);
    return Calculate_LPF(velocity_raw, last_velocity, alpha);
}

float get_velocity(Motor_HandleTypeDef *motor)
{
    return motor->MotorAlg.Velocity;
}

float get_velocity_raw(Motor_HandleTypeDef *motor)
{
    return motor->MotorData.Velocity_raw;
}

float Calculate_PID(float target, float feedback, float dt ,PID_t* pid)
{
    float error = target - feedback;
    
    // 计算积分项
    pid->This_I += error * dt ;
    pid->This_I = Limit(pid->This_I, pid->integral_max, pid->integral_min); // 限制积分项防止积分饱和

    // 计算微分项
    float derivative = (error - pid->last_error) / dt;

    // 计算PID输出
    float output = pid->KP * error + pid->KI * pid->This_I + pid->KD * derivative;
    output = Limit(output, pid->output_max, pid->output_min); // 限制输出范围

    // 更新历史误差
    pid->last_error = error;

    return output;
}

void update_svpwm(Motor_HandleTypeDef *motor)
{
    static float K = 0 , Ux = 0 , Uy = 0 , Uz = 0 , Tx = 0 ,Ty = 0,Tz = 0;
	static float Ta = 0 , Tb = 0 ,Tc = 0 ;

    update_Park_N(motor);
    update_Sector(motor);

    K=(sqrt_3*1)/(motor->MotorConfig.UMAX/2);
	Ux = motor->MotorAlg.Ubeta;
	Uy = (sqrt_3/2.0f)*motor->MotorAlg.Ualpha - 0.5f*motor->MotorAlg.Ubeta;
	Uz = (sqrt_3/2.0f)*motor->MotorAlg.Ualpha + 0.5f*motor->MotorAlg.Ubeta;

    switch (motor->MotorAlg.Sector)
	{
	case 1:
		Tx = K*Uy ;
		Ty = K*Ux ;
		break;
	case 2:
		Tx = -K*Uy ;
		Ty = K*Uz ;
		break;
	case 3:
		Tx = K*Ux ;
		Ty = -K*Uz ;
		break;
	case 4:
		Tx = -K*Ux ;
		Ty = -K*Uy ;
		break;
	case 5:
		Tx = -K*Uz ;
		Ty = K*Uy ;
		break;
	case 6:
		Tx = K*Uz ;
		Ty = -K*Ux ;
		break;
	default:/*打印报错信息*/
		break;
	}
    	Tz = 0.5f*(1-Tx-Ty) ;
	if(Tx + Ty > 1)
	{
		Tx = Tx/(Tx+Ty)*1;
		Ty = Ty/(Tx+Ty)*1;
	}

	switch(motor->MotorAlg.Sector)
	{
		case 1 : 
			Tc = Tz ;
			Tb = Tz + Ty ;
			Ta = Tz + Ty + Tx ;
			break;
		case 2 : 
			Tc = Tz ;
			Ta = Tz + Ty ;
			Tb = Tz + Ty + Tx ;			
			break;
		case 3 : 
			Ta = Tz ;
			Tc = Tz + Ty ;
			Tb = Tz + Ty + Tx ;	
			break;
		case 4 : 
			Ta = Tz ;
			Tb = Tz + Ty ;
			Tc = Tz + Ty + Tx ;	
			break;
		case 5 : 
			Tb = Tz ;
			Ta = Tz + Ty ;
			Tc = Tz + Ty + Tx ;	
			break;
		case 6 : 
			Tb = Tz ;
			Tc = Tz + Ty ;
			Ta = Tz + Ty + Tx ;
			break;
		default:
			break;
	}

	float a = mymap(Ta,-1,1,0,1);
	float b = mymap(Tb,-1,1,0,1);
	float c = mymap(Tc,-1,1,0,1);

    set_pwm(motor,a,b,c);
}

void set_svpwm(Motor_HandleTypeDef *motor, float Uq , float Ud ,float angle_el)
{
    static float K = 0 , Ux = 0 , Uy = 0 , Uz = 0 , Tx = 0 ,Ty = 0,Tz = 0;
	static float Ta = 0 , Tb = 0 ,Tc = 0 ;
    float Ualpha = 0 , Ubeta = 0 ;
    
    float *Upark = Calculate_Park_N(Uq , Ud , angle_el);
    Ualpha = Upark[0];
    Ubeta  = Upark[1];
    
    int Sector = Calculate_Sector(Ualpha , Ubeta);
    
    K=(sqrt_3*1)/(motor->MotorConfig.UMAX/2);
    Ux = Ubeta;
    Uy = (sqrt_3/2.0f)*Ualpha - 0.5f*Ubeta;
    Uz = (sqrt_3/2.0f)*Ualpha + 0.5f*Ubeta;

    switch (Sector)
	{
        case 1:
            Tx = K*Uy ;
            Ty = K*Ux ;
            break;
        case 2:
            Tx = -K*Uy ;
            Ty = K*Uz ;
            break;
        case 3:
            Tx = K*Ux ;
            Ty = -K*Uz ;
            break;
        case 4:
            Tx = -K*Ux ;
            Ty = -K*Uy ;
            break;
        case 5:
            Tx = -K*Uz ;
            Ty = K*Uy ;
            break;
        case 6:
            Tx = K*Uz ;
            Ty = -K*Ux ;
            break;
        default:
            break;
	}
    	Tz = 0.5f*(1-Tx-Ty) ;
	if(Tx + Ty > 1)
	{
		Tx = Tx/(Tx+Ty)*1;
		Ty = Ty/(Tx+Ty)*1;
	}

	switch(Sector)
	{
		case 1 : 
			Tc = Tz ;
			Tb = Tz + Ty ;
			Ta = Tz + Ty + Tx ;
			break;
		case 2 : 
			Tc = Tz ;
			Ta = Tz + Ty ;
			Tb = Tz + Ty + Tx ;			
			break;
		case 3 : 
			Ta = Tz ;
			Tc = Tz + Ty ;
			Tb = Tz + Ty + Tx ;	
			break;
		case 4 : 
			Ta = Tz ;
			Tb = Tz + Ty ;
			Tc = Tz + Ty + Tx ;	
			break;
		case 5 : 
			Tb = Tz ;
			Ta = Tz + Ty ;
			Tc = Tz + Ty + Tx ;	
			break;
		case 6 : 
			Tb = Tz ;
			Tc = Tz + Ty ;
			Ta = Tz + Ty + Tx ;
			break;
		default:
			break;
	}

	float a = mymap(Ta,-1,1,0,1);
	float b = mymap(Tb,-1,1,0,1);
	float c = mymap(Tc,-1,1,0,1);
    set_pwm_nodir(motor,a,b,c);
}

void update_spwm(Motor_HandleTypeDef *motor)
{
    update_Park_N(motor);
    update_Clark_N(motor);
    set_pwm(motor,motor->MotorAlg.UA/motor->MotorConfig.UMAX, motor->MotorAlg.UB/motor->MotorConfig.UMAX , motor->MotorAlg.UC/motor->MotorConfig.UMAX);
}

void set_spwm(Motor_HandleTypeDef *motor,float Uq, float Ud ,float angle_el)
{
    float *Upark = Calculate_Park_N(Uq , Ud , angle_el);
    float *Uclark = Calculate_Clark_N(Upark[0] , Upark[1] , motor->MotorConfig.UMAX);
    set_pwm_nodir(motor,Uclark[0]/motor->MotorConfig.UMAX , Uclark[1]/motor->MotorConfig.UMAX , Uclark[2]/motor->MotorConfig.UMAX);
}

float get_dt(Motor_HandleTypeDef *motor)
{
    return motor->time.dt;
}

float update_dt(Motor_HandleTypeDef *motor)
{
    motor->MotorDrv.Update_dt(&motor->time);
    return motor->time.dt;
}

float update_IaIbIc(Motor_HandleTypeDef *motor)
{
    static float Ia_ = 0 , Ib_ = 0 , Ic_ = 0 ;
    if(motor->MotorDrv.Update_Ia_raw!=NULL | motor->MotorDrv.Update_Ib_raw!=NULL | motor->MotorDrv.Update_Ic_raw!=NULL)
    {
        motor->MotorData.CurrentData.I_raw.IA_raw = motor->MotorDrv.Update_Ia_raw();
        motor->MotorData.CurrentData.I_raw.IB_raw = motor->MotorDrv.Update_Ib_raw();
        motor->MotorData.CurrentData.I_raw.IC_raw = motor->MotorDrv.Update_Ic_raw();
        
        motor->MotorAlg.IA = motor->MotorDrv.Cal_Ia(motor->MotorData.CurrentData.I_raw.IA_raw , motor->MotorData.IA_offset);
        motor->MotorAlg.IB = motor->MotorDrv.Cal_Ib(motor->MotorData.CurrentData.I_raw.IB_raw , motor->MotorData.IB_offset);
        motor->MotorAlg.IC = motor->MotorDrv.Cal_Ic(motor->MotorData.CurrentData.I_raw.IC_raw , motor->MotorData.IC_offset);
        
        switch (motor->MotorConfig.DIR)
        {
            case 1:
            {
                Ia_ = motor->MotorAlg.IA ;
                Ib_ = motor->MotorAlg.IB ;
                Ic_ = motor->MotorAlg.IC ;

                motor->MotorAlg.IA = Ia_ ;
                motor->MotorAlg.IB = Ib_ ;
                motor->MotorAlg.IC = Ic_ ;
            }break;
            case 2:
            {
                Ia_ = motor->MotorAlg.IB ;
                Ib_ = motor->MotorAlg.IA ;
                Ic_ = motor->MotorAlg.IC ;

                motor->MotorAlg.IA = Ia_ ;
                motor->MotorAlg.IB = Ib_ ;
                motor->MotorAlg.IC = Ic_ ;
            }break;
            case 3:
            {
                Ia_ = motor->MotorAlg.IC ;
                Ib_ = motor->MotorAlg.IB ;
                Ic_ = motor->MotorAlg.IA ;

                motor->MotorAlg.IA = Ia_ ;
                motor->MotorAlg.IB = Ib_ ;
                motor->MotorAlg.IC = Ic_ ;
            }break;
            case 4:
            {
                Ia_ = motor->MotorAlg.IC ;
                Ib_ = motor->MotorAlg.IA ;
                Ic_ = motor->MotorAlg.IB ;

                motor->MotorAlg.IA = Ia_ ;
                motor->MotorAlg.IB = Ib_ ;
                motor->MotorAlg.IC = Ic_ ;
            }break;
            case 5:
            {
                Ia_ = motor->MotorAlg.IB ;
                Ib_ = motor->MotorAlg.IC ;
                Ic_ = motor->MotorAlg.IA ;

                motor->MotorAlg.IA = Ia_ ;
                motor->MotorAlg.IB = Ib_ ;
                motor->MotorAlg.IC = Ic_ ;
            }break;
            case 6:
            {
                Ia_ = motor->MotorAlg.IA ;
                Ib_ = motor->MotorAlg.IC ;
                Ic_ = motor->MotorAlg.IB ;

                motor->MotorAlg.IA = Ia_ ;
                motor->MotorAlg.IB = Ib_ ;
                motor->MotorAlg.IC = Ic_ ;
            }break;
            default:
            {
                Ia_ = motor->MotorAlg.IA ;
                Ib_ = motor->MotorAlg.IB ;
                Ic_ = motor->MotorAlg.IC ;

                motor->MotorAlg.IA = Ia_ ;
                motor->MotorAlg.IB = Ib_ ;
                motor->MotorAlg.IC = Ic_ ;
            }break;

        }

        return 1 ;
    }
    else
    {
        /*打印报错信息*/
        return 0 ;
    }
}

float get_Ia(Motor_HandleTypeDef *motor)
{
    return motor->MotorAlg.IA;
}

float get_Ib(Motor_HandleTypeDef *motor)
{
    return motor->MotorAlg.IB;
}

float get_Ic(Motor_HandleTypeDef *motor)
{
    return motor->MotorAlg.IC;
}

void update_Ioffset(Motor_HandleTypeDef *motor)
{
    for(int i=0 ; i<1000 ; i++)
    {
        motor->MotorData.IA_offset += motor->MotorDrv.Cal_Ia(motor->MotorDrv.Update_Ia_raw() , 0);
        motor->MotorData.IB_offset += motor->MotorDrv.Cal_Ib(motor->MotorDrv.Update_Ib_raw() , 0);
        motor->MotorData.IC_offset += motor->MotorDrv.Cal_Ic(motor->MotorDrv.Update_Ic_raw() , 0);
    }
    motor->MotorData.IA_offset = motor->MotorData.IA_offset/1000 ;
    motor->MotorData.IB_offset = motor->MotorData.IB_offset/1000 ;
    motor->MotorData.IC_offset = motor->MotorData.IC_offset/1000 ;
}

float get_Ia_offset(Motor_HandleTypeDef *motor)
{
    return motor->MotorData.IA_offset;
}

float get_Ib_offset(Motor_HandleTypeDef *motor)
{
    return motor->MotorData.IB_offset;
}

float get_Ic_offset(Motor_HandleTypeDef *motor)
{
    return motor->MotorData.IC_offset;
}

void update_pole_pairs_sensor_block(Motor_HandleTypeDef *motor)
{
    static float velocity_integral = 0.0f;
    set_svpwm(motor,motor->MotorConfig.UMAX*0.5f,0.0f, 0.0f);
    motor->MotorDrv.Delayms(500);
    for(int i=0 ; i<1000 ; i++)
    {
        update_dt(motor);  //预热dt，防止因初次启动产生的极小dt干扰后面的速度计算
        update_angle(motor);
        update_velocity_raw(motor);
    }

    for(int i=0 ; i<1000 ; i++)
    {
        update_dt(motor);  //预热dt，防止因初次启动产生的极小dt干扰后面的速度计算
        update_angle(motor);
        update_velocity_raw(motor);
        velocity_integral += motor->MotorData.Velocity_raw*motor->time.dt;
        // printf("%f,%f,%f,%f\n",(angle_end - angle_start),velocity_integral,motor->MotorData.Velocity_raw,motor->MotorAlg.angle);
        set_svpwm(motor,motor->MotorConfig.UMAX*0.5f,0.0f, Limit_angle_el((float)i*0.01f));
        motor->MotorDrv.Delayms(1);
    }
    motor->MotorDrv.Delayms(500);

    motor->MotorConfig.Pole_pairs = (uint32_t)round(myabs((float)(1000*0.01f)/(velocity_integral)));
    // printf("%d,%f\n",motor->MotorConfig.Pole_pairs,(myabs((float)(1000*0.01f)/(velocity_integral))));
    // printf("%f,%f,%f,%f\n",(angle_end - angle_start),velocity_integral,motor->MotorData.Velocity_raw,motor->MotorAlg.angle);
    set_svpwm(motor,0.0f, 0.0f , 0.0f); 
}

void update_pole_pairs_sensor_nonblock(Motor_HandleTypeDef *motor)
{
    static int flag = 0;
    static float angle_start = 0;
    static float angle_end = 0;
    
    float velocity_target = 3.0f; 
    float time_init = 0.5f;
    float time_prep = 1.0f;
    float time_process = 5.0f;

    static uint8_t state = 0 ;
    static float total_time = 0 ;
    static float velocity_integral = 0.0f;

    if(flag == 0)
    {
        total_time += update_dt(motor);
        update_angle(motor);
        update_velocity_raw(motor);
        if(total_time < time_init)
        {
            state = 0;
        }
        else if(total_time >= time_init && total_time < (time_init+time_prep))
        {
            state = 1;
        }
        else if(total_time >= (time_init+time_prep) && total_time < (time_init+time_prep+time_process))
        {
            state = 2;
        }
        else if(total_time >= (time_init+time_prep+time_process))
        {
            state = 3;
        }
        
        switch (state)
        {
            case 0:
            {
                set_svpwm(motor,0.0f, 0.0f , 0.0f);
            }break;
            case 1:
            {
                set_svpwm(motor, motor->MotorConfig.UMAX*0.5f,0.0f,0.0f);
                angle_start = motor->MotorAlg.angle;
            }break;
            case 2:
            {
                velocity_integral += motor->MotorData.Velocity_raw*motor->time.dt;
                set_svpwm(motor,motor->MotorConfig.UMAX*0.5f,0.0f, Limit_angle_el((float)velocity_target*(total_time-time_init-time_prep)));
                // motor->MotorDrv.Delayms(1);
                angle_end = motor->MotorAlg.angle;
            }break;
            case 3:
            {
                angle_end = motor->MotorAlg.angle;
                motor->MotorConfig.Pole_pairs = (uint32_t)round(myabs((float)velocity_target*(total_time-time_init-time_prep)/(velocity_integral)));
                set_svpwm(motor,0.0f, 0.0f , 0.0f);
                // total_time = 0.0f;
                // velocity_integral = 0.0f;
                flag = 1;
                // printf("%d\n",motor->MotorConfig.Pole_pairs);
            }
            default:
            {
                /* 打印报错信息 */
            }break;
        }
    }
    printf("%d\n",motor->MotorConfig.Pole_pairs);

}

void update_2DIR_sensor_block(Motor_HandleTypeDef *motor)
{
    float velocity_target = 0.01f; 
    float velocity_integral = 0.0f;

    for(int i=0 ; i<1000 ; i++)
    {
        update_dt(motor);  //预热dt，防止因初次启动产生的极小dt干扰后面的速度计算
        update_angle(motor);
        update_velocity_raw(motor);
        set_svpwm(motor,0.001f*i*motor->MotorConfig.UMAX*0.5f, motor->MotorConfig.UMAX*0.5f - 0.001f*i*motor->MotorConfig.UMAX*0.5f , 0.0f);
        motor->MotorDrv.Delayms(1);
    }

    for(int i=0 ; i<2000 ; i++)
    {
        update_dt(motor);
        update_angle(motor);
        update_velocity_raw(motor);
        // if(myabs(motor->MotorData.Velocity_raw) > 2*velocity_target)
        // {
        //     motor->MotorData.Velocity_raw = 0.0f ;
        // }
        set_svpwm(motor,motor->MotorConfig.UMAX*0.5f,3.1f, Limit_angle_el((float)i*velocity_target));
        velocity_integral += motor->MotorData.Velocity_raw;
        // printf("%f,%f\n",motor->MotorData.Velocity_raw,velocity_integral);
        motor->MotorDrv.Delayms(1);
    }
    motor->MotorDrv.Delayms(500);

    if(velocity_integral>0)
    {
        motor->MotorConfig.DIR = 1;
    }
    else if(velocity_integral<0)
    {
        motor->MotorConfig.DIR = 2;
    }
    else
    {
        /*传感器异常报错*/
    }
    set_svpwm(motor,0.0f,0.0f,0);
}

void update_2DIR_sensor_nonblock(Motor_HandleTypeDef *motor)
{
    float velocity_target = 10.0f; 
    float time_init = 0.5f;
    float time_prep = 1.0f;
    float time_process = 2.0f;
    // float time_finish = 0.5f;

    static uint8_t state = 0 ;
    static float total_time = 0.0f ;
    static float velocity_integral = 0.0f;

    total_time += update_dt(motor); //预热
    update_angle(motor);
    update_velocity_raw(motor);
    if(total_time < time_init)
    {
        state = 0;
    }
    else if(total_time >= time_init && total_time < (time_init+time_prep))
    {
        state = 1;
    }
    else if(total_time >= (time_init+time_prep) && total_time < (time_init+time_prep+time_process))
    {
        state = 2;
    }
    else if(total_time >= (time_init+time_prep+time_process))
    {
        state = 3;
    }

    switch (state)
    {
        case 0:
        {
            set_svpwm(motor,0.0f, 0.0f , 0.0f);
        }break;
        case 1:
        {
            float K =  (total_time-time_init)/time_process;
            set_svpwm(motor,0.0f,K*motor->MotorConfig.UMAX*0.5f, 0.0f);
        }break;
        case 2:
        {
            set_svpwm(motor,motor->MotorConfig.UMAX*0.5f,3.0f, Limit_angle_el((float)(total_time-time_init-time_prep)*velocity_target));
            velocity_integral += motor->MotorData.Velocity_raw;
        }break;
        case 3:
        {
            if(velocity_integral>0)
            {
                motor->MotorConfig.DIR = 1;
            }
            else if(velocity_integral<0)
            {
                motor->MotorConfig.DIR = 2;
            }
            else
            {
                /*传感器异常报错*/
            }
            set_svpwm(motor,0.0f, 0.0f , 0.0f);
            total_time = 0.0f;
            velocity_integral = 0.0f;
        }
        default:
        {
            /* 打印报错信息 */
        }break;
    }
    // printf("%f,%d,%f,%f,%f\n",velocity_integral,motor->MotorConfig.DIR,motor->MotorData.Velocity_raw,motor->MotorAlg.angle,motor->time.dt);
}

void update_angle_el_zero_sensor_block(Motor_HandleTypeDef *motor)
{
    int sample_per = 10 ; //每极对采样10个
    int sample_total = motor->MotorConfig.Pole_pairs * sample_per ; //总采样数
    float *angle_el_zero = (float*)calloc(sample_total,sizeof(float));  //按照采样数定义动态数组
    float angle_el_zero_all = 0;
    if(angle_el_zero == NULL)
    {   
        //打印报错信息
        return;
    }

    set_svpwm(motor,0.0f, motor->MotorConfig.UMAX*0.5f , 0.0f);
    motor->MotorDrv.Delayms(500);
    for(int i = 0; i<sample_total ;i++)
    {
        set_svpwm(motor,motor->MotorConfig.UMAX*0.5f,0,Limit_angle_el(i*(2*PI*motor->MotorConfig.Pole_pairs)/sample_total));
        motor->MotorDrv.Delayms(1);
        angle_el_zero[i] = ( ((float)i) / ((float)sample_total) ) * 2*PI - motor->MotorDrv.Cal_Angle( motor->MotorDrv.Update_Angle_raw() );
    }
    motor->MotorDrv.Delayms(500);
    for(int i = sample_total-1;i>=0;i--)
    {
        set_svpwm(motor,motor->MotorConfig.UMAX*0.5f,0,Limit_angle_el(i*(2*PI*motor->MotorConfig.Pole_pairs)/sample_total));
        motor->MotorDrv.Delayms(1);
        angle_el_zero[i] += ( ((float)i) / ((float)sample_total) ) * 2*PI - motor->MotorDrv.Cal_Angle( motor->MotorDrv.Update_Angle_raw() );
        angle_el_zero[i] /= 2 ;
    }
    for(int i = 0; i<sample_total ;i++)
    {
        angle_el_zero_all += angle_el_zero[i];
    }
    motor->MotorConfig.angle_el_zero = angle_el_zero_all/(float)sample_total;

    set_svpwm(motor,0.0f, 0.0f , 0.0f);
    free((void*)angle_el_zero);
}

void update_angle_el_zero_sensor_nonblock(Motor_HandleTypeDef *motor)
{
    static uint8_t state = 0;
    static float total_time = 0.0f;
    static int sample_count = 0;
    static int sample_total = 0;
    static float *angle_el_zero = NULL;
    static float angle_el_zero_all = 0.0f;

    total_time += get_dt(motor);

    // 状态判断：按时间/采样进度划分阶段
    if (state == 0)
    {
        // 初始化阶段（首次调用触发，不依赖时间）
    }
    else if (state == 1)
    {
        // 励磁稳定阶段：持续0.5s
        if (total_time >= 0.5f)
        {
            total_time = 0.0f;
            sample_count = 0;
            state = 2;
        }
    }
    else if (state == 2)
    {
        // 正向采样阶段：采样数达到总采样数则切换
        if (sample_count >= sample_total)
        {
            total_time = 0.0f;
            sample_count = sample_total - 1;
            state = 3;
        }
    }
    else if (state == 3)
    {
        // 反向采样阶段：采样数减至-1则切换
        if (sample_count < 0)
        {
            total_time = 0.0f;
            state = 4;
        }
    }
    else if (state == 4)
    {
        // 计算收尾阶段：执行一次后切换至结束
        state = 5;
    }


    switch (state)
    {
        case 0:
            total_time = 0.0f;
            sample_count = 0;
            angle_el_zero_all = 0.0f;
            sample_total = motor->MotorConfig.Pole_pairs * 10;
            
            angle_el_zero = (float*)calloc(sample_total, sizeof(float));
            if (angle_el_zero == NULL)
            {
                state = 5;
                break;
            }

            set_svpwm(motor, 0.0f, motor->MotorConfig.UMAX * 0.5f, 0.0f);
            state = 1;
            break;

        case 1:
            // d轴励磁，等待转子稳定
            set_svpwm(motor, 0.0f, motor->MotorConfig.UMAX * 0.5f, 0.0f);
            break;

        case 2:
            {
                float target_el_angle = Limit_angle_el((float)sample_count * (2 * PI * motor->MotorConfig.Pole_pairs) / (float)sample_total);
                set_svpwm(motor, motor->MotorConfig.UMAX * 0.5f, 0.0f, target_el_angle);
                angle_el_zero[sample_count] = target_el_angle  - motor->MotorDrv.Cal_Angle(motor->MotorDrv.Update_Angle_raw());
                sample_count++;
                total_time = 0.0f;
            }
            break;

        case 3:
            {
                float target_el_angle = Limit_angle_el((float)sample_count * (2 * PI * motor->MotorConfig.Pole_pairs) / (float)sample_total);
                set_svpwm(motor, motor->MotorConfig.UMAX * 0.5f, 0.0f, target_el_angle);
                float reverse_error = target_el_angle - motor->MotorDrv.Cal_Angle(motor->MotorDrv.Update_Angle_raw());
                angle_el_zero[sample_count] = (angle_el_zero[sample_count] + reverse_error) / 2.0f;
                sample_count--;
                total_time = 0.0f;
            }
            break;

        case 4:
            for (int i = 0; i < sample_total; i++)
            {
                angle_el_zero_all += angle_el_zero[i];
            }
            motor->MotorConfig.angle_el_zero = angle_el_zero_all / (float)sample_total;
            set_svpwm(motor, 0.0f, 0.0f, 0.0f);
            free(angle_el_zero);
            angle_el_zero = NULL;
            break;

        case 5:
            // 校准结束/错误状态
            break;

        default:
            set_svpwm(motor, 0.0f, 0.0f, 0.0f);
            if (angle_el_zero != NULL)
            {
                free(angle_el_zero);
                angle_el_zero = NULL;
            }
            state = 0;
            break;
    }
}

void map_samples_to_lut(float *error_arr, int N_SAMPLES, float *lut_arr, int N_LUT)
{
    for (int j = 0; j < N_LUT; j++)  // 遍历 LUT 每个索引
    {
        // 步骤 1：计算当前 LUT 索引对应的机械角度（0~2π）
        float lut_angle = (2 * PI * j) / (float)N_LUT;

        // 步骤 2：定位对应的左采样点 i
        int i = (int)floor( (j * N_SAMPLES) / (float)N_LUT );
        // 处理边界：i 不能超过 N_SAMPLES-1
        i = (i >= N_SAMPLES) ? N_SAMPLES - 1 : i;
        // 右采样点（周期闭环）
        int i_next = (i + 1) % N_SAMPLES;

        // 步骤 3：计算左右采样点的角度
        float sample_angle_i = (2 * PI * i) / (float)N_SAMPLES;
        float sample_angle_next = (2 * PI * i_next) / (float)N_SAMPLES;

        // 步骤 4：计算插值比例（处理 i_next=0 时的周期跨越）
        float delta_angle;
        if (i_next == 0)
        {
            // 最后一个采样点到第一个采样点的角度差（跨越 2π 边界）
            delta_angle = (2 * PI - sample_angle_i) + sample_angle_next;
        }
        else
        {
            delta_angle = sample_angle_next - sample_angle_i;
        }
        float ratio = (lut_angle - sample_angle_i) / delta_angle;
        // 限制 ratio 在 0~1 范围内（避免浮点误差导致越界）
        ratio = (ratio < 0) ? 0 : (ratio > 1) ? 1 : ratio;

        // 步骤 5：线性插值得到 LUT 值
        lut_arr[j] = error_arr[i] * (1 - ratio) + error_arr[i_next] * ratio;
    }
}

void update_NLLUT_encoder_sensor_block(Motor_HandleTypeDef *motor)
{
    int sample_per = 10 ; //每极对采样10个
    int sample_total = motor->MotorConfig.Pole_pairs * sample_per ; //总采样数
    float *angle_el_zero = (float*)calloc(sample_total,sizeof(float));  //按照采样数定义动态数组
    float angle_el_zero_all = 0;
    if(angle_el_zero == NULL)
    {   
        //打印报错信息
        return;
    }

    set_svpwm(motor,0.0f, motor->MotorConfig.UMAX*0.5f , 0.0f);
    motor->MotorDrv.Delayms(500);
    for(int i = 0; i<sample_total ;i++)
    {
        set_svpwm(motor,motor->MotorConfig.UMAX*0.5f,0,Limit_angle_el(i*(2*PI*motor->MotorConfig.Pole_pairs)/sample_total));
        motor->MotorDrv.Delayms(1);
        angle_el_zero[i] = ( ((float)i) / ((float)sample_total) ) * 2*PI - motor->MotorDrv.Cal_Angle( motor->MotorDrv.Update_Angle_raw() );
    }
    motor->MotorDrv.Delayms(500);
    for(int i = sample_total-1;i>=0;i--)
    {
        set_svpwm(motor,motor->MotorConfig.UMAX*0.5f,0,Limit_angle_el(i*(2*PI*motor->MotorConfig.Pole_pairs)/sample_total));
        motor->MotorDrv.Delayms(1);
        angle_el_zero[i] += ( ((float)i) / ((float)sample_total) ) * 2*PI - motor->MotorDrv.Cal_Angle( motor->MotorDrv.Update_Angle_raw() );
        angle_el_zero[i] /= 2 ;
    }
    for(int i = 0; i<sample_total ;i++)
    {
        angle_el_zero_all += angle_el_zero[i];
    }
    motor->MotorConfig.angle_el_zero = angle_el_zero_all/(float)sample_total;
    for(int i = 0; i<sample_total ;i++)
    {
        angle_el_zero[i] -= motor->MotorConfig.angle_el_zero; 
    }
    int LUT_total = sizeof(motor->MotorConfig.NLLUT_encoder)/sizeof(float);
    map_samples_to_lut(angle_el_zero,sample_total,motor->MotorConfig.NLLUT_encoder,LUT_total);
    set_svpwm(motor,0,0,0);
}

void update_NLLUT_encoder_sensor_nonblock(Motor_HandleTypeDef *motor)
{
    // 静态变量：保存校准上下文（跨调用不丢失）
    static uint8_t state = 0;                  // 状态机：0-初始化 1-励磁稳定 2-正向采样 3-反向采样 4-算零点/去零漂 5-映射LUT 6-收尾 7-错误
    static float total_time = 0.0f;            // 累计时间（替代Delayms）
    static int sample_count = 0;               // 采样计数器（正向递增，反向递减）
    static int sample_total = 0;               // 总采样数（极对数×10）
    static float *angle_el_zero = NULL;        // 误差采样数组（正向+反向平均）
    static float angle_el_zero_all = 0.0f;     // 零点偏移总和
    static int LUT_total = 0;                  // NLLUT数组长度（预存避免重复计算）

    // 1. 累加控制周期时间差（非阻塞计时核心）
    total_time += get_dt(motor);

    // 2. 状态切换判断（按时间/采样进度触发）
    if (state == 0) {}  // 初始化：首次调用触发，不依赖时间
    else if (state == 1)
    {
        // 励磁稳定：累计0.5s后切换
        if (total_time >= 0.5f)
        {
            total_time = 0.0f;
            sample_count = 0;
            state = 2;
        }
    }
    else if (state == 2)
    {
        // 正向采样：采样数达总采样数后切换
        if (sample_count >= sample_total)
        {
            total_time = 0.0f;
            sample_count = sample_total - 1;
            state = 3;
        }
    }
    else if (state == 3)
    {
        // 反向采样：采样数<0后切换
        if (sample_count < 0)
        {
            total_time = 0.0f;
            state = 4;
        }
    }
    else if (state == 4)
    {
        // 算零点/去零漂：执行一次后切换
        state = 5;
    }
    else if (state == 5)
    {
        // 映射LUT：执行一次后切换
        state = 6;
    }
    else if (state == 6)
    {
        // 收尾：执行一次后切换至结束
        state = 7;
    }

    // 3. 状态机逻辑执行
    switch (state)
    {
        // 状态0：初始化（分配内存、重置参数）
        case 0:
            // 重置上下文
            total_time = 0.0f;
            sample_count = 0;
            angle_el_zero_all = 0.0f;
            sample_total = motor->MotorConfig.Pole_pairs * 10;
            LUT_total = sizeof(motor->MotorConfig.NLLUT_encoder) / sizeof(float);

            // 分配采样数组
            angle_el_zero = (float*)calloc(sample_total, sizeof(float));
            if (angle_el_zero == NULL || LUT_total == 0)
            {
                state = 7;
                break;
            }

            // 启动d轴励磁
            set_svpwm(motor, 0.0f, motor->MotorConfig.UMAX * 0.5f, 0.0f);
            state = 1;
            break;

        // 状态1：d轴励磁稳定（等待转子停稳）
        case 1:
            set_svpwm(motor, 0.0f, motor->MotorConfig.UMAX * 0.5f, 0.0f);
            break;

        // 状态2：正向采样（按顺序采全周期误差）
        case 2:
        {
            // 计算当前目标电角度
            float target_el_angle = Limit_angle_el((float)sample_count * (2 * PI * motor->MotorConfig.Pole_pairs) / (float)sample_total);
            // 驱动至目标角度
            set_svpwm(motor, motor->MotorConfig.UMAX * 0.5f, 0.0f, target_el_angle);
            // 采样误差（理论电角度 - 实际编码器角度）
            angle_el_zero[sample_count] = ( (float)sample_count / (float)sample_total ) * 2 * PI- motor->MotorDrv.Cal_Angle(motor->MotorDrv.Update_Angle_raw());      
            sample_count++;
            total_time = 0.0f;  // 确保采样间隔
        }
        break;

        // 状态3：反向采样（逆序采样，误差平均）
        case 3:
        {
            // 计算当前目标电角度（同正向对应点）
            float target_el_angle = Limit_angle_el((float)sample_count * (2 * PI * motor->MotorConfig.Pole_pairs) / (float)sample_total);
            // 驱动至目标角度
            set_svpwm(motor, motor->MotorConfig.UMAX * 0.5f, 0.0f, target_el_angle);
            // 采样反向误差并与正向平均
            float reverse_error = ( (float)sample_count / (float)sample_total ) * 2 * PI- motor->MotorDrv.Cal_Angle(motor->MotorDrv.Update_Angle_raw());angle_el_zero[sample_count] = (angle_el_zero[sample_count] + reverse_error) / 2.0f;
            sample_count--;
            total_time = 0.0f;  // 确保采样间隔
        }
        break;

        // 状态4：计算零点偏移 + 误差去零漂
        case 4:
            // 算角度零点偏移（angle_el_zero）
            for (int i = 0; i < sample_total; i++)
            {
                angle_el_zero_all += angle_el_zero[i];
            }
            motor->MotorConfig.angle_el_zero = angle_el_zero_all / (float)sample_total;
            // 误差数组减去零点偏移（保留纯非线性误差）
            for (int i = 0; i < sample_total; i++)
            {
                angle_el_zero[i] -= motor->MotorConfig.angle_el_zero;
            }
            break;

        // 状态5：采样点映射到NLLUT数组
        case 5:
            map_samples_to_lut(angle_el_zero, sample_total, motor->MotorConfig.NLLUT_encoder, LUT_total);
            break;

        // 状态6：收尾（停止PWM、释放内存）
        case 6:
            set_svpwm(motor, 0.0f, 0.0f, 0.0f);
            free(angle_el_zero);
            angle_el_zero = NULL;
            break;

        // 状态7：结束/错误处理
        case 7:
            // 错误时释放残留内存
            if (angle_el_zero != NULL)
            {
                free(angle_el_zero);
                angle_el_zero = NULL;
            }
            break;

        // 默认：异常重置
        default:
            set_svpwm(motor, 0.0f, 0.0f, 0.0f);
            if (angle_el_zero != NULL)
            {
                free(angle_el_zero);
                angle_el_zero = NULL;
            }
            state = 0;
            break;
    }
}

