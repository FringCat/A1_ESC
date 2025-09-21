#include "foc_alg.h"
#include <stdio.h>
#include <math.h>
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
    if (e > r) {
        return r;
    } else if (e < -r) {
        return -r;
    } else {
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
    motor->MotorAlg.angle_el = Calculate_angle_el(motor->MotorConfig.Pole_pairs, motor->MotorDrv.Cal_Angle(motor->MotorDrv.Update_Angle_raw()), motor->MotorConfig.angle_el_zero);
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
	float _Ub = Limit(motor->MotorAlg.UB/motor->MotorConfig.UMAX  , 1 , 0 );
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

void set_pwm(Motor_HandleTypeDef *motor,float Ua , float Ub ,float Uc)
{
    float _Ua = Limit(Ua/motor->MotorConfig.UMAX , 1 , 0 );//计算并限制ABC相所需的占空比
	float _Ub = Limit(Ub/motor->MotorConfig.UMAX  , 1 , 0 );
	float _Uc = Limit(Uc/motor->MotorConfig.UMAX , 1 , 0 );

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

void set_pwm_nodir(Motor_HandleTypeDef *motor,float Ua , float Ub ,float Uc)
{
    float _Ua = Limit(Ua/motor->MotorConfig.UMAX , 1 , 0 );//计算并限制ABC相所需的占空比
    float _Ub = Limit(Ub/motor->MotorConfig.UMAX  , 1 , 0 );
    float _Uc = Limit(Uc/motor->MotorConfig.UMAX , 1 , 0 );

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
    motor->MotorAlg.Velocity = Calculate_velocity_LPF(motor->MotorAlg.angle, motor->MotorAlg.last_angle, motor->time.dt, motor->MotorData.Velocity_LPF.last_output, motor->MotorData.Velocity_LPF.alpha);
    motor->MotorData.Velocity_LPF.last_output = motor->MotorAlg.Velocity; // 更新滤波器的上次输出值
    motor->MotorAlg.last_angle = motor->MotorAlg.angle; // 更新上一时刻角度
    return motor->MotorAlg.Velocity;
}

float update_velocity_raw(Motor_HandleTypeDef *motor)
{
    motor->MotorData.Velocity_raw = Calculate_velocity_raw(motor->MotorAlg.angle, motor->MotorAlg.last_angle, motor->time.dt);
    motor->MotorAlg.last_angle = motor->MotorAlg.angle; // 更新上一时刻角度
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
    set_pwm(motor,motor->MotorAlg.UA , motor->MotorAlg.UB , motor->MotorAlg.UC);
}

void set_spwm(Motor_HandleTypeDef *motor,float Uq, float Ud ,float angle_el)
{
    float *Upark = Calculate_Park_N(Uq , Ud , angle_el);
    float *Uclark = Calculate_Clark_N(Upark[0] , Upark[1] , motor->MotorConfig.UMAX);
    set_pwm_nodir(motor,Uclark[0] , Uclark[1] , Uclark[2]);
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
        motor->MotorData.I_raw.IA_raw = motor->MotorDrv.Update_Ia_raw();
        motor->MotorData.I_raw.IB_raw = motor->MotorDrv.Update_Ib_raw();
        motor->MotorData.I_raw.IC_raw = motor->MotorDrv.Update_Ic_raw();
        
        motor->MotorAlg.IA = motor->MotorDrv.Cal_Ia(motor->MotorData.I_raw.IA_raw , motor->MotorData.IA_offset);
        motor->MotorAlg.IB = motor->MotorDrv.Cal_Ib(motor->MotorData.I_raw.IB_raw , motor->MotorData.IB_offset);
        motor->MotorAlg.IC = motor->MotorDrv.Cal_Ic(motor->MotorData.I_raw.IC_raw , motor->MotorData.IC_offset);
        
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
    float angle_el_start = 0 , angle_el_end = 0 ;
    set_svpwm(motor,0.0f, motor->MotorConfig.UMAX*0.5f , 0.0f);
    motor->MotorDrv.Delayms(2000);
    angle_el_start = update_angle_el(motor);
    for(int i=0 ; i<10000 ; i++)
    {
        set_svpwm(motor,0.0f, motor->MotorConfig.UMAX*0.5f , Limit_angle_el((float)i*0.001f));
        motor->MotorDrv.Delayms(1);
    }
    motor->MotorDrv.Delayms(2000);
    angle_el_end = update_angle_el(motor);
    motor->MotorConfig.Pole_pairs = (uint32_t)round(myabs((float)(10000*0.001f)/(angle_el_end - angle_el_start)));
    set_svpwm(motor,0.0f, 0.0f , 0.0f);
}

void update_pole_pairs_sensor_nonblock(Motor_HandleTypeDef *motor)
{
    static float angle_el_start = 0 , angle_el_end = 0 ;
    static uint8_t state = 0 ;
    static float total_time = 0 ;

    total_time += get_dt(motor);
    if(total_time < 2.0f)
    {
        state = 0;
    }
    else if(total_time >= 2.0f && total_time < 4.0f)
    {
        state = 1;
    }
    else if(total_time >= 4.0f && total_time < 14.0f)
    {
        state = 2;
    }
    else if(total_time >= 14.0f)
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
            set_svpwm(motor,0.0f, motor->MotorConfig.UMAX*0.5f , 0.0f);
            angle_el_start = update_angle_el(motor);
        }break;
        case 2:
        {
            set_svpwm(motor,0.0f, motor->MotorConfig.UMAX*0.5f , Limit_angle_el((total_time - 4.0f)*1.0f));
        }break;
        case 3:
        {
            angle_el_end = update_angle_el(motor);
            motor->MotorConfig.Pole_pairs = (uint32_t)round(myabs((float)(10.0f)/(angle_el_end - angle_el_start)));
            set_svpwm(motor,0.0f, 0.0f , 0.0f);
        }
        default:
        {
            /* 打印报错信息 */
        }break;
    }
}