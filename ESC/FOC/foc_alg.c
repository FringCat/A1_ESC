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

float update_angle_el(Motor_HandleTypeDef *motor, float angle) 
{
    motor->MotorAlg.angle_el = Limit_angle_el(angle * motor->MotorConfig.Pole_pairs - motor->MotorConfig.angle_el_zero);
    return motor->MotorAlg.angle_el;
}

float Calculate_angle_el(float Pole_pairs,float angle,float angle_el_zero) 
{
    return Limit_angle_el(angle * Pole_pairs - angle_el_zero);
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

	static float Upark_N[2];
	
	Upark_N[0] = motor->MotorAlg.Ud*cos(motor->MotorAlg.angle_el) - motor->MotorAlg.Uq*sin(motor->MotorAlg.angle_el); //Park逆变换 ①Ualpha = Ud * cosθ - Uq * sinθ
    Upark_N[1] = motor->MotorAlg.Uq*cos(motor->MotorAlg.angle_el) + motor->MotorAlg.Ud*sin(motor->MotorAlg.angle_el); //           ②Ubeta  = Uq * cosθ + Ud * sinθ
	
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
    static float Uclark_N[3];
    
    //Clark逆变换:
    Uclark_N[0] = motor->MotorAlg.Ualpha + motor->MotorConfig.UMAX/2;                 // ①Ua = Ualpha ;
    Uclark_N[1] = (sqrt_3*motor->MotorAlg.Ubeta-motor->MotorAlg.Ualpha)/2 + motor->MotorConfig.UMAX/2; // ②Ub = (√3 * Ubeta - Ualpha)/2 ;
    Uclark_N[2] = -(motor->MotorAlg.Ualpha + sqrt_3*motor->MotorAlg.Ubeta)/2 + motor->MotorConfig.UMAX/2;// ③Uc = ( -Ualpha - √3 * Ubeta )/2;
    
    motor->MotorAlg.UA = Uclark_N[0];
    motor->MotorAlg.UB = Uclark_N[1];
    motor->MotorAlg.UC = Uclark_N[2];
    
    return Uclark_N;
    
}

void update_pwm(Motor_HandleTypeDef *motor)
{
    float _Ua = Limit(motor->MotorAlg.UA/motor->MotorConfig.UMAX , 1 , 0 );//计算并限制ABC相所需的占空比
	float _Ub = Limit(motor->MotorAlg.UB/motor->MotorConfig.UMAX  , 1 , 0 );
	float _Uc = Limit(motor->MotorAlg.UC/motor->MotorConfig.UMAX , 1 , 0 );

    if(motor->MotorDrv.SetPWM_A!=NULL | motor->MotorDrv.SetPWM_B!=NULL | motor->MotorDrv.SetPWM_C!=NULL)
    {
        switch (motor->MotorConfig.DIR)
        {
            case 1:
            {
                motor->MotorDrv.SetPWM_A(_Ua);
                motor->MotorDrv.SetPWM_B(_Ub);
                motor->MotorDrv.SetPWM_C(_Uc);
            }break;
            case 2:
            {
                motor->MotorDrv.SetPWM_A(_Ub);
                motor->MotorDrv.SetPWM_B(_Ua);
                motor->MotorDrv.SetPWM_C(_Uc);
            }break;
            case 3:
            {
                motor->MotorDrv.SetPWM_A(_Uc);
                motor->MotorDrv.SetPWM_B(_Ub);
                motor->MotorDrv.SetPWM_C(_Ua);
            }break;
            case 4:
            {
                motor->MotorDrv.SetPWM_A(_Uc);
                motor->MotorDrv.SetPWM_B(_Ua);
                motor->MotorDrv.SetPWM_C(_Ub);
            }break;
            case 5:
            {
                motor->MotorDrv.SetPWM_A(_Ub);
                motor->MotorDrv.SetPWM_B(_Uc);
                motor->MotorDrv.SetPWM_C(_Ua);
            }break;
            case 6:
            {
                motor->MotorDrv.SetPWM_A(_Ua);
                motor->MotorDrv.SetPWM_B(_Uc);
                motor->MotorDrv.SetPWM_C(_Ub);
            }break;
            default:
            {
                motor->MotorDrv.SetPWM_A(_Ua);
                motor->MotorDrv.SetPWM_B(_Ub);
                motor->MotorDrv.SetPWM_C(_Uc);
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

    if(motor->MotorDrv.SetPWM_A!=NULL | motor->MotorDrv.SetPWM_B!=NULL | motor->MotorDrv.SetPWM_C!=NULL)
    {
        switch (motor->MotorConfig.DIR)
        {
            case 1:
            {
                motor->MotorDrv.SetPWM_A(_Ua);
                motor->MotorDrv.SetPWM_B(_Ub);
                motor->MotorDrv.SetPWM_C(_Uc);
            }break;
            case 2:
            {
                motor->MotorDrv.SetPWM_A(_Ub);
                motor->MotorDrv.SetPWM_B(_Ua);
                motor->MotorDrv.SetPWM_C(_Uc);
            }break;
            case 3:
            {
                motor->MotorDrv.SetPWM_A(_Uc);
                motor->MotorDrv.SetPWM_B(_Ub);
                motor->MotorDrv.SetPWM_C(_Ua);
            }break;
            case 4:
            {
                motor->MotorDrv.SetPWM_A(_Uc);
                motor->MotorDrv.SetPWM_B(_Ua);
                motor->MotorDrv.SetPWM_C(_Ub);
            }break;
            case 5:
            {
                motor->MotorDrv.SetPWM_A(_Ub);
                motor->MotorDrv.SetPWM_B(_Uc);
                motor->MotorDrv.SetPWM_C(_Ua);
            }break;
            case 6:
            {
                motor->MotorDrv.SetPWM_A(_Ua);
                motor->MotorDrv.SetPWM_B(_Uc);
                motor->MotorDrv.SetPWM_C(_Ub);
            }break;
            default:
            {
                motor->MotorDrv.SetPWM_A(_Ua);
                motor->MotorDrv.SetPWM_B(_Ub);
                motor->MotorDrv.SetPWM_C(_Uc);
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

    if(motor->MotorDrv.SetPWM_A!=NULL | motor->MotorDrv.SetPWM_B!=NULL | motor->MotorDrv.SetPWM_C!=NULL)
    {
        motor->MotorDrv.SetPWM_A(_Ua);
        motor->MotorDrv.SetPWM_B(_Ub);
        motor->MotorDrv.SetPWM_C(_Uc);
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

float update_velocity(Motor_HandleTypeDef *motor)
{
    if (motor->time.dt <= 0) 
    {
        return motor->MotorAlg.Velocity; // 避免除以零
    }

    // motor->MotorData.Velocity_raw = (motor->MotorAlg.angle - motor->MotorData.LastAngle) / motor->time.dt ;
    // 计算原始速度
    if(fabs(motor->MotorAlg.angle -  motor->MotorData.LastAngle) > (0.8f*2*PI))
	{
		if((motor->MotorAlg.angle-  motor->MotorData.LastAngle)<0){motor->MotorData.Velocity_raw = (2*PI -  motor->MotorData.LastAngle + motor->MotorAlg.angle)/motor->time.dt ;}//正转
		else if((motor->MotorAlg.angle -  motor->MotorData.LastAngle)>=0){motor->MotorData.Velocity_raw = -(2*PI - motor->MotorAlg.angle +  motor->MotorData.LastAngle)/motor->time.dt ;}//反转
	}
	else 
	{
		motor->MotorData.Velocity_raw = (motor->MotorAlg.angle -  motor->MotorData.LastAngle)/motor->time.dt ;
	}

    // 应用低通滤波器平滑速度
    motor->MotorAlg.Velocity = Calculate_LPF(motor->MotorData.Velocity_raw, motor->MotorData.LastVelocity, motor->MotorData.alpha);

    // 更新历史值
    motor->MotorData.LastAngle = motor->MotorAlg.angle;
    motor->MotorData.LastVelocity = motor->MotorAlg.Velocity;

    return motor->MotorAlg.Velocity;
}

float get_velocity(Motor_HandleTypeDef *motor)
{
    return motor->MotorAlg.Velocity;
}

float Calculate_velocity(float angle, float last_angle, float dt, float last_velocity, float alpha)
{
    if (dt <= 0) 
    {
        return last_velocity; // 避免除以零
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

    // 应用低通滤波器平滑速度
    float velocity = Calculate_LPF(velocity_raw, last_velocity, alpha);

    return velocity;
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