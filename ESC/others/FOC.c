#include "as5047p.h"
#include "FOC.h"
extern TIM_HandleTypeDef htim1;
/*===================================================移植时需要修改的部分==============================================================*/
#define Upower 24.0f
#define Ipower 24.0f
#define A_PWM_Period 4250
#define B_PWM_Period 4250
#define C_PWM_Period 4250
#define PWMPeriod A_PWM_Period

#define PI 3.14159265359f
#define _3PI_2 4.71238898038f
#define sqrt_3 1.732050807568877293f 

#define I_ADC_CONV    0.0040283203125
#define U_ADC_CONV    0.000244140625*6                                                  
#define _1_SQRT3   0.57735026919f
#define _2_SQRT3   1.15470053838f

Motor_t Motor;

void DriftOffsets(void)
{
	uint16_t detect_rounds = 100;
	for(int i = 0; i < detect_rounds; i++)
	{
		Motor.IA_offset += ((double)(HAL_ADC_GetValue(&hadc1))*I_ADC_CONV);
		Motor.IB_offset += ((double)(HAL_ADC_GetValue(&hadc1))*I_ADC_CONV);

	}
	Motor.IA_offset = Motor.IA_offset / detect_rounds;
	Motor.IB_offset = Motor.IB_offset / detect_rounds;

}

void SetMotor_Velocity(float Velocity_motor)
{
	Motor.time.dt = Motor.Getdt(&Motor.time);
	float Iq1 = PID_velocity(&Motor,Velocity_motor);
	SVPWM(&Motor,Iq1,GetElectricalAngle(&Motor));
	// printf("%f,%f\n",Motor.Velocity,Iq1);
}

void FOCparamInit(Motor_t* motor) 
{
    if (&motor == NULL) 
	{
        return; // 确保传入的指针有效
    }

    // 使用 memset 将整个结构体清零
    memset(&motor, 0, sizeof(Motor_t*));
}

uint32_t ticks = 0;
double Getdt(Time_t* time)
{
	if(ticks == 0)
	{
		time->PastTime = time->ThisTime;
		time->ThisTime =__HAL_TIM_GET_COUNTER(&htim3);
		if(time->ThisTime > time->PastTime)
		{
			time->dt = (double)(time->ThisTime - time->PastTime)/(double)(170000000.0/300.0);
		}
		else
		{
			time->dt = (double)(65535 - time->PastTime + time->ThisTime)/(double)(170000000.0/300.0);
		}
		ticks = 1 ;
	}
    
	return  time->dt ;
}
void FOC_SetPWM(uint16_t a,uint16_t b,uint16_t c)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,a);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,b);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,c);
}
void FOC_Delayms(uint16_t ms)
{
	HAL_Delay(ms);
}
float FOC_GetAngle_Sensor(void)
{
	return GetAngle(); //AS5047p
}
void FOC_GetIaIbIc(void)
{
	// static float Last_IA=0,Last_IB=0,Last_IC=0;
	// Motor.IA = ((double)(adc_convert(ADC0_CH0_A0))*I_ADC_CONV-Motor.IA_offset);
	// Motor.IB = ((double)(adc_convert(ADC0_CH6_A6))*I_ADC_CONV-Motor.IB_offset);
	// Motor.IC = -((double)(adc_convert(ADC0_CH3_A3))*I_ADC_CONV-Motor.IC_offset);
	// Motor.IA = (double)(adc_convert(ADC0_CH0_A0));
	// Motor.IB = (double)(adc_convert(ADC0_CH1_A1));
	// Motor.IC = (double)(adc_convert(ADC0_CH3_A3));
	// Motor.IA = LowPassFilter(Last_IA,Motor.IA,0.01/(0.01+Motor.time.dt));
	// Motor.IB = LowPassFilter(Last_IB,Motor.IB,0.01/(0.01+Motor.time.dt));
	// Motor.IC = LowPassFilter(Last_IC,Motor.IC,0.01/(0.01+Motor.time.dt));
	// Last_IA = Motor.IA;
	// Last_IB = Motor.IB;
	// Last_IC = Motor.IC;
}

void FOC_init(void)
{
	FOCparamInit(&Motor);
	
	Motor.Delayms = FOC_Delayms;
	Motor.GetAngle = FOC_GetAngle_Sensor;
	Motor.Getdt = Getdt;
	Motor.SetPWM = FOC_SetPWM;
	Motor.GetIaIbIc = FOC_GetIaIbIc;

	Motor.angle_el_zero = 0.517368;//2.540426;

	Motor.motor_number = 1;
	Motor.DIR = -1 ;//电机1单速度环
	Motor.PID_param[0].KP= 0.3;
	Motor.PID_param[0].KI= 0.1;
	Motor.T = 0.01;
	Motor.Pole_pairs = 14;

	Motor.PID_param[1].KP= 2.6;
	Motor.PID_param[1].KI= 4.8;
	Motor.PID_param[1].KD= 0.0;	

	Motor.PID_param[2].KP= 2.0;//电机1三环速度环
	Motor.PID_param[2].KI= 0.0;

	Motor.PID_param[3].KP= 0.4;//电机1三环角度环
	Motor.PID_param[3].KI= 0.0;
	Motor.PID_param[3].KD= 0.0;

	Motor.PID_param[4].KP= 154.0;//电机1三环角速度环  p0.126
	Motor.PID_param[4].KI= 0.0;
	Motor.PID_param[4].KD= 0.00;

}
// float GetAngle(void)
// {
// 	  u16 Angle_Digit = 0 ;
// 	  float Angle = 0 ;
// 	  Angle_Digit = AS5600_ReadTwoByte(_raw_ang_hi,_raw_ang_lo); 
// 	  Angle=Angle_Digit*360/4096*(PI/180);

// 	return Angle;

// }
// float GetAngle_(void)
// { 
//  	static float full_rotations = 0.0;
//     static float Last_Angle = 0.0;
//  	float D_Angle = 0.0;
//  	float Angle = FOC_GetAngle_Sensor();
//  	D_Angle = Angle - Last_Angle;

//  	if( fabs(D_Angle) > (0.8f*2.0f*PI) )
//  	{
//  		full_rotations = full_rotations + ((D_Angle > 0) ? -1 :1);
//  	}
	
//  	Last_Angle = Angle;
//  	return (full_rotations * 2.0f * PI + Last_Angle);
//  }
/*==========================================================================================================================*/

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

float Limit_ElectricalAngle(float ElectricalAngle)
{
	/*
	电角度限制函数，用于将获取到的角度值通通限制在 2π之内，
	一方面减少计算量，更重要的是防止由于随着电机旋转圈数的增加，
	反馈回来的角度值无限制地叠加，使得存放角度值的变量发生溢出，这是很危险的.
	言归正传，只要时刻对存放角度值的变量进行以 2π 为单位的取余，就可以让该变量保持在 2π 之内 ，防止溢出.
	限制的实现思路就好比把某个正弦值 2kπ+ a 中的 a 提取出来----把2kπ给取余掉.
	函数输入 电机角度或电角度(angle) ，返回 float类型的取余限制后的角度值 .
	*/
	
  float a = fmod(ElectricalAngle, 2*PI);    //对输入的角度(angle)进行以 2π为单位的取余，fmod 是取余函数，
	                                //其功能类似C语言的取余符号“%”，返回值是取余后在 2π之内 的值.
	
  return (a >= 0 ? a : (a + 2*PI)); //在返回时仍需注意:fmod函数如果对 小于2π 的值进行取余，
									//那么则会返回 -(2π- a) 这样的负值，相当于在a的基础上再减一个2π，使之变成了负值,
	                                //负值的角度我们是不能直接使用的，所以我们先加个2π把它变成正值，
	                                //当然你也可以理解为:正值才是他原本的值.
	
	/*
	备注:在 FOC_Config.h 头文件中，本库定义了默认使用的π(宏:PI),
	在本库的例程中默认使用该宏(PI)来进行有关π的计算,使用者可根据情况按照自己需求更改.
	*/
	
}
float GetElectricalAngle(Motor_t* motor) //用于转动
{
	/*
	电角度计算函数，输入 电机当前角度(angle) 和 电机的极对数(pole_pairs),
	返回当前电机的 电角度(电机角度*极对数),
	返回值是 float类型的电角度(angle * pole_pairs).
	*/
	
    return Limit_ElectricalAngle(motor->DIR*motor->angle * motor->Pole_pairs - motor->angle_el_zero);//电机电角度 = 电机当前机械角度(angle) * 电机极对数(pole_pairs)

	/*
	备注:在 FOC_Config.h 头文件中，本库定义了电机默认使用的电机极对数(宏:Pole_pairs),
	在本库的例程中默认使用该宏(Pole_pairs)来进行有关电机极对数的计算,使用者可根据情况按照自己需求更改.
	*/
} 

float GetElectricalAngle__(Motor_t* motor)//用于计算 
{
	/*
	电角度计算函数，输入 电机当前角度(angle) 和 电机的极对数(pole_pairs),
	返回当前电机的 电角度(电机角度*极对数),
	返回值是 float类型的电角度(angle * pole_pairs).
	*/
    return Limit_ElectricalAngle(motor->angle * motor->Pole_pairs - motor->angle_el_zero);//电机电角度 = 电机当前机械角度(angle) * 电机极对数(pole_pairs)

	/*
	备注:在 FOC_Config.h 头文件中，本库定义了电机默认使用的电机极对数(宏:Pole_pairs),
	在本库的例程中默认使用该宏(Pole_pairs)来进行有关电机极对数的计算,使用者可根据情况按照自己需求更改.
	*/
} 

float angle_el_zero1 = 0;
float GetElectricalAngle_(float DIR,float Pole_pairs,float angle) 
{
	extern float angle_el_zero1;
    return Limit_ElectricalAngle(DIR*angle * Pole_pairs - angle_el_zero1); ;//电机电角度 = 电机当前机械角度(angle) * 电机极对数(pole_pairs)

} 


float Limit(float value , float high , float low)
{
	/*限制值函数，用于限制某些输入参数的大小，为了安全(?)，例如占空比，电压等等，
	  输入需要限制的目标参数值(value),最大值(high),最小值(low),
		返回 float类型的被限制后的值.
	*/
	return (value)<(low)?(low):((value)>(high)?(high):(value));//如果目标参数超出最大/最小值的范围，就把这个值锁死在最大/最小值
}

float *Park_N(float Uq , float ElectricalAngle)
{
	/*
	Park逆变换函数，输入 Uq/Iq(控制力矩用的电压/电流) 和 电角度(电机极对数*电机当前角度) ，
	返回经过 Park逆变换 后的 Ualpha 与 Ubeta ，
	返回值是一个 指向float类型的数组的指针 ，数组的第一个数是 Ualpha ,第二个是 Ubeta.
	*/
	static float Upark_N[2];//Upark_N[2]存储着 经过Park逆变换后的 Ualpha 和 Ubeta .
	
	Upark_N[0] =  -Uq*sin(ElectricalAngle); //Park逆变换 ①Ualpha = Ud * cosθ - Uq * sinθ
    Upark_N[1] =   Uq*cos(ElectricalAngle); //           ②Ubeta  = Uq * cosθ + Ud * sinθ
	return Upark_N;
	
	/* 备注: Ud一般设置为0 , 所以这里的Park逆变换看起来少了一部分，就是 Ud 那部分 */
}

float *Park_N2(float Uq , float Ud , float ElectricalAngle)
{
	/*
	Park逆变换函数，输入 Uq/Iq(控制力矩用的电压/电流) 和 电角度(电机极对数*电机当前角度) ，
	返回经过 Park逆变换 后的 Ualpha 与 Ubeta ，
	返回值是一个 指向float类型的数组的指针 ，数组的第一个数是 Ualpha ,第二个是 Ubeta.
	*/
	static float Upark_N[2];//Upark_N[2]存储着 经过Park逆变换后的 Ualpha 和 Ubeta .
	
	Upark_N[0] = Ud*cos(ElectricalAngle) - Uq*sin(ElectricalAngle); //Park逆变换 ①Ualpha = Ud * cosθ - Uq * sinθ
    Upark_N[1] = Uq*cos(ElectricalAngle) + Ud*sin(ElectricalAngle); //           ②Ubeta  = Uq * cosθ + Ud * sinθ
	return Upark_N;
	
	/* 备注: Ud一般设置为0 , 所以这里的Park逆变换看起来少了一部分，就是 Ud 那部分 */
}

float *Clark_N(float Ualpha ,float Ubeta)
{
  /*
	Clark逆变换函数，输入 Ualpha 和 Ubeta ，
	返回经过 Clark逆变换 后的 Ua ,Ub ,Uc ，这三个值用于设置三相输出的PWM占空比，
	返回值也是一个 float类型的数组指针(Uclark_N) ，数组的第一个数是 Ua ，第二个是 Ub , 第三个是 Uc.
	*/	
	static float Uclark_N[3];//Uclark_N[3]存储着 经过Clark逆变换后的 Ua 和 Ub ,Uc .
	
	//Clark逆变换:
	Uclark_N[0] = Ualpha + Upower/2;                 // ①Ua = Ualpha ;
	Uclark_N[1] = (sqrt(3)*Ubeta-Ualpha)/2 + Upower/2; // ②Ub = (√3 * Ubeta - Ualpha)/2 ;
	Uclark_N[2] = -(Ualpha + sqrt(3)*Ubeta)/2 + Upower/2;// ③Uc = ( -Ualpha - √3 * Ubeta )/2;
	
	return Uclark_N;
	
	/*
	备注:在 FOC_Config.h 头文件中，本库定义了默认使用的电源电压(宏:Upower),
	在本库的例程中默认使用该宏(Upower)来进行有关电源电压的计算,使用者可根据情况按照自己需求更改.
	*/
}

void SPWM(Motor_t* motor,float Ua , float Ub ,float Uc)
{
	/*
	设置三相PWM输出函数，该函数采取SPWM的输出方式，在ABC三相输出Ua，Ub，Uc的PWM波，
	通过设置定时器外设的输出比较寄存器(ARR)值来调节占空比.
	输入ABC三相的电压值(Ua,Ub,Uc)，无返回值.
	*/
	
	float __Ua = Limit(Ua , 8 , 0 );//计算并限制ABC相所需的占空比
	float __Ub = Limit(Ub , 8 , 0 );
	float __Uc = Limit(Uc , 8 , 0 );
	
	float _Ua = Limit(__Ua/Upower , 1 , 0 );//计算并限制ABC相所需的占空比
	float _Ub = Limit(__Ub/Upower , 1 , 0 );
	float _Uc = Limit(__Uc/Upower , 1 , 0 );
	
	motor->SetPWM((uint16_t)(_Ua*A_PWM_Period),(uint16_t)(_Ub*B_PWM_Period),(uint16_t)(_Uc*C_PWM_Period));//设置ABC三相占空比
}



void FOC_SPWM(Motor_t* motor,float Uq,float angle)
{
	float *Upark_N;                                    
	float *Uclark_N; 
	
	Upark_N = Park_N(Limit(Uq,Upower/2,-(Upower/2)) , GetElectricalAngle(motor));
	Uclark_N = Clark_N(Upark_N[0],Upark_N[1]);

	SPWM(motor,Uclark_N[0],Uclark_N[1],Uclark_N[2]);
}

void FOC_SPWM_(Motor_t* motor,int pwm_duty,float angle)
{
	float Uq;
	float *Upark_N;                                    
	float *Uclark_N; 
	Uq = mymap((float)pwm_duty,-1000,1000,-6,6);
	Upark_N = Park_N(Limit(Uq,Upower/2,-(Upower/2)) , GetElectricalAngle(motor));
	Uclark_N = Clark_N(Upark_N[0],Upark_N[1]);

	SPWM(motor,Uclark_N[0],Uclark_N[1],Uclark_N[2]);
}


int GetSector( float Ualpha , float Ubeta )
{
	if((Ualpha>0.0f) && (Ubeta>0.0f) && (Ubeta/Ualpha < sqrt_3)){return 1 ;}
	else if((Ubeta>0.0f) && (Ubeta/myabs(Ualpha)>sqrt_3)){return 2 ;}
	else if((Ualpha<0.0f) && (Ubeta>0.0f) && (-Ubeta/Ualpha < sqrt_3)){return 3 ;}
	else if((Ualpha<0.0f) && (Ubeta<0.0f) && (Ubeta/Ualpha < sqrt_3)){return 4 ;}
	else if((Ubeta<0.0f) && (-Ubeta/myabs(Ualpha)>sqrt_3)){return 5 ;}
	else if((Ualpha>0.0f) && (Ubeta<0.0f) && (-Ubeta/Ualpha < sqrt_3)){return 6 ;}
	else {return 0;}
}

void MotorStart(Motor_t* motor)
{   
	// FOC_SPWM(3,_3PI_2);
	SVPWM2(motor,6,0,_3PI_2);
	FOC_Delayms(1000);
	motor->angle = motor->GetAngle() ;
	motor->angle_el_zero = GetElectricalAngle(motor);//确定零角度
	printf("%f\n",motor->angle_el_zero);
	// FOC_SPWM(0,_3PI_2);
	SVPWM2(motor,0,0,_3PI_2);
	FOC_Delayms(200);
	
}

void MotorStart_Mult(void)
{   
	// FOC_SPWM(3,_3PI_2);
	SVPWM2(&Motor,6,0,_3PI_2);
	FOC_Delayms(2000);
	Motor.angle = Motor.GetAngle() ;
	Motor.angle_el_zero = GetElectricalAngle(&Motor);//确定零角度
	// FOC_SPWM(0,_3PI_2);
	SVPWM2(&Motor,0,0,_3PI_2);
	FOC_Delayms(200);
	SVPWM2(&Motor,1,0,_3PI_2);
	printf("%f\n",Motor.angle_el_zero);
}

float LowPassFilter(float Last_Output , float Input , float alpha)
{
	float Output = 0 ;
	Output = alpha * Last_Output + (1 - alpha) * (Input);
	return Output ;
}	

float GetVelocity(Motor_t* motor)
{
	static float dt = 0 ;
	dt = motor->Getdt(&motor->time);
	if(motor->DIR == 1)
	{
		motor->angle = 2*PI - motor->GetAngle();//有感
	}
	else
	{
		motor->angle  = motor->GetAngle();
	}
	// Angle = FOC_GetAngle_Sensorless_SMO();//无感
	float alpha = (motor->T) / (motor->T + motor->time.dt);

	if(fabs(motor->angle - motor->LastAngle) > (0.8f*2*PI))
	{
		if((motor->angle - motor->LastAngle)<0){motor->Velocity_raw = (2*PI - motor->LastAngle + motor->angle)/motor->time.dt ;}//正转
		else if((motor->angle - motor->LastAngle)>=0){motor->Velocity_raw = -(2*PI - motor->angle + motor->LastAngle)/motor->time.dt ;}//反转
	}
	else 
	{
		motor->Velocity_raw = (motor->angle - motor->LastAngle)/motor->time.dt ;
	}
	// Velocity_NoFilter = (Angle - LastAngle)/dt ; ;
	// Angle = LowPassFilter(Angle,LastAngle,alpha);
	// Velocity_NoFilter = (Angle - LastAngle)/dt ;
	motor->Velocity = LowPassFilter(motor->LastVelocity,motor->Velocity_raw,alpha);
	// printf("%f,%f,%f\n",Last_Velocity,Angle,dt);

	motor->LastAngle = motor->angle ;
	motor->LastVelocity = motor->Velocity;

	return motor->Velocity;
}

float GetVelocity_(Motor_t* motor)
{
	// static float dt = 0 ;
	// dt = motor->Getdt();
	// motor->time.dt = dt;
	// if(motor->DIR == 1)
	// {
	// 	motor->angle = 2*PI - motor->GetAngle();//有感
	// }
	// else
	// {
		motor->angle  = motor->GetAngle();

	// }
	// Angle = FOC_GetAngle_Sensorless_SMO();//无感
	float alpha = (motor->T) / (motor->T + motor->time.dt);

	if(fabs(motor->angle - motor->LastAngle) > (0.8f*2*PI))
	{
		if((motor->angle - motor->LastAngle)<0){motor->Velocity_raw = (2*PI - motor->LastAngle + motor->angle)/motor->time.dt ;}//正转
		else if((motor->angle - motor->LastAngle)>=0){motor->Velocity_raw = -(2*PI - motor->angle + motor->LastAngle)/motor->time.dt ;}//反转
	}
	else 
	{
		motor->Velocity_raw = (motor->angle - motor->LastAngle)/motor->time.dt ;
	}
	// Velocity_NoFilter = (Angle - LastAngle)/dt ; ;
	// Angle = LowPassFilter(Angle,LastAngle,alpha);
	// Velocity_NoFilter = (Angle - LastAngle)/dt ;
	motor->Velocity = LowPassFilter(motor->LastVelocity,motor->Velocity_raw,alpha);
	motor->LastAngle = motor->angle ;
	motor->LastVelocity = motor->Velocity;

	
	motor->Velocity = motor->DIR*motor->Velocity;
	// printf("%f,%f,%f\n",Last_Velocity,Angle,dt);

	return motor->Velocity;
}

float PID_velocity(Motor_t* motor,float TargetVelocity)
{
	// static float Kp = 0.008 ;//
	// static float Ki = 0.03 ;//

	GetVelocity(motor) ;	

	motor->PID_param[0].error = (TargetVelocity - motor->Velocity);
	
	motor->PID_param[0].This_I = motor->PID_param[0].Last_I + motor->PID_param[0].KI*motor->time.dt*motor->PID_param[0].error;
	motor->PID_param[0].This_I = Limit(motor->PID_param[0].This_I,Ipower/2,-Ipower/2);
	
	motor->PID_param[0].Output = motor->PID_param[0].KP*motor->PID_param[0].error + motor->PID_param[0].This_I;
	motor->PID_param[0].Output = Limit(motor->PID_param[0].Output , Ipower/2 , -Ipower/2);
	
	motor->PID_param[0].Last_I = motor->PID_param[0].This_I;

	// printf("%f,%f,%f,%f\n",NowVelocity,TargetVelocity,Output,dt);//20 6

	return  motor->PID_param[0].Output ;
}

float PID_Iq(Motor_t* motor,float TargetIq)
{
	static float Last_Iq = 0;
	motor->time.dt = motor->Getdt(&motor->time);
	motor->GetIaIbIc();
	motor->angle = motor->GetAngle();
	motor->angle_el = GetElectricalAngle(motor);

	// float mid =  (1.f/3) * (motor->IA + motor->IB + motor->IC);
	// motor->IA = motor->IA - mid ;
	// motor->IB = motor->IB - mid ;
	// motor->IC = motor->IC - mid ;
	// motor->IB = -( motor->IA + motor->IC );
	motor->Ialpha = motor->IA;
	motor->Ibeta = _1_SQRT3 * motor->IA + _2_SQRT3 * motor->IB;
	
    motor->Iq = motor->Ibeta*cos(motor->angle_el) - motor->Ialpha*sin(motor->angle_el);
	motor->Id = motor->Ialpha*cos(motor->angle_el) + motor->Ibeta*sin(motor->angle_el);

	// motor->Iq = LowPassFilter(Last_Iq,motor->Iq,0.01/(0.01+motor->time.dt));
    // Last_Iq = motor->Iq ;
	
	// printf("%f,%f,%f,%f,%f\n",motor->Ialpha,motor->Ibeta,motor->Ualpha,motor->Ubeta,motor->Iq);
	// printf("%f,%f,%f,%f\n",TargetIq,motor->Iq,motor->PID_param[1].Output,motor->PID_param[1].error);
	motor->PID_param[1].error = (TargetIq - motor->Iq);

	motor->PID_param[1].This_I = motor->PID_param[1].Last_I + motor->PID_param[1].KI*motor->time.dt*motor->PID_param[1].error;
	motor->PID_param[1].This_I = Limit(motor->PID_param[1].This_I,6,-6);

	motor->PID_param[1].Output = motor->PID_param[1].KP*motor->PID_param[1].error + motor->PID_param[1].This_I+motor->PID_param[1].KD*(motor->PID_param[1].error-motor->PID_param[1].last_error);;
	motor->PID_param[1].Output = Limit(motor->PID_param[1].Output,6,-6);

	motor->PID_param[1].Last_I = motor->PID_param[1].This_I;
	motor->PID_param[1].last_error=motor->PID_param[1].error;

}	
// float PID_I(float TargetIq)
// {
// 	static float error = 0 ;
// 	static float Lasterror = 0;
// 	static float LastI = 0 ;
// 	static float ThisI = 0 ;

// 	static float Kp = 0.09 ;//0.08 ;//0.02 ;
// 	static float Ki = 0.2 ;//0.06 ;//0.02;//0.2 ;
// 	static float Kd = 0.0;
	 
// 	static float Output = 0 ;
// 	static float LastIA = 0 ;
// 	static float LastIB = 0 ;
// 	static float LastIq = 0 ;

// 	static float Ialpha = 0;
// 	static float Ibeta = 0;
// 	static float Iq = 0 ;
// 	static float Id = 0 ;

//     // float angle = FOC_GetAngle_Sensor();
// 	float angle_el = GetElectricalAngle(Angle_all, 7);
// 	float dt = FOC_Getdt_I();

// 	value[0] = adc_convert(ADC0_CH0_A0);
// 	value[1] = adc_convert(ADC0_CH1_A1);
// 	// value[2] = adc_convert(ADC0_CH0_A0);

// 	IA = ((double)(value[0])*I_ADC_CONV-offset_ia )*5;
//     IB = ((double)(value[1])*I_ADC_CONV-offset_ib )*5; 
//     // IC = ((double)(value[2])*I_ADC_CONV-offset_ib)*5;
	
// 	// IA = LowPassFilter(LastIA,IA,0.01);
// 	// IB = LowPassFilter(LastIB,IB,0.01); 

// 	Ialpha = IA;
// 	Ibeta = _1_SQRT3 * IA + _2_SQRT3 * IB;

// 	Iq = Ibeta*cos(angle_el) - Ialpha*sin(angle_el);
// 	Id = Ialpha*cos(angle_el) + Ibeta*sin(angle_el);
// 	Iq = LowPassFilter(LastIq,Iq,0.01);

// 	error = (TargetIq - Iq) ;

// 	ThisI = LastI + Ki*dt*error;
// 	ThisI = Limit(ThisI,Upower/2,-Upower/2);

// 	Output = Kp*error + ThisI + Kd*(error-Lasterror)/dt;
// 	Output = Limit(Output , Upower/2 , -Upower/2);
	
// 	LastI = ThisI;
// 	Lasterror = error;
// 	LastIA = IA;
// 	LastIB = IB;
// 	LastIq = Iq;
// 	// printf("%f,%f,%f\n",TargetIq,Iq,Id);
// 	// printf("%f,%f\n",TargetIq,Iq);
// 	// printf("%f,%f\n",Ialpha,Ibeta);
// 	printf("%f,%f,%f\n",IA,IB,Iq);

// 	// sprintf(Buffer_UART,"%f,%f,%f,%f\n",V,TargetIq,Iq,Id);
// 	// sprintf(Buffer_UART,"%f,%f,%f\n",TargetIq,Iq,Id);
// 	// HAL_UART_Transmit_DMA(&huart1,Buffer_UART,strlen(Buffer_UART));    

// 	return Output;
// }


// float* PID_IF(float TargetIq,float Angle)
// {
// 	static int i = 0;
// 	static float error = 0 ;
// 	static float Lasterror = 0;
// 	static float LastI = 0 ;
// 	static float ThisI = 0 ;

// 	static float error_ = 0 ;
// 	static float Lasterror_ = 0;
// 	static float LastI_ = 0 ;
// 	static float ThisI_ = 0 ;

// 	static float Kp = 3.8;//0.02 ;//0.08 ;//0.02 ;
// 	static float Ki = 2.5;//0.05 ;//0.06 ;//0.02;//0.2 ;
// 	static float Kd = 0.0;

// 	static float Kp_ = 2;//0.02 ;//0.08 ;//0.02 ;
// 	static float Ki_ = 0.1;//0.05 ;//0.06 ;//0.02;//0.2 ;
// 	static float Kd_ = 0.0;
	
// 	static float Output[2];

// 	static float LastIA = 0 ;
// 	static float LastIB = 0 ;
// 	static float LastIq = 0 ;

// 	static float Ialpha = 0;
// 	static float Ibeta = 0;
// 	static float Iq = 0 ;
// 	static float Id = 0 ;

//     float angle = Angle;
// 	float angle_el = GetElectricalAngle(angle, 7);
// 	float dt = FOC_Getdt_I();

// 	//  for(int j=0;j<3;j++)
// 	// {
// 	// 	HAL_ADC_Start(&hadc1);
// 	// 	HAL_ADC_PollForConversion(&hadc1,1000);
// 	// 	value[j]=HAL_ADC_GetValue(&hadc1);
// 	// }
// 	// IA = -((double)(value[0])*I_ADC_CONV-offset_ia )*5;
//     // IB = -((double)(value[1])*I_ADC_CONV-offset_ib )*5; 
//     // IC = -((double)(value[2])*I_ADC_CONV-offset_ib)*5;
	
// 	// IA = LowPassFilter(LastIA,IA,0.01);
// 	// IB = LowPassFilter(LastIB,IB,0.01); 

// 	Ialpha = IA;
// 	Ibeta = _1_SQRT3 * IA + _2_SQRT3 * IB;

// 	Iq = Ibeta*cos(angle_el) - Ialpha*sin(angle_el);
// 	Id = Ialpha*cos(angle_el) + Ibeta*sin(angle_el);
// 	// Iq = LowPassFilter(LastIq,Iq,0.01);

// 	error = (TargetIq - Iq) ;//对q轴求误差

// 	ThisI = LastI + Ki*dt*error;
// 	ThisI = Limit(ThisI,Ipower/2,-Ipower/2);

// 	Output[0] = Kp*error + ThisI + Kd*(error-Lasterror)/dt;
// 	Output[0] = Limit(Output[0] , Ipower/2 , -Ipower/2);
	
// 	error_ = (0 - Id) ; //对d轴求误差

// 	ThisI_ = LastI_ + Ki_*dt*error_;
// 	ThisI_ = Limit(ThisI_,Ipower/2,-Ipower/2);

// 	Output[1] = Kp_*error_ + ThisI_ + Kd_*(error_-Lasterror_)/dt;
// 	Output[1] = Limit(Output[1], Ipower/2 , -Ipower/2);

// 	LastI = ThisI;
// 	Lasterror = error;
// 	LastI_ = ThisI_;
// 	Lasterror_ = error_;

// 	LastIA = IA;
// 	LastIB = IB;
// 	LastIq = Iq;
// 	// printf("%f,%f,%f\n",TargetIq,Iq,Id);
// 	// printf("%f,%f\n",TargetIq,Iq);
// 	// printf("%f,%f\n",Ialpha,Ibeta);
// 	// if(i>30)
// 	// {printf("%f\n",Iq);i=0;}
// 	// i++;
// 	// printf("%f,%f,%f\n",IA,IB,IC);
// 	// sprintf(Buffer_UART,"%f,%f\n",IA,IB);
// 	// sprintf(Buffer_UART,"%f,%f,%f,%f\n",V,TargetIq,Iq,Id);
// 	// sprintf(Buffer_UART,"%f,%f,%f\n",TargetIq,Iq,Id);
// 	// HAL_UART_Transmit_DMA(&huart1,Buffer_UART,strlen(Buffer_UART));    

// 	return Output;
// }

void SVPWM(Motor_t* motor,float Uq,float angle)
{
	float *Upark_N ;
	static float K = 0 , Ux = 0 , Uy = 0 , Uz = 0 , Tx = 0 ,Ty = 0,Tz = 0;
	static float Ta = 0 , Tb = 0 ,Tc = 0 ;
	int sector = 0 ;
	Upark_N = Park_N(Limit(motor->DIR*Uq,Ipower/2,-(Ipower/2)) , angle);
		
	sector = GetSector(Upark_N[0],Upark_N[1]);

	motor->Ualpha  = Upark_N[0];
	motor->Ubeta = Upark_N[1];

	K=(sqrt_3*1)/(Ipower/2);
	Ux = Upark_N[1];
	Uy = (sqrt_3/2.0f)*Upark_N[0] - 0.5f*Upark_N[1];
	Uz = (sqrt_3/2.0f)*Upark_N[0] + 0.5f*Upark_N[1];
	
	switch (sector)
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

	switch(sector)
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

	motor->SetPWM((uint16_t)(a*A_PWM_Period),(uint16_t)(b*B_PWM_Period),(uint16_t)(c*C_PWM_Period));

}

void SVPWM__(Motor_t* motor,int pwm_duty,float angle)
{
	float Uq;
	float *Upark_N ;
	static float K = 0 , Ux = 0 , Uy = 0 , Uz = 0 , Tx = 0 ,Ty = 0,Tz = 0;
	static float Ta = 0 , Tb = 0 ,Tc = 0 ;
	int sector = 0 ;
	Uq = mymap((float)pwm_duty,-1000,1000,-6,6);
	Upark_N = Park_N(Limit(motor->DIR*Uq,Ipower/2,-(Ipower/2)) , angle);
		
	sector = GetSector(Upark_N[0],Upark_N[1]);

	motor->Ualpha  = Upark_N[0];
	motor->Ubeta = Upark_N[1];

	K=(sqrt_3*1)/(Ipower/2);
	Ux = Upark_N[1];
	Uy = (sqrt_3/2.0f)*Upark_N[0] - 0.5f*Upark_N[1];
	Uz = (sqrt_3/2.0f)*Upark_N[0] + 0.5f*Upark_N[1];
	
	switch (sector)
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

	switch(sector)
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

	motor->SetPWM((uint16_t)(a*A_PWM_Period),(uint16_t)(b*B_PWM_Period),(uint16_t)(c*C_PWM_Period));

}
void SVPWM2(Motor_t* motor,float Uq,float Ud,float angle)
{
	float *Upark_N ;
	static float K = 0 , Ux = 0 , Uy = 0 , Uz = 0 , Tx = 0 ,Ty = 0,Tz = 0;
	static float Ta = 0 , Tb = 0 ,Tc = 0 ;  
	int sector = 0 ;
	Upark_N = Park_N2(Limit(Uq,Ipower/2,-(Ipower/2)) , Limit(Ud,Ipower/2,-(Ipower/2)) ,GetElectricalAngle(motor));
		
	sector = GetSector(Upark_N[0],Upark_N[1]);

	K=(sqrt_3*1)/(Ipower/2);
	Ux = Upark_N[1];
	Uy = (sqrt_3/2.0f)*Upark_N[0] - 0.5f*Upark_N[1];
	Uz = (sqrt_3/2.0f)*Upark_N[0] + 0.5f*Upark_N[1];
	
	switch (sector)
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

	switch(sector)
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

	motor->SetPWM((uint16_t)(a*A_PWM_Period*0.8f),(uint16_t)(b*B_PWM_Period*0.8f),(uint16_t)(c*C_PWM_Period*0.8f));

}

// void IF_Start(float TargetVelocity)
// {
// 	static float LastAngle = 0 ;
// 	static float *OUTPUT_;
// 	float dt = FOC_Getdt_IF();
// 	LastAngle = Limit_ElectricalAngle(LastAngle+TargetVelocity*dt);
// 	float *OUTPUT = PID_IF(0.3,LastAngle);

// 	OUTPUT_ = FOC_Sensorless(TargetVelocity,OUTPUT[0]);	
// 	printf("%f,%f,%f\n",OUTPUT_[0],LastAngle,OUTPUT_[1]);
// 	// SVPWM2(OUTPUT[0],OUTPUT[1],LastAngle);
// 	SVPWM(OUTPUT[0],LastAngle);
// }
void VF_Start(Motor_t* motor,float TargetVelocity,float acc ,float VF_uq_delta)
{
	static float LastAngle = 0 ;
	static float Velocity = 0;
	static float T = 0 ;
	float dt = motor->Getdt(&motor->time);
	T+=dt;
	if(Velocity<TargetVelocity)
	{
		Velocity+=acc;
	}
	else
	{
		Velocity = TargetVelocity;
	}
	// if(T<8)//8秒后切闭环
	// {
		LastAngle = Limit_ElectricalAngle(LastAngle+Velocity*dt);
		// OUTPUT_ = FOC_Sensorless(TargetVelocity,2.0f + Velocity * VF_uq_delta);	
		// printf("%f,%f,%f,%f\n",OUTPUT_[0],LastAngle,OUTPUT_[1],OUTPUT_[3]);
		printf("%f,%f,%f\n",Velocity,LastAngle,GetAngle());
		SVPWM(motor,2.0f + Velocity * VF_uq_delta,GetElectricalAngle_(-1,LastAngle, 7));
	// }
	// else
	// {
	// 	OUTPUT_ = FOC_Sensorless(50,OUTPUT_[3]);	
	// 	printf("%f,%f,%f,%f\n",OUTPUT_[0],LastAngle,OUTPUT_[1],OUTPUT_[3]);
	// 	SVPWM(OUTPUT_[3],OUTPUT_[0]);
	// }
}


float* PLL(float Valpha, float Vbeta ,float Ts) 
{
	static float Kp = 16.5;
	static float Ki = 6550;
	static float Velocity_Filter = 0 ;
	static float Theta_err = 0;
	static float I = 0 ;
	static float Theta = 0;
	static float Velocity = 0 ;

	static float OUTPUT[3] ;
	Theta_err = -1 * Valpha * cos(Theta) + Vbeta * sin(Theta) * -1;
	
	I += Ts * Ki * Theta_err;
	Velocity = Kp * Theta_err + I;
	
	Velocity_Filter = Velocity_Filter * 0.9 + Velocity * 0.1;

	Theta += Ts * Velocity;  

	Theta = Limit_ElectricalAngle(Theta);

	OUTPUT[0] = Theta;
	OUTPUT[1] = Velocity;
	OUTPUT[2] = Ts;
	return OUTPUT;
}
