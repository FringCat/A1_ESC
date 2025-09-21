#include "as5047p.h"
#include "FOC.h"
extern TIM_HandleTypeDef htim1;
/*===================================================��ֲʱ��Ҫ�޸ĵĲ���==============================================================*/
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
        return; // ȷ�������ָ����Ч
    }

    // ʹ�� memset �������ṹ������
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
	Motor.DIR = -1 ;//���1���ٶȻ�
	Motor.PID_param[0].KP= 0.3;
	Motor.PID_param[0].KI= 0.1;
	Motor.T = 0.01;
	Motor.Pole_pairs = 14;

	Motor.PID_param[1].KP= 2.6;
	Motor.PID_param[1].KI= 4.8;
	Motor.PID_param[1].KD= 0.0;	

	Motor.PID_param[2].KP= 2.0;//���1�����ٶȻ�
	Motor.PID_param[2].KI= 0.0;

	Motor.PID_param[3].KP= 0.4;//���1�����ǶȻ�
	Motor.PID_param[3].KI= 0.0;
	Motor.PID_param[3].KD= 0.0;

	Motor.PID_param[4].KP= 154.0;//���1�������ٶȻ�  p0.126
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
	��Ƕ����ƺ��������ڽ���ȡ���ĽǶ�ֵͨͨ������ 2��֮�ڣ�
	һ������ټ�����������Ҫ���Ƿ�ֹ�������ŵ����תȦ�������ӣ�
	���������ĽǶ�ֵ�����Ƶص��ӣ�ʹ�ô�ŽǶ�ֵ�ı���������������Ǻ�Σ�յ�.
	�Թ�������ֻҪʱ�̶Դ�ŽǶ�ֵ�ı��������� 2�� Ϊ��λ��ȡ�࣬�Ϳ����øñ��������� 2�� ֮�� ����ֹ���.
	���Ƶ�ʵ��˼·�ͺñȰ�ĳ������ֵ 2k��+ a �е� a ��ȡ����----��2k�и�ȡ���.
	�������� ����ǶȻ��Ƕ�(angle) ������ float���͵�ȡ�����ƺ�ĽǶ�ֵ .
	*/
	
  float a = fmod(ElectricalAngle, 2*PI);    //������ĽǶ�(angle)������ 2��Ϊ��λ��ȡ�࣬fmod ��ȡ�ຯ����
	                                //�书������C���Ե�ȡ����š�%��������ֵ��ȡ����� 2��֮�� ��ֵ.
	
  return (a >= 0 ? a : (a + 2*PI)); //�ڷ���ʱ����ע��:fmod��������� С��2�� ��ֵ����ȡ�࣬
									//��ô��᷵�� -(2��- a) �����ĸ�ֵ���൱����a�Ļ������ټ�һ��2�У�ʹ֮����˸�ֵ,
	                                //��ֵ�ĽǶ������ǲ���ֱ��ʹ�õģ����������ȼӸ�2�а��������ֵ��
	                                //��Ȼ��Ҳ�������Ϊ:��ֵ������ԭ����ֵ.
	
	/*
	��ע:�� FOC_Config.h ͷ�ļ��У����ⶨ����Ĭ��ʹ�õĦ�(��:PI),
	�ڱ����������Ĭ��ʹ�øú�(PI)�������йئеļ���,ʹ���߿ɸ�����������Լ��������.
	*/
	
}
float GetElectricalAngle(Motor_t* motor) //����ת��
{
	/*
	��Ƕȼ��㺯�������� �����ǰ�Ƕ�(angle) �� ����ļ�����(pole_pairs),
	���ص�ǰ����� ��Ƕ�(����Ƕ�*������),
	����ֵ�� float���͵ĵ�Ƕ�(angle * pole_pairs).
	*/
	
    return Limit_ElectricalAngle(motor->DIR*motor->angle * motor->Pole_pairs - motor->angle_el_zero);//�����Ƕ� = �����ǰ��е�Ƕ�(angle) * ���������(pole_pairs)

	/*
	��ע:�� FOC_Config.h ͷ�ļ��У����ⶨ���˵��Ĭ��ʹ�õĵ��������(��:Pole_pairs),
	�ڱ����������Ĭ��ʹ�øú�(Pole_pairs)�������йص���������ļ���,ʹ���߿ɸ�����������Լ��������.
	*/
} 

float GetElectricalAngle__(Motor_t* motor)//���ڼ��� 
{
	/*
	��Ƕȼ��㺯�������� �����ǰ�Ƕ�(angle) �� ����ļ�����(pole_pairs),
	���ص�ǰ����� ��Ƕ�(����Ƕ�*������),
	����ֵ�� float���͵ĵ�Ƕ�(angle * pole_pairs).
	*/
    return Limit_ElectricalAngle(motor->angle * motor->Pole_pairs - motor->angle_el_zero);//�����Ƕ� = �����ǰ��е�Ƕ�(angle) * ���������(pole_pairs)

	/*
	��ע:�� FOC_Config.h ͷ�ļ��У����ⶨ���˵��Ĭ��ʹ�õĵ��������(��:Pole_pairs),
	�ڱ����������Ĭ��ʹ�øú�(Pole_pairs)�������йص���������ļ���,ʹ���߿ɸ�����������Լ��������.
	*/
} 

float angle_el_zero1 = 0;
float GetElectricalAngle_(float DIR,float Pole_pairs,float angle) 
{
	extern float angle_el_zero1;
    return Limit_ElectricalAngle(DIR*angle * Pole_pairs - angle_el_zero1); ;//�����Ƕ� = �����ǰ��е�Ƕ�(angle) * ���������(pole_pairs)

} 


float Limit(float value , float high , float low)
{
	/*����ֵ��������������ĳЩ��������Ĵ�С��Ϊ�˰�ȫ(?)������ռ�ձȣ���ѹ�ȵȣ�
	  ������Ҫ���Ƶ�Ŀ�����ֵ(value),���ֵ(high),��Сֵ(low),
		���� float���͵ı����ƺ��ֵ.
	*/
	return (value)<(low)?(low):((value)>(high)?(high):(value));//���Ŀ������������/��Сֵ�ķ�Χ���Ͱ����ֵ���������/��Сֵ
}

float *Park_N(float Uq , float ElectricalAngle)
{
	/*
	Park��任���������� Uq/Iq(���������õĵ�ѹ/����) �� ��Ƕ�(���������*�����ǰ�Ƕ�) ��
	���ؾ��� Park��任 ��� Ualpha �� Ubeta ��
	����ֵ��һ�� ָ��float���͵������ָ�� ������ĵ�һ������ Ualpha ,�ڶ����� Ubeta.
	*/
	static float Upark_N[2];//Upark_N[2]�洢�� ����Park��任��� Ualpha �� Ubeta .
	
	Upark_N[0] =  -Uq*sin(ElectricalAngle); //Park��任 ��Ualpha = Ud * cos�� - Uq * sin��
    Upark_N[1] =   Uq*cos(ElectricalAngle); //           ��Ubeta  = Uq * cos�� + Ud * sin��
	return Upark_N;
	
	/* ��ע: Udһ������Ϊ0 , ���������Park��任����������һ���֣����� Ud �ǲ��� */
}

float *Park_N2(float Uq , float Ud , float ElectricalAngle)
{
	/*
	Park��任���������� Uq/Iq(���������õĵ�ѹ/����) �� ��Ƕ�(���������*�����ǰ�Ƕ�) ��
	���ؾ��� Park��任 ��� Ualpha �� Ubeta ��
	����ֵ��һ�� ָ��float���͵������ָ�� ������ĵ�һ������ Ualpha ,�ڶ����� Ubeta.
	*/
	static float Upark_N[2];//Upark_N[2]�洢�� ����Park��任��� Ualpha �� Ubeta .
	
	Upark_N[0] = Ud*cos(ElectricalAngle) - Uq*sin(ElectricalAngle); //Park��任 ��Ualpha = Ud * cos�� - Uq * sin��
    Upark_N[1] = Uq*cos(ElectricalAngle) + Ud*sin(ElectricalAngle); //           ��Ubeta  = Uq * cos�� + Ud * sin��
	return Upark_N;
	
	/* ��ע: Udһ������Ϊ0 , ���������Park��任����������һ���֣����� Ud �ǲ��� */
}

float *Clark_N(float Ualpha ,float Ubeta)
{
  /*
	Clark��任���������� Ualpha �� Ubeta ��
	���ؾ��� Clark��任 ��� Ua ,Ub ,Uc ��������ֵ�����������������PWMռ�ձȣ�
	����ֵҲ��һ�� float���͵�����ָ��(Uclark_N) ������ĵ�һ������ Ua ���ڶ����� Ub , �������� Uc.
	*/	
	static float Uclark_N[3];//Uclark_N[3]�洢�� ����Clark��任��� Ua �� Ub ,Uc .
	
	//Clark��任:
	Uclark_N[0] = Ualpha + Upower/2;                 // ��Ua = Ualpha ;
	Uclark_N[1] = (sqrt(3)*Ubeta-Ualpha)/2 + Upower/2; // ��Ub = (��3 * Ubeta - Ualpha)/2 ;
	Uclark_N[2] = -(Ualpha + sqrt(3)*Ubeta)/2 + Upower/2;// ��Uc = ( -Ualpha - ��3 * Ubeta )/2;
	
	return Uclark_N;
	
	/*
	��ע:�� FOC_Config.h ͷ�ļ��У����ⶨ����Ĭ��ʹ�õĵ�Դ��ѹ(��:Upower),
	�ڱ����������Ĭ��ʹ�øú�(Upower)�������йص�Դ��ѹ�ļ���,ʹ���߿ɸ�����������Լ��������.
	*/
}

void SPWM(Motor_t* motor,float Ua , float Ub ,float Uc)
{
	/*
	��������PWM����������ú�����ȡSPWM�������ʽ����ABC�������Ua��Ub��Uc��PWM����
	ͨ�����ö�ʱ�����������ȽϼĴ���(ARR)ֵ������ռ�ձ�.
	����ABC����ĵ�ѹֵ(Ua,Ub,Uc)���޷���ֵ.
	*/
	
	float __Ua = Limit(Ua , 8 , 0 );//���㲢����ABC�������ռ�ձ�
	float __Ub = Limit(Ub , 8 , 0 );
	float __Uc = Limit(Uc , 8 , 0 );
	
	float _Ua = Limit(__Ua/Upower , 1 , 0 );//���㲢����ABC�������ռ�ձ�
	float _Ub = Limit(__Ub/Upower , 1 , 0 );
	float _Uc = Limit(__Uc/Upower , 1 , 0 );
	
	motor->SetPWM((uint16_t)(_Ua*A_PWM_Period),(uint16_t)(_Ub*B_PWM_Period),(uint16_t)(_Uc*C_PWM_Period));//����ABC����ռ�ձ�
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
	motor->angle_el_zero = GetElectricalAngle(motor);//ȷ����Ƕ�
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
	Motor.angle_el_zero = GetElectricalAngle(&Motor);//ȷ����Ƕ�
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
		motor->angle = 2*PI - motor->GetAngle();//�и�
	}
	else
	{
		motor->angle  = motor->GetAngle();
	}
	// Angle = FOC_GetAngle_Sensorless_SMO();//�޸�
	float alpha = (motor->T) / (motor->T + motor->time.dt);

	if(fabs(motor->angle - motor->LastAngle) > (0.8f*2*PI))
	{
		if((motor->angle - motor->LastAngle)<0){motor->Velocity_raw = (2*PI - motor->LastAngle + motor->angle)/motor->time.dt ;}//��ת
		else if((motor->angle - motor->LastAngle)>=0){motor->Velocity_raw = -(2*PI - motor->angle + motor->LastAngle)/motor->time.dt ;}//��ת
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
	// 	motor->angle = 2*PI - motor->GetAngle();//�и�
	// }
	// else
	// {
		motor->angle  = motor->GetAngle();

	// }
	// Angle = FOC_GetAngle_Sensorless_SMO();//�޸�
	float alpha = (motor->T) / (motor->T + motor->time.dt);

	if(fabs(motor->angle - motor->LastAngle) > (0.8f*2*PI))
	{
		if((motor->angle - motor->LastAngle)<0){motor->Velocity_raw = (2*PI - motor->LastAngle + motor->angle)/motor->time.dt ;}//��ת
		else if((motor->angle - motor->LastAngle)>=0){motor->Velocity_raw = -(2*PI - motor->angle + motor->LastAngle)/motor->time.dt ;}//��ת
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

// 	error = (TargetIq - Iq) ;//��q�������

// 	ThisI = LastI + Ki*dt*error;
// 	ThisI = Limit(ThisI,Ipower/2,-Ipower/2);

// 	Output[0] = Kp*error + ThisI + Kd*(error-Lasterror)/dt;
// 	Output[0] = Limit(Output[0] , Ipower/2 , -Ipower/2);
	
// 	error_ = (0 - Id) ; //��d�������

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
	// if(T<8)//8����бջ�
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
