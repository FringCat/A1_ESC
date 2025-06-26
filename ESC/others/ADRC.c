#include "ADRC.h"
extern ADRC_t ADRC;
extern float v0;
int factorial(int n) {
    if (n == 0 || n == 1) {  // 基例，0! = 1! = 1
        return 1;
    } else {
        return n * factorial(n - 1);  // 递归调用
    }
}
float h_ = 0.76;
void ADRC_Init(ADRC_t* ADRC)
{
    memset(ADRC,0,sizeof(ADRC_t));

    ADRC->TD.r = 0.94 ;
    ADRC->TD.h = 0.001 ;

    ADRC->ESO.w0 = 33.0;
    ADRC->ESO.beta[0] = 3*ADRC->ESO.w0 ;
    ADRC->ESO.beta[1] = 3*ADRC->ESO.w0*ADRC->ESO.w0 ;
    ADRC->ESO.beta[2] = ADRC->ESO.w0*ADRC->ESO.w0*ADRC->ESO.w0 ;
    ADRC->ESO.b = 1000.9 ;

    ADRC->NLSEF.k1 = 2.0 ;
    ADRC->NLSEF.k2 = 0.0 ;
    ADRC->NLSEF.a1 = 0.8 ;
    ADRC->NLSEF.a2 = 0.5 ;
    ADRC->NLSEF.r1 = 10.8 ;
    ADRC->NLSEF.r2 = 0.2 ;
    
}

// void ESO(ADRC_t* ADRC,float y)
// {
//     float z1 = ADRC->ESO.z[0];
//     float *z_prev = (float *)malloc(sizeof(ADRC->ESO.z));
//     memset(z_prev,0,sizeof(ADRC->ESO.z));
//     for(int i = 0;i<(sizeof(ADRC->ESO.z)/sizeof(float));i++)
//     {
//         z_prev[i] = ADRC->ESO.z[i];
//     }
//     for(int i = 0;i<(sizeof(ADRC->ESO.z)/sizeof(float));i++)
//     {   
//         if(i == (sizeof(ADRC->ESO.z)/sizeof(float))-2)
//         {
//             ADRC->ESO.z[i] += (z_prev[i+1] + ADRC->ESO.beta[i] *( y - z1 ) + ADRC->ESO.u * ADRC->ESO.b);           
//         }
//         else if(i == (sizeof(ADRC->ESO.z)/sizeof(float))-1)
//         {
//             ADRC->ESO.z[i] += (ADRC->ESO.beta[i] *( y - z1 ));  //观测到的扰动
//         }
//         else
//         {
//             ADRC->ESO.z[i] += (z_prev[i+1] + ADRC->ESO.beta[i] *( y - z1 ));
//         }
        
//     }
//     free(z_prev);
// }
void ESO(ADRC_t* ADRC, float y)  // 添加步长参数h
{
    float z1 = ADRC->ESO.z[0];
    ADRC->ESO.u = (ADRC->NLSEF.output - ADRC->ESO.z[2]) / ADRC->ESO.b;
    // 三阶ESO标准形式
    ADRC->ESO.z[0] += ADRC->TD.h * (ADRC->ESO.z[1] + ADRC->ESO.beta[0] * (y - z1));
    ADRC->ESO.z[1] = ADRC->TD.h * (ADRC->ESO.z[2] + ADRC->ESO.beta[1] * (y - z1) + ADRC->ESO.b * ADRC->ESO.u);
    ADRC->ESO.z[2] = ADRC->TD.h * (ADRC->ESO.beta[2] * (y - z1));  // 扰动观测
}

// float fhan1(float e1 , float* e2,float r ,float h,int order)
// {
//     float output = 0;
    
//     if(fabs(e1)  <  r)
//     {
//         printf("%f,%f,%f,%f\n",v0,ADRC.TD.v[0],ADRC.TD.v[1],1.0);

//     //     output = (h/r) *e1;
//     //    for (int i = 0; i < (order); i++) 
//     //    {
//     //         if(i==0)
//     //         {
//     //             continue;
//     //         }
//     //         output += (h / pow(r, i+1)) * e2[i];
//     //    }
//     output = pow(3*r,(-2.0/3.0)) * Sat(e1,r)*h;
//         // printf("%f,%f\n",output,1.0);
//     }
//     else
//     {
        
//         float switch_fun = e1;
//         // switch_fun = e1+(e2[1]*fabs(e2[1]))/(2*r*h);
//         for(int i = 1;i<order;i++)
//         {
//             float a = 0;
//             if(order%2 == 1) //奇
//             { 
//                 a = pow(e2[i],order);
//             }
//             else if(order%2 == 0)//偶
//             {
//                 a = pow(e2[i],order/2) * pow(fabs(e2[i]),order/2);
//             }
//             else
//             {
//                 printf("error!");
//             }
//             switch_fun += a/((float)factorial(order) * pow(h,order-1) *pow(r,order-1));
//         }
//         output = r*Sat(switch_fun,r);
//         printf("%f,%f,%f,%f\n",v0,ADRC.TD.v[0],ADRC.TD.v[1],2.0);

//         // printf("%f,%f\n",output,2.0);
//     }
//     return output;
// }

// float fhan1(float e1, float* e2, float r, float h,int order) 
// {
//     float output = 0;
//     order = 2;  // 假设为三阶系统，直接固定阶数

//     if (fabs(e1) <= r) 
//     {
//         printf("%f,%f,%f,%f\n", v0, ADRC.TD.v[0], ADRC.TD.v[1], 1.0);
//         output = pow(3*r, (-2.0/3.0)) * Sat(e1, r);  
//     } else 
//     {
//         float switch_fun = e1 + pow(e2[1], 3)/(factorial(3)*pow(h,2)*pow(r,2));  // 三阶特定的计算
//         output = r * Sat(switch_fun, r);
//         printf("%f,%f,%f,%f\n", v0, ADRC.TD.v[0], ADRC.TD.v[1], 2.0);
//     }
//     return output;
// }
float fhan1(float e1, float* e2, float r, float h,int order) 
{
    float output = 0;

    if (fabs(e1) <= r) 
    {
        // printf("%f,%f,%f,%f\n", v0, ADRC.TD.v[0], ADRC.TD.v[1], 1.0);
        output = pow(3*r, (-2.0/3.0)) * Sat(e1, r) *h;  
    } else 
    {
        float switch_fun = e1 + pow(e2[1], 3)/(factorial(3)*pow(h,2)*pow(r,2));  // 三阶特定的计算
        output = pow(r,r) * Sat(switch_fun, r)*h;
        // printf("%f,%f,%f,%f\n", v0, ADRC.TD.v[0], ADRC.TD.v[1], 2.0);
    }
    return output;
}
void TD(ADRC_t* ADRC, float v0) {
    // float derivatives[2];  // 二阶系统，直接定义大小为2的数组

    // 计算导数
    
    // 第二阶导数：直接调用 fhan1 函数，传递当前误差和状态变量
    ADRC->TD.v[1] = fhan1(v0 - ADRC->TD.v[0], ADRC->TD.v, ADRC->TD.r,h_,2);
    ADRC->TD.v[0] += ADRC->TD.v[1];  // 第一阶导数
    // // 更新状态变量
    // for (int i = 0; i < 2; i++) {
    //     ADRC->TD.v[i] += ADRC->TD.h*derivatives[i];  // 积分更新状态
    // }
}
// void TD(ADRC_t* ADRC,float v0)
// {   
//     int order = sizeof(ADRC->TD.v)/sizeof(float);
//     float* derivatives = (float*)malloc(sizeof(ADRC->TD.v)); // 假设order较小，否则动态分配
//     for (int i = 0; i < order; i++)
//     {
//         if(i==order-1)
//         {
//              derivatives[i] = ADRC->TD.r * fhan1(v0-ADRC->TD.v[0], ADRC->TD.v ,ADRC->TD.r,ADRC->TD.h,(sizeof(ADRC->TD.v)/sizeof(float))) ;
//             //  derivatives[i] = fhan3(ADRC->TD.v[0]-v0, ADRC->TD.v ,ADRC->TD.r,ADRC->TD.h,(sizeof(ADRC->TD.v)/sizeof(float))) ; 
//         }
//         else
//         {
//             derivatives[i] =  ADRC->TD.v[i+1];
//             // ADRC->TD.v[i] +=  ADRC->TD.v[i+1]*ADRC->TD.h;
//         }
//     }
//     for (int i = 0; i < order; i++) 
//     {
//         ADRC->TD.v[i] += derivatives[i] ;
//     }
//     free(derivatives);
// }
float fsg(float x ,float d)
{
    return ( sgn(x+d) - sgn(x-d) )/2;
}


float fhan2(float e1 , float* e2,float r ,float h,int order)
{
    float d = r*h*h,
    a0 = h*e2[1], 
    y = e1 + a0,
    a1 = sqrt(d*(d+8*fabs(y))),
    a2 = a0 + sgn(y)*(a1-d)/2,
    a = (a0+y)*fsg(y,d)+a2*(1-fsg(y,d));
    printf("%f,%f,%f\n",v0,ADRC.TD.v[0],ADRC.TD.v[1]);
    return r*(a/d)*fsg(y,d)+r*sgn(a)*(1-fsg(a,d));
     
}

float fhan3(float e1 , float* e2,float r ,float h,int order)
{
    static float h_all = 0;
    h_all+= h;
    float fh =  (2*(e2[0]-e1)*r)/(3.1415926*(1+r*r*(1/e1)*(1/e1)*(1/e1)*(1/e1)));
    float switch_fun = sgn(e1+(e2[1]*fabs(e2[1]))/(2*r*h));
    printf("%f,%f,%f\n",v0,ADRC.TD.v[0],ADRC.TD.v[1]);
    return -fh;
}


float flag = 1.0 ;
float fal(float e,float a,float r)
{
    if(fabs(e)<=r)
    {
        flag = 1.0 ;
        return (1/(pow(r,1-a)))*e;
    }
    else
    {
        flag = 2.0 ;
        return pow(fabs(e),a)*sgn(e);
    }
}
float NLSEF1(ADRC_t* ADRC)
{
    // ADRC->NLSEF.output = ADRC->NLSEF.k1*fal(ADRC->TD.v[0]-ADRC->ESO.z[0],ADRC->NLSEF.a1,ADRC->NLSEF.r1)+ADRC->NLSEF.k2*fal(ADRC->TD.v[1]-ADRC->ESO.z[1],ADRC->NLSEF.a2,ADRC->NLSEF.r2)+ADRC->ESO.z[2]/ADRC->ESO.b;
    ADRC->NLSEF.output = ADRC->NLSEF.k1*fal(ADRC->TD.v[0]-ADRC->ESO.z[0],ADRC->NLSEF.a1,ADRC->NLSEF.r1);
    return ADRC->NLSEF.output;
}
// float NLSEF1(ADRC_t* ADRC)
// {
//     float target = 20 ;
//     ADRC->NLSEF.output = ADRC->NLSEF.k1*fal(target-ADRC->ESO.z[0],ADRC->NLSEF.a1,ADRC->NLSEF.r1)+ADRC->NLSEF.k2*fal(ADRC->TD.v[1]-ADRC->ESO.z[1],ADRC->NLSEF.a2,ADRC->NLSEF.r2);
//     return ADRC->NLSEF.output;
// }
