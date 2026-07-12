#ifndef PID_H
#define PID_H
#include "main.h"

typedef struct
{
    float Kp; // Proportional gain
    float Ki; // Integral gain
    float Kd; // Derivative gain
    float SP; // 用户设定值

    uint32_t t_k_1;    // 上一次时间,单位us;无符号差值可处理计时回绕
    float err_k_1;     // 上一次的误差
    float err_int_k_1; // 上一次的积分误差

    float UpperLimit; // PID输出上限
    float LowerLimit; // PID输出下限

} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd);
void PID_ChangeSP(PID_TypeDef *pid, float SP);
float PID_Compute(PID_TypeDef *pid, float FB);
void PID_LimitConfig(PID_TypeDef *pid, float UpperLimit, float LowerLimit);
void PID_Reset(PID_TypeDef *pid); // 重置PID状态

#endif
