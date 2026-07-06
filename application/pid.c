#include "pid.h"
#include "delay.h"

// 对PID参数进行初始化
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->SP = 0.0f; // 初始化设定值为0
    pid->t_k_1 = 0; // 初始化上一次的时间为0
    pid->err_k_1 = 0.0f; // 初始化上一次的误差为0
    pid->err_int_k_1 = 0.0f; // 初始化上一次的积分误差为0
    pid->UpperLimit = 3.4f * 10e8f; // 初始化PID输出上限为一个较大的值
    pid->LowerLimit = -3.4f * 10e8f; // 初始化PID输出下限为一个较小的值
}

// 改变PID的设定值
void PID_ChangeSP(PID_TypeDef *pid, float SP)
{
    pid->SP = SP;
}

void PID_LimitConfig(PID_TypeDef *pid, float UpperLimit, float LowerLimit)
{
    pid->UpperLimit = UpperLimit;
    pid->LowerLimit = LowerLimit;
}


// 执行一次pid运算
float PID_Compute(PID_TypeDef *pid, float FB)
{
    float err = pid->SP - FB; // 计算误差

    uint64_t t_k = GetUs(); // 获取当前时间 (微秒)
    float dt = (t_k - pid->t_k_1) / 1000000.0f; // 计算时间差 (秒)

    float err_dev = 0.0f;
    float err_int = 0.0f;

    if(pid->t_k_1 != 0) // 确保不是第一次调用
    {
        err_dev = (err - pid->err_k_1) / dt; // 计算误差的导数
        err_int = pid->err_int_k_1 + (pid->err_k_1 + err) * dt * 0.5f; // 计算误差的积分 (梯形积分法)
    }
    
    float COp = pid->Kp * err; // 比例项
    float COi = pid->Ki * err_int; // 积分项
    float COd = pid->Kd * err_dev; // 微分项
    
    // 积分限幅
    if(COi > pid->UpperLimit)
    {
        COi = pid->UpperLimit;
    }
    else if(COi < pid->LowerLimit)
    {
       COi = pid->LowerLimit;
    }

    float CO = COp + COi + COd; // PID输出

    // 限制PID输出在设定的上下限之间
    if (CO > pid->UpperLimit) // 限制PID输出上限
    {
        CO = pid->UpperLimit;
    }
    else if (CO < pid->LowerLimit) // 限制PID输出下限
    {
        CO = pid->LowerLimit;
    }

    // 更新PID状态
    pid->t_k_1 = t_k; // 更新上一次的时间
    pid->err_k_1 = err; // 更新上一次的误差
    pid->err_int_k_1 = err_int; // 更新上一次的积分误差

    return CO;
}

void PID_Reset(PID_TypeDef *pid)
{
    pid->t_k_1 = 0; // 重置上一次的时间
    pid->err_k_1 = 0.0f; // 重置上一次的误差
    pid->err_int_k_1 = 0.0f; // 重置上一次的积分误差
}
