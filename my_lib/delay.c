/*
 * delay.c
 *
 * Created on: 2025年5月6日
 * Author: gaoxi
 */

#include "delay.h"

// 用于缓存微秒计算系数 (72MHz下为72)
static uint32_t g_fac_us = 0; 

//
// @简介：初始化延迟函数
// @注意：只需在 main 函数开头调用一次
//
void Delay_Init(void)
{
    // 计算每微秒需要多少个时钟周期
    // 避免在 GetUs 中重复进行除法运算
    g_fac_us = SystemCoreClock / 1000000;
}

//
// @简介：毫秒级延迟
// @参数：Delay - 延迟时长 (ms)
//
void Delay(uint32_t Delay)
{
    // HAL_Delay 是官方提供的阻塞延时，直接用它最稳
    HAL_Delay(Delay);
}

//
// @简介：获取当前系统时间 (ms)
//
uint32_t GetTick(void)
{
    return HAL_GetTick();
}

/**
 * @brief  获取当前的微秒级时间戳
 * @retval 当前的微秒数
 */
uint32_t GetUs(void)
{
    uint32_t ms1, ms2, val;
    uint32_t load = SysTick->LOAD;

    // 防止用户忘了调 Delay_Init，这里自动补救一下
    if(g_fac_us == 0) 
    {
        Delay_Init();
    }

    /* 双重读取法：防止在读取 VAL 的瞬间 HAL_GetTick 刚好进位 */
    do {
        ms1 = HAL_GetTick(); // 直接调用 HAL 库函数
        val = SysTick->VAL;  // SysTick 计数值 (向下计数)
        ms2 = HAL_GetTick();
    } while (ms1 != ms2);    // 如果两次读的毫秒不一样，说明跨秒了，重读一次

    /* 核心计算公式 */
    // (LOAD - val) 是当前毫秒内已经走过的 tick 数
    return (ms1 * 1000) + ((load - val) / g_fac_us);
}

//
// @简介：微秒级延迟
// @参数：us - 要延迟的时间 (us)
//
void DelayUs(uint32_t us)
{
    // 获取起始时刻
    uint32_t start = GetUs();
    
    // 利用无符号整型溢出特性，安全计算时间差
    while((GetUs() - start) < us);
}
