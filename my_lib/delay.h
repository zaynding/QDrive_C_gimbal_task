/*
 * delay.h
 *
 * Created on: 2025年5月6日
 * Author: gaoxi
 */

#ifndef INC_DELAY_H_
#define INC_DELAY_H_

#include "main.h" // 这一行必须有，为了包含 stm32f1xx_hal.h

// 函数声明
void Delay_Init(void);          // 初始化
void Delay(uint32_t ms);        // 毫秒延时
void DelayUs(uint32_t us);      // 微秒延时

uint32_t GetTick(void);         // 获取系统时间(ms)
uint32_t GetUs(void);           // 获取系统时间(us)

#endif /* INC_DELAY_H_ */
