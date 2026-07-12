#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

void Control_Init(void);
void Control_Proc(void);
void Control_RequestUpdateFromISR(void);
void Control_Task(void);

uint8_t Control_Start(void);
void Control_Stop(void);
uint8_t Control_IsStarted(void);
uint8_t Control_AreMotorsReady(void);
uint8_t Control_IsIMUOnline(void);
uint32_t Control_GetIMUAgeMs(void);
void Control_RetryMotorEnable(void);

uint8_t Control_ResetAttitude(void);
uint8_t Control_EnableStability(void);
void Control_DisableStability(void);
uint8_t Control_IsStabilityEnabled(void);

void SetGimbal0Speed(float speed_rpm);   //yaw轴目标转速(rpm)
void SetGimbal1Speed(float speed_rpm);   //pitch轴目标转速(rpm)

#endif
