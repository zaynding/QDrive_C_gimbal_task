#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

typedef enum
{
    GIMBAL_STATE_UNINITIALIZED = 0,
    GIMBAL_STATE_CALIBRATING,
    GIMBAL_STATE_RUNNING,
    GIMBAL_STATE_RECOVERING,
    GIMBAL_STATE_FAULT
} GimbalState_t;

void Gimbal_Init(void);
void Gimbal_Task(void);
GimbalState_t Gimbal_GetState(void);

#endif
