#ifndef BMI_DECTION_H
#define BMI_DECTION_H

typedef enum
{
    USART1_EXIT_NONE = 0,
    USART1_EXIT_MOTOR_NOT_READY,
    USART1_EXIT_IMU_OFFLINE,
    USART1_EXIT_CONTROL_STOPPED
} USART1_ExitReason_t;

void USART1_SetExitReason(USART1_ExitReason_t reason);
void USART1_Proc(void);

#endif
