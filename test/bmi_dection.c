#include "usart.h"
#include "bmi_dection.h"

static volatile USART1_ExitReason_t exit_reason;

// 记录等待USART1进程发送的退出原因.
void USART1_SetExitReason(USART1_ExitReason_t reason)
{
    if (exit_reason == USART1_EXIT_NONE)
        exit_reason = reason;
}

// 通过USART1发送一次云台退出运行的原因.
void USART1_Proc(void)
{
    static uint8_t motor_message[] = "EXIT:MOTOR_NOT_READY\r\n";
    static uint8_t imu_message[] = "EXIT:IMU_OFFLINE\r\n";
    static uint8_t control_message[] = "EXIT:CONTROL_STOPPED\r\n";
    uint8_t *message;
    uint16_t length;
    USART1_ExitReason_t reason = exit_reason;

    if (reason == USART1_EXIT_NONE)
        return;

    exit_reason = USART1_EXIT_NONE;
    if (reason == USART1_EXIT_MOTOR_NOT_READY)
    {
        message = motor_message;
        length = sizeof(motor_message) - 1U;
    }
    else if (reason == USART1_EXIT_IMU_OFFLINE)
    {
        message = imu_message;
        length = sizeof(imu_message) - 1U;
    }
    else
    {
        message = control_message;
        length = sizeof(control_message) - 1U;
    }

    HAL_UART_Transmit(&huart1, message, length, 20U);
}
