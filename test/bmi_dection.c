#include "usart.h"
#include "bmi_dection.h"
#include "bmi088.h"
#include "control.h"
#include <stdio.h>

static volatile USART1_ExitReason_t exit_reason;
static uint8_t imu_phase_snapshot;
static uint8_t imu_drdy_snapshot;
static uint32_t imu_spi_state_snapshot;
static uint32_t imu_spi_error_snapshot;
static uint32_t imu_age_snapshot;

// 记录等待USART1进程发送的退出原因.
void USART1_SetExitReason(USART1_ExitReason_t reason)
{
    if (exit_reason == USART1_EXIT_NONE)
    {
        if (reason == USART1_EXIT_IMU_OFFLINE)
        {
            imu_phase_snapshot = BMI088_GetDMAPhase();
            imu_spi_state_snapshot = BMI088_GetSPIState();
            imu_spi_error_snapshot = BMI088_GetSPIError();
            imu_drdy_snapshot = BMI088_GetGyroDRDYLevel();
            imu_age_snapshot = Control_GetIMUAgeMs();
        }
        exit_reason = reason;
    }
}

// 通过USART1发送一次云台退出运行的原因.
void USART1_Proc(void)
{
    static uint8_t motor_message[] = "EXIT:MOTOR_NOT_READY\r\n";
    static uint8_t control_message[] = "EXIT:CONTROL_STOPPED\r\n";
    static uint8_t imu_message[128];
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
        int len = snprintf((char *)imu_message, sizeof(imu_message),
                           "EXIT:IMU_OFFLINE PHASE=%u SPI=%lu ERR=0x%08lX DRDY=%u AGE=%lu\r\n",
                           (unsigned int)imu_phase_snapshot,
                           (unsigned long)imu_spi_state_snapshot,
                           (unsigned long)imu_spi_error_snapshot,
                           (unsigned int)imu_drdy_snapshot,
                           (unsigned long)imu_age_snapshot);

        if (len < 0)
            return;
        if (len >= (int)sizeof(imu_message))
            len = sizeof(imu_message) - 1U;
        message = imu_message;
        length = (uint16_t)len;
    }
    else
    {
        message = control_message;
        length = sizeof(control_message) - 1U;
    }

    HAL_UART_Transmit(&huart1, message, length, 20U);
}
