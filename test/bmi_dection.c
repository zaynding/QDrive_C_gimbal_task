#include "usart.h"
#include "task.h"
#include "bmi_dection.h"
#include "stdio.h"
#include "bmi088.h"

void USART1_Proc(void)
{
    PERIODIC(100)  
    char buf[200];
    int len = sprintf(buf, "yaw: %.3f rad, ready: %d\r\n",
                      BMI088_GetYaw(), BMI088_YawIsReady()
                      );
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, HAL_MAX_DELAY);
}
