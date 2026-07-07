#include "usart.h"
#include "bmi_dection.h"
#include "stdio.h"
#include "receive.h"

void USART1_Proc(void)
{
    static uint32_t last_rx_time_ms = 0;
    const VisionErr_t *vision = Vision_GetErr();

    if (vision->rx_time_ms == 0 || vision->rx_time_ms == last_rx_time_ms)
    {
        return;
    }
    last_rx_time_ms = vision->rx_time_ms;

    char buf[160];
    int len = snprintf(buf, sizeof(buf),
                       "vision: valid=%u, ex=%d, ey=%d, yaw=%.4f rad, pitch=%.4f rad\r\n",
                       (unsigned int)vision->valid,
                       (int)vision->ex_px,
                       (int)vision->ey_px,
                       vision->yaw_err_rad,
                       vision->pitch_err_rad);

    if (len < 0)
    {
        return;
    }
    if (len >= (int)sizeof(buf))
    {
        len = sizeof(buf) - 1;
    }

    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, HAL_MAX_DELAY);
}


