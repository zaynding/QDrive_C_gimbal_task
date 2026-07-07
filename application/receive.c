// 本文件处理视觉模块发来的数据:
// 帧格式为 "$valid,ex,ey\n", 例如 "$1,-45,30\n".
// valid 表示靶纸是否有效, ex/ey 是图像平面上的像素偏差.
//
// 通信框架: USART6 + ReceiveToIdle DMA. 每收到一帧或出现串口空闲,
// HAL 会调用 HAL_UARTEx_RxEventCallback(), 在回调中解析本次数据.
#include "receive.h"
#include "usart.h"
#include "qmath.h"
#include "main.h"

/* -------------------- 内部常量和缓冲区 -------------------- */

// "$1,-320,-180\n" 约 14 字节, 64 字节作为安全缓冲.
#define VISION_RX_BUF_SIZE   64

static uint8_t s_rx_buf[VISION_RX_BUF_SIZE];
static VisionErr_t s_vision;


/* -------------------- 内部函数 -------------------- */

static int parse_int(const uint8_t *buf, uint16_t len, uint16_t *pos, int32_t *out)
{
    uint16_t i = *pos;
    int32_t val = 0;
    int32_t sign = 1;
    uint8_t got = 0;

    if (i < len && (buf[i] == '+' || buf[i] == '-'))
    {
        if (buf[i] == '-')
        {
            sign = -1;
        }
        i++;
    }

    while (i < len && buf[i] >= '0' && buf[i] <= '9')
    {
        val = val * 10 + (buf[i] - '0');
        i++;
        got = 1;
    }

    if (!got)
    {
        return -1;
    }

    *out = sign * val;
    *pos = i;
    return 0;
}

static int parse_frame(const uint8_t *buf, uint16_t len)
{
    uint16_t i = 0;
    int32_t valid = 0;
    int32_t ex = 0;
    int32_t ey = 0;

    while (i < len && buf[i] != '$')
    {
        i++;
    }
    if (i >= len)
    {
        return -1;
    }
    i++;

    if (parse_int(buf, len, &i, &valid) != 0) return -1;
    if (i >= len || buf[i] != ',') return -1;
    i++;

    if (parse_int(buf, len, &i, &ex) != 0) return -1;
    if (i >= len || buf[i] != ',') return -1;
    i++;

    if (parse_int(buf, len, &i, &ey) != 0) return -1;

    s_vision.valid = (valid != 0) ? 1 : 0;
    s_vision.ex_px = (int16_t)ex;
    s_vision.ey_px = (int16_t)ey;

    if (s_vision.valid)
    {
        // ex 为正表示目标在图像右侧, 云台 yaw 应向右修正.
        s_vision.yaw_err_rad = qatan2((float)ex, VISION_FX);

        // ey 为正表示目标在图像下侧, 取反后让向上为正.
        s_vision.pitch_err_rad = qatan2((float)(-ey), VISION_FY);
    }

    s_vision.rx_time_ms = HAL_GetTick();
    return 0;
}


static void vision_start_dma(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, s_rx_buf, VISION_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
}


/* -------------------- 公开接口 -------------------- */

void Vision_Init(void)
{
    s_vision.valid = 0;
    s_vision.ex_px = 0;
    s_vision.ey_px = 0;
    s_vision.yaw_err_rad = 0.0f;
    s_vision.pitch_err_rad = 0.0f;
    s_vision.rx_time_ms = 0;

    vision_start_dma();
}

const VisionErr_t *Vision_GetErr(void)
{
    return &s_vision;
}

uint8_t Vision_IsOnline(uint32_t timeout_ms)
{
    return ((HAL_GetTick() - s_vision.rx_time_ms) <= timeout_ms) ? 1 : 0;
}


/* -------------------- HAL 中断回调 -------------------- */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART6)
    {
        parse_frame(s_rx_buf, Size);
        vision_start_dma();
    }
}
