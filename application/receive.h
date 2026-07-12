#ifndef RECEIVE_H
#define RECEIVE_H

#include <stdint.h>

/*
 * 树莓派发送 ex=target_x-223, ey=target_y-221.
 * 以下参数来自640x480相机标定;当前按约定忽略畸变.
 */
#define VISION_IMG_W 640.0f
#define VISION_IMG_H 480.0f
#define VISION_CX 298.1835178f
#define VISION_CY 230.6594528f
#define VISION_FX 776.9054244f
#define VISION_FY 582.5955349f
#define VISION_LASER_X 223.0f
#define VISION_LASER_Y 221.0f

typedef struct
{
    uint8_t valid; // 1: 靶纸有效, 0: 未识别到靶纸

    int16_t ex_px; // 水平方向像素偏差, 右为正
    int16_t ey_px; // 垂直方向像素偏差, 下为正

    float yaw_err_rad;   // yaw 方向角度误差, 单位 rad
    float pitch_err_rad; // pitch 方向角度误差, 单位 rad

    uint32_t rx_time_ms; // 最后一帧合法报文的接收时间戳
} VisionErr_t;

/* ==================== 视觉接收接口 ==================== */

/**
 * @brief  初始化视觉串口接收, 使用 USART6 + ReceiveToIdle DMA.
 * @note   在 MX_USART6_UART_Init() 之后、主循环之前调用一次.
 */
void Vision_Init(void);

/**
 * @brief  获取最新一帧视觉数据.
 * @return 指向内部 VisionErr_t 的只读指针.
 */
const VisionErr_t *Vision_GetErr(void);

/**
 * @brief  判断视觉链路是否在线.
 * @param  timeout_ms  超时时间, 单位 ms.
 * @return 1: 在线, 0: 超时.
 */
uint8_t Vision_IsOnline(uint32_t timeout_ms);

#endif
