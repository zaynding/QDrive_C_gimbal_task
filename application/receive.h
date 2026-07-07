#ifndef RECEIVE_H
#define RECEIVE_H

#include <stdint.h>

/*
 * 视觉参数
 * 图像分辨率: 640 x 360, 16:9
 * 相机 FOV: 对角约 90 度
 * 估算: HFOV 约 82.15 度, VFOV 约 52.23 度
 * fx = fy 约 367.15 px
 */
#define VISION_IMG_W        640.0f
#define VISION_IMG_H        360.0f
#define VISION_CX           320.0f
#define VISION_CY           180.0f
#define VISION_FX           367.15f
#define VISION_FY           367.15f

typedef struct
{
    uint8_t valid;          // 1: 靶纸有效, 0: 未识别到靶纸

    int16_t ex_px;          // 水平方向像素偏差, 右为正
    int16_t ey_px;          // 垂直方向像素偏差, 下为正

    float yaw_err_rad;      // yaw 方向角度误差, 单位 rad
    float pitch_err_rad;    // pitch 方向角度误差, 单位 rad

    uint32_t rx_time_ms;    // 最后一帧有效视觉数据的接收时间戳
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
