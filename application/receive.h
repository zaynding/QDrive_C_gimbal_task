#ifndef RECEIVE_H
#define RECEIVE_H

#include <stdint.h>

/*
 * ���������
 * �ֱ��ʣ�640 �� 360��16:9
 * �Խ� FOV��90��
 * �Ƶã�HFOV �� 82.15�㣬VFOV �� 52.23��
 * fx = fy �� 367.15 px
 */
#define VISION_IMG_W        640.0f
#define VISION_IMG_H        360.0f
#define VISION_CX           320.0f
#define VISION_CY           180.0f
#define VISION_FX           367.15f
#define VISION_FY           367.15f

typedef struct
{
    uint8_t valid;            // 1��������Ч��0��δʶ�𵽰���

    int16_t ex_px;            // ˮƽ��������Ϊ��
    int16_t ey_px;            // ��ֱ��������Ϊ��

    float yaw_err_rad;      // yaw �Ƕ�����Ϊ��
    float pitch_err_rad;    // pitch �Ƕ�����Ϊ��

    uint32_t rx_time_ms;      // ���һ���յ������ݵ�ʱ��
} VisionErr_t;


/* ==================== ����ӿ� ==================== */

/**
 * @brief  �����Ӿ����ݽ��գ������ж� + DMA��
 * @note   ���� MX_USART6_UART_Init() ֮����ѭ��֮ǰ����һ��
 */
void Vision_Init(void);

/**
 * @brief  ��ȡ����һ֡���������ֻ����
 * @return ָ���ڲ� VisionErr_t �ĳ���ָ��
 */
const VisionErr_t *Vision_GetErr(void);

/**
 * @brief  �ж��Ӿ���·�Ƿ����ߣ����һ֡�Ƿ��ڳ�ʱʱ�����յ���
 * @param  timeout_ms  ���������֡�������λ ms��
 * @return 1�����ߣ�0����ʱ
 */
uint8_t Vision_IsOnline(uint32_t timeout_ms);


/* ==================== 底盘世界坐标(从底盘MCU经UART接收) ====================
 * 世界系遵循IMU.md第9节(FRD): +X前/北 +Y右/东 +Z下
 * 原点在靶纸位置,故(x,y)是小车相对靶纸的位置,单位米 */
typedef struct
{
    float    x;            //小车世界X坐标,单位米(靶纸=原点)
    float    y;            //小车世界Y坐标,单位米(靶纸=原点)
    uint8_t  valid;        //1:坐标有效 0:未收到有效坐标
    uint32_t rx_time_ms;   //最后一次收到坐标帧的时间戳
} ChassisPos_t;

/**
 * @brief  UART接收底盘世界坐标的处理函数(占位,协议由底盘MCU定义)
 */
void Chassis_RxProc(void);

/**
 * @brief  读取最新底盘世界位置(只读)
 */
const ChassisPos_t *Chassis_GetPos(void);

/**
 * @brief  判断底盘坐标是否在线(timeout_ms内收到过)
 */
uint8_t Chassis_IsOnline(uint32_t timeout_ms);


#endif
