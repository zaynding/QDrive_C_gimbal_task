//本文件处理视觉传来的数据:靶纸在图像平面上的水平、竖直位移ex、ey,转换为角度误差err
//视觉传来的信息格式:$valid,ex,ey\n (靶纸有效标记valid,水平位移ex,竖直位移ey)
//例如: $1,-45,30\n   表示  靶纸有效,水平向左 -45px,竖直向下 30px
//
//通信框架:USART6 + DMA2_Stream1带空闲中断 + DMA每收满一帧数据就产生中断
//        由 HAL 自动调用 HAL_UARTEx_RxEventCallback,在该回调中解析接收的帧

#include "receive.h"
#include "usart.h"
#include "qmath.h"
#include "main.h"

/* -------------------- 内部常量和缓冲区 -------------------- */

// DMA 接收缓冲区,约为一帧 "$1,-320,-180\n" 的 14 字节,取 64 作为安全值
#define VISION_RX_BUF_SIZE   64

// DMA 从串口读入数据存放的原始字节缓冲
static uint8_t     s_rx_buf[VISION_RX_BUF_SIZE];

//最新视觉数据(靶纸检测结果),供外部通过 Vision_GetErr 只读查询
static VisionErr_t s_vision;

//最新底盘世界坐标(靶纸为原点),由Chassis_RxProc()更新
static ChassisPos_t s_chassis;


/* -------------------- 内部函数 -------------------- */

/**
 * @brief  从缓冲区读出一个十进制整数,支持前缀 '+' / '-'
 * @param  buf   数据缓冲区
 * @param  len   数据缓冲区有效长度
 * @param  pos   [输入/输出] 当前解析位置,成功后指向数字之后的第一个字符
 * @param  out   [输出] 读出的整数值
 * @return 0:成功,-1:解析位置没有有效数字,格式错误
 */
static int parse_int(const uint8_t *buf, uint16_t len, uint16_t *pos, int32_t *out)
{
    uint16_t i    = *pos;
    int32_t  val  = 0;
    int32_t  sign = 1;
    uint8_t  got  = 0;       // 是否读到至少一位数字

    // 读可选符号位
    if (i < len && (buf[i] == '+' || buf[i] == '-'))
    {
        if (buf[i] == '-') sign = -1;
        i++;
    }

    // 十位累加读取
    while (i < len && buf[i] >= '0' && buf[i] <= '9')
    {
        val = val * 10 + (buf[i] - '0');
        i++;
        got = 1;
    }

    if (!got) return -1;     // 没有读到任何数字,格式错误

    *out = sign * val;
    *pos = i;
    return 0;
}

/**
 * @brief  解析一帧 "$valid,ex,ey\n"格式,成功则更新 s_vision
 * @param  buf  DMA 接收缓冲区
 * @param  len  接收到的字节总数
 * @return 0:帧格式成功,-1:帧格式错误(此时保留上一帧数据)
 */
static int parse_frame(const uint8_t *buf, uint16_t len)
{
    uint16_t i     = 0;
    int32_t  valid = 0, ex = 0, ey = 0;

    // 1. 查找帧头 '$',丢弃帧头之前的所有杂字节
    while (i < len && buf[i] != '$') i++;
    if (i >= len) return -1;         // 本帧里没有帧头
    i++;                             // 跳过 '$'

    // 2. 递次读取 valid, ex, ey 三个整数字段,字段间用 ',' 分隔
    if (parse_int(buf, len, &i, &valid) != 0) return -1;
    if (i >= len || buf[i] != ',')  return -1;   i++;
    if (parse_int(buf, len, &i, &ex)    != 0) return -1;
    if (i >= len || buf[i] != ',')  return -1;   i++;
    if (parse_int(buf, len, &i, &ey)    != 0) return -1;
    // 末尾 '\r' / '\n' 不强制验证,飬,为简化逻辑只要三个字段都读成功就认为帧有效

    // 3. 写入视觉全局状态
    s_vision.valid = (valid != 0) ? 1 : 0;
    s_vision.ex_px = (int16_t)ex;
    s_vision.ey_px = (int16_t)ey;

    // 4. 仅在帧有效时计算方向角,无效时保留前次方向角不变,保存小车姿态
    if (s_vision.valid)
    {
        // yaw方向为水平ex 正值表示目标在图像右侧 → yaw_err 云台应向右转
        s_vision.yaw_err_rad   = qatan2((float)ex, VISION_FX);

        // pitch方向为竖直ey 负值表示向下,取反使向上为正,得到最终角度偏差
        s_vision.pitch_err_rad = qatan2((float)(-ey), VISION_FY);
    }

    // 5. 记录接帧时间戳,供后续脱线时间判断使用
    s_vision.rx_time_ms = HAL_GetTick();
    return 0;
}

/**
 * @brief  启动一轮中断 + DMA接收
 * @note   NORMAL 模式下 DMA 每次中断都需要重新启动
 */
static void vision_start_dma(void)
{
    // 接收 VISION_RX_BUF_SIZE 字节,直到收到空闲码元,触发 HAL_UARTEx_RxEventCallback
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, s_rx_buf, VISION_RX_BUF_SIZE);

    // 关闭 DMA 半满中断,只在"满了 / 空闲"时触发帧完成回调
    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
}


/* -------------------- 公开接口实现 -------------------- */

void Vision_Init(void)
{
    s_vision.valid      = 0;
    s_vision.rx_time_ms = 0;
    vision_start_dma();
}

const VisionErr_t *Vision_GetErr(void)
{
    return &s_vision;
}

/* -------------------- 底盘世界坐标(从底盘MCU经UART接收) -------------------- */

//UART接收底盘世界坐标的处理函数(占位,协议由底盘MCU定义,后续填充)
void Chassis_RxProc(void)
{
    // TODO: 解析底盘帧 -> s_chassis.{x, y, valid, rx_time_ms}
}

//读取最新底盘世界位置(只读)
const ChassisPos_t *Chassis_GetPos(void)
{
    return &s_chassis;
}

//1表示底盘坐标在timeout_ms内收到过,0表示超时
uint8_t Chassis_IsOnline(uint32_t timeout_ms)
{
    return (s_chassis.valid && (HAL_GetTick() - s_chassis.rx_time_ms) <= timeout_ms) ? 1 : 0;
}

uint8_t Vision_IsOnline(uint32_t timeout_ms)
{
    // HAL_GetTick 为 32 位无符号整数,支持自然回绕,计时逻辑仍然正确
    return ((HAL_GetTick() - s_vision.rx_time_ms) <= timeout_ms) ? 1 : 0;
}


/* -------------------- HAL 中断回调函数 -------------------- */

/**
 * @brief  串口空闲 / DMA 中断回调函数由 HAL 库自动调用
 * @note   每收到一帧(满或空闲触发),HAL 自动调用这个回调函数
 * @param  huart  中断回调的串口句柄
 * @param  Size   接收已获得的字节数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART6)
    {
        parse_frame(s_rx_buf, Size);   //解析本帧,失败则保留上一帧数据
        vision_start_dma();            //重新启动接收,等待下一帧
    }
}
