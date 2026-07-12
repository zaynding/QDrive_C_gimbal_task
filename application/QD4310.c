//
// Created by jason on 25-10-16.
//
#include "main.h"
#include "QD4310.h"

static volatile float yaw_angle;
static volatile float pitch_angle;
static volatile uint32_t yaw_feedback_time_ms;
static volatile uint32_t pitch_feedback_time_ms;
static volatile uint8_t yaw_feedback_valid;
static volatile uint8_t pitch_feedback_valid;
static volatile uint8_t yaw_enabled;
static volatile uint8_t pitch_enabled;

// 限制函数，用于替代C++的std::clamp
static float QD4310_Clamp(float value, float min, float max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

// 发送命令到电机
void QD4310_SendCommand(QD4310_t *motor, QD4310_Command_t cmd, int16_t value)
{
    static uint8_t TxBuffer[3];
    TxBuffer[0] = (uint8_t)cmd;
    // 将int16_t值拆分为两个字节
    TxBuffer[1] = (uint8_t)(value & 0xFF);
    TxBuffer[2] = (uint8_t)((value >> 8) & 0xFF);

    uint32_t txMailbox = CAN_TX_MAILBOX0;
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = 0x400 + motor->id;
    TxHeader.ExtId = 0x400 + motor->id;
    TxHeader.TransmitGlobalTime = DISABLE;
    TxHeader.DLC = 3;
    HAL_CAN_AddTxMessage(motor->hcan, &TxHeader, TxBuffer, &txMailbox);
    while (HAL_CAN_GetTxMailboxesFreeLevel(motor->hcan) == 0)
        ;
}

// 更新电机状态
void QD4310_Update(QD4310_t *motor, const uint8_t feedback[8])
{
    motor->enabled = feedback[0] & 0x01;

    // 重构int16_t值从字节数组
    int16_t current_raw = (int16_t)((feedback[3] << 8) | feedback[2]);
    motor->current = (float)current_raw * 10.0f / INT16_MAX;

    int16_t speed_raw = (int16_t)((feedback[5] << 8) | feedback[4]);
    motor->speed = (float)speed_raw * 1000.0f / 32767.0f;

    uint16_t angle_raw = (uint16_t)((feedback[7] << 8) | feedback[6]);
    motor->angle = (float)angle_raw * QD4310_TWO_PI / UINT16_MAX;
}

// 初始化CAN反馈接收和中断.
void QD4310_FeedbackInit(void)
{
    CAN_FilterTypeDef filter = {0};

    yaw_feedback_valid = 0U;
    pitch_feedback_valid = 0U;
    yaw_enabled = 0U;
    pitch_enabled = 0U;

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
        Error_Handler();
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
        Error_Handler();
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        Error_Handler();
}

// 返回Yaw电机当前编码器角度.
float QD4310_GetYawAngle(void)
{
    return yaw_angle;
}

// 返回Pitch电机当前编码器角度.
float QD4310_GetPitchAngle(void)
{
    return pitch_angle;
}

// 判断Yaw电机反馈是否在线.
uint8_t QD4310_IsYawOnline(uint32_t timeout_ms)
{
    return yaw_feedback_valid && ((HAL_GetTick() - yaw_feedback_time_ms) <= timeout_ms);
}

// 判断Pitch电机反馈是否在线.
uint8_t QD4310_IsPitchOnline(uint32_t timeout_ms)
{
    return pitch_feedback_valid && ((HAL_GetTick() - pitch_feedback_time_ms) <= timeout_ms);
}

// 返回Yaw电机反馈中的使能状态.
uint8_t QD4310_IsYawEnabled(void)
{
    return yaw_enabled;
}

// 返回Pitch电机反馈中的使能状态.
uint8_t QD4310_IsPitchEnabled(void)
{
    return pitch_enabled;
}

// 接收电机CAN反馈并更新两轴编码器角度和状态.
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    uint16_t angle_raw;
    float angle;

    if (hcan != &hcan1)
        return;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        return;

    if (rx_header.IDE == CAN_ID_STD && rx_header.RTR == CAN_RTR_DATA &&
        rx_header.DLC == 8U && (rx_header.StdId == 0x500 || rx_header.StdId == 0x501))
    {
        angle_raw = (uint16_t)((rx_data[7] << 8) | rx_data[6]);
        angle = (float)angle_raw * QD4310_TWO_PI / UINT16_MAX;

        if (rx_header.StdId == 0x500)
        {
            yaw_angle = angle;
            yaw_enabled = rx_data[0] & 0x01U;
            yaw_feedback_time_ms = HAL_GetTick();
            yaw_feedback_valid = 1U;
        }
        else
        {
            pitch_angle = angle;
            pitch_enabled = rx_data[0] & 0x01U;
            pitch_feedback_time_ms = HAL_GetTick();
            pitch_feedback_valid = 1U;
        }
    }
}

// 使能电机
void QD4310_Enable(QD4310_t *motor)
{
    QD4310_SendCommand(motor, QD4310_CMD_ENABLE, 0x0000);
}

// 失能电机
void QD4310_Disable(QD4310_t *motor)
{
    QD4310_SendCommand(motor, QD4310_CMD_DISABLE, 0x0000);
}

// 设置电机角度
void QD4310_SetAngle(QD4310_t *motor, float angle)
{
    angle = QD4310_Clamp(angle, 0.0f, QD4310_TWO_PI);
    int16_t angle_value = (int16_t)(angle / QD4310_TWO_PI * UINT16_MAX);
    QD4310_SendCommand(motor, QD4310_CMD_ANGLE, angle_value);
}

// 设置电机步进角度
void QD4310_SetStepAngle(QD4310_t *motor, float step_angle)
{
    step_angle = QD4310_Clamp(step_angle, QD4310_MIN_STEPANGLE, QD4310_MAX_STEPANGLE);
    int16_t step_angle_value = (int16_t)(step_angle / QD4310_MAX_STEPANGLE * INT16_MAX);
    QD4310_SendCommand(motor, QD4310_CMD_STEP_ANGLE, step_angle_value);
}

// 设置电机转速
void QD4310_SetSpeed(QD4310_t *motor, float speed)
{
    speed = QD4310_Clamp(speed, QD4310_MIN_SPEED, QD4310_MAX_SPEED);
    int16_t speed_value = (int16_t)(speed / QD4310_MAX_SPEED * INT16_MAX);
    QD4310_SendCommand(motor, QD4310_CMD_SPEED, speed_value);
}

// 设置电机低速
void QD4310_SetLowSpeed(QD4310_t *motor, float speed)
{
    speed = QD4310_Clamp(speed, QD4310_MIN_SPEED, QD4310_MAX_SPEED);
    int16_t speed_value = (int16_t)(speed / QD4310_MAX_SPEED * INT16_MAX);
    QD4310_SendCommand(motor, QD4310_CMD_LOW_SPEED, speed_value);
}

// 设置电机电流
void QD4310_SetCurrent(QD4310_t *motor, float current)
{
    current = QD4310_Clamp(current, QD4310_MIN_CURRENT, QD4310_MAX_CURRENT);
    int16_t current_value = (int16_t)(current / QD4310_MAX_CURRENT * INT16_MAX);
    QD4310_SendCommand(motor, QD4310_CMD_CURRENT, current_value);
}
