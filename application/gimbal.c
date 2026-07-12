#include "gimbal.h"
#include "control.h"
#include "bmi088.h"
#include "receive.h"
#include "user_key.h"
#include "main.h"

#define MOTOR_ENABLE_RETRY_MS (10U)
#define LASER_VISION_TIMEOUT_MS (100U)

static GimbalState_t gimbal_state = GIMBAL_STATE_UNINITIALIZED;
static uint32_t last_motor_retry_ms;

// 按固定周期重试电机使能.
static void retry_motor_enable(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - last_motor_retry_ms) >= MOTOR_ENABLE_RETRY_MS)
    {
        last_motor_retry_ms = now;
        Control_RetryMotorEnable();
    }
}

// 完成姿态复位、自稳开启和控制启动.
static uint8_t start_stability_control(void)
{
    if (!Control_AreMotorsReady() || !Control_IsIMUOnline())
        return 0U;
    if (!Control_ResetAttitude())
        return 0U;
    if (!Control_EnableStability())
        return 0U;
    return Control_Start();
}

// 根据视觉目标状态自动控制激光.
static void update_tracking_laser(void)
{
    static GimbalState_t previous_state = GIMBAL_STATE_UNINITIALIZED;
    const VisionErr_t *vision = Vision_GetErr();

    if (gimbal_state == GIMBAL_STATE_RUNNING)
    {
        uint8_t target_valid = vision->valid && Vision_IsOnline(LASER_VISION_TIMEOUT_MS);
        User_Laser_Set(target_valid);
    }
    else if (previous_state == GIMBAL_STATE_RUNNING)
    {
        User_Laser_Set(0U);
    }

    previous_state = gimbal_state;
}

// 初始化云台控制和BMI088并进入标定状态.
void Gimbal_Init(void)
{
    Control_Init();
    Control_Stop();

    if (BMI088_Init() != BMI088_OK)
    {
        gimbal_state = GIMBAL_STATE_FAULT;
        return;
    }

    last_motor_retry_ms = HAL_GetTick();
    gimbal_state = GIMBAL_STATE_CALIBRATING;
}

// 在主循环中运行云台启动、监测和故障恢复状态机.
void Gimbal_Task(void)
{
    Control_Task();

    switch (gimbal_state)
    {
        case GIMBAL_STATE_CALIBRATING:
            retry_motor_enable();
            if (BMI088_YawIsReady() && start_stability_control())
                gimbal_state = GIMBAL_STATE_RUNNING;
            break;

        case GIMBAL_STATE_RUNNING:
            if (!Control_AreMotorsReady() || !Control_IsIMUOnline() || !Control_IsStarted())
            {
                Control_Stop();
                gimbal_state = GIMBAL_STATE_RECOVERING;
            }
            break;

        case GIMBAL_STATE_RECOVERING:
            retry_motor_enable();
            if (BMI088_YawIsReady() && start_stability_control())
                gimbal_state = GIMBAL_STATE_RUNNING;
            break;

        case GIMBAL_STATE_FAULT:
            break;

        case GIMBAL_STATE_UNINITIALIZED:
        default:
            break;
    }

    update_tracking_laser();
}

// 返回当前云台运行状态.
GimbalState_t Gimbal_GetState(void)
{
    return gimbal_state;
}
