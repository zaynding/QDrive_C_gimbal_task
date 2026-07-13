#include "control.h"
#include "receive.h"
#include "pid.h"
#include "QD4310.h"
#include "bmi088.h"
#include "bmi_dection.h"
#include "can.h"
#include <math.h>

/* 角度PID输出目标角速度,速度闭环由QD4310内部完成. */
#define RADPS_TO_RPM (60.0f / QD4310_TWO_PI)
#define YAW_MOTOR_DIR (+1.0f)
#define PITCH_MOTOR_DIR (+1.0f)
#define VISION_TIMEOUT_MS (100U)
#define MOTOR_TIMEOUT_MS (50U)
#define IMU_TIMEOUT_MS (10U)
#define ERR_LP_ALPHA (0.3f)
#define W_MAX_RADPS (4.0f)
#define YAW_MAX_RAD (QD4310_PI / 6.0f)
#define PITCH_MAX_RAD (0.5f)

static PID_TypeDef pid_yaw;
static PID_TypeDef pid_pitch;
static QD4310_t gimbal_motor0;
static QD4310_t gimbal_motor1;

static float yaw_ref;
static float pitch_ref;
static float yaw_err_lp;
static float pitch_err_lp;

static float pitch_imu_zero;
static float yaw_motor_zero;
static float pitch_motor_zero;

static uint32_t last_vision_time_ms;
static uint8_t vision_frame_seen;
static uint8_t control_initialized;
static uint8_t ref_initialized;
static uint8_t attitude_reset;
static uint8_t stability_enabled;
static uint8_t control_started;
static volatile uint8_t control_update_pending;
static volatile uint32_t last_imu_update_ms;

// 将角度限制到[-pi, pi].
static float wrap_pi(float angle)
{
    while (angle > QD4310_PI)
        angle -= QD4310_TWO_PI;
    while (angle < -QD4310_PI)
        angle += QD4310_TWO_PI;
    return angle;
}

// 读取Yaw和Pitch电机编码器角度.
static void get_motor_angle(float *yaw, float *pitch)
{
    *yaw = QD4310_GetYawAngle();
    *pitch = QD4310_GetPitchAngle();
}

// 计算开启自稳时使用的惯性空间角度.
static void get_stability_angle(float *yaw, float *pitch)
{
    float motor_pitch = QD4310_GetPitchAngle();

    *yaw = BMI088_GetYaw();
    *pitch = (BMI088_GetPitch() - pitch_imu_zero) +
             wrap_pi(motor_pitch - pitch_motor_zero);
}

// 根据自稳状态选择当前角度反馈.
static void get_feedback_angle(float *yaw, float *pitch)
{
    if (stability_enabled)
        get_stability_angle(yaw, pitch);
    else
        get_motor_angle(yaw, pitch);
}

// 切换角度反馈来源并保持切换前后的控制误差不变.
static void switch_feedback(uint8_t enable)
{
    float old_yaw;
    float old_pitch;
    float new_yaw;
    float new_pitch;

    get_feedback_angle(&old_yaw, &old_pitch);
    stability_enabled = enable;
    get_feedback_angle(&new_yaw, &new_pitch);

    if (ref_initialized)
    {
        yaw_ref += new_yaw - old_yaw;
        pitch_ref += new_pitch - old_pitch;
    }
    else
    {
        yaw_ref = new_yaw;
        pitch_ref = new_pitch;
        ref_initialized = 1;
    }
}

// 初始化PID、电机和控制状态.
void Control_Init(void)
{
    /* Example gains converted from discrete 1 kHz PID to the dt-based PID. */
    PID_Init(&pid_yaw, 3.3f, 0.1f, 0.2f);
    PID_Init(&pid_pitch, 3.3f, 0.1f, 0.2f);
    PID_LimitConfig(&pid_yaw, W_MAX_RADPS, -W_MAX_RADPS);
    PID_LimitConfig(&pid_pitch, W_MAX_RADPS, -W_MAX_RADPS);
    PID_ChangeSP(&pid_yaw, 0.0f);
    PID_ChangeSP(&pid_pitch, 0.0f);

    gimbal_motor0.id = 0;
    gimbal_motor0.hcan = &hcan1;
    gimbal_motor1.id = 1;
    gimbal_motor1.hcan = &hcan1;

    QD4310_FeedbackInit();
    QD4310_Enable(&gimbal_motor0);
    QD4310_Enable(&gimbal_motor1);

    yaw_ref = 0.0f;
    pitch_ref = 0.0f;
    yaw_err_lp = 0.0f;
    pitch_err_lp = 0.0f;
    pitch_imu_zero = 0.0f;
    yaw_motor_zero = 0.0f;
    pitch_motor_zero = 0.0f;
    last_vision_time_ms = 0U;
    vision_frame_seen = 0U;
    ref_initialized = 0U;
    attitude_reset = 0U;
    stability_enabled = 0U;
    control_started = 0U;
    control_update_pending = 0U;
    last_imu_update_ms = 0U;
    control_initialized = 1U;
}

// 由IMU中断通知主循环执行一次控制更新.
void Control_RequestUpdateFromISR(void)
{
    control_update_pending = 1U;
    last_imu_update_ms = HAL_GetTick();
}

// 在主循环中处理一次待执行的控制周期.
void Control_Task(void)
{
    uint32_t primask;

    if (!control_update_pending)
        return;

    primask = __get_PRIMASK();
    __disable_irq();
    control_update_pending = 0U;
    if (!primask)
        __enable_irq();

    Control_Proc();
}

// 判断两轴电机反馈是否在线且电机已经使能.
uint8_t Control_AreMotorsReady(void)
{
    return QD4310_IsYawOnline(MOTOR_TIMEOUT_MS) &&
           QD4310_IsPitchOnline(MOTOR_TIMEOUT_MS) &&
           QD4310_IsYawEnabled() && QD4310_IsPitchEnabled();
}

// 判断IMU数据是否仍在持续更新.
uint8_t Control_IsIMUOnline(void)
{
    return last_imu_update_ms != 0U &&
           ((HAL_GetTick() - last_imu_update_ms) <= IMU_TIMEOUT_MS);
}

// 返回距离最近一次IMU数据更新经过的毫秒数.
uint32_t Control_GetIMUAgeMs(void)
{
    if (last_imu_update_ms == 0U)
        return 0xFFFFFFFFU;
    return HAL_GetTick() - last_imu_update_ms;
}

// 对离线或未使能的电机重新发送使能命令.
void Control_RetryMotorEnable(void)
{
    if (!QD4310_IsYawOnline(MOTOR_TIMEOUT_MS) || !QD4310_IsYawEnabled())
        QD4310_Enable(&gimbal_motor0);
    if (!QD4310_IsPitchOnline(MOTOR_TIMEOUT_MS) || !QD4310_IsPitchEnabled())
        QD4310_Enable(&gimbal_motor1);
}

// 重置Yaw并记录Pitch姿态与编码器零点.
uint8_t Control_ResetAttitude(void)
{
    float old_yaw;
    float old_pitch;
    float new_yaw;
    float new_pitch;

    if (!control_initialized || !BMI088_YawIsReady())
        return 0U;

    get_feedback_angle(&old_yaw, &old_pitch);
    BMI088_YawReset();
    pitch_imu_zero = BMI088_GetPitch();
    yaw_motor_zero = QD4310_GetYawAngle();
    pitch_motor_zero = QD4310_GetPitchAngle();
    attitude_reset = 1U;
    get_feedback_angle(&new_yaw, &new_pitch);

    if (stability_enabled && ref_initialized)
    {
        yaw_ref += new_yaw - old_yaw;
        pitch_ref += new_pitch - old_pitch;
    }

    return 1U;
}

// 开启IMU惯性空间角度反馈.
uint8_t Control_EnableStability(void)
{
    if (!control_initialized || !attitude_reset || !BMI088_YawIsReady())
        return 0U;
    if (!stability_enabled)
        switch_feedback(1U);
    return 1U;
}

// 关闭自稳并切换回电机编码器反馈.
void Control_DisableStability(void)
{
    if (control_initialized && stability_enabled)
        switch_feedback(0U);
}

// 返回当前是否已经开启自稳.
uint8_t Control_IsStabilityEnabled(void)
{
    return stability_enabled;
}

// 对齐当前目标角并启动云台控制.
uint8_t Control_Start(void)
{
    float yaw_fb;
    float pitch_fb;

    if (!control_initialized || !attitude_reset || !Control_AreMotorsReady())
        return 0U;

    get_feedback_angle(&yaw_fb, &pitch_fb);
    yaw_ref = yaw_fb;
    pitch_ref = pitch_fb;
    yaw_err_lp = 0.0f;
    pitch_err_lp = 0.0f;
    PID_Reset(&pid_yaw);
    PID_Reset(&pid_pitch);
    ref_initialized = 1U;
    control_started = 1U;
    return 1U;
}

// 停止云台控制并将两轴目标速度清零.
void Control_Stop(void)
{
    if (!control_initialized)
        return;

    control_started = 0U;
    SetGimbal0Speed(0.0f);
    SetGimbal1Speed(0.0f);
    PID_Reset(&pid_yaw);
    PID_Reset(&pid_pitch);
    yaw_err_lp = 0.0f;
    pitch_err_lp = 0.0f;
}

// 返回云台控制是否处于运行状态.
uint8_t Control_IsStarted(void)
{
    return control_started;
}

// 完成一次视觉目标更新、角度PID计算和电机速度下发.
void Control_Proc(void)
{
    float yaw_fb;
    float pitch_fb;
    float yaw_error;
    float pitch_error;
    float yaw_e;
    float pitch_e;
    float w_yaw;
    float w_pitch;
    const VisionErr_t *vision;

    if (!control_initialized || !control_started)
        return;

    if (!Control_AreMotorsReady() || !Control_IsIMUOnline())
    {
        if (!Control_AreMotorsReady())
            USART1_SetExitReason(USART1_EXIT_MOTOR_NOT_READY);
        else
            USART1_SetExitReason(USART1_EXIT_IMU_OFFLINE);
        Control_Stop();
        return;
    }

    get_feedback_angle(&yaw_fb, &pitch_fb);
    if (!ref_initialized)
    {
        yaw_ref = yaw_fb;
        pitch_ref = pitch_fb;
        yaw_err_lp = 0.0f;
        pitch_err_lp = 0.0f;
        ref_initialized = 1U;
    }

    vision = Vision_GetErr();
    if (vision->valid && vision->rx_time_ms != 0U &&
        Vision_IsOnline(VISION_TIMEOUT_MS) &&
        (!vision_frame_seen || vision->rx_time_ms != last_vision_time_ms))
    {
        float target_x = VISION_LASER_X + (float)vision->ex_px;
        float target_y = VISION_LASER_Y + (float)vision->ey_px;

        yaw_error = -(atan2f(target_x - VISION_CX, VISION_FX) -
                      atan2f(VISION_LASER_X - VISION_CX, VISION_FX));
        pitch_error = -(atan2f(target_y - VISION_CY, VISION_FY) -
                        atan2f(VISION_LASER_Y - VISION_CY, VISION_FY));

        yaw_ref = yaw_fb + yaw_error;
        pitch_ref = pitch_fb + pitch_error;
        last_vision_time_ms = vision->rx_time_ms;
        vision_frame_seen = 1U;
    }

    yaw_e = wrap_pi(yaw_ref - yaw_fb);
    pitch_e = wrap_pi(pitch_ref - pitch_fb);
    yaw_err_lp += ERR_LP_ALPHA * (yaw_e - yaw_err_lp);
    pitch_err_lp += ERR_LP_ALPHA * (pitch_e - pitch_err_lp);

    w_yaw = PID_Compute(&pid_yaw, -yaw_err_lp);
    w_pitch = PID_Compute(&pid_pitch, -pitch_err_lp);

    {
        float yaw_from_zero = wrap_pi(QD4310_GetYawAngle() - yaw_motor_zero);
        float yaw_motor_command = w_yaw * YAW_MOTOR_DIR;
        float pitch_from_zero = wrap_pi(QD4310_GetPitchAngle() - pitch_motor_zero);
        float pitch_motor_command = w_pitch * PITCH_MOTOR_DIR;

        if ((yaw_motor_command > 0.0f && yaw_from_zero >= YAW_MAX_RAD) ||
            (yaw_motor_command < 0.0f && yaw_from_zero <= -YAW_MAX_RAD))
        {
            yaw_ref = yaw_fb;
            yaw_err_lp = 0.0f;
            PID_Reset(&pid_yaw);
            w_yaw = 0.0f;
        }

        if ((pitch_motor_command > 0.0f && pitch_from_zero >= PITCH_MAX_RAD) ||
            (pitch_motor_command < 0.0f && pitch_from_zero <= -PITCH_MAX_RAD))
        {
            pitch_ref = pitch_fb;
            pitch_err_lp = 0.0f;
            PID_Reset(&pid_pitch);
            w_pitch = 0.0f;
        }
    }

    SetGimbal0Speed(w_yaw * RADPS_TO_RPM * YAW_MOTOR_DIR);
    SetGimbal1Speed(w_pitch * RADPS_TO_RPM * PITCH_MOTOR_DIR);
}

// 设置Yaw电机目标速度.
void SetGimbal0Speed(float speed_rpm)
{
    QD4310_SetSpeed(&gimbal_motor0, speed_rpm);
}

// 设置Pitch电机目标速度.
void SetGimbal1Speed(float speed_rpm)
{
    QD4310_SetSpeed(&gimbal_motor1, speed_rpm);
}
