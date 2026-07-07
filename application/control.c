#include "control.h"
#include "receive.h"
#include "pid.h"
#include "QD4310.h"
#include "bmi088.h"
#include "can.h"
#include <math.h>

/* =====================================================================
 * 控制思路:绝对空间角单角度环
 *
 *   树莓派err ─►[累加器]─► θref ─►(θref − θ反馈)─►[平滑]─► 角度PID ─► Wref(rad/s)
 *                                       ▲                                  │
 *               θ反馈 = 加速度计+陀螺仪融合的绝对空间角                     │ rad/s→rpm
 *               (yaw:BMI088_GetYaw / pitch:BMI088_GetPitch)                ▼
 *                                                                  QD4310_SetSpeed
 *
 * 说明:
 *  - 累加器:树莓派(视觉)每帧发来角误差err,θref += err。视觉较慢,故只在收到
 *    有效帧时累加,两帧之间θref保持不变;高频的角度环用IMU绝对角持续跟踪θref。
 *    视觉丢失时同样不累加 → θref锁死,云台停在最后瞄准的世界方向。
 *  - MCU只做外层"角度环",输出目标角速度;内层"速度环"由QD4310内部闭环,
 *    故直接把角速度换算成rpm用QD4310_SetSpeed下发(不在MCU再做速度PID)。
 * =====================================================================*/

/* ---- 调参/硬件映射(上电标定时调整) ---- */
#define RADPS_TO_RPM        (60.0f / QD4310_TWO_PI)  //云台rad/s->电机rpm换算
#define YAW_MOTOR_DIR       (+1.0f)   //yaw电机转向反了改成-1.0f
#define PITCH_MOTOR_DIR     (+1.0f)   //pitch电机转向反了改成-1.0f
#define VISION_TIMEOUT_MS   (100U)    //视觉信号超时阈值,ms
#define ERR_LP_ALPHA        (0.3f)    //角度误差平滑系数[0,1],=1则不平滑
#define W_MAX_RADPS         (100.0f)  //角度环输出角速度限幅,rad/s(*9.55≈955rpm<1000)

//角度环PID:输入=角度误差(rad),输出=云台角速度(rad/s)
static PID_TypeDef pid_yaw;
static PID_TypeDef pid_pitch;
static QD4310_t gimbal_motor0; //yaw电机(水平转动)
static QD4310_t gimbal_motor1; //pitch电机(俯仰转动)

//累加器输出的目标角(绝对空间角,rad,连续不wrap),视觉丢失时保持不变
static float yaw_ref;
static float pitch_ref;
//平滑处理后的角度误差(rad)
static float yaw_err_lp;
static float pitch_err_lp;
static uint8_t ref_inited = 0;   //0:目标角还没用当前反馈初始化过

//把角度归一化到[-pi,pi]
static float wrap_pi(float a)
{
    while (a >  QD4310_PI) a -= QD4310_TWO_PI;
    while (a < -QD4310_PI) a += QD4310_TWO_PI;
    return a;
}

void Control_Init(void)
{
    PID_Init(&pid_yaw, 0.5f, 0.1f, 0.05f);
    PID_Init(&pid_pitch, 0.5f, 0.1f, 0.05f);

    //输出是角速度(rad/s),限幅保证换算成rpm后不超QD4310量程
    PID_LimitConfig(&pid_yaw, W_MAX_RADPS, -W_MAX_RADPS);
    PID_LimitConfig(&pid_pitch, W_MAX_RADPS, -W_MAX_RADPS);

    //云台电机在CAN1上,id分别为0(yaw)和1(pitch)
    gimbal_motor0.id   = 0;
    gimbal_motor0.hcan = &hcan1;
    gimbal_motor1.id   = 1;
    gimbal_motor1.hcan = &hcan1;

    QD4310_Enable(&gimbal_motor0);
    QD4310_Enable(&gimbal_motor1);

    ref_inited = 0;
}

void Control_Proc(void)
{
    //(1) 反馈:加速度计+陀螺仪融合出的绝对空间角
    float yaw_fb   = BMI088_GetYaw();     //连续世界yaw,rad
    float pitch_fb = BMI088_GetPitch();   //绝对pitch,rad

    //开机第一帧:累加器对齐当前反馈,避免上电瞬间猛甩
    if (!ref_inited)
    {
        yaw_ref   = yaw_fb;
        pitch_ref = pitch_fb;
        yaw_err_lp   = 0.0f;
        pitch_err_lp = 0.0f;
        ref_inited = 1;
    }

    //(2) 累加器:视觉有效时把树莓派发来的角误差累加进目标角;
    //    丢失/无新帧则不累加 → θref保持,云台锁住当前朝向
    const VisionErr_t *v = Vision_GetErr();
    uint8_t vision_ok = (v->valid && Vision_IsOnline(VISION_TIMEOUT_MS));

    if (vision_ok)
    {
        yaw_ref   += v->yaw_err_rad;
        pitch_ref += v->pitch_err_rad;
    }

    //(3) 角度误差 = θref − θ反馈,连续角先wrap到[-pi,pi]再平滑处理
    float yaw_e   = wrap_pi(yaw_ref   - yaw_fb);
    float pitch_e = wrap_pi(pitch_ref - pitch_fb);
    yaw_err_lp   += ERR_LP_ALPHA * (yaw_e   - yaw_err_lp);
    pitch_err_lp += ERR_LP_ALPHA * (pitch_e - pitch_err_lp);

    //(4) 角度环PID:传SP=0、FB=-err,等价于内部err=角度误差(误差正→输出角速度正)
    PID_ChangeSP(&pid_yaw, 0.0f);
    PID_ChangeSP(&pid_pitch, 0.0f);
    float w_yaw   = PID_Compute(&pid_yaw,   -yaw_err_lp);     //rad/s
    float w_pitch = PID_Compute(&pid_pitch, -pitch_err_lp);   //rad/s

    //(5) 角速度(rad/s) → 电机rpm,应用方向系数,下发(速度环在QD4310内部)
    SetGimbal0Speed(w_yaw   * RADPS_TO_RPM * YAW_MOTOR_DIR);
    SetGimbal1Speed(w_pitch * RADPS_TO_RPM * PITCH_MOTOR_DIR);
}

//电机0: 云台yaw轴目标转速(rpm)
void SetGimbal0Speed(float speed_rpm)
{
    QD4310_SetSpeed(&gimbal_motor0, speed_rpm);
}

//电机1: 云台pitch轴目标转速(rpm)
void SetGimbal1Speed(float speed_rpm)
{
    QD4310_SetSpeed(&gimbal_motor1, speed_rpm);
}
