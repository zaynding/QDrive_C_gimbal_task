#include "control.h"
#include "receive.h"
#include "pid.h"
#include "QD4310.h"
#include "task.h"
#include "bmi088.h"
#include "can.h"
#include <math.h>

/* ---- 调参/硬件映射(上电标定时调整) ---- */
#define RADPS_TO_RPM        (60.0f / QD4310_TWO_PI)  //云台rad/s->电机rpm换算
#define YAW_MOTOR_DIR       (+1.0f)   //电机转向反了改成-1.0f
#define PITCH_MOTOR_DIR     (+1.0f)   //电机转向反了改成-1.0f
#define VISION_TIMEOUT_MS   (100U)    //视觉信号超时阈值,ms
#define CHASSIS_TIMEOUT_MS  (200U)    //底盘坐标超时阈值,ms

//云台角度环PID:输入=角度误差(rad),输出=云台角速度(rad/s)
static PID_TypeDef pid_yaw;
static PID_TypeDef pid_pitch;
static QD4310_t gimbal_motor0; //yaw电机(水平转动)
static QD4310_t gimbal_motor1; //pitch电机(俯仰转动)

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

    PID_LimitConfig(&pid_yaw, 500.0f, -500.0f);
    PID_LimitConfig(&pid_pitch, 500.0f, -500.0f);

    //云台电机在CAN1上,id分别为0(yaw)和1(pitch)
    gimbal_motor0.id   = 0;
    gimbal_motor0.hcan = &hcan1;
    gimbal_motor1.id   = 1;
    gimbal_motor1.hcan = &hcan1;

    QD4310_Enable(&gimbal_motor0);
    QD4310_Enable(&gimbal_motor1);
}

void Control_Proc(void)
{
    /* =====================================================================
     * yaw是世界系单一角度环:
     *   反馈(唯一)   = 云台IMU_yaw.底盘旋转会改变云台测到的yaw,被环路压制,
     *                 天然解耦无需前馈
     *   目标值(设定) = 靶子的世界绝对方向:
     *     (3) 视觉有效: 当前yaw_fb + 视觉yaw误差
     *     (2) 视觉无效: 用底盘世界坐标atan2计算绝对角度
     * =====================================================================*/

    //(1) 解耦: 什么都不产出,yaw_fb是唯一反馈
    float yaw_fb = BMI088_GetYaw();   //rad,连续世界yaw(见bmi088.c)

    const VisionErr_t *v = Vision_GetErr();
    uint8_t vision_ok = (v->valid && Vision_IsOnline(VISION_TIMEOUT_MS));

    float yaw_sp;
    if (vision_ok)
    {
        //(3) 视觉闭环: 靶子世界方向 = 当前yaw + 视觉误差
        yaw_sp = yaw_fb + v->yaw_err_rad;
    }
    else
    {
        //(2) 解算: 用底盘世界坐标计算靶子的绝对世界角度
        //方向 车->靶 = (0-x, 0-y). 世界系是FRD(IMU.md第9节),atan2符号正确
        //注意: 上电时让车头朝向世界+X,然后BMI088_YawReset()使IMU_yaw零点与绝对角对齐
        const ChassisPos_t *c = Chassis_GetPos();
        if (!Chassis_IsOnline(CHASSIS_TIMEOUT_MS))
        {
            //无视觉目标 && 无底盘坐标 -> 云台停住
            PID_Reset(&pid_yaw);
            PID_Reset(&pid_pitch);
            SetGimbal0Speed(0.0f);
            SetGimbal1Speed(0.0f);
            return;
        }
        yaw_sp = atan2f(-c->y, -c->x);
    }

    //yaw角度环: 把世界角度误差压制到0
    //PID内部err=SP-FB,传入SP=0,FB=-yaw_err等价于err=yaw_err,误差为正时输出为正
    float yaw_err = wrap_pi(yaw_sp - yaw_fb);
    PID_ChangeSP(&pid_yaw, 0.0f);
    float w_yaw = PID_Compute(&pid_yaw, -yaw_err);   //云台yaw角速度,rad/s

    //pitch: 暂时纯视觉速度环(IMU_pitch还没解算).无目标时停止
    float w_pitch = 0.0f;
    if (vision_ok)
    {
        PID_ChangeSP(&pid_pitch, 0.0f);
        w_pitch = PID_Compute(&pid_pitch, -v->pitch_err_rad);   //rad/s
    }
    else
    {
        PID_Reset(&pid_pitch);   //待机时复位,防止积分饱和和微分冲击
    }

    //(3) 云台角速度(rad/s) -> 电机rpm,应用方向系数,发送
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
