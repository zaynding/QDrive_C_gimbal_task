#include "bmi088.h"
#include "control.h"
#include "qmath.h"
#include "spi.h"
#include "gpio.h"

extern SPI_HandleTypeDef hspi1;

/* BMI088 片选引脚(与Core/Inc/main.h的INT1_ACCEL/INT1_GYRO区分开):
 *   PA4 = ACCEL CS, PB0 = GYRO CS
 *   PC4 = INT1_ACCEL (加速度计DRDY, EXTI4)
 *   PC5 = INT1_GYRO  (陀螺仪DRDY,   EXTI9_5)
 */
#define CS1_ACCEL_GPIO_Port GPIOA
#define CS1_ACCEL_Pin       GPIO_PIN_4
#define CS1_GYRO_GPIO_Port  GPIOB
#define CS1_GYRO_Pin        GPIO_PIN_0

#define ACCEL CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin
#define GYRO  CS1_GYRO_GPIO_Port, CS1_GYRO_Pin

/* ==================== 传感器物理量 & 状态 ==================== */
static float ax, ay, az;   //加速度计值,单位m/s^2
static float gx, gy, gz;   //陀螺仪值,单位rad/s
static float temperature;  //温度值,单位摄氏度
static uint8_t bmi088_error = BMI088_OK;

//yaw欧拉角解算(车体FRD)
static float yaw = 0.0f;             //连续积分的绝对yaw,单位rad
static float gz_bias = 0.0f;         //车体yaw角速度零偏,单位rad/s
static float yaw_calib_accum = 0.0f; //开机静止标定累加
static uint16_t yaw_calib_cnt = 0;
static uint8_t yaw_calibrated = 0;   //0:标定中 1:标定完成

//pitch姿态融合状态(Mahony四元数互补滤波
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float pitch = 0.0f;
#define MAHONY_TWO_KP   (2.0f * 0.5f)   //比例增益,越大越信任加速度计(收敛快但抖)
#define MAHONY_SAMPLE_S (0.001f)        //采样周期=陀螺DRDY周期(1kHz)

/* ==================== DMA异步读取状态机 ==================== */
/* 硬件链路: 陀螺DRDY(PC5)每1ms下降沿 -> EXTI9_5_IRQHandler
 *        -> HAL_GPIO_EXTI_Callback -> 启动 gyro DMA 读(7字节)
 *        -> HAL_SPI_TxRxCpltCallback: gyro相 -> 解析 + 启动 accel DMA 读(8字节)
 *                                     accel相 -> 解析 + Mahony + yaw积分 + Control_Proc
 */
typedef enum {
    IMU_PHASE_IDLE  = 0,
    IMU_PHASE_GYRO  = 1,   //DMA正在读陀螺
    IMU_PHASE_ACCEL = 2    //DMA正在读加速度计
} imu_phase_t;

static volatile imu_phase_t imu_phase = IMU_PHASE_IDLE;

//陀螺: 命令1B + 数据6B = 7B (陀螺无dummy)
//加计: 命令1B + dummy1B + 数据6B = 8B
static uint8_t gyro_tx[7]  __attribute__((aligned(4)))  = {0x02 | 0x80, 0,0,0,0,0,0};
static uint8_t gyro_rx[7]  __attribute__((aligned(4)));
static uint8_t accel_tx[8] __attribute__((aligned(4))) = {0x12 | 0x80, 0,0,0,0,0,0,0};
static uint8_t accel_rx[8] __attribute__((aligned(4)));

/* ==================== 基础SPI寄存器读写(仅初始化用,阻塞) ==================== */
static uint8_t spi_rw(uint8_t data)
{
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

static void reg_write(GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t reg, uint8_t data)
{
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    spi_rw(reg);
    spi_rw(data);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
}

static uint8_t reg_read(GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t reg, uint8_t dummy)
{
    uint8_t data;

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    spi_rw(reg | 0x80);
    if (dummy)
        spi_rw(0x00);
    data = spi_rw(0x00);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    return data;
}

/* ==================== 快速倒数平方根==================== */
//替代 1.0f/sqrtf(x),Newton-Raphson一次迭代精度足够Mahony归一化用
static float inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - halfx * y * y);
    return y;
}

/* ==================== BMI088初始化 ==================== */
uint8_t BMI088_Init(void)
{
    HAL_GPIO_WritePin(ACCEL, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GYRO, GPIO_PIN_SET);
    HAL_Delay(10);

    reg_read(ACCEL, 0x00, 1);          //加速度计上电后首次读操作会失败,丢弃
    HAL_Delay(1);
    reg_write(ACCEL, 0x7E, 0xB6);      //软复位
    HAL_Delay(80);

    reg_read(ACCEL, 0x00, 1);
    HAL_Delay(1);
    if (reg_read(ACCEL, 0x00, 1) != 0x1E)   //校验chip id
    {
        bmi088_error = BMI088_ACCEL_ID_ERROR;
        return bmi088_error;
    }

    reg_write(ACCEL, 0x7D, 0x04);   //开启加速度计
    HAL_Delay(1);
    reg_write(ACCEL, 0x7C, 0x00);   //进入正常模式
    HAL_Delay(1);
    reg_write(ACCEL, 0x40, 0xAB);   //输出速率800hz,普通滤波
    HAL_Delay(1);
    reg_write(ACCEL, 0x41, 0x00);   //量程3g
    HAL_Delay(1);
    reg_write(ACCEL, 0x53, 0x08);   //INT1 输出使能,推挽,低有效
    HAL_Delay(1);
    reg_write(ACCEL, 0x58, 0x04);   //数据就绪映射到INT1
    HAL_Delay(1);

    if (reg_read(ACCEL, 0x41, 1) != 0x00)
    {
        bmi088_error = BMI088_ACCEL_CONFIG_ERROR;
        return bmi088_error;
    }

    reg_write(GYRO, 0x14, 0xB6);    //软复位
    HAL_Delay(80);

    reg_read(GYRO, 0x00, 0);
    HAL_Delay(1);
    if (reg_read(GYRO, 0x00, 0) != 0x0F)    //校验chip id
    {
        bmi088_error = BMI088_GYRO_ID_ERROR;
        return bmi088_error;
    }

    reg_write(GYRO, 0x0F, 0x00);    //量程2000deg/s
    HAL_Delay(1);
    reg_write(GYRO, 0x10, 0x82);    //输出速率1000hz
    HAL_Delay(1);
    reg_write(GYRO, 0x11, 0x00);    //进入正常模式
    HAL_Delay(1);
    reg_write(GYRO, 0x15, 0x80);    //INT3使能,推挽,低有效
    HAL_Delay(1);
    reg_write(GYRO, 0x16, 0x00);
    HAL_Delay(1);
    reg_write(GYRO, 0x18, 0x01);    //数据就绪映射到INT3(PC5)
    HAL_Delay(1);

    if (reg_read(GYRO, 0x0F, 0) != 0x00)
    {
        bmi088_error = BMI088_GYRO_CONFIG_ERROR;
        return bmi088_error;
    }

    //初始化到此为止,EXTI由CubeMX使能,首次陀螺DRDY下降沿即启动DMA链
    imu_phase = IMU_PHASE_IDLE;
    return BMI088_OK;
}

/* ==================== 姿态解算 ==================== */
//yaw:开机静止标定零偏后,纯积分陀螺z轴(加计对yaw无观测,故不融合)
//原始为FLU,车体为FRD: gyro_z_body = -gyro_z_raw
static void yaw_solve_step(void)
{
    float r = -gz;   //rad/s

    if (!yaw_calibrated)
    {
        yaw_calib_accum += r;
        //1kHz采样,采1000个≈1s静止标定
        if (++yaw_calib_cnt >= 1000U)
        {
            gz_bias = yaw_calib_accum / 1000.0f;
            yaw_calibrated = 1;
        }
        return;
    }

    yaw += (r - gz_bias) * MAHONY_SAMPLE_S;
}

//pitch:Mahony互补滤波(6轴,无磁力计),移植自例程MahonyAHRS.h
//加速度计校正陀螺的pitch/roll漂移,得到绝对pitch
static void pitch_solve_step(void)
{
    float gx_ = gx, gy_ = gy, gz_ = gz;
    float recip_norm;

    //1.加计有效时,用重力方向的观测误差校正陀螺(比例项,Ki=0)
    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f))
    {
        recip_norm = inv_sqrt(ax * ax + ay * ay + az * az);
        float axn = ax * recip_norm;
        float ayn = ay * recip_norm;
        float azn = az * recip_norm;

        float halfvx = q1 * q3 - q0 * q2;
        float halfvy = q0 * q1 + q2 * q3;
        float halfvz = q0 * q0 - 0.5f + q3 * q3;

        float halfex = ayn * halfvz - azn * halfvy;
        float halfey = azn * halfvx - axn * halfvz;
        float halfez = axn * halfvy - ayn * halfvx;

        gx_ += MAHONY_TWO_KP * halfex;
        gy_ += MAHONY_TWO_KP * halfey;
        gz_ += MAHONY_TWO_KP * halfez;
    }

    //2.积分四元数
    gx_ *= 0.5f * MAHONY_SAMPLE_S;
    gy_ *= 0.5f * MAHONY_SAMPLE_S;
    gz_ *= 0.5f * MAHONY_SAMPLE_S;
    float qa = q0, qb = q1, qc = q2;
    q0 += -qb * gx_ - qc * gy_ - q3 * gz_;
    q1 +=  qa * gx_ + qc * gz_ - q3 * gy_;
    q2 +=  qa * gy_ - qb * gz_ + q3 * gx_;
    q3 +=  qa * gz_ + qb * gy_ - qc * gx_;

    //3.归一化(用Quake快速倒数平方根)
    recip_norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recip_norm;
    q1 *= recip_norm;
    q2 *= recip_norm;
    q3 *= recip_norm;

    //4.提取pitch = asin(-2*(q1q3 - q0q2)),用qmath加速
    pitch = qasin(-2.0f * (q1 * q3 - q0 * q2));
}

/* ==================== DMA传输链 ==================== */
static void start_gyro_dma(void)
{
    HAL_GPIO_WritePin(GYRO, GPIO_PIN_RESET);
    imu_phase = IMU_PHASE_GYRO;
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, gyro_tx, gyro_rx, 7) != HAL_OK)
    {
        HAL_GPIO_WritePin(GYRO, GPIO_PIN_SET);
        imu_phase = IMU_PHASE_IDLE;
    }
}

static void start_accel_dma(void)
{
    HAL_GPIO_WritePin(ACCEL, GPIO_PIN_RESET);
    imu_phase = IMU_PHASE_ACCEL;
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, accel_tx, accel_rx, 8) != HAL_OK)
    {
        HAL_GPIO_WritePin(ACCEL, GPIO_PIN_SET);
        imu_phase = IMU_PHASE_IDLE;
    }
}

//把陀螺 rx[1..6] (小端16位×3) 转成 gx/gy/gz (rad/s)
static void parse_gyro(void)
{
    int16_t raw;
    raw = (int16_t)((gyro_rx[2] << 8) | gyro_rx[1]);
    gx = raw * 2000.0f / 32768.0f * 3.14159265f / 180.0f;
    raw = (int16_t)((gyro_rx[4] << 8) | gyro_rx[3]);
    gy = raw * 2000.0f / 32768.0f * 3.14159265f / 180.0f;
    raw = (int16_t)((gyro_rx[6] << 8) | gyro_rx[5]);
    gz = raw * 2000.0f / 32768.0f * 3.14159265f / 180.0f;
}

//把加计 rx[2..7] (跳过cmd+dummy) 转成 ax/ay/az (m/s^2)
static void parse_accel(void)
{
    int16_t raw;
    raw = (int16_t)((accel_rx[3] << 8) | accel_rx[2]);
    ax = raw * 3.0f * 9.80665f / 32768.0f;
    raw = (int16_t)((accel_rx[5] << 8) | accel_rx[4]);
    ay = raw * 3.0f * 9.80665f / 32768.0f;
    raw = (int16_t)((accel_rx[7] << 8) | accel_rx[6]);
    az = raw * 3.0f * 9.80665f / 32768.0f;
}

/* ==================== HAL回调(重写weak) ==================== */
//陀螺DRDY(PC5,EXTI9_5)下降沿 -> 启动一轮 gyro→accel DMA链
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_5)          //INT1_GYRO
    {
        //若上一轮还没完成(理论上1ms周期足够),这次跳过防止DMA冲突
        if (imu_phase == IMU_PHASE_IDLE)
        {
            start_gyro_dma();
        }
    }
    //INT1_ACCEL(PC4)在本设计中不启动独立链,accel由gyro链末尾串读
}

//SPI DMA传输完成: 分阶段推进状态机 -> Mahony -> yaw积分 -> Control_Proc
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) return;

    if (imu_phase == IMU_PHASE_GYRO)
    {
        HAL_GPIO_WritePin(GYRO, GPIO_PIN_SET);
        parse_gyro();
        start_accel_dma();   //链式启动accel读
    }
    else if (imu_phase == IMU_PHASE_ACCEL)
    {
        HAL_GPIO_WritePin(ACCEL, GPIO_PIN_SET);
        parse_accel();

        //一轮采样齐了 -> 姿态融合 -> 通知裸机控制任务
        yaw_solve_step();
        pitch_solve_step();

        imu_phase = IMU_PHASE_IDLE;
        Control_RequestUpdateFromISR();
    }
}

// SPI或DMA异常后释放片选并允许下一次数据就绪中断重新采样.
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1)
        return;

    HAL_GPIO_WritePin(GYRO, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ACCEL, GPIO_PIN_SET);
    imu_phase = IMU_PHASE_IDLE;
}

/* ==================== 只读getter ==================== */
float BMI088_GetAx(void) { return ax; }
float BMI088_GetAy(void) { return ay; }
float BMI088_GetAz(void) { return az; }

float BMI088_GetGx(void) { return gx; }
float BMI088_GetGy(void) { return gy; }
float BMI088_GetGz(void) { return gz; }

float BMI088_GetYaw(void)   { return yaw; }
float BMI088_GetPitch(void) { return pitch; }

void BMI088_YawReset(void)      { yaw = 0.0f; }
uint8_t BMI088_YawIsReady(void) { return yaw_calibrated; }

float BMI088_GetTemperature(void) { return temperature; }
uint8_t BMI088_GetError(void)     { return bmi088_error; }
