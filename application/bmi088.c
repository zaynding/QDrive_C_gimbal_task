#include "bmi088.h"
#include "spi.h"
#include "gpio.h"

extern SPI_HandleTypeDef hspi1;

/* BMI088 片选引脚：（PA4=ACCEL, PB0=GYRO） */
#define CS1_ACCEL_GPIO_Port GPIOA
#define CS1_ACCEL_Pin       GPIO_PIN_4
#define CS1_GYRO_GPIO_Port  GPIOB
#define CS1_GYRO_Pin        GPIO_PIN_0

#define ACCEL CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin
#define GYRO  CS1_GYRO_GPIO_Port, CS1_GYRO_Pin

static float ax, ay, az;   //加速度计值,单位m/s^2
static float gx, gy, gz;   //陀螺仪值,单位rad/s
static float temperature;  //温度值,单位摄氏度
static uint8_t bmi088_error = BMI088_OK;

//yaw欧拉角解算相关状态(车体FRD坐标系)
static float yaw = 0.0f;             //连续积分的绝对yaw,单位rad
static float gz_bias = 0.0f;         //车体yaw角速度零偏,单位rad/s
static float yaw_calib_accum = 0.0f; //开机标定时的角速度累加和
static uint16_t yaw_calib_cnt = 0;   //开机标定已采集的样本数
static uint8_t yaw_calibrated = 0;   //0:标定中 1:标定完成

static uint8_t spi_rw(uint8_t data)
{
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

//cs:片选引脚(加速度计/陀螺仪共用同一套收发逻辑,靠片选区分芯片)
static void reg_write(GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t reg, uint8_t data)
{
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    spi_rw(reg);
    spi_rw(data);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
}

//dummy:加速度计读操作在地址后多插一个哑字节,陀螺仪没有,靠这个参数区分
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

static void reg_read_buf(GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t reg, uint8_t dummy, uint8_t *buf, uint8_t len)
{
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    spi_rw(reg | 0x80);
    if (dummy)
        spi_rw(0x00);
    while (len--)
    {
        *buf++ = spi_rw(0x00);
    }
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
}

//BMI088初始化
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
    reg_write(ACCEL, 0x53, 0x08);
    HAL_Delay(1);
    reg_write(ACCEL, 0x58, 0x04);
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
    reg_write(GYRO, 0x15, 0x80);    //开启中断
    HAL_Delay(1);
    reg_write(GYRO, 0x16, 0x00);
    HAL_Delay(1);
    reg_write(GYRO, 0x18, 0x01);
    HAL_Delay(1);

    if (reg_read(GYRO, 0x0F, 0) != 0x00)
    {
        bmi088_error = BMI088_GYRO_CONFIG_ERROR;
        return bmi088_error;
    }

    bmi088_error = BMI088_Update();
    return bmi088_error;
}

//yaw欧拉角解算:开机静止标定零偏后,纯积分陀螺z轴角速度得到连续yaw
//     加速度计只能校正roll/pitch,对yaw无观测,故不做卡尔曼/加计融合。
//原始为FLU,车体为FRD: gyro_z_body = -gyro_z_raw, r>0表示车头右转。
static void BMI088_YawSolve(void)
{
    float r = -gz;   //raw(FLU) -> body(FRD)的yaw角速度,单位rad/s

    //开机假设静止约1s(200*5ms),对角速度求平均作为零偏
    if (!yaw_calibrated)
    {
        yaw_calib_accum += r;
        if (++yaw_calib_cnt >= 200U)
        {
            gz_bias = yaw_calib_accum / 200.0f;
            yaw_calibrated = 1;
        }
        return;      //标定期间yaw保持0
    }

    //去零偏后积分,连续不wrap(可超过+-pi),供云台解耦delta驱动使用
    yaw += (r - gz_bias) * 0.005f;
}

//BMI088进程函数,每5ms采样一次
void BMI088_Proc(void)
{
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - last_tick) < 5U)
    {
        return;
    }

    last_tick = now;
    bmi088_error = BMI088_Update();

    BMI088_YawSolve();   //更新yaw欧拉角
}

//读取BMI088原始值并转换为物理量
uint8_t BMI088_Update(void)
{
    uint8_t buf[6];
    int16_t raw;

    reg_read_buf(ACCEL, 0x12, 1, buf, 6);
    raw = (buf[1] << 8) | buf[0];
    ax = raw * 3.0f * 9.80665f / 32768.0f;
    raw = (buf[3] << 8) | buf[2];
    ay = raw * 3.0f * 9.80665f / 32768.0f;
    raw = (buf[5] << 8) | buf[4];
    az = raw * 3.0f * 9.80665f / 32768.0f;

    reg_read_buf(GYRO, 0x02, 0, buf, 6);
    raw = (buf[1] << 8) | buf[0];
    gx = raw * 2000.0f / 32768.0f * 3.14159265f / 180.0f;
    raw = (buf[3] << 8) | buf[2];
    gy = raw * 2000.0f / 32768.0f * 3.14159265f / 180.0f;
    raw = (buf[5] << 8) | buf[4];
    gz = raw * 2000.0f / 32768.0f * 3.14159265f / 180.0f;

    reg_read_buf(ACCEL, 0x22, 1, buf, 2);
    raw = (buf[0] << 3) | (buf[1] >> 5);
    if (raw > 1023)
        raw -= 2048;
    temperature = raw * 0.125f + 23.0f;

    return BMI088_OK;
}	

float BMI088_GetAx(void)
{
    return ax;
}

float BMI088_GetAy(void)
{
    return ay;
}

float BMI088_GetAz(void)
{
    return az;
}

float BMI088_GetGx(void)
{
    return gx;
}

float BMI088_GetGy(void)
{
    return gy;
}

float BMI088_GetGz(void)
{
    return gz;
}

//读取连续yaw角,单位rad(未wrap,正方向为车头右转)
float BMI088_GetYaw(void)
{
    return yaw;
}

//将当前朝向设为yaw零点
void BMI088_YawReset(void)
{
    yaw = 0.0f;
}

//返回1表示开机零偏标定已完成
uint8_t BMI088_YawIsReady(void)
{
    return yaw_calibrated;
}

float BMI088_GetTemperature(void)
{
    return temperature;
}

uint8_t BMI088_GetError(void)
{
    return bmi088_error;
}
