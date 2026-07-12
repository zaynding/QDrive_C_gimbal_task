#ifndef __BMI088_H
#define __BMI088_H


#include "main.h"

#define BMI088_OK                 0
#define BMI088_ACCEL_ID_ERROR     0x01
#define BMI088_GYRO_ID_ERROR      0x02
#define BMI088_ACCEL_CONFIG_ERROR 0x03
#define BMI088_GYRO_CONFIG_ERROR  0x04

uint8_t BMI088_Init(void);
uint8_t BMI088_DMA_IRQHandler(void);

float BMI088_GetAx(void);
float BMI088_GetAy(void);
float BMI088_GetAz(void);

float BMI088_GetGx(void);
float BMI088_GetGy(void);
float BMI088_GetGz(void);

float BMI088_GetYaw(void);        //连续yaw欧拉角,单位rad
float BMI088_GetPitch(void);      //绝对pitch欧拉角(加速度计+陀螺仪融合),单位rad
void  BMI088_YawReset(void);      //当前朝向设为yaw零点
uint8_t BMI088_YawIsReady(void);  //1:开机零偏标定完成

float BMI088_GetTemperature(void);
uint8_t BMI088_GetError(void);
uint8_t BMI088_GetDMAPhase(void);
uint32_t BMI088_GetSPIState(void);
uint32_t BMI088_GetSPIError(void);
uint8_t BMI088_GetGyroDRDYLevel(void);


#endif
