/*
 * si2c.h
 *
 *  Created on: May 6, 2025
 *      Author: gaoxi
 */

#ifndef INC_SI2C_H_
#define INC_SI2C_H_


#include "main.h"

typedef struct
{
	GPIO_TypeDef *SCL_GPIOx; // SCL引脚的组编号
	uint32_t SCL_GPIO_Pin;   // SCL引脚的引脚编号

	GPIO_TypeDef *SDA_GPIOx; // SCL引脚的组编号
	uint32_t SDA_GPIO_Pin;   // SDA引脚的引脚编号

} SI2C_TypeDef;

void My_SI2C_Init(SI2C_TypeDef *SI2C);
int My_SI2C_SendBytes(SI2C_TypeDef *SI2C, uint8_t Addr, const uint8_t *pData, uint16_t Size);
int My_SI2C_ReceiveBytes(SI2C_TypeDef *SI2C, uint8_t Addr, uint8_t *pBuffer, uint16_t Size);
int My_SI2C_RegReadBytes(SI2C_TypeDef *SI2C, uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Size);
int My_SI2C_RegWriteBytes(SI2C_TypeDef *SI2C, uint8_t Addr, uint8_t Reg, const uint8_t *pData, uint16_t Size);


#endif /* INC_SI2C_H_ */
