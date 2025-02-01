#ifndef __I2C_H__
#define __I2C_H__

#include "system.h"

#define I2C_SCL_PIN		GPIO_Pin_8								/*连接到SCL时钟线的GPIO*/
#define I2C_SDA_PIN		GPIO_Pin_9								/*连接到SDA数据线的GPIO*/

// SCL = 1
#define I2C_SCL_1  GPIO_SetBits(GPIOA, I2C_SCL_PIN)		
// SCL = 0	
#define I2C_SCL_0  GPIO_ResetBits(GPIOA, I2C_SCL_PIN)		
// SDA = 1
#define I2C_SDA_1  GPIO_SetBits(GPIOC, I2C_SDA_PIN)		
// SDA = 0
#define I2C_SDA_0  GPIO_ResetBits(GPIOC, I2C_SDA_PIN)		
// 读SDA口线状态 
#define I2C_SDA_READ  GPIO_ReadInputDataBit(GPIOC, I2C_SDA_PIN)	

void I2C_GPIO_Init(void);
void i2c_Start(void);
void i2c_Stop(void);
void i2c_Ack(void);
void i2c_NAck(void);
uint8_t i2c_WaitAck(void);
void i2c_SendByte(uint8_t byte);
uint8_t i2c_ReadByte(void);



#endif	// __I2C_H__

