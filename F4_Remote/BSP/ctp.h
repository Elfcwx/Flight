#ifndef __CTP_H__
#define __CTP_H__

#include "system.h"
#include "i2c.h"
#include "touch.h"
#include "systick.h"

//与电容触摸屏连接的芯片引脚(未包含I2C引脚) 
//IO操作函数	 
// FT6336复位引脚
#define FT_RST_H   				GPIO_SetBits(GPIOA, GPIO_Pin_9);	
#define FT_RST_L				GPIO_ResetBits(GPIOA, GPIO_Pin_9);
// FT6336中断引脚	
#define FT_INT_H    			GPIO_SetBits(GPIOA, GPIO_Pin_10);
#define FT_INT_L				GPIO_ResetBits(GPIOA, GPIO_Pin_10);


//I2C读写命令	
#define FT_CMD_WR 				0X70    	//写命令
#define FT_CMD_RD 				0X71		//读命令
  
//FT6336 部分寄存器定义 
#define FT_REG_NUM_FINGER       0x02		//触摸状态寄存器

#define FT_TP1_REG 				0X03	  //第一个触摸点数据地址
#define FT_TP2_REG 				0X09		//第二个触摸点数据地址
#define FT_TP3_REG 				0X0F		//第三个触摸点数据地址
#define FT_TP4_REG 				0X15		//第四个触摸点数据地址
#define FT_TP5_REG 				0X1B		//第五个触摸点数据地址  

void FT6336_Init(void);
u8 FT6336_Scan(u8 mode);

#endif	// __CTP_H__

