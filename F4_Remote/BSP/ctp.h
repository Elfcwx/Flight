#ifndef __CTP_H__
#define __CTP_H__

#include "system.h"
#include "i2c.h"
#include "touch.h"
#include "systick.h"

//����ݴ��������ӵ�оƬ����(δ����I2C����) 
//IO��������	 
// FT6336��λ����
#define FT_RST_H   				GPIO_SetBits(GPIOA, GPIO_Pin_9);	
#define FT_RST_L				GPIO_ResetBits(GPIOA, GPIO_Pin_9);
// FT6336�ж�����	
#define FT_INT_H    			GPIO_SetBits(GPIOA, GPIO_Pin_10);
#define FT_INT_L				GPIO_ResetBits(GPIOA, GPIO_Pin_10);


//I2C��д����	
#define FT_CMD_WR 				0X70    	//д����
#define FT_CMD_RD 				0X71		//������
  
//FT6336 ���ּĴ������� 
#define FT_REG_NUM_FINGER       0x02		//����״̬�Ĵ���

#define FT_TP1_REG 				0X03	  //��һ�����������ݵ�ַ
#define FT_TP2_REG 				0X09		//�ڶ������������ݵ�ַ
#define FT_TP3_REG 				0X0F		//���������������ݵ�ַ
#define FT_TP4_REG 				0X15		//���ĸ����������ݵ�ַ
#define FT_TP5_REG 				0X1B		//��������������ݵ�ַ  

void FT6336_Init(void);
u8 FT6336_Scan(u8 mode);

#endif	// __CTP_H__

