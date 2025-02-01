#include "i2c.h"

void I2C_GPIO_Init(void)
{
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		
	GPIO_InitTypeDef GPIO_InitStructure;
	// SCL
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// SDA
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_I2C3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_I2C3);

}

void I2C_Config_Init(void)
{
	I2C_GPIO_Init();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);
	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;				// I2CӦ��ʹ��
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;		// 7λѰַģʽ
	I2C_InitStructure.I2C_ClockSpeed = 100000;		// I2Cʱ��Ƶ�� --- 100khz
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;			// SCL�ߵ�ʱ��ռ�ձ�
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;		// I2Cģʽ
	I2C_InitStructure.I2C_OwnAddress1 = 0x0A;				// I2C�����豸��ַ
	I2C_Init(I2C3, &I2C_InitStructure);
	
	I2C_Cmd(I2C3, ENABLE);
}

void I2C_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	u32 Timeout;
	Timeout = 10000;
	
	while(I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)
	{
		Timeout --;
		if (Timeout == 0)
		{
			break;
		}
	}
}

void I2C_Write(u8 devAddr_WR, u8 reg, u8 data)
{
	// ������ʼ�ź�
	I2C_GenerateSTART(I2C3, ENABLE);
	// ���EV5�¼�
	I2C_WaitEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT);
	
	// �����豸��ַ
	I2C_Send7bitAddress(I2C3, devAddr_WR, I2C_Direction_Transmitter);
	// ���EV6�¼�
	I2C_WaitEvent(I2C3, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	// ����Ҫд����ڴ浥Ԫ��ַ
	I2C_SendData(I2C3, reg);
	// ���EV8�¼�
	I2C_WaitEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	// ����Ҫд�������
	I2C_SendData(I2C3, data);
	// ���EV8_2�¼�
	I2C_WaitEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	// ����ֹͣ�ź�
	I2C_GenerateSTOP(I2C3, ENABLE);
}

void I2C_Read_Buffer(u8 devAddr_RD, u8 reg, u8 *pBuffer, u16 NumByteToRead)
{
	// ������ʼ�ź�
	I2C_GenerateSTART(I2C3, ENABLE);
	// ���EV5�¼�
	I2C_WaitEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT);
	
	// �����豸��ַ
	I2C_Send7bitAddress(I2C3, devAddr_RD, I2C_Direction_Transmitter);
	// ���EV6�¼�
	I2C_WaitEvent(I2C3, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	// ����Ҫ�������ڴ浥Ԫ��ַ
	I2C_SendData(I2C3, reg);
	// ���EV8�¼�
	I2C_WaitEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	// �ڶ�����ʼ�ź�
	I2C_GenerateSTART(I2C3, ENABLE);
	// ���EV5�¼�
	I2C_WaitEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT);
	
	// �����豸��ַ
	I2C_Send7bitAddress(I2C3, devAddr_RD, I2C_Direction_Receiver);
	// ���EV6�¼�
	I2C_WaitEvent(I2C3, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
	
	while(NumByteToRead)
	{
		if(NumByteToRead == 1){
			I2C_AcknowledgeConfig(I2C3, DISABLE);
			I2C_GenerateSTOP(I2C3, ENABLE);
		}
		// ���EV7�¼�
		I2C_WaitEvent(I2C3, I2C_EVENT_MASTER_BYTE_RECEIVED);
		*pBuffer = I2C_ReceiveData(I2C3);
		pBuffer++;
		NumByteToRead--;
	}
	
	I2C_AcknowledgeConfig(I2C3, ENABLE);
}


