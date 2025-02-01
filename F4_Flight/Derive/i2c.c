#include "i2c.h"

void I2C_GPIO_Init(void)
{
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		
	GPIO_InitTypeDef GPIO_InitStructure;
	// SCL
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// SDA
	GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	I2C_SCL_1;
	I2C_SDA_1;
}

/**
  * @brief SDA线输入模式配置
  * @param None
  * @retval None
  */
void SDA_Input_Mode(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
  * @brief SDA线输出模式配置
  * @param None
  * @retval None
  */
void SDA_Output_Mode(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

}

static void i2c_Delay(void)
{
	uint8_t i;

	for (i=0; i<20; i++);
}

void i2c_Start()
{
	SDA_Output_Mode();
	I2C_SDA_1;
	I2C_SCL_1;
	i2c_Delay();
	I2C_SDA_0;
	i2c_Delay();
	I2C_SCL_0;
	i2c_Delay();
}

void i2c_Stop()
{
	SDA_Output_Mode();
	I2C_SCL_0;
	I2C_SDA_0;
	i2c_Delay();
	I2C_SCL_1;
	I2C_SDA_1;
	i2c_Delay();
	
}

void i2c_Ack(void)
{
	SDA_Output_Mode();
	I2C_SCL_0;
	I2C_SDA_0;	/*CPU驱动SDA=0*/
	i2c_Delay();
	I2C_SCL_1;	/*CPU产生1个时钟*/
	i2c_Delay();
	I2C_SCL_0;
}

void i2c_NAck(void)
{
	SDA_Output_Mode();
	I2C_SCL_0;
	I2C_SDA_1;	/*CPU驱动SDA = 1*/
	i2c_Delay();
	I2C_SCL_1;	/*CPU产生1个时钟*/
	i2c_Delay();
	I2C_SCL_0;
}

uint8_t i2c_WaitAck(void)
{
	uint8_t wait = 0;
	
	I2C_SDA_1;	/*CPU释放SDA总线*/
	SDA_Input_Mode();
	i2c_Delay();
	I2C_SCL_1;	/*CPU驱动SCL=1,此时器件会返回ACK应答*/
	i2c_Delay();
	while(I2C_SDA_READ)	/*CPU读取SDA口线状态*/
	{
		wait++;
		if (wait > 250)
        {
            i2c_Stop();
			SDA_Output_Mode();
            return 0;
        }
	}
	I2C_SCL_0;
	SDA_Output_Mode();
	return 1;
}

void i2c_SendByte(uint8_t byte)
{
	uint8_t i;
	SDA_Output_Mode();
	/* 先发送字节的高位bit7 */
	for(i = 0; i < 8; i++)
	{
		I2C_SCL_0;
		if(byte & 0x80)
		{
			I2C_SDA_1;
		}else{
			I2C_SDA_0;
		}
		byte <<= 1;	/*左移一个bit*/
		i2c_Delay();
		I2C_SCL_1;
		i2c_Delay();	

	}
	I2C_SCL_0;	
	i2c_Delay();
}

uint8_t i2c_ReadByte(void)
{
	uint8_t i;
	uint8_t value;
	SDA_Input_Mode();
	/*读到第1个bit为数据的bit7*/
	value = 0;
	for (i = 0; i < 8; i++)
	{
		I2C_SCL_0;
		i2c_Delay();
		I2C_SCL_1;
		value <<= 1;
		if (I2C_SDA_READ)
		{
			value |= 0x01;
		}
		i2c_Delay();
	}
	I2C_SCL_0;
	SDA_Output_Mode();
	return value;
}
