#include "spi.h"

void NRF_GPIO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);// 使能 GPIOA 时钟 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;// 复用功能 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;// 推挽输出 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;// 100MHz 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;// 上拉 
	GPIO_Init(GPIOA, &GPIO_InitStructure);// 初始化 
	// 配置引脚复用映射 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1); // PB5复用为 SPI1 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1); // PB6复用为 SPI1 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1); // PB7复用为 SPI1
}

void LCD_GPIO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);// 使能 GPIOB 时钟 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;// 复用功能 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;// 推挽输出 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;// 100MHz 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;// 上拉 
	GPIO_Init(GPIOB, &GPIO_InitStructure);// 初始化 
	// 配置引脚复用映射 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2); // PB13复用为 SPI2
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2); // PB14复用为 SPI2 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2); // PB15复用为 SPI2
}

void SPI1_Config_Init(void)
{
	NRF_GPIO_Init();
	SPI_InitTypeDef  SPI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);// 使能 SPI1 时钟 
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  // 设置SPI全双工 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;  // 设置SPI工作模式:主SPI 
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; // 设置SPI的数据大小: 8位帧结构 
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;// 串行同步时钟的空闲状态为高电平 
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; // 数据捕获于第一个时钟沿 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;   // NSS信号由软件管理 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; // 预分频256 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; // 数据传输从MSB位开始 
	SPI_InitStructure.SPI_CRCPolynomial = 7; // CRC值计算的多项式 
	SPI_Init(SPI1, &SPI_InitStructure); // 根据指定的参数初始化外设SPIx寄存器 
	  
	SPI_Cmd(SPI1, ENABLE); //使能SPI1  
}

void SPI2_Config_Init(void)
{
	LCD_GPIO_Init();
	SPI_InitTypeDef  SPI_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);// 使能 SPI1 时钟 
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  // 设置SPI全双工 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;  // 设置SPI工作模式:主SPI 
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; // 设置SPI的数据大小: 8位帧结构 
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;// 串行同步时钟的空闲状态为高电平 
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; // 数据捕获于第一个时钟沿 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;   // NSS信号由软件管理 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; // 预分频256 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; // 数据传输从MSB位开始 
	SPI_InitStructure.SPI_CRCPolynomial = 7; // CRC值计算的多项式 
	SPI_Init(SPI2, &SPI_InitStructure); // 根据指定的参数初始化外设SPIx寄存器 
	  
	SPI_Cmd(SPI2, ENABLE); //使能SPI1  
}


// 返回值:读取到的字节 
u8 SPI1_SwapByte(u8 TxData) 
{      
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空   
	SPI_I2S_SendData(SPI1, TxData); //通过外设SPIx发送一个byte  数据 
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} //等待接收完 
	return SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据 
} 

// 设置SPI1速度
//---------------------------------------------------
// SPI_DivideFrequency_2    2分频： 50MHz
// SPI_DivideFrequency_4    4分频： 25MHz
// SPI_DivideFrequency_8    8分频： 12.5MHz
// SPI_DivideFrequency_16    16分频：6.25MHz
// SPI_DivideFrequency_32    32分频：3.125MHz

void SPI1_SetSpeed(u8 SPI_DivideFrequency)
{
    SPI1->CR1&=0XFFC7;
    SPI1->CR1|=SPI_DivideFrequency;    //设置SPI2速度  
}

u8 SPI2_ReadByte(void)
{
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET){} //等待接收完 
	return SPI_I2S_ReceiveData(SPI2); //返回通过SPIx最近接收的数据
}

void SPI2_WriteByte(u8 TxData) 
{      
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空   
	SPI_I2S_SendData(SPI2, TxData); //通过外设SPIx发送一个byte  数据 
	 
} 

