#include "usart6.h"

#define USART6_RX_BUFFER_SIZE	128
u8 USART6RxBuffer[USART6_RX_BUFFER_SIZE];
RingBuffer USART6_RingBuffer;

void USART6_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART6时钟	
	
	//串口6对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); 
	
  	//USART6端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
  
	//Usart6 NVIC 配置
  	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

   	//USART6 初始化设置
	USART_InitStructure.USART_BaudRate = 115200;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  	USART_Init(USART6, &USART_InitStructure); //初始化串口6

  	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//开启串口接受中断	
	USART_Cmd(USART6, ENABLE);                    //使能串口6 	

	RingBuffer_Init(&USART6_RingBuffer, USART6RxBuffer, USART6_RX_BUFFER_SIZE);
}


uint16_t USART6_GetData(uint8_t *buf, uint16_t len)
{
    return ringbuffer_out(&USART6_RingBuffer, buf, len);
}


void USART6_IRQHandler(void)
{
	if (USART_GetITStatus(USART6, USART_IT_RXNE) == SET)
	{
		uint8_t res = USART_ReceiveData(USART6);
		ringbuffer_in_check(&USART6_RingBuffer, (uint8_t *)&res, 1); /*!< 将接收到的数据放入FIFO */
	}
}

