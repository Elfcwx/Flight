#include "nrf24l01.h"

const u8 TX_ADDRESS[TX_ADR_WIDTH]={0x1F,0x2E,0x3D,0x4C,0x5B}; 	
const u8 RX_ADDRESS[RX_ADR_WIDTH]={0x1F,0x2E,0x3D,0x4C,0x5B};

void NRF24L01_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	// CSN
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// CE
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	// IRQ
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA, GPIO_Pin_4);

    SPI_Config_Init();
	NRF_CE_L; 			
	SPI_CSN_H;
}

u8 NRF24L01_Check(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
    u8 i;
    SPI1_SetSpeed(SPI_BaudRatePrescaler_8); //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）        
    SPI_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.        
    SPI_Read_Buf(TX_ADDR,buf,5); //读出写入的地址  
    for(i=0;i<5;i++){
		if(buf[i]!=0XA5){
			break;
		}
	}
    if(i!=5){
		return 1;//检测24L01错误    
    }
	return 0;         //检测到24L01
}	
//向寄存器写入值
u8 SPI_Write_Reg(u8 reg,u8 value)
{
	u8 status;	
   	SPI_CSN_L;        
  	status = SPI1_SwapByte(reg);
  	SPI1_SwapByte(value);  
  	SPI_CSN_H;   
    
  	return(status);       			
}

//读取寄存器值
u8 SPI_Read_Reg(u8 reg)
{
	u8 reg_val;	    
 	SPI_CSN_L;  	
  	SPI1_SwapByte(reg);   
  	reg_val = SPI1_SwapByte(0XFF);	// 0xff 空操作
  	SPI_CSN_H;   
    
  	return(reg_val);        
}	

//读出寄存器中连续len个字节长度的值
u8 SPI_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	u8 status,i;	    
  	SPI_CSN_L;         	
  	status = SPI1_SwapByte(reg);	   
 	for(i=0;i<len;i++){
		pBuf[i]=SPI1_SwapByte(0XFF);
	}
        
  	SPI_CSN_H;
    
  	return status;        
}

//向寄存器写入连续len个字节的值
u8 SPI_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,i;
    
 	SPI_CSN_L;
  	status = SPI1_SwapByte(reg);
  	for(i=0; i<len; i++){
		SPI1_SwapByte(*pBuf++);
	}
  	SPI_CSN_H;   
    
  	return status;         
}

u8 NRF24L01_TxPacket(u8 *txbuf)
{
	u8 sta;
	SPI1_SetSpeed(SPI_BaudRatePrescaler_8);
	NRF_CE_L;   
	SPI_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);
 	NRF_CE_H;		//启动发送
	while(NRF_IRQ != 0);		//等待发送完成
	sta=SPI_Read_Reg(STATUS);		//读取状态寄存器的值	   
	SPI_Write_Reg(NRF_WRITE_REG+STATUS,sta);	//清除TX_DS或MAX_RT中断标志
	if(sta&MAX_TX){		//达到最大重发次数	
		SPI_Write_Reg(FLUSH_TX,0xff);		//清除TX FIFO寄存器 		
		return FLUSH_TX;
	}
	if(sta&TX_OK){		//发送完成
		return TX_OK;
	}
	return 0xff;		//其他原因发送失败
}

u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;
	SPI1_SetSpeed(SPI_BaudRatePrescaler_8);
	sta = SPI_Read_Reg(STATUS); 	 					    //状态标志位
	SPI_Write_Reg(NRF_WRITE_REG+STATUS,sta); 		//清除中断标志
	if(sta&RX_OK)											//接收成功
	{
		SPI_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);		
		SPI_Write_Reg(FLUSH_RX,0xff);
		return 0; 
	}	   
	return 1;
}

void NRF24L01_RxMode(void)
{
	NRF_CE_L;
	
    SPI_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
      
    SPI_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //使能通道0的自动应答    
    SPI_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址       
    SPI_Write_Reg(NRF_WRITE_REG+RF_CH,40);         //设置RF通信频率  
	// 射频RF计算公式：2400+RF_CH(Mhz) 给的40，也就是2440Mhz	
    SPI_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度         
    SPI_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);//设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
	SPI_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式   
	NRF_CE_H; //CE为高,进入接收模式 
}

void NRF24L01_TxMode(void)
{
	NRF_CE_L;
	SPI_Write_Buf(NRF_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
    SPI_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK      
 
    SPI_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    
    SPI_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  
    SPI_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
    SPI_Write_Reg(NRF_WRITE_REG+RF_CH,40);       //设置RF通道为40
    SPI_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
    SPI_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断    
	NRF_CE_H;  //CE为高,10us后启动发送
}

