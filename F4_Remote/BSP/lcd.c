#include "lcd.h"

//LCD�Ļ�����ɫ�ͱ���ɫ	   
u16 POINT_COLOR=0x0000;	//������ɫ
u16 BACK_COLOR=0xFFFF;  //����ɫ 

//����LCD��Ҫ����
//Ĭ��Ϊ����
_lcd_dev lcddev;
	 					    
//д�Ĵ�������
//data:�Ĵ���ֵ
void LCD_WR_REG(u8 reg)
{
	SPI_CS_L;		
	SPI_DC_L;//RS=0 ����	
	
	SPI2_WriteByte(reg);

	SPI_CS_H;				
}
//д���ݺ���
//data:�Ĵ���ֵ
void LCD_WR_DATA_8Bit(u8 data)
{
	SPI_CS_L;		
	SPI_DC_H;//RS=1 ����
	
	SPI2_WriteByte(data);
	SPI_CS_H;			
}

void LCD_WR_DATA_16Bit(u16 data)
{
	SPI_CS_L;		
	SPI_DC_H;//RS=1 ����
	
	SPI2_WriteByte((data>>8)&0xff);
	SPI2_WriteByte(data&0xff);
	SPI_CS_H;
}

//д�Ĵ���
//LCD_Reg:�Ĵ������
//LCD_RegValue:Ҫд���ֵ
void LCD_WriteReg(u8 LCD_Reg,u16 LCD_RegValue)
{	
	LCD_WR_REG(LCD_Reg);  
	LCD_WR_DATA_16Bit(LCD_RegValue);	    		 
}   

//��ʼдGRAM
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.wramcmd);
} 
//LCDдGRAM
//RGB_Code:��ɫֵ
//void LCD_WriteRAM(u16 RGB_Code)
//{							    
//	LCD_WR_DATA_16Bit(RGB_Code);
//}

//��mdk -O1ʱ���Ż�ʱ��Ҫ����
//��ʱi
void opt_delay(u8 i)
{
	while(i--);
}

//LCD������ʾ
void LCD_DisplayOn(void)
{					   
	LCD_WR_REG(0X29);	//������ʾ
}	 
//LCD�ر���ʾ
void LCD_DisplayOff(void)
{	   
	LCD_WR_REG(0X28);	//�ر���ʾ
}   
//���ù��λ��
//Xpos:������
//Ypos:������
void LCD_SetCursor(u16 Xpos, u16 Ypos)
{	 	    
		LCD_WR_REG(lcddev.setxcmd); 
		LCD_WR_DATA_16Bit(Xpos); 			 
		LCD_WR_REG(lcddev.setycmd); 
		LCD_WR_DATA_16Bit(Ypos); 		 
} 		 
   
//����
//x,y:����
//POINT_COLOR:�˵����ɫ
void LCD_DrawPoint(u16 x,u16 y)
{
	LCD_SetCursor(x,y);		//���ù��λ�� 
	LCD_WriteRAM_Prepare();	//��ʼд��GRAM
	LCD_WR_DATA_16Bit(POINT_COLOR); 
}	 
//���ٻ���
//x,y:����
//color:��ɫ
void LCD_Fast_DrawPoint(u16 x,u16 y,u16 color)
{	   
		//���ù��λ��
		LCD_SetCursor(x,y); 	 
		//д����ɫ
		LCD_WriteReg(lcddev.wramcmd,color);
}


//dir:����ѡ�� 	0-0����ת��1-180����ת��2-270����ת��3-90����ת
void LCD_Display_Dir(u8 dir)
{
	if(dir==0||dir==1)			//����
	{
			lcddev.dir=0;	//����
			lcddev.width=240;
			lcddev.height=320;

			lcddev.wramcmd=0X2C;
			lcddev.setxcmd=0X2A;
			lcddev.setycmd=0X2B;
		
		if(dir==0)        //0-0����ת
		{
			LCD_WR_REG(0x36); 
			LCD_WR_DATA_8Bit((0<<3)|(0<<7)|(0<<6)|(0<<5));
		}else							//1-180����ת
		{
			LCD_WR_REG(0x36); 
			LCD_WR_DATA_8Bit((0<<3)|(1<<7)|(1<<6)|(0<<5));		
		}
		
	}else if(dir==2||dir==3)
	{
		
			lcddev.dir=1;	//����
			lcddev.width=320;
			lcddev.height=240;

			lcddev.wramcmd=0X2C;
			lcddev.setxcmd=0X2A;
			lcddev.setycmd=0X2B; 

				if(dir==2)				//2-270����ת
				{
					LCD_WR_REG(0x36); 
					LCD_WR_DATA_8Bit((0<<3)|(1<<7)|(0<<6)|(1<<5));

				}else							//3-90����ת
				{
					LCD_WR_REG(0x36); 
					LCD_WR_DATA_8Bit((0<<3)|(0<<7)|(1<<6)|(1<<5));
				}		
	}	


	//������ʾ����	
	LCD_WR_REG(lcddev.setxcmd); 
	LCD_WR_DATA_8Bit(0);
	LCD_WR_DATA_8Bit(0);
	LCD_WR_DATA_16Bit(lcddev.width-1);
	LCD_WR_REG(lcddev.setycmd); 
	LCD_WR_DATA_8Bit(0);
	LCD_WR_DATA_8Bit(0);
	LCD_WR_DATA_16Bit(lcddev.height-1);  
	
	
}	 
//���ô���,���Զ����û������굽�������Ͻ�(sx,sy).
//sx,sy:������ʼ����(���Ͻ�)
//width,height:���ڿ�Ⱥ͸߶�,�������0!!
//�����С:width*height. 
void LCD_Set_Window(u16 sx,u16 sy,u16 width,u16 height)
{    
	u16 twidth,theight;
	twidth=sx+width-1;
	theight=sy+height-1;

	LCD_WR_REG(lcddev.setxcmd); 
	LCD_WR_DATA_16Bit(sx);	 
	LCD_WR_DATA_16Bit(twidth);   
	LCD_WR_REG(lcddev.setycmd); 
	LCD_WR_DATA_16Bit(sy);
	LCD_WR_DATA_16Bit(theight);

}
//��ʼ��lcd
void LCD_Init(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	// CSN
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// DC RST
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	SPI2_Config_Init();
	PWM_Init();
	
	SPI_RST_H;
	delay_ms(1);
	SPI_RST_L;
	delay_ms(10);
	SPI_RST_H;
	delay_ms(120);  


//************* Start Initial Sequence **********//
delay_ms(120);                //delay_ms 120ms
LCD_WR_REG(0x11);     
delay_ms(120);                //delay_ms 120ms
LCD_WR_REG(0x36);     
LCD_WR_DATA_8Bit(0x00);   
LCD_WR_REG(0x3A);     
LCD_WR_DATA_8Bit(0x55);   
LCD_WR_REG(0xB2);     
LCD_WR_DATA_8Bit(0x0C);   
LCD_WR_DATA_8Bit(0x0C);   
LCD_WR_DATA_8Bit(0x00);   
LCD_WR_DATA_8Bit(0x33);   
LCD_WR_DATA_8Bit(0x33);   
LCD_WR_REG(0xB7);     
LCD_WR_DATA_8Bit(0x75);   
LCD_WR_REG(0xBB);     
LCD_WR_DATA_8Bit(0x15);   
LCD_WR_REG(0xC0);     
LCD_WR_DATA_8Bit(0x2C);   
LCD_WR_REG(0xC2);     
LCD_WR_DATA_8Bit(0x01);   
LCD_WR_REG(0xC3);     
LCD_WR_DATA_8Bit(0x13);    
LCD_WR_REG(0xC4);     
LCD_WR_DATA_8Bit(0x23);   
LCD_WR_REG(0xC6);     
LCD_WR_DATA_8Bit(0x0F);   
LCD_WR_REG(0xD0);     
LCD_WR_DATA_8Bit(0xA4);   
LCD_WR_DATA_8Bit(0xA1);   
LCD_WR_REG(0xD6);     
LCD_WR_DATA_8Bit(0xA1);
LCD_WR_REG(0x21);     
LCD_WR_REG(0xE0);
LCD_WR_DATA_8Bit(0xD0);
LCD_WR_DATA_8Bit(0x08);
LCD_WR_DATA_8Bit(0x10);
LCD_WR_DATA_8Bit(0x0D);
LCD_WR_DATA_8Bit(0x0C);
LCD_WR_DATA_8Bit(0x07);
LCD_WR_DATA_8Bit(0x37);
LCD_WR_DATA_8Bit(0x53);
LCD_WR_DATA_8Bit(0x4C);
LCD_WR_DATA_8Bit(0x39);
LCD_WR_DATA_8Bit(0x15);
LCD_WR_DATA_8Bit(0x15);
LCD_WR_DATA_8Bit(0x2A);
LCD_WR_DATA_8Bit(0x2D);
LCD_WR_REG(0xE1);
LCD_WR_DATA_8Bit(0xD0);
LCD_WR_DATA_8Bit(0x0D);
LCD_WR_DATA_8Bit(0x12);
LCD_WR_DATA_8Bit(0x08);
LCD_WR_DATA_8Bit(0x08);
LCD_WR_DATA_8Bit(0x15);
LCD_WR_DATA_8Bit(0x34);
LCD_WR_DATA_8Bit(0x34);
LCD_WR_DATA_8Bit(0x4A);
LCD_WR_DATA_8Bit(0x36);
LCD_WR_DATA_8Bit(0x12);
LCD_WR_DATA_8Bit(0x13);
LCD_WR_DATA_8Bit(0x2B);
LCD_WR_DATA_8Bit(0x2F);   
LCD_WR_REG(0x29); // Display on

} 



//��ȡ��ĳ�����ɫֵ	 
//x,y:����
//����ֵ:�˵����ɫ
u16 LCD_ReadPoint(u16 x,u16 y)
{
	u8 r,g,b,reg=0x2e;
 	u16 color;
	if(x>=lcddev.width||y>=lcddev.height)return 0;	//�����˷�Χ,ֱ�ӷ���		   
	LCD_SetCursor(x,y);
	SPI_CS_L;		
	SPI_DC_L;
		
	SPI2_WriteByte(reg);

	//��һ�οն� �����ηֱ�ΪR G B
	SPI2_ReadByte();
	r = SPI2_ReadByte();
	g = SPI2_ReadByte();
	b = SPI2_ReadByte();
	
	color = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);		
	SPI_CS_H;		
	return color;
}		
  
//��������
//color:Ҫ���������ɫ
void LCD_Clear(u16 color)
{
	u32 index=0;      
	u32 totalpoint=lcddev.width;
	totalpoint*=lcddev.height; 			//�õ��ܵ���
	LCD_SetCursor(0x00,0x0000);	//���ù��λ�� 
	LCD_WriteRAM_Prepare();     		//��ʼд��GRAM	  	  
	for(index=0;index<totalpoint;index++)LCD_WR_DATA_16Bit(color);
}  
//��ָ�����������ָ����ɫ
//�����С:(xend-xsta+1)*(yend-ysta+1)
//xsta
//color:Ҫ������ɫ
void LCD_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color)
{          
	u16 i,j;
	u16 xlen=0;
	u16 temp;
	if((lcddev.id==0X6804)&&(lcddev.dir==1))	//6804������ʱ�����⴦��  
	{
		temp=sx;
		sx=sy;
		sy=lcddev.width-ex-1;	  
		ex=ey;
		ey=lcddev.width-temp-1;
 		lcddev.dir=0;	 
 		lcddev.setxcmd=0X2A;
		lcddev.setycmd=0X2B;  	 			
		LCD_Fill(sx,sy,ex,ey,color);  
 		lcddev.dir=1;	 
  		lcddev.setxcmd=0X2B;
		lcddev.setycmd=0X2A;  	 
 	}else
	{
		xlen=ex-sx+1;	 
		for(i=sy;i<=ey;i++)
		{
		 	LCD_SetCursor(sx,i);      				//���ù��λ�� 
			LCD_WriteRAM_Prepare();     			//��ʼд��GRAM	  
			for(j=0;j<xlen;j++)LCD_WR_DATA_16Bit(color);	//���ù��λ�� 	    
		}
	}
}  
//��ָ�����������ָ����ɫ��			 
//(sx,sy),(ex,ey):�����ζԽ�����,�����СΪ:(ex-sx+1)*(ey-sy+1)   
//color:Ҫ������ɫ
void LCD_Color_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 *color)
{  
	u16 height,width;
	u16 i,j;
	width=ex-sx+1; 			//�õ����Ŀ��
	height=ey-sy+1;			//�߶�
 	for(i=0;i<height;i++)
	{
 		LCD_SetCursor(sx,sy+i);   	//���ù��λ�� 
		LCD_WriteRAM_Prepare();     //��ʼд��GRAM
		for(j=0;j<width;j++)LCD_WR_DATA_16Bit(color[i*width+j]);//д������ 
	}  
}
//����
//x1,y1:�������
//x2,y2:�յ�����  
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 
	delta_x=x2-x1; //������������ 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; //���õ������� 
	else if(delta_x==0)incx=0;//��ֱ�� 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;//ˮƽ�� 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; //ѡȡ�������������� 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )//������� 
	{  
		LCD_DrawPoint(uRow,uCol);//���� 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
}    
//������	  
//(x1,y1),(x2,y2):���εĶԽ�����
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
	LCD_DrawLine(x1,y1,x2,y1);
	LCD_DrawLine(x1,y1,x1,y2);
	LCD_DrawLine(x1,y2,x2,y2);
	LCD_DrawLine(x2,y1,x2,y2);
}
//��ָ��λ�û�һ��ָ����С��Բ
//(x,y):���ĵ�
//r    :�뾶
void LCD_Draw_Circle(u16 x0,u16 y0,u8 r)
{
	int a,b;
	int di;
	a=0;b=r;	  
	di=3-(r<<1);             //�ж��¸���λ�õı�־
	while(a<=b)
	{
		LCD_DrawPoint(x0+a,y0-b);             //5
 		LCD_DrawPoint(x0+b,y0-a);             //0           
		LCD_DrawPoint(x0+b,y0+a);             //4               
		LCD_DrawPoint(x0+a,y0+b);             //6 
		LCD_DrawPoint(x0-a,y0+b);             //1       
 		LCD_DrawPoint(x0-b,y0+a);             
		LCD_DrawPoint(x0-a,y0-b);             //2             
  	LCD_DrawPoint(x0-b,y0-a);             //7     	         
		a++;
		//ʹ��Bresenham�㷨��Բ     
		if(di<0)di +=4*a+6;	  
		else
		{
			di+=10+4*(a-b);   
			b--;
		} 						    
	}
} 									  
//��ָ��λ����ʾһ���ַ�
//x,y:��ʼ����
//num:Ҫ��ʾ���ַ�:" "--->"~"
//size:�����С 12/16/24
//mode:���ӷ�ʽ(1)���Ƿǵ��ӷ�ʽ(0)
void LCD_ShowChar(u16 x,u16 y,u8 num,u8 size,u8 mode)
{  							  
    u8 temp,t1,t;
	u16 y0=y;
	u8 csize=(size/8+((size%8)?1:0))*(size/2);		//�õ�����һ���ַ���Ӧ������ռ���ֽ���	
 	num=num-' ';//�õ�ƫ�ƺ��ֵ��ASCII�ֿ��Ǵӿո�ʼȡģ������-' '���Ƕ�Ӧ�ַ����ֿ⣩
	for(t=0;t<csize;t++)
	{   
		if(size==12)temp=asc2_1206[num][t]; 	 	//����1206����
		else if(size==16)temp=asc2_1608[num][t];	//����1608����
		else if(size==24)temp=asc2_2412[num][t];	//����2412����
		else return;								//û�е��ֿ�
		for(t1=0;t1<8;t1++)
		{			    
			if(temp&0x80)LCD_Fast_DrawPoint(x,y,POINT_COLOR);
			else if(mode==0)LCD_Fast_DrawPoint(x,y,BACK_COLOR);
			temp<<=1;
			y++;
			if(y>=lcddev.height)return;		//��������
			if((y-y0)==size)
			{
				y=y0;
				x++;
				if(x>=lcddev.width)return;	//��������
				break;
			}
		}  	 
	}  	    	   	 	  
}   
//m^n����
//����ֵ:m^n�η�.
u32 LCD_Pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}			 
//��ʾ����,��λΪ0,����ʾ
//x,y :�������	 
//len :���ֵ�λ��
//size:�����С
//color:��ɫ 
//num:��ֵ(0~4294967295);	 
void LCD_ShowNum(u16 x,u16 y,u32 num,u8 len,u8 size)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/LCD_Pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+(size/2)*t,y,' ',size,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,0); 
	}
} 
//��ʾ����,��λΪ0,������ʾ
//x,y:�������
//num:��ֵ(0~999999999);	 
//len:����(��Ҫ��ʾ��λ��)
//size:�����С
//mode:
//[7]:0,�����;1,���0.
//[6:1]:����
//[0]:0,�ǵ�����ʾ;1,������ʾ.
void LCD_ShowxNum(u16 x,u16 y,u32 num,u8 len,u8 size,u8 mode)
{  
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/LCD_Pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				if(mode&0X80)LCD_ShowChar(x+(size/2)*t,y,'0',size,mode&0X01);  
				else LCD_ShowChar(x+(size/2)*t,y,' ',size,mode&0X01);  
 				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,mode&0X01); 
	}
} 
//��ʾ�ַ���
//x,y:�������
//width,height:�����С  
//size:�����С
//*p:�ַ�����ʼ��ַ		  
void LCD_ShowString(u16 x,u16 y,u16 width,u16 height,u8 size,u8 *p)
{
	u8 x0=x;
	width+=x;
	height+=y;
    while((*p<='~')&&(*p>=' '))//�ж��ǲ��ǷǷ��ַ�!
    {       
        if(x>=width){x=x0;y+=size;}
        if(y>=height)break;//�˳�
        LCD_ShowChar(x,y,*p,size,0);
        x+=size/2;
        p++;
    }
}


//���� 16*16
void GUI_DrawFont16(u16 x, u16 y,u8 *s,u8 mode)
{
	u8 i,j;
	u16 k;
	u16 HZnum;
	u16 x0=x;
	HZnum=get_tfont16_size();	//�Զ�ͳ�ƺ�����Ŀ
	
			
	for (k=0;k<HZnum;k++) 
	{
	  if ((tfont16[k].Index[0]==*(s))&&(tfont16[k].Index[1]==*(s+1)))
	  { 	LCD_Set_Window(x,y,16,16);
				LCD_WriteRAM_Prepare();
		    for(i=0;i<16*2;i++)
		    {
				for(j=0;j<8;j++)
		    	{	
					if(!mode) //�ǵ��ӷ�ʽ
					{
						if(tfont16[k].Msk[i]&(0x80>>j))	LCD_WR_DATA_16Bit(POINT_COLOR);
						else LCD_WR_DATA_16Bit(BACK_COLOR);
					}
					else
					{
						//POINT_COLOR=fc;
						if(tfont16[k].Msk[i]&(0x80>>j))	LCD_DrawPoint(x,y);//��һ����
						x++;
						if((x-x0)==16)
						{
							x=x0;
							y++;
							break;
						}
					}

				}
				
			}
			
			
		}				  	
		continue;  //���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
	}

	LCD_Set_Window(0,0,lcddev.width,lcddev.height);//�ָ�����Ϊȫ��  
} 

//���� 24*24
void GUI_DrawFont24(u16 x, u16 y, u8 *s,u8 mode)
{
	u8 i,j;
	u16 k;
	u16 HZnum;
	u16 x0=x;
	HZnum=get_tfont24_size();	//�Զ�ͳ�ƺ�����Ŀ
		
			for (k=0;k<HZnum;k++) 
			{
			  if ((tfont24[k].Index[0]==*(s))&&(tfont24[k].Index[1]==*(s+1)))
			  { 	LCD_Set_Window(x,y,24,24);
						LCD_WriteRAM_Prepare();
				    for(i=0;i<24*3;i++)
				    {
							for(j=0;j<8;j++)
							{
								if(!mode) //�ǵ��ӷ�ʽ
								{
									if(tfont24[k].Msk[i]&(0x80>>j))	LCD_WR_DATA_16Bit(POINT_COLOR);
									else LCD_WR_DATA_16Bit(BACK_COLOR);
								}
							else
							{
								//POINT_COLOR=fc;
								if(tfont24[k].Msk[i]&(0x80>>j))	LCD_DrawPoint(x,y);//��һ����
								x++;
								if((x-x0)==24)
								{
									x=x0;
									y++;
									break;
								}
							}
						}
					}
					
					
				}				  	
				continue;  //���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
			}

	LCD_Set_Window(0,0,lcddev.width,lcddev.height);//�ָ�����Ϊȫ��  
}

//���� 32*32
void GUI_DrawFont32(u16 x, u16 y, u8 *s,u8 mode)
{
	u8 i,j;
	u16 k;
	u16 HZnum;
	u16 x0=x;
	HZnum = get_tfont32_size();	//�Զ�ͳ�ƺ�����Ŀ
	for (k=0;k<HZnum;k++) 
			{
			  if ((tfont32[k].Index[0]==*(s))&&(tfont32[k].Index[1]==*(s+1)))
			  { 	LCD_Set_Window(x,y,32,32);
						LCD_WriteRAM_Prepare();
				    for(i=0;i<32*4;i++)
				    {
						for(j=0;j<8;j++)
				    	{
							if(!mode) //�ǵ��ӷ�ʽ
							{
								if(tfont32[k].Msk[i]&(0x80>>j))	LCD_WR_DATA_16Bit(POINT_COLOR);
								else LCD_WR_DATA_16Bit(BACK_COLOR);
							}
							else
							{
								//POINT_COLOR=fc;
								if(tfont32[k].Msk[i]&(0x80>>j))	LCD_DrawPoint(x,y);//��һ����
								x++;
								if((x-x0)==32)
								{
									x=x0;
									y++;
									break;
								}
							}
						}
					}
					
					
				}				  	
				continue;  //���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
			}
	
	LCD_Set_Window(0,0,lcddev.width,lcddev.height);//�ָ�����Ϊȫ��  
} 



//��ʾ���ֻ����ַ���
void Show_Str(u16 x, u16 y,u8 *str,u8 size,u8 mode)
{			
	u16 x0=x;							  	  
  	u8 bHz=0;     //�ַ��������� 
    while(*str!=0)//����δ����
    { 
        if(!bHz)
        {
			if(x>(lcddev.width-size/2)||y>(lcddev.height-size)) 
			return; 
	        if(*str>0x80)bHz=1;//���� 
	        else              //�ַ�
	        {          
		        if(*str==0x0D)//���з���
		        {         
		            y+=size;
		            x=x0;
		            str++; 
		        }  
		        else
				{
					if(size>=24)//�ֿ���û�м���12X24 16X32��Ӣ������,��8X16����
					{ 						
					LCD_ShowChar(x,y,*str,24,mode);
					x+=12; //�ַ�,Ϊȫ�ֵ�һ�� 
					}
					else
					{
					LCD_ShowChar(x,y,*str,size,mode);
					x+=size/2; //�ַ�,Ϊȫ�ֵ�һ�� 
					}
				} 
				str++; 
		        
	        }
        }else//���� 
        {   
				if(x>(lcddev.width-size)||y>(lcddev.height-size)) 
				return;  
							bHz=0;//�к��ֿ�    
				if(size==32)
				GUI_DrawFont32(x,y,str,mode);	 	
				else if(size==24)
				GUI_DrawFont24(x,y,str,mode);	
				else
				GUI_DrawFont16(x,y,str,mode);
					
						str+=2; 
						x+=size;//��һ������ƫ��	    
        }						 
    }   
}


//��ʾ40*40ͼƬ
void Gui_Drawbmp16(u16 x,u16 y,const unsigned char *p) //��ʾ40*40ͼƬ
{
  	int i; 
	unsigned char picH,picL; 
	LCD_Set_Window(x,y,40,40);
	LCD_WriteRAM_Prepare();	
	
    for(i=0;i<40*40;i++)
	{	
	 	picL=*(p+i*2);	//���ݵ�λ��ǰ
		picH=*(p+i*2+1);				
		LCD_WR_DATA_16Bit(picH<<8|picL);  						
	}	
	LCD_Set_Window(0,0,lcddev.width,lcddev.height);//�ָ���ʾ����Ϊȫ��	

}

//������ʾ
void Gui_StrCenter(u16 x, u16 y, u8 *str,u8 size,u8 mode)
{
	u16 x1;
	u16 len=strlen((const char *)str);
	if(size>16)
	{
		x1=(lcddev.width-len*(size/2))/2;
	}else
	{
		x1=(lcddev.width-len*8)/2;
	}
	
	Show_Str(x+x1,y,str,size,mode);
} 


void Load_Drow_Dialog(void)
{
	LCD_Clear(WHITE);//����   
 	POINT_COLOR=BLUE;//��������Ϊ��ɫ
	BACK_COLOR=WHITE;
	LCD_ShowString(lcddev.width-24,0,200,16,16,"RST");//��ʾ��������
  POINT_COLOR=RED;//���û�����ɫ 
}
////////////////////////////////////////////////////////////////////////////////
//���ݴ�����ר�в���
//��ˮƽ��
//x0,y0:����
//len:�߳���
//color:��ɫ
void gui_draw_hline(u16 x0,u16 y0,u16 len,u16 color)
{
	if(len==0)return;
	LCD_Fill(x0,y0,x0+len-1,y0,color);	
}
//��ʵ��Բ
//x0,y0:����
//r:�뾶
//color:��ɫ
void gui_fill_circle(u16 x0,u16 y0,u16 r,u16 color)
{											  
	u32 i;
	u32 imax = ((u32)r*707)/1000+1;
	u32 sqmax = (u32)r*(u32)r+(u32)r/2;
	u32 x=r;
	gui_draw_hline(x0-r,y0,2*r,color);
	for (i=1;i<=imax;i++) 
	{
		if ((i*i+x*x)>sqmax)// draw lines from outside  
		{
 			if (x>imax) 
			{
				gui_draw_hline (x0-i+1,y0+x,2*(i-1),color);
				gui_draw_hline (x0-i+1,y0-x,2*(i-1),color);
			}
			x--;
		}
		// draw lines from inside (center)  
		gui_draw_hline(x0-x,y0+i,2*x,color);
		gui_draw_hline(x0-x,y0-i,2*x,color);
	}
}  

//��һ������
//(x1,y1),(x2,y2):��������ʼ����
//size�������Ĵ�ϸ�̶�
//color����������ɫ
void lcd_draw_bline(u16 x1, u16 y1, u16 x2, u16 y2,u8 size,u16 color)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 
	if(x1<size|| x2<size||y1<size|| y2<size)return; 
	delta_x=x2-x1; //������������ 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; //���õ������� 
	else if(delta_x==0)incx=0;//��ֱ�� 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;//ˮƽ�� 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; //ѡȡ�������������� 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )//������� 
	{  
		gui_fill_circle(uRow,uCol,size,color);//���� 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
}

