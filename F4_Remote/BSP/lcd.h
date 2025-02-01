#ifndef __LCD_H__
#define __LCD_H__

#include "system.h"	 
#include "spi.h"
#include "pwm.h"
#include "systick.h"
#include "string.h"
#include "stdlib.h"
#include "fonts.h" 
	 
/////////////////////////////////////�û�������///////////////////////////////////	 
//֧�ֺ��������ٶ����л�
#define USE_LCM_DIR  	  0   	//����Һ����˳ʱ����ת���� 	0-0����ת��1-180����ת��2-270����ת��3-90����ת
//#define USE_TP_TYPE   	CTP 	//���崥������  CTP = ����ģʽ , RTP = ����ģʽ

//#define CTP 0X80		//����ģʽ

	 
//LCD��Ҫ������
typedef struct  
{										    
	u16 width;			//LCD ���
	u16 height;			//LCD �߶�
	u16 id;				//LCD ID
	u8  dir;			//���������������ƣ�0��������1��������	
	u8	wramcmd;		//��ʼдgramָ��
	u8 setxcmd;			//����x����ָ��
	u8  setycmd;		//����y����ָ��	 
}_lcd_dev; 	  

//LCD����
extern _lcd_dev lcddev;	//����LCD��Ҫ����
//LCD�Ļ�����ɫ�ͱ���ɫ	   
extern u16  POINT_COLOR;//Ĭ�Ϻ�ɫ    
extern u16  BACK_COLOR; //������ɫ.Ĭ��Ϊ��ɫ

////////////////////////////////////////////////////////////////////
//-----------------LCD�˿ڶ���----------------// 

// Ƭѡ�˿�
#define SPI_CS_H		GPIO_SetBits(GPIOB, GPIO_Pin_12);	
#define SPI_CS_L		GPIO_ResetBits(GPIOB, GPIO_Pin_12);
// ��λ															  	
#define SPI_RST_H 		GPIO_SetBits(GPIOC, GPIO_Pin_7);
#define SPI_RST_L 		GPIO_ResetBits(GPIOC, GPIO_Pin_7);
// ����/����
#define SPI_DC_H		GPIO_SetBits(GPIOC, GPIO_Pin_6);
#define SPI_DC_L		GPIO_ResetBits(GPIOC, GPIO_Pin_6);
// ���� ���� 
#define	LCD_LED_H		GPIO_SetBits(GPIOA, GPIO_Pin_11);
#define	LCD_LED_L		GPIO_ResetBits(GPIOA, GPIO_Pin_11);

	
	 

//������ɫ
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	 0x001F  
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 //��ɫ
#define BRRED 			 0XFC07 //�غ�ɫ
#define GRAY  			 0X8430 //��ɫ
//GUI��ɫ

#define DARKBLUE      	 0X01CF	//����ɫ
#define LIGHTBLUE      	 0X7D7C	//ǳ��ɫ  
#define GRAYBLUE       	 0X5458 //����ɫ
//������ɫΪPANEL����ɫ 
 
#define LIGHTGREEN     	 0X841F //ǳ��ɫ 
#define LGRAY 					 0XC618 //ǳ��ɫ(PANNEL),���屳��ɫ

#define LGRAYBLUE        0XA651 //ǳ����ɫ(�м����ɫ)
#define LBBLUE           0X2B12 //ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)
	    															  
	    															  
void LCD_Init(void);													   	//��ʼ��
void LCD_DisplayOn(void);													//����ʾ
void LCD_DisplayOff(void);													//����ʾ
void LCD_Clear(u16 Color);	 												//����
void LCD_SetCursor(u16 Xpos, u16 Ypos);										//���ù��
void LCD_DrawPoint(u16 x,u16 y);											//����
void LCD_Fast_DrawPoint(u16 x,u16 y,u16 color);								//���ٻ���
void LCD_Draw_Circle(u16 x0,u16 y0,u8 r);						 			//��Բ
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2);							//����
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2);		   				//������
void LCD_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color);		   				//��䵥ɫ
void LCD_Color_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 *color);				//���ָ����ɫ
void LCD_ShowChar(u16 x,u16 y,u8 num,u8 size,u8 mode);						//��ʾһ���ַ�
void LCD_ShowNum(u16 x,u16 y,u32 num,u8 len,u8 size);  						//��ʾһ������
void LCD_ShowxNum(u16 x,u16 y,u32 num,u8 len,u8 size,u8 mode);				//��ʾ ����
void LCD_ShowString(u16 x,u16 y,u16 width,u16 height,u8 size,u8 *p);		//��ʾһ���ַ���,12/16����

void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue);
void LCD_WriteRAM_Prepare(void);
void Load_Drow_Dialog(void);
void LCD_Set_Window(u16 sx,u16 sy,u16 width,u16 height);	//���ô���				
void Show_Str(u16 x, u16 y,u8 *str,u8 size,u8 mode);//��ʾ����
void Gui_Drawbmp16(u16 x,u16 y,const unsigned char *p); //��ʾ40*40 ͼƬ
void Gui_StrCenter(u16 x, u16 y,u8 *str,u8 size,u8 mode);//������ʾ
void LCD_Display_Dir(u8 dir); //����LCD��ʾ����
void lcd_draw_bline(u16 x1, u16 y1, u16 x2, u16 y2,u8 size,u16 color);//��һ������
void gui_fill_circle(u16 x0,u16 y0,u16 r,u16 color);//��ʵ��Բ
void gui_draw_hline(u16 x0,u16 y0,u16 len,u16 color);//��ˮƽ�� ���ݴ�����ר�в���
u16 LCD_ReadPoint(u16 x,u16 y);//��ȡָ��������ɫ



#endif	// __LCD_H__

