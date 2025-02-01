#include "touch.h"

_m_tp_dev tp_dev=
{
	0,
	0, 
	0,
	0,
};					

//��������ʼ��  		    
//����ֵ:0,û�н���У׼
//       1,���й�У׼
void TP_Init(void)
{	
	FT6336_Init();			
	LCD_Clear(WHITE);//���� 									 
}

const u16 POINT_COLOR_TBL[CT_MAX_TOUCH]={RED,GREEN,BLUE,BROWN,GRED}; 

//���ݴ��������Ժ���
void ctp_test(void)
{
	u8 t=0; 	    
 	u16 lastpos[5][2];		//���һ�ε����� 
	while(1)
	{
		FT6336_Scan(0);
		for(t=0;t<CT_MAX_TOUCH;t++)//���5�㴥��
		{
			if((tp_dev.sta)&(1<<t))//�ж��Ƿ��е㴥����
			{
				if(tp_dev.x[t]<lcddev.width&&tp_dev.y[t]<lcddev.height)//��LCD��Χ��
				{
					if(lastpos[t][0]==0XFFFF)
					{
						lastpos[t][0] = tp_dev.x[t];
						lastpos[t][1] = tp_dev.y[t];
					}
					lcd_draw_bline(lastpos[t][0],lastpos[t][1],tp_dev.x[t],tp_dev.y[t],2,POINT_COLOR_TBL[t]);//����
					lastpos[t][0]=tp_dev.x[t];
					lastpos[t][1]=tp_dev.y[t];
					if(tp_dev.x[t]>(lcddev.width-24)&&tp_dev.y[t]<16)
					{
						Load_Drow_Dialog();//���
					}								
				}
			}else lastpos[t][0]=0XFFFF;
		}
	}	
}

