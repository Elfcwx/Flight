#include "touch.h"

_m_tp_dev tp_dev=
{
	0,
	0, 
	0,
	0,
};					

//触摸屏初始化  		    
//返回值:0,没有进行校准
//       1,进行过校准
void TP_Init(void)
{	
	FT6336_Init();			
	LCD_Clear(WHITE);//清屏 									 
}

const u16 POINT_COLOR_TBL[CT_MAX_TOUCH]={RED,GREEN,BLUE,BROWN,GRED}; 

//电容触摸屏测试函数
void ctp_test(void)
{
	u8 t=0; 	    
 	u16 lastpos[5][2];		//最后一次的数据 
	while(1)
	{
		FT6336_Scan(0);
		for(t=0;t<CT_MAX_TOUCH;t++)//最多5点触摸
		{
			if((tp_dev.sta)&(1<<t))//判断是否有点触摸？
			{
				if(tp_dev.x[t]<lcddev.width&&tp_dev.y[t]<lcddev.height)//在LCD范围内
				{
					if(lastpos[t][0]==0XFFFF)
					{
						lastpos[t][0] = tp_dev.x[t];
						lastpos[t][1] = tp_dev.y[t];
					}
					lcd_draw_bline(lastpos[t][0],lastpos[t][1],tp_dev.x[t],tp_dev.y[t],2,POINT_COLOR_TBL[t]);//画线
					lastpos[t][0]=tp_dev.x[t];
					lastpos[t][1]=tp_dev.y[t];
					if(tp_dev.x[t]>(lcddev.width-24)&&tp_dev.y[t]<16)
					{
						Load_Drow_Dialog();//清除
					}								
				}
			}else lastpos[t][0]=0XFFFF;
		}
	}	
}

