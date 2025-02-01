#include "systick.h"
#include "FreeRTOS.h"
#include "task.h"


static uint32_t g_fac_us = 0;       /* us延时倍乘数 */
extern void xPortSysTickHandler(void);

/**
 * @brief     systick中断服务函数,使用OS时用到
 * @param     ticks : 延时的节拍数  
 * @retval    无
 */  
void SysTick_Handler(void)
{

    /* OS 开始跑了,才执行正常的调度处理 */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        xPortSysTickHandler();
    }
}

/**
 * @brief     初始化延迟函数
 * @param     sysclk: 系统时钟频率, 即CPU频率(rcc_c_ck), 180MHz
 * @retval    无
 */  

void SysTick_Init(uint16_t SYSCLK)
{
	/* 如果需要支持OS */
    uint32_t reload;

    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);	/* SYSTICK使用外部时钟源,频率为HCLK */
    g_fac_us = SYSCLK;                                  /* 不论是否使用OS,g_fac_us都需要使用 */
  /* 如果需要支持OS. */
    reload = SYSCLK;                                    /* 每秒钟的计数次数 单位为M */
    reload *= 1000000 / configTICK_RATE_HZ;            	/* 根据delay_ostickspersec设定溢出时间,reload为24位
                                                         * 寄存器,最大值:16777216,在180M下,约合0.0932s左右  */
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;          /* 开启SYSTICK中断 */
    SysTick->LOAD = reload;                             /* 每1/delay_ostickspersec秒中断一次 */
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;           /* 开启SYSTICK */
}

void delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;        /* LOAD的值 */
    ticks = nus * g_fac_us;                 /* 需要的节拍数 */
    told = SysTick->VAL;                    /* 刚进入时的计数器值 */
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;        /* 这里注意一下SYSTICK是一个递减的计数器就可以了 */
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks) 
            {
                break;                      /* 时间超过/等于要延迟的时间,则退出 */
            }
        }
    }
        
} 

/**
 * @brief     延时nms
 * @param     nms: 要延时的ms数 (0< nms <= 65535) 
 * @retval    无
 */
void delay_ms(uint16_t nms)
{
    uint32_t i;
    for (i=0; i<nms; i++)
    {
        delay_us(1000);
    }
}

