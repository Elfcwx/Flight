#include "pwm.h"

void PWM_Init(void)
{	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_TIM1);
	
	TIM_InternalClockConfig(TIM1);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 400;		//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 100 - 1;	//PSC			pwmÖÜÆÚÎª 2500hz--- 0.4msÂö¿í
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStacture;
	TIM_OCInitStacture.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStacture.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStacture.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStacture.TIM_Pulse = 0;		// CCR 
	TIM_OC1Init(TIM1, &TIM_OCInitStacture);
	TIM_OC2Init(TIM1, &TIM_OCInitStacture);
	TIM_OC3Init(TIM1, &TIM_OCInitStacture);
	TIM_OC4Init(TIM1, &TIM_OCInitStacture);
	TIM_OCStructInit(&TIM_OCInitStacture);
	TIM_Cmd(TIM1, ENABLE);
}

void SET_BLK_PWM(u16 value)
{
	TIM1->CCR4 = value;
}

