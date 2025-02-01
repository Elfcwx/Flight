#include "adc.h"

void Battery_ADC_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//使能 GPIOB 时钟

	// 初始化 ADC1 通道5 IO口
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;  // PB1 通道9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;  // 模式为模拟输入
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 不带上下拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
}

void ADC_Config(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);// 使能ADC1时钟
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);   // ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);    // 复位结束
	
	// 初始化 CCR 通用控制寄存器配置
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; // DMA失能
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;  // 独立模式
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;  // 预分频值设置	保证ADC1的时钟频率不超过36MHz
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;// 两个采样阶段之间延迟5个时钟
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	// 初始化 ADC1 相关参数
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  // 关闭连续转换
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  // 右对齐
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; // 禁止触发检测，使用软件触发
	ADC_InitStructure.ADC_NbrOfConversion = 1;  // 1个转换在规则序列中
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;  // 12位模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE; // 非扫描模式
	ADC_Init(ADC1,&ADC_InitStructure); // ADC初始化
	
	ADC_Cmd(ADC1,ENABLE);   // 开启AD转换器
}

//获取ADC值
//channel：通道值 0~16
//返回值：转换结果
u16 Get_ADC_Value(u8 channel)
{
	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_480Cycles);//设置ADC规则组通道，一个序列，采样时间480
	ADC_SoftwareStartConv(ADC1);//使能指定的ADC1的软件转换启动功能
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));//读取状态寄存器的状态位EOC，等待转换结束
	return ADC_GetConversionValue(ADC1);//返回最近一次的ADC1规则组的转换结果
}
//获取通道ch的转换值，取times次，然后平均
//channel：通道编号  times：获取次数
//返回值：通道ch的times次转换结果平均值
u16 Get_ADC_Average(u8 channel, u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val = temp_val + Get_ADC_Value(channel); //取times次通道值进行求和
		delay_ms(5);
	}
	return temp_val/times; //返回平均值
}
