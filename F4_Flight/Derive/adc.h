#ifndef __ADC_H__
#define __ADC_H__

#include "system.h"
#include "systick.h"

void Battery_ADC_Init(void);
void ADC_Config(void);
u16 Get_ADC_Value(u8 channel);
u16 Get_ADC_Average(u8 channel, u8 times);

#endif	//__ADC_H__
