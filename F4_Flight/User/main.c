#include "system.h"
#include "systick.h"
#include "usart1.h"
#include "led.h"
#include "pwm.h"
#include "mpu6050.h"
#include "posture.h"
#include "nrf24l01.h"
#include "usart6.h"
#include "adc.h"
#include "task_manager.h"

int main()
{
	SysTick_Init(100);
	delay_ms(1000);
	USART1_Init(115200);
	USART6_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

	Led_Init();
	
	while(MPU6050_Init());
	
	NRF24L01_Init();
	while(NRF24L01_Check()); 
	NRF24L01_RxMode();
	PWM_Init();
//	Battery_ADC_Init();
//	ADC_Config();
	delay_ms(10);
	
	MPU6050_GyroOffset();
	//Kalman_Init();
	Led_Red_Twinkle(5);
	MPU6050_AccOffset();
	Flight_task();
}


