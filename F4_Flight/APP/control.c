#include "control.h"
/* 
************************************************************************************************
		 y
									 M4				 M1				  +PITCH
		 |							   \			/		 	 		 |
		 |								\	  |	   /			 		 |
		 |									  |					 		 |
	-----|------ x						------|-------		  -ROLL ----------- +ROLL
		 |							    /	  |	   \			 		 |	
		 |							   /	  |		\					 |
		 |							 M3				 M2					 |
	mpu6050放置方向						四旋翼电机朝向			  	  -PITCH		向上倾斜角度正负号
	
***********************************************************************************************
*/							


uint8_t IMU_FLAG;
uint8_t REMOTE_FLAG;

int16_t M1, M2, M3, M4;

float e_gyro_x;
float e_gyro_y;
float pwm_x;
float pwm_y;

int16_t motor = 0;

void Flight_Check(void)
{
	if(Pitch > -45 && Pitch < 45 && Roll > -45 && Roll < 45)
	{
		IMU_FLAG = 1;
	}else{
		IMU_FLAG = 0;
		DEBugFly_Error();
	}
	
}

void Control(uint8_t outerEN)
{
	Flight_Check();
	if(IMU_FLAG)
	{
		if(outerEN){
			e_gyro_x = Pitch_Pid(nrf_Pitch, Pitch);
			e_gyro_y = Roll_Pid(nrf_Roll, Roll);
		}

		pwm_x = Gx_Pid(e_gyro_x, gx);
		pwm_y = Gy_Pid(e_gyro_y, gy);
		
//		printf("pwmx: %f\r\n", pwm_x);
//		printf("pwmy: %f\r\n", pwm_y);
		M1 = (1000 + MOTO_Speed) + pwm_x - pwm_y;
		M2 = (1000 + MOTO_Speed) - pwm_x - pwm_y;
		M3 = (1000 + MOTO_Speed) - pwm_x + pwm_y;
		M4 = (1000 + MOTO_Speed) + pwm_x + pwm_y;

//		printf("nrfpitch: %f\r\n", nrf_Pitch);
//		printf("nrfroll: %f\r\n", nrf_Roll);
//		printf("MOTO_Speed: %d\r\n", MOTO_Speed);
		
//		printf("M1: %d\r\n", M1);
//		printf("M2: %d\r\n", M2);
//		printf("M3: %d\r\n", M3);
//		printf("M4: %d\r\n", M4);
		
		Motor_PWM_Out(M1, M2, M3, M4);
		
	}
}

void DEBugFly_Error(void)
{
	LED1_ON;
	
	Motor_PWM_Out(1000, 1000, 1000, 1000);
}

void Fly_Error(void)
{
	LED1_ON;
	M1-=3;
	M2-=3;
	M3-=3;
	M4-=3;
	Motor_PWM_Out(M1, M2, M3, M4);
}
