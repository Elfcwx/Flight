#ifndef __POSTURE_H__
#define __POSTURE_H__

#include "system.h"
#include "mpu6050.h"
#include "math.h"
#include "usart1.h"

#define G			9.80665f		      	// m/s^2	
#define RadtoDeg    57.324841f				//弧度到角度 (弧度 * 180/3.1415)
#define DegtoRad    0.0174533f				//角度到弧度 (角度 * 3.1415/180)
 
extern float Roll, Pitch, Yaw;
extern float gx, gy, gz;
void IMU_Data(uint8_t count);
void Posture_Update(float ax, float ay, float az, float gx, float gy, float gz);

#endif	// __POSTURE_H__
