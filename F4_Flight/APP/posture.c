#include "posture.h"

#define Kp	2.0f
#define Ki	0.005f
#define halfT	0.003f

MPU6050_RAW_Data Acc_Raw_Data;
MPU6050_RAW_Data Gyro_Raw_Data;

float Roll, Pitch, Yaw;
float q0=1, q1=0, q2=0, q3=0;
float gx=0, gy=0, gz=0;
float ax=0, ay=0, az=0;
float exInt, eyInt, ezInt;
float q0_yaw = 1.0f, q1_yaw = 0.0f, q2_yaw = 0.0f, q3_yaw = 0.0f;    //弥补Mahony算法在无地磁情况解算Yaw轴满足不了大扰动要求的现象

void IMU_Data(uint8_t update)
{
	MPU_Get_AccelData(&Acc_Raw_Data);
	MPU_Get_GyroData(&Gyro_Raw_Data);
	
	ax = Acc_Raw_Data.x * G;
	ay = Acc_Raw_Data.y * G;
	az = Acc_Raw_Data.z * G;

	gx = Gyro_Raw_Data.x * DegtoRad;
	gy = Gyro_Raw_Data.y * DegtoRad;
	gz = Gyro_Raw_Data.z * DegtoRad;
//	printf("gx: %f\r\n", gx);
//	printf("gy: %f\r\n", gy);
	if(update)
	{
		Posture_Update(ax, ay, az, gx, gy, gz);
	}
}

void Posture_Update(float ax, float ay, float az, float gx, float gy, float gz)
{
	float norm; 
	float ex, ey, ez;
	float vx, vy, vz;
	
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q1q1 = q1*q1;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
	
	float	q0_yawq0_yaw = q0_yaw * q0_yaw;
	float	q1_yawq1_yaw = q1_yaw * q1_yaw;
	float	q2_yawq2_yaw = q2_yaw * q2_yaw;
	float	q3_yawq3_yaw = q3_yaw * q3_yaw;
	float	q1_yawq2_yaw = q1_yaw * q2_yaw;
	float	q0_yawq3_yaw = q0_yaw * q3_yaw;
	
	//Yaw轴四元数的微分方程，先单独解出yaw的姿态
	q0_yaw = q0_yaw + (-q1_yaw * gx - q2_yaw * gy - q3_yaw * gz) * halfT;	//halfT采样时间的一半
	q1_yaw = q1_yaw + (q0_yaw * gx + q2_yaw * gz - q3_yaw * gy) * halfT;
	q2_yaw = q2_yaw + (q0_yaw * gy - q1_yaw * gz + q3_yaw * gx) * halfT;
	q3_yaw = q3_yaw + (q0_yaw * gz + q1_yaw * gy - q2_yaw * gx) * halfT;
	
	//规范化Yaw轴四元数
	norm = sqrt(q0_yawq0_yaw + q1_yawq1_yaw + q2_yawq2_yaw + q3_yawq3_yaw);
	q0_yaw = q0_yaw / norm;
	q1_yaw = q1_yaw / norm;
	q2_yaw = q2_yaw / norm;
	q3_yaw = q3_yaw / norm;
	
	if(ax * ay * az	== 0)//如果加速度数据无效，或者自由坠落，不结算
		return ;
	
	//加速度计测量的重力方向(机体坐标系)
	norm = sqrt(ax*ax + ay*ay + az*az);
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;
	
	//四元数推出的实际重力方向(机体坐标系)
	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 -q2q2 + q3q3;
	
	//叉积误差
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);
	
	//叉积误差积分为角速度
	exInt = exInt + ex * Ki;
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;
	
	//角速度补偿
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
	
	//更新四元数
	q0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy) * halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx) * halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx) * halfT;
	
	//单位化四元数
	norm = sqrt(q0q0 + q1q1 + q2q2 + q3q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	
	//四元数反解欧拉角
	Pitch = atan2(2 * q2q3 + 2 * q0q1, -2 * q1q1 - 2 * q2q2 + 1) * 57.3f;
	Roll = asin(-2 * q1q3 + 2 * q0q2) * 57.3f;
	Yaw = atan2(2 * q1_yawq2_yaw + 2 * q0_yawq3_yaw, -2 * q2_yawq2_yaw - 2 * q3_yawq3_yaw + 1)	* 57.3f;
}



