#include "pid.h"

//pitch角度环参数
float pitch_Kp = 5;
float pitch_Ki = 0;
float pitch_Kd = 2;

//roll角度环参数
float roll_Kp = 5;
float roll_Ki = 0;
float roll_Kd = 2;


//x轴角速度环参数
float gx_Kp = 0.73f;
float gx_Ki = 0;
float gx_Kd = 2;

//y轴角速度环参数
float gy_Kp = 0.73f;
float gy_Ki = 0;
float gy_Kd = 2;

// 位置环pid参数
float pos_Kp = 0.025f;
float pos_Ki = 0.004f;
float pos_Kd = 0.0f;

// 速度环pid参数
float vec_Kp = 0.028f;
float vec_Ki = 0.00007f;
float vec_Kd = 0.0f;



float Pitch_Pid(float recv_pitch, float imu_pitch)
{
	static float last_error, error_sum;
	float error, x_gyro;

	error = recv_pitch - imu_pitch;
	error_sum += roll_Ki * error;
	error_sum = error_sum > 20 ? 20 : (error_sum < -20 ? -20 : error_sum);
	
	x_gyro = roll_Kp * error + error_sum + roll_Kd * (error - last_error);
	last_error = error;
	
	return x_gyro;
	
}
float Roll_Pid(float recv_roll, float imu_roll)
{
	static float last_error, error_sum;
	float error, y_gyro;
	
	error = recv_roll - imu_roll;
	error_sum += pitch_Ki * error;
	error_sum = error_sum > 20 ? 20 : (error_sum < -20 ? -20 : error_sum);
	
	y_gyro = pitch_Kp * error + error_sum + pitch_Kd * (error - last_error);
	last_error = error;
	
	return y_gyro;
	
}

float Gx_Pid(float e_gyro_x, float gyro_x)
{
	static float last_error, error_sum;
	float error, x_pwm;
	
	error = e_gyro_x - gyro_x;
	error_sum += gy_Ki * error;
	error_sum = error_sum > 200 ? 200 : (error_sum < -200 ? -200 : error_sum);
	
	x_pwm = gy_Kp * error + error_sum + gy_Kd * (error - last_error);
	x_pwm = x_pwm > 500 ? 500 : (x_pwm < -500 ? -500 : x_pwm);
	last_error = error;
	
	return x_pwm;
	
}
float Gy_Pid(float e_gyro_y, float gyro_y)
{
	static float last_error, error_sum;
	float error, y_pwm;
	
	error = e_gyro_y - gyro_y;
	
	error_sum += gy_Ki * error;
	// 积分限幅
	error_sum = error_sum > 200 ? 200 : (error_sum < -200 ? -200 : error_sum);
	
	
	y_pwm = gy_Kp * error + error_sum + gy_Kd * (error - last_error);
	y_pwm = y_pwm > 500 ? 500 : (y_pwm < -500 ? -500 : y_pwm);
	last_error = error;
	
	return y_pwm;
}
/* 
e_pos 为期望位置，在定点悬停时，给0
pos 为实际位置，通过光流计测量位移

ret : e_vec 为期望速度
*/
float Pos_Pid(float e_pos, float pos)
{
	static float last_error, error_sum;
	float error, e_vec;
	
	error = e_pos - pos;
	
	error_sum += pos_Ki * error;
	// 积分限幅
	error_sum = error_sum > 200 ? 200 : (error_sum < -200 ? -200 : error_sum);
	
	e_vec = pos_Kp * error + error_sum + pos_Kd * (error - last_error);
	
	last_error = error;
	
	return e_vec;
}

/* 
e_vec 为期望速度，通过位置环输出得到
vec 为实际速度，通过光流计测量速度

ret : e_angle 为期望角度
*/
float Vec_Pid(float e_vec, float vec)
{
	static float last_error, error_sum;
	float error, e_angle;
	
	error = e_vec - vec;
	
	error_sum += pos_Ki * error;
	// 积分限幅
	error_sum = error_sum > 200 ? 200 : (error_sum < -200 ? -200 : error_sum);
	
	e_angle = pos_Kp * error + error_sum + pos_Kd * (error - last_error);
	
	last_error = error;
	
	return e_angle;
}
