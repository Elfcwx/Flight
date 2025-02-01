#ifndef __PID_H__
#define __PID_H__

#include "system.h"


float Pitch_Pid(float recv_pitch, float imu_pitch);
float Roll_Pid(float recv_roll, float imu_roll);
float Gx_Pid(float e_gyro_x, float gyro_x);
float Gy_Pid(float e_gyro_y, float gyro_y);
float Pos_Pid(float e_pos, float pos);
float Vec_Pid(float e_vec, float vec);

#endif	//__PID_H__
