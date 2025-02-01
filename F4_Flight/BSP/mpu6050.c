#include "mpu6050.h"

float gyro_offset[3];

const float acc_offset_fac[3] = {1.0031,0.9932,0.9784};
const float acc_offset_bias[3] = {-0.0749,0.0261,0.1296};

static uint8_t GYRO_OFFSET_FLAG = 0;
static uint8_t ACC_OFFSET_FLAG = 0;

Kalman _kalman;
float lpf_factor = 0.2;


u8 MPU6050_Init(void)
{
	uint8_t res;
	I2C_GPIO_Init();
	delay_ms(100);
	while(!MPU_WriteReg(MPU_PWR_MGMT1_REG, 0x80));		//复位mpu6050
	delay_ms(100);
	while(!MPU_WriteReg(MPU_PWR_MGMT1_REG, 0x00));		//唤醒mpu6050
	while(!MPU_Set_Gyro_Fsr(3));					//陀螺仪量程
	while(!MPU_Set_Accel_Fsr(2));					//加速度计量程
	while(!MPU_WriteReg(MPU_INT_EN_REG, 0x00));		//关闭所有中断
	while(!MPU_WriteReg(MPU_USER_CTRL_REG, 0x00));		//i2c主模式关闭
	while(!MPU_WriteReg(MPU_FIFO_EN_REG, 0x00));		//关闭FIFO
	while(!MPU_WriteReg(MPU_INTBP_CFG_REG, 0x80));		//INT引脚低电平有效

	res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
	
	if(res == MPU_ADDR)
	{
		while(!MPU_WriteReg(MPU_PWR_MGMT1_REG,0X01));	//设置CLKSEL,PLL X轴为参考
		while(!MPU_WriteReg(MPU_PWR_MGMT2_REG,0X00));	//加速度与陀螺仪都工作
		while(!MPU_Set_Rate(50));						//设置采样率为50Hz
	}else{
		return 1;
	}

	return 0;
}


/**
 * @brief       MPU6050设置陀螺仪传感器量程范围
 * @param       frs: 0 --> ±250dps
 *                   1 --> ±500dps
 *                   2 --> ±1000dps
 *                   3 --> ±2000dps
 */
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_WriteReg(MPU_GYRO_CFG_REG,fsr<<3); //设置陀螺仪满量程范围
}


/**
 * @brief       MPU6050设置加速度传感器量程范围
 * @param       frs: 0 --> ±2g
 *                   1 --> ±4g
 *                   2 --> ±8g
 *                   3 --> ±16g
 */
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_WriteReg(MPU_ACCEL_CFG_REG,fsr<<3); //设置加速度传感器满量程范围  
}


/**********************************************
函数名称：MPU_Set_Rate
函数功能：设置MPU6050的采样率(假定Fs=1KHz)
函数参数：rate:4~1000(Hz)  初始化中rate取50
函数返回值：0,设置成功  其他,设置失败
**********************************************/
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_WriteReg(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);											//自动设置LPF为采样率的一半
}

/**********************************************
函数名称：MPU_Set_LPF
函数功能：设置MPU6050的数字低通滤波器
函数参数：lpf:数字低通滤波频率(Hz)
函数返回值：0,设置成功  其他,设置失败
**********************************************/
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_WriteReg(MPU_CFG_REG,data);//设置数字低通滤波器  
}

u8 MPU_WriteReg(uint8_t reg_addr, uint8_t reg_data)
{
	uint8_t ERROR_Flag;
	i2c_Start();
	i2c_SendByte(MPU_I2C_ADDR);
	ERROR_Flag = i2c_WaitAck();
	i2c_SendByte(reg_addr);
	ERROR_Flag = i2c_WaitAck();
	i2c_SendByte(reg_data);
	ERROR_Flag = i2c_WaitAck();
	i2c_Stop();
	
	return ERROR_Flag;
}

u8 MPU_Read_Byte(u8 reg)
{
	uint8_t res;
	
	i2c_Start();
	i2c_SendByte(MPU_I2C_ADDR);
	i2c_WaitAck();

	i2c_SendByte(reg);
	i2c_WaitAck();
	i2c_Start();
	i2c_SendByte(MPU_I2C_ADDR|1);
	i2c_WaitAck();
	res = i2c_ReadByte();
	i2c_NAck();
	i2c_Stop();
	return res;
}

u8 MPU_ReadData(uint8_t reg, uint8_t len, uint8_t *buf)
{
	uint8_t i, ERROR_Flag;
	
	i2c_Start();
	i2c_SendByte(MPU_I2C_ADDR);
	ERROR_Flag = i2c_WaitAck();
	i2c_SendByte(reg);
	ERROR_Flag = i2c_WaitAck();
	
	i2c_Start();
	i2c_SendByte(MPU_I2C_ADDR|1);
	ERROR_Flag = i2c_WaitAck();
	for(i=0; i<len; i++)
	{
		buf[i] = i2c_ReadByte();
		
		/* 每读完1个字节后，需要发送Ack， 最后一个字节不需要Ack，发Nack */
		if(i != len-1)
		{
			i2c_Ack();
		}else
		{
			i2c_NAck();
		}
	}
	i2c_Stop();
	
	return ERROR_Flag;
}


void MPU6050_GyroOffset(void)
{
	uint8_t i;
	MPU6050_RAW_Data offset;
	float sum_x=0, sum_y=0, sum_z=0;
	offset.x=0;
	offset.y=0;
	offset.z=0;
	for(i=0; i<200; i++)
	{
		MPU_Get_GyroData(&offset);
		sum_x += offset.x;
		sum_y += offset.y;
		sum_z += offset.z;
		delay_ms(10);
	}
	gyro_offset[0] = sum_x/200;
	gyro_offset[1] = sum_y/200;
	gyro_offset[2] = sum_z/200;
	GYRO_OFFSET_FLAG = 1;
}

void MPU6050_AccOffset(void)
{
	ACC_OFFSET_FLAG = 1;
}
void Kalman_Init(void)
{
	_kalman.Last_P = 0.02;			
	_kalman.Now_P = 0;		
	_kalman.out = 0;			
	_kalman.Kg = 0;		
	_kalman.Q = 0.001;
	_kalman.R = 0.543;
}

/**
 *卡尔曼滤波器
 *@param 	Kalman *kfp 卡尔曼结构体参数
 *   		float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
float KalmanFilter(Kalman *kfp,float input)
{
   //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
   kfp->Now_P = kfp->Last_P + kfp->Q;
   //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
   kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
   //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
   kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//因为这一次的预测值就是上一次的输出值
   //更新协方差方程: 本次的系统协方差付给 kfp->LastP 为下一次运算准备。
   kfp->Last_P = (1-kfp->Kg) * kfp->Now_P;
   return kfp->out;
}

int16_t lpf_gyro(int16_t old_gyro, int16_t cur_gyro)
{
	return old_gyro * (1 - lpf_factor) + cur_gyro * lpf_factor;
}

void MPU_Get_GyroData(MPU6050_RAW_Data *gyro_data)
{
	uint8_t buf[6], ret;
	int16_t recv_gyro_data[3];
	static int16_t last_gyro[3] = {0, 0, 0};
	ret = MPU_ReadData(MPU_GYRO_XOUTH_REG, 6, buf);
	if(ret == 1)
	{
		recv_gyro_data[0]=((int16_t)buf[0]<<8) | buf[1];
		recv_gyro_data[1]=((int16_t)buf[2]<<8) | buf[3];
		recv_gyro_data[2]=((int16_t)buf[4]<<8) | buf[5];
	}
	//printf("%d,%d\n", recv_gyro_data[0], last_gyro[0]);
	if(GYRO_OFFSET_FLAG)
	{
		recv_gyro_data[0] = lpf_gyro(last_gyro[0], recv_gyro_data[0]);
		recv_gyro_data[1] = lpf_gyro(last_gyro[1], recv_gyro_data[1]);
		recv_gyro_data[2] = lpf_gyro(last_gyro[2], recv_gyro_data[2]);
		
		last_gyro[0] = recv_gyro_data[0];
		last_gyro[1] = recv_gyro_data[1];
		last_gyro[2] = recv_gyro_data[2];
	}
	gyro_data->x = (float)recv_gyro_data[0] * Gyro_Range_fac;
	gyro_data->y = (float)recv_gyro_data[1] * Gyro_Range_fac;
	gyro_data->z = (float)recv_gyro_data[2] * Gyro_Range_fac;
	
	if(GYRO_OFFSET_FLAG)
	{
		gyro_data->x -= gyro_offset[0];
		gyro_data->y -= gyro_offset[1];
		gyro_data->z -= gyro_offset[2];
//		printf("gx-offset: %f\r\n", gyrp_offset[0]);
//		printf("gx: %f\r\n", gyro_data->x);
	}
}

void MPU_Get_AccelData(MPU6050_RAW_Data *acc_data)
{
	uint8_t buf[6], ret;
	int16_t recv_acc_data[3];
	ret = MPU_ReadData(MPU_ACCEL_XOUTH_REG, 6, buf);
	if(ret == 1)
	{
		recv_acc_data[0]=((int16_t)buf[0]<<8) | buf[1];
		recv_acc_data[1]=((int16_t)buf[2]<<8) | buf[3];
		recv_acc_data[2]=((int16_t)buf[4]<<8) | buf[5];
	}
	
//	if(ACC_OFFSET_FLAG)
//	{
//		recv_acc_data[0] = KalmanFilter(&_kalman,recv_acc_data[0]);
//		recv_acc_data[1] = KalmanFilter(&_kalman,recv_acc_data[1]);
//		recv_acc_data[2] = KalmanFilter(&_kalman,recv_acc_data[2]);
//		printf("faz: %d\r\n", recv_acc_data[2]);
//	}

	acc_data->x = (float)recv_acc_data[0] * Acc_Range_fac;
	acc_data->y = (float)recv_acc_data[1] * Acc_Range_fac;
	acc_data->z = (float)recv_acc_data[2] * Acc_Range_fac;
	
	if(ACC_OFFSET_FLAG)
	{
		// 满足线性关系 Kx+b
		acc_data->x = acc_offset_fac[0] * acc_data->x + acc_offset_bias[0];
		acc_data->y = acc_offset_fac[1] * acc_data->y + acc_offset_bias[1];
		acc_data->z = acc_offset_fac[2] * acc_data->z + acc_offset_bias[2];
//		printf("az: %f\r\n", acc_data->z);
	}
	
}





