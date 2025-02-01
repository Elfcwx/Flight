#ifndef __OPTICAL_H__
#define __OPTICAL_H__

#include "system.h"
#include "usart6.h"
#include "usart1.h"

#define BEGIN_BYTE			0xFE	//开始状态字
#define DATA_BYTE			0x0A	//数据包字节数
#define END_BYTE			0x55	//结束状态字
#define LIGHT_ERR			0x00	//光流数据不可用
#define LIGHT_RIGHT			0xF5	//光流数据可用

/*通讯数据结构*/
typedef struct
{
    uint8_t startByte1;
    uint8_t startByte2;
	uint16_t data[10];
    uint8_t checkXOR;
	uint8_t endByte;
} OpticFlow_pack;

typedef struct
{
	int16_t flow_x;			//X 像素点累计时间内的累加位移
	int16_t flow_y;			//Y 像素点累计时间内的累加位移
	uint16_t height;			//激光测距距离(mm)，
	uint16_t integration_timespan;		//上一次发送光流数据到本次发送光流数据的累计时间us
	uint8_t valid;			//状态值：0(0x00)为光流数据不可用，245(0xF5)为光流数据可用
	uint8_t laser_belief;		//激光测距置信度，比如 0x64 表示激光测距置信度为100%
	float dt;
	float x_rad;
	float y_rad;

}OpticFlow_Data;



u8 OpticFlow_Write(u8 res);
void unpack(OpticFlow_pack *packet);
u8 OpticFlow_Update(void);

extern OpticFlow_Data _opticflow_Data;
extern float x_speed;
extern float y_speed;
extern float x_offset;
extern float y_offset;
#endif	//__OPTICAL_H__
