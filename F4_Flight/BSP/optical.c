#include "optical.h"

OpticFlow_pack Rx_Pack;
OpticFlow_Data _opticflow_Data;
float x_speed;
float y_speed;
float x_offset;
float y_offset;
uint8_t receive;

static enum
{
	waitForFirstByte,		// 等待第一个字节状态
	waitForSecondByte,		// 等待第二个字节状态
	waitForData,			// 等待数据接收状态
	waitForCheckXOR,		// 等待检查校验值状态
	waitForEndByte,			// 等待结束字节状态
}RxState = waitForFirstByte;	// 初始化等待第一个字节状态

u8 OpticFlow_Update(void)
{
	if(USART6_GetData(&receive, 1)){
		if(OpticFlow_Write(receive)){
			unpack(&Rx_Pack);
			return 1;
		}
		return 0;
	}else{
		return 0;
	}
}

u8 OpticFlow_Write(u8 res)
{
	static uint8_t checkXOR = 0, dataIndex = 0;
	switch (RxState)
	{
		case waitForFirstByte:
			if(res == BEGIN_BYTE)
			{
				RxState = waitForSecondByte;
				Rx_Pack.startByte1 = res;
			}
			break;
		case waitForSecondByte:
			if(res == DATA_BYTE)
			{
				RxState = waitForData;
				Rx_Pack.startByte2 = res;
				dataIndex = 0;
			}else{
				RxState = waitForFirstByte;
			}
			break;
		case waitForData:
			Rx_Pack.data[dataIndex] = res;
			dataIndex++;
			checkXOR = checkXOR ^ res;
			if(dataIndex == 10)
			{
				RxState = waitForCheckXOR;
			}
			break;
		case waitForCheckXOR:
			if(checkXOR == res)
			{
				Rx_Pack.checkXOR = res;
				RxState = waitForEndByte;
			}else{
				RxState = waitForFirstByte;
			}
			break;
		case waitForEndByte:
			if(res == END_BYTE)
			{
				Rx_Pack.endByte = res;
				RxState = waitForFirstByte;
				checkXOR = 0;
				return 1;
			}else{
				RxState = waitForFirstByte;
			}
			break;
		default:
			RxState = waitForFirstByte;
			break;	
	}
	return 0;
}

void unpack(OpticFlow_pack *packet)
{
	_opticflow_Data.flow_x = packet->data[1]<<8 | packet->data[0];
	_opticflow_Data.flow_y = packet->data[3]<<8 | packet->data[2];
	_opticflow_Data.integration_timespan = packet->data[5]<<8 | packet->data[4];
	_opticflow_Data.height = packet->data[7]<<8 | packet->data[6];
	_opticflow_Data.valid = packet->data[8];
	_opticflow_Data.laser_belief = packet->data[9];
	_opticflow_Data.dt = (float)_opticflow_Data.integration_timespan * 0.000001f;
	_opticflow_Data.x_rad = (float)_opticflow_Data.flow_x / 10000.0f;
	_opticflow_Data.y_rad = (float)_opticflow_Data.flow_y / 10000.0f;
	printf("time: %f\r\n", _opticflow_Data.dt);
	printf("x_rad: %f\r\n", _opticflow_Data.x_rad);
	printf("y_rad: %f\r\n", _opticflow_Data.y_rad);
	x_speed = _opticflow_Data.x_rad * (float)_opticflow_Data.height / _opticflow_Data.dt;
	x_offset += x_speed * _opticflow_Data.dt;
	y_speed = _opticflow_Data.y_rad * (float)_opticflow_Data.height / _opticflow_Data.dt;
	y_offset += y_speed * _opticflow_Data.dt;
}                                  

