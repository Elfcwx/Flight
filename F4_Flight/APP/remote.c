#include "remote.h"

u8 nrf24l01_buf[33];
uint8_t nrf_rx;
float nrf_Roll;
float nrf_Pitch;
float nrf_Yaw;
uint16_t speed;
uint16_t MOTO_Speed;

uint8_t unlockF;
uint8_t descentF;
uint8_t onLedF;
uint8_t takeOffF;
//float battery;

void Remote_Receive(void)
{
	NRF24L01_RxMode();
	nrf_rx = NRF24L01_RxPacket(nrf24l01_buf);
	if(nrf_rx == 0)
	{
		if (nrf24l01_buf[2] == 0xff) {
				nrf_Roll = nrf24l01_buf[1];
			} else {
				nrf_Roll = -nrf24l01_buf[1];
			}
			
			if (nrf24l01_buf[4] == 0xff) {
				nrf_Pitch = nrf24l01_buf[3];
			} else {
				nrf_Pitch = -nrf24l01_buf[3];
			}
			
			if (nrf24l01_buf[6] == 0xff) {
				nrf_Yaw = nrf24l01_buf[5];
			} else {
				nrf_Yaw = -nrf24l01_buf[5];
			}
			speed = ((uint16_t)nrf24l01_buf[7]<<8)|nrf24l01_buf[8];
			unlockF = nrf24l01_buf[9];
			descentF = nrf24l01_buf[10];
			onLedF = nrf24l01_buf[11];
			takeOffF = nrf24l01_buf[12];
			
			MOTO_Speed = (float)(speed - 460) / 3240 * 800; //max 1800
			
	}
}

void Send_Remote(void)
{
	NRF24L01_TxMode();
	
	
	
}



