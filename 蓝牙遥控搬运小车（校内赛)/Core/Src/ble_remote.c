#include "ble_remote.h"
#include <math.h>
remote_t remote;


uint8_t uart_to_remote(uint8_t *buff)
{
	if(buff[0]!=BLE_REMOTE_HEAD||buff[BLE_REMOTE_LENGTH-1]!=BLE_REMOTE_TAIL)
	{
		return REMOTE_ERROR;
	}
	if(uart_to_rocker(&buff[1],&(remote.rocker[0]))==REMOTE_ERROR||uart_to_rocker(&buff[5],&(remote.rocker[1])))
	{
		return REMOTE_ERROR;
	}
	remote.Button[0]=(buff[9]&0x80)>>7;
	remote.Button[1]=(buff[9]&0x40)>>6;
	remote.Button[2]=(buff[9]&0x20)>>5;
	remote.Button[3]=(buff[9]&0x10)>>4;
	remote.Switch[0]=(buff[9]&0x08)>>3;
	remote.Switch[1]=(buff[9]&0x04)>>2;
	remote.Switch[2]=(buff[9]&0x02)>>1;
	remote.Switch[3]=buff[9]&0x01;
	return REMOTE_OK;
}
float degree_to_rad(int16_t degree)
{
	return PI*degree/180;
}

uint8_t uart_to_rocker(uint8_t *buff, rocker_t *rocker)
{
	rocker->angle=(uint16_t)buff[3]<<8|buff[2];
	rocker->distance=(uint16_t)buff[1]<<8|buff[0];
	if(rocker->distance<ROCKER_DISTANCE_MIN||rocker->distance>ROCKER_DISTANCE_MAX)
	{
		return REMOTE_ERROR;
	}
	else if(rocker->distance==ROCKER_DISTANCE_MIN)
	{
		if(rocker->angle==-1)
		{
			rocker->angle=0;
			rocker->x_position=rocker->y_position=0;
			return REMOTE_OK;
		}
		else
		{
			return REMOTE_ERROR;
		}
	}
	else if(rocker->angle<ROCKER_DISTANCE_MIN||rocker->angle>ROCKER_DISTANCE_MAX)
	{
		return REMOTE_ERROR;
	}
	float rad = degree_to_rad(rocker->angle);
	rocker->x_position=(float)rocker->distance*cos(rad);
	rocker->y_position=(float)rocker->distance*sin(rad);
	return REMOTE_OK;	
}

const remote_t *get_remote_control_point(void)
{
	return &remote;
}

