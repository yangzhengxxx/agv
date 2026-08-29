#ifndef __LIDAR_H
#define __LIDAR_H
#include "stm32f4xx.h"
#include "stdint.h"

void newcanLidarTrans(void);
extern unsigned char Lidar_Status, Lidar1_Status, lidarlossStop,Lidarflag;;
extern int Length_Obstacle, Length_Obstacle1, Length_Obstacle2;
extern uint8_t Lidarcanbuf[8];
#endif
