#ifndef __4G_COMNET_H
#define __4G_COMNET_H
#include "bsp_debug_usart.h"
#include "delay.h"
#include "string.h"

typedef struct ///??径点类型
{
	s32 MovepathX; ///单位厘米cm
	s32 MovepathY;
	s32 Movedir; //角度，不??弧度
} Movepoint;

typedef union ///位置??换值；用联合体??换十进制的十??进制形式
{
	u8 data[4];
	s32 tmpdata;
} trandata;

/////??径点序列
extern s32 StartpointX; //起点坐标
extern s32 StartpointY;
extern s32 Startdir;  //起点方向
extern s32 CurpointX; //当前点坐??
extern s32 CurpointY;
extern s32 Curdir;	   ///当前点方??
extern s32 NextpointX; //下个点坐??
extern s32 NextpointY;
extern s32 Nextdir;	   ///下个点方??
extern s32 CurPathNo;  ///当前??径编??
extern s32 NextPathNo; ///下条??径编??
extern u32 link_count; //失联计数
extern u8 Task_Count;  //任务点的数量
extern u8 Checksum_Error;
extern u8 release_flag;
extern u8 release_canbuf[20][10];
extern u8 task_conflag, PointNum;

extern u8 arrive; ////到达??标位??标???，0：没到达 1：到达；
extern u8 t8266revbuf[24];
extern u8 t8266sendbuf1[24];
extern u8 t8266sendbuf2[24];
extern u8 t8266sendbuf3[24];
extern u8 oksendbuf[24];
extern u8 oksendbuf11[24];
extern u8 AGVupsendbuf[12];
extern u8 cangpsbuf[8];//AGV
extern int16_t cangpsbuf2[100][12];//AGV
extern int16_t pointbuf[100][12];
extern int16_t AGVpointbuf[100][12];//AGV
extern u8 fflag;

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void RM04_Send_Data(u8 *buf, u8 len);
void RM04_Receive_Data(u8 *buf, u8 *len);
void comm_init_trans(void);
void comm_path_trans(void);
void datadeal2(int pointnum);

// FLASH相关

extern u8 datatemp[];
extern u8 flashbuf[20][8];
extern u8 Sysrelease_flag;
extern int k;
extern u8 AGVtasklimit;//防止任务一直赋值更新

/////////////////////////////////////////////////////////////////////////////
static u8 t8266sendbuf[24] = {0x61, 0x00, 0x00, 0x50, 0x10, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x5A, 0x00, 0x00, 0x00};

#endif
