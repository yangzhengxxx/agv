#include <lidar.h>
#include "stm32f4xx.h"
#include <stdio.h>
#include <rs16_lidar.h>
#include <run.h>
#include "bsp_debug_usart.h"
#include "4g_comnet.h"

//***********Lidar参数
unsigned char Lidar_Status, Lidar1_Status, lidarlossStop;//0雷达正常1前雷达丢失2后雷达丢失3前后雷达丢失
unsigned char Lidar_jiaoyancount;
int Length_Obstacle, Length_Obstacle1, Length_Obstacle2;
uint8_t Lidarcanbuf[8];

// Lidar_Status 前激光雷达状态，Lidar1_Status 后激光雷达状态
// Length_Obstacle 前激光雷达障碍距离,Length_Obstacle1 后激光雷达障碍距离
// Length_Obstacle2 侧雷达
// lidarlossStop 多线雷达多次数据丢失导致停车

// 前激光雷达数据读取和刹车控制函数，LidarTestSwitch 1-直行，2-转弯，3-倒车
void newcanLidarTrans(void)//AGV待改20250914
{
static u8 BSMcnt = 0, Obstacle_flag = 1;
	int sum = 0;
	sum = Lidarcanbuf[0] + Lidarcanbuf[1] + Lidarcanbuf[2];
	if (Lidarcanbuf[7] == sum)
	{
		Lidar_jiaoyancount=0;
		lidarLoss = 0;
		lidarLossNum = 0; // 8-29
		u8 checkValue = 0;
			int n;
       unsigned char  L01, L02, L03, L04, L05, L06, L07, L08;
			  Lidar_Status= Lidarcanbuf[0];//新增雷达状态
		    L01 = Lidarcanbuf[1];                              // 前1米//解析部分待改
        L02 = Lidarcanbuf[2];//后雷达

			if (LidarTestSwitch == 3)
			{
					Length_Obstacle = L01;
			}
			if (LidarTestSwitch == 4)
			{
					Length_Obstacle = L01;
			}
			if (LidarTestSwitch == 15)
			{
					Length_Obstacle1 = L02;
			}
			
			
			if (LidarTestSwitch == 3)
      {

        if (Length_Obstacle <= 15) // 12-15 150-300
        {
            LidarObstacle = 1; // 9-5
            printf("##3-15##LidarObstacle=%d\n", Length_Obstacle);
            if (AGV_getVehicle > 0)
	          {
							
							EHBControl(20);
						}
			 EPB_Park_Request = 2;
			 BrakePressureReq = 0;
        }
        else
				{
            LidarObstacle = 0;
					if((TaskCount-1)>=0)
					{ if(now_task[TaskCount-1][4]==0)
						 DrivingSpeed=10;
					  else
						 DrivingSpeed = now_task[TaskCount-1][4]*1; 
				  }
					else{printf("TaskCount is error");}
					
				}
       }

		  if (LidarTestSwitch == 15)
       {

        if (Length_Obstacle1 <= 10) // 12-15 150-300
        {
            LidarObstacle = 1; // 9-5
            printf("##B15-7 and Secter##LidarObstacle=%d\n", Length_Obstacle1);
            if (AGV_getVehicle > 0)
						{
                
							EHBControl(20);
						}
				EPB_Park_Request = 2;
				BrakePressureReq = 0;
        }
        else
				{
            LidarObstacle = 0;
					if((TaskCount-1)>=0)
					{ if(now_task[TaskCount-1][4]==0)
						 DrivingSpeed=10;
					  else
						 DrivingSpeed = now_task[TaskCount-1][4]*1; 
				  }
					else{printf("TaskCount is error");}
				}
       }	
			 	if (LidarTestSwitch == 4)//AGV转弯用
       {

        if (Length_Obstacle <= 9) // 12-15 150-300
        {
            LidarObstacle = 1; // 9-5
            printf("##B4-9 and Secter##LidarObstacle=%d\n", Length_Obstacle);
            if (AGV_getVehicle > 0)
						{
                
							EHBControl(20);
						}
				EPB_Park_Request = 2;
				BrakePressureReq = 0;
        }
        else
				{
            LidarObstacle = 0;
					if((TaskCount-1)>=0)
					{ if(now_task[TaskCount-1][4]==0)
						 DrivingSpeed=10;
					  else
						 DrivingSpeed = now_task[TaskCount-1][4]*1; 
				  }
					else{printf("TaskCount is error");}
				}
       }	


      printf("L0e %u %u  \r\n", L01, L02);
			printf("lidarstatus %d lidardistance  %u %u \r\n",  Lidarcanbuf[0], Lidarcanbuf[1],  Lidarcanbuf[2]);

	}
	else
	{
		Lidarflag=0;
		Lidar_jiaoyancount++;
		if (Lidar_jiaoyancount>10&&AGV_getVehicle > 0)//多次校验不通过刹车2025917
		{
		   EHBControl(20);
		}
	  printf("lidar jiaoyan error");
	}



}


