#include "bsp_debug_usart.h"
#include "bsp_can1.h"
#include "delay.h"
#include "rs16_lidar.h"
#include "run.h"
#include "math.h"
#include "4g_comnet.h"

unsigned int taillose = 0;
unsigned char RSLidarcanbuf[8];
unsigned char Tx_canbuf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int rslidar_x = 0, rslidar_y = 0, rslidar_angle;
float Rs_Angle = 0;
u8 MatchingLoss = 0; //多次超差停车
u8 matchingcnt = 0;	 // lidar漂计数器
u8 imuflag = 0, matchingflag = 0;
float imuinit_angle = 0;
float imucom_angle = 0;
float endlinedis = 0;
int imucnt = 0;

//__align(8) u8 UART5_TX_BUF[400];

//u8 UART5_RX_BUF[400];

u16 UART5_RX_STA = 0;
u8 UART5_RX_END = 0;
//void DEBUG_UART5_IRQHandler(void)
//{
//	u8 res;
//	if (USART_GetITStatus(DEBUG_UART5, USART_IT_RXNE) != RESET)
//	{
//		USART_ClearITPendingBit(UART5, USART_IT_RXNE);
//		res = USART_ReceiveData(DEBUG_UART5);
//		//		  USART_SendData(USART1, res);
//		if ((UART5_RX_STA & 0x80) == 0)
//		{
//			if (UART5_RX_STA == 0 && res != 0xff)
//			{
//				taillose++;
//				UART5_RX_STA = 0;
//				return;
//			}
//			UART5_RX_BUF[UART5_RX_STA & 0x7F] = res;
//			if (++UART5_RX_STA >= 13)
//			{
//				if (UART5_RX_BUF[12] != 0xfa)
//				{
//					taillose++;
//					// USART1->DR = '&';
//					//  memset(UART5_RX_BUF, 0, sizeof(UART5_RX_BUF));
//					UART5_RX_STA = 0;
//				}
//				else
//				{
//					UART5_RX_STA |= 0x80;
//				}
//			}
//		}
//	}
//}

//void LidarData_Deal(void)//原
//{
//	int j;
//	int sum = 0;
//	float rslidar_angle2 = 0;
//	//static u8 enterimu = 0;
//	if (UART5_RX_STA & 0x80)
//	{
//		UART5_RX_END = 1;
//		MatchingLoss = 0;
//		for (j = 1; j < 11; j++)
//			sum = sum + UART5_RX_BUF[j];
//		if (UART5_RX_BUF[11] == (sum & 0xff))
//		{
//			if (UART5_RX_BUF[2] < 128)
//			{
//				rslidar_x = (UART5_RX_BUF[2] * 65536 + UART5_RX_BUF[3] * 256 + UART5_RX_BUF[4]);
//			}
//			else
//			{
//				rslidar_x = -((UART5_RX_BUF[2] - 128) * 65536 + UART5_RX_BUF[3] * 256 + UART5_RX_BUF[4]);
//			}
//			if (UART5_RX_BUF[5] < 128)
//			{
//				rslidar_y = (UART5_RX_BUF[5] * 65536 + UART5_RX_BUF[6] * 256 + UART5_RX_BUF[7]);
//			}
//			else
//			{
//				rslidar_y = -((UART5_RX_BUF[5] - 128) * 65536 + UART5_RX_BUF[6] * 256 + UART5_RX_BUF[7]);
//			}
//			rslidar_angle = (UART5_RX_BUF[8] * 256 + UART5_RX_BUF[9]);
//			rslidar_angle2 = rslidar_angle - 18000;
//			if (rslidar_angle2 < 0)
//				rslidar_angle2 = rslidar_angle2 + 36000;
//			Rs_Angle = (float)((rslidar_angle2) / 10.0);
//			imuinit_angle = Rs_Angle;
//			printf("Lidarbuf %x %x %x %x %x %x %x %x %x %x\r\n", UART5_RX_BUF[2], UART5_RX_BUF[3], UART5_RX_BUF[4], UART5_RX_BUF[5], UART5_RX_BUF[6], UART5_RX_BUF[7], UART5_RX_BUF[8], UART5_RX_BUF[9], UART5_RX_BUF[10], UART5_RX_BUF[11]);
//		}
//		else
//		{
//			printf("ERROR! sum=%x  buf[11]=%d\r\n", sum, UART5_RX_BUF[11]);
//		}
//	}

//	memset(UART5_RX_BUF, 0, sizeof(UART5_RX_BUF));
//	UART5_RX_STA = 0;
//	printf("lidar_x = %d lidar_y = %d lidar_angle = %f\r\n", rslidar_x, rslidar_y, Rs_Angle);
//}

//void Endlinedis(void)
//{
//	endlinedis = sqrt((rslidar_x - goal_x) * (rslidar_x - goal_x) + (rslidar_y - goal_y) * (rslidar_y - goal_y)); //计算离目标点的距离
//	imucnt = endlinedis / (DrivingSpeed / 3.6 * 100);
//	printf("Line run need imucnt:%d \n", imucnt);
//}


void LidarData_Deal(void)//多线数据处理新版
{
	int j;
	int sum = 0;
	float rslidar_angle2 = 0;
	//static u8 enterimu = 0;

//	if (USART5_RX_END == 1)
//	{
//		printf("hello");
	if (t8266revbuf[0] == 't')
	//if (t8266revbuf[0] == 0x74)
	{
		for (j = 1; j < 11; j++)
			sum = sum + t8266revbuf[j];
		MatchingLoss = 0;
			int n;

		if (t8266revbuf[11]==(sum & 0xff))
		{
			if (t8266revbuf[2] < 128)
			{
				rslidar_x = (t8266revbuf[2] * 65536 + t8266revbuf[3] * 256 + t8266revbuf[4]);
			}
			else
			{
				rslidar_x = -((t8266revbuf[2] - 128) * 65536 + t8266revbuf[3] * 256 + t8266revbuf[4]);
			}
			if (t8266revbuf[5] < 128)
			{
				rslidar_y = (t8266revbuf[5] * 65536 + t8266revbuf[6] * 256 + t8266revbuf[7]);
			}
			else
			{
				rslidar_y = -((t8266revbuf[5] - 128) * 65536 + t8266revbuf[6] * 256 + t8266revbuf[7]);
			}
			rslidar_angle = (t8266revbuf[8] * 256 + t8266revbuf[9]);
			rslidar_angle2 = rslidar_angle - 18000;
			if (rslidar_angle2 < 0)
				rslidar_angle2 = rslidar_angle2 + 36000;
			Rs_Angle = (float)((rslidar_angle2) / 10.0);
			imuinit_angle = Rs_Angle;
//			printf("rslidar_x=%d,rslidar_y=%d",rslidar_x,rslidar_y);
			printf("Lidarbuf %x %x %x %x %x %x %x %x %x %x\r\n", t8266revbuf[2], t8266revbuf[3], t8266revbuf[4], t8266revbuf[5], t8266revbuf[6], t8266revbuf[7], t8266revbuf[8], t8266revbuf[9], t8266revbuf[10], t8266revbuf[11]);
		}
		else
		{
			printf("Multi-line lidarERROR! 校验未通过");
		}
	}
//}
}