#include "stm32f4xx.h"
#include "bsp_debug_usart.h"
#include "bsp_can1.h"
#include "bsp_can2.h"
#include "string.h"
#include "stdio.h"
#include "stm32f4xx_usart.h"
#include "timer.h"
#include "delay.h"
#include "4g_comnet.h"
#include "run.h"
#include "key.h"
#include "rs16_lidar.h"
#include <lidar.h>
int buttonDown = 0;
typedef union /// 位置转换值；用联合体转换十进制的十六进制形式
{
	u8 data[4];
	s32 tmpdata;
} testdata;

//////////////

CanTxMsg TxMessage1; // CAN1发送缓冲区
CanRxMsg RxMessage1; // CAN1接收缓冲区
CanTxMsg TxMessage2; // CAN2发送缓冲区
CanRxMsg RxMessage2; // CAN2接收缓冲区
// uint8_t MCU1canbuf[8];
u8 tempbuf[12] = {'a', 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x0A, 0x0B, 0x0C};

__IO uint32_t flag1 = 0; // 用于CAN1标志是否接收到数据，在中断函数中赋值
__IO uint32_t flag2 = 0; // 用于CAN2标志是否接收到数据，在中断函数中赋值
__IO uint32_t flag3 = 0;

uint16_t ReceiveData = 0;
unsigned char task_num = 0;
// task_num,任务计数器

// 自检函数
void SelfTest(void)
{
 
	SelfTestResultposition = StartPositionTest(); // 出发位置检测
	printf("position check result = %d\r\n", SelfTestResultposition);
//	SelfTestResultlidar = LidarSelfTest(); // 激光雷达自检
//	printf("lidar check result = %d\r\n", SelfTestResultlidar);
	if (SelfTestResultposition == 1)
	{
		SelfTestResult = 1;
		AGVtasklimit=0;
	}
	else if(SelfTestResultposition == 0)
	{
		SelfTestResult = 0;
	}
	else 
    SelfTestResult = 2;
}

// 获取系统任务函数
void GetTaskSystem(void)
{
	unsigned char GTFB = 1;
	TaskCount = 0;
	
	GTFB = GetTaskForBuf(); // 从缓存中获取任务数据
	if (GTFB == 0)
	{
		printf("Get system task OK!\r\n");
		AGVtaskover=0;
		task_flag = 0; // 获取完任务，任务开关置零
		task_status = 9;
		if (EPB_System_Status == 1)
			EPBControl1();
	}
	else
	{
		printf("Wait system task\r\n");
		task_status = 0;
		delay_ms(30);
	}
}

// 获取任务函数
//五菱

// 运行函数
void Run(void)
{

	uint8_t usart3firstsend = 0;
	Init(); // 初始化//
	while (1)
	{
		printf("\r\n Start!!！\r\n");
		HomeTrans();
		EPB12Trans();
		
//    LidarRead();	//测试用
		if(uart_tx_flag)
		{
		uart_tx_flag = 0;
		usart5Create();  // 更新发送缓冲区
		//  发送数据
		UART5_SendArray(AGVupsendbuf, 12);

		
		}

//		if(AGVwait == 1)
//			{
//				EHBControl(20);
//				printf("wait!!!");//20251206QHY新增等待点
//			}
//		else
//			{
//				EHBControl(0);
//			}
		
		if(AGV_remote_status==1)//遥控状态  内容代添加
		{
/*ZQY20260224
			memset(cangpsbuf2, 0, sizeof cangpsbuf2); //清空数组  
			memset(now_task, 0, sizeof now_task);
			memset(pointbuf, 0, sizeof pointbuf);
			task_status = 0;//2025819改为0	
			Task_Count=0;
			AGVtaskover=1;
*/
			FB_IS_PAUSED = 0;//ZQY20260225
		  SelfTestResult=0;
			printf("remoting!!!");
			shield_flag=1;//2025921
			arrive_flag=0;//2025921
			GetPosition();		  // 获取车辆GPS位置
	    printf("car_x=%d  car_y=%d  car_angle=%f\r\n", car_x, car_y, Rs_Angle);

		}
		if(AGV_remote_status==3)//驻车状态  清零
		{
			
			memset(cangpsbuf2, 0, sizeof cangpsbuf2); //清空数组  
			memset(now_task, 0, sizeof now_task);
			memset(pointbuf, 0, sizeof pointbuf);
			RevokeControl();
			Expect_EPS_Angle=0;
			Expect_speed=0;
			Expect_Gear=0;
			auto_start=1;//AGV自动驾驶开，可打开可关闭
			taskseq=0;
			SelfTestResult=0;
			newAGVtaskover=0;
			arrive_flag=0;
			printf("AGV_remote_status==3!!!\n");
//			if(AGV_pole_status ==2)
//			{
//			auto_start=1;
//			}
		}
/*ZQY20260202注释掉
		if(AGV_remote_status==3 && AGV_pole_status ==2)
		{
			auto_start=0;//20251103
			printf("AUTO_START===0!!!\r\n");
		}
*/			
		if(FB_DISPATCHING_STOP_SW==1)//AGV急停
		{

			memset(cangpsbuf2, 0, sizeof cangpsbuf2); //清空数组  
			memset(now_task, 0, sizeof now_task);
			memset(pointbuf, 0, sizeof pointbuf);
			RevokeControl();
			Expect_EPS_Angle=0;
			Expect_speed=0;
			Expect_Gear=0;
			//auto_start=0;//AGV自动驾驶开，可打开可关闭
			auto_start=1;//AGV自动驾驶开，可打开可关闭ZQY20260202注释掉173行，智驾模式改为1
			SelfTestResult=0;
			//EHBControl(20);
			if(AGV_getVehicle!=0)//20251104
			{
				EHBControl(20);
			}
			else
			{
				EHBControl(0);
			}
			newAGVtaskover=0;
			printf("emergencystop!!!");

		}
		//if(FB_DISPATCHING_STOP_SW==0 && LidarObstacle == 0 && lidarLoss==0)
		//if(FB_DISPATCHING_STOP_SW==0)
		//{
		//	EHBControl(0);//20251103
		//}
		
		if (AGV_remote_status==2)//切换自驾
		{
      HomeTrans();
			if(FB_IS_PAUSED==1)//暂停
			{
				Expect_EPS_Angle=0;
				Expect_speed=0;
				EPBControl();
				if(EPB_System_Status==1)
				{
				   EHBControl(0);
				}
				else
				{
					EHBControl(20);
				}
				printf("agv pauseing！！！");
			}
			
			
			if (SelfTestResult == 0)
			{
				comm_path_trans();
				SelfTest(); // AGV自检
				printf("Expect_speed=%d",Expect_speed);
			}

			// 自检结果为2，只能遥控
			if (SelfTestResult == 2)
			{
				printf("startposition is error!!please  change remote");
				auto_start=0;//AGV自动驾驶关闭
				remote_control_status = 1;//未使用
				
			}

			////自检结果为1，可以自动驾驶
			if (SelfTestResult == 1)
			{
				auto_start=1;//AGV自驾打开
				//u8 AGVtasklimit=0;//ZQY 20260622 add
				// 自检通过进入自动驾驶状态
				if (auto_start == 1)
				{

						EPB12Trans();	
					  comm_path_trans();
							printf("taskseq=%d ", taskseq);//20251015
					    printf("task_status=%d ", task_status);//AGV调试用
					    printf("Length_Obstacle=%d Length_Obstacle1=%d Lidar_Status=%d LidarObstacle=%d OutTolerance=%d lidarLoss=%d", Length_Obstacle, Length_Obstacle1, Lidar_Status, LidarObstacle, OutTolerance, lidarLoss);//AGV调试用
					    printf("Expect_speed=%d",Expect_speed);
						if(car_x<=200&&car_x>=-200&&car_y<=200&&car_y>=-200)//2025929
								{
									shield_flag=1;//2025929
								}//2025929
								else{shield_flag=0;}//2025929
//				    LidarRead();		  // 前激光雷达检测 LidarObstacle
//					    if(Expect_Gear== 2&&Length_Obstacle<=10)
//							{
//								EHBControl(20);//AGVEHB
//								if(AGV_getVehicle==0) BrakePressureReq=0;
//								printf("***qinlidardistance=%d",Length_Obstacle);
//							}
//							
//							if(Expect_Gear== 1&&Length_Obstacle1<=6)
//							{
//								EHBControl(20);//AGVEHB
//								if(AGV_getVehicle==0) BrakePressureReq=0;
//								printf("***houlidardistance=%d",Length_Obstacle1);
//							}
							if(lidarLoss==1)
							{
								Expect_EPS_Angle = 0;//2025929
								Expect_speed = 0;//20251009
								if(AGV_getVehicle!=0) EHBControl(20);
								if(AGV_getVehicle==0) {BrakePressureReq=0;EPBControl();}
								printf("one line lidar lose");
							
							}
							//if(AGV_getVehicle==0){Expect_EPS_Angle = 0;}//2025929

							//if (LidarObstacle == 0 && OutTolerance == 0 && lidarLoss == 0 && FB_IS_PAUSED==0 && AGVwait == 0)//AGV增加没有被暂停
							if (LidarObstacle == 0 && OutTolerance == 0 && lidarLoss == 0 && FB_IS_PAUSED==0)// && AGVwait == 0)//AGV增加没有被暂停
							{

								// 获取调度系统任务
								if (task_status == 0)
								{

									//EHBControl(10);//AGVEHB
									//delay_ms(20);
									EHBControl(0);
									if (EPB_System_Status != 1)
										EPBControl();
									GetTaskSystem(); // 调度获取任务，且释放EPB
								}

								// 直线行驶
								if (task_status == 1)
								{
									arrive_flag=0;
									LidarRead();		  // 前激光雷达检测 LidarObstacle
									GPSL = LineProcess(); // 直行处理AGV改
									if (GPSL == 0)
										SelfTestResult = 2;
									// printf("GPSL=%d GPSError=%d SelfTestResult=%d", GPSL, GPSError, SelfTestResult);
								}

								// 车辆转弯
								if (task_status == 2)
								{
									arrive_flag=0;
									LidarRead();  // 前激光雷达检测 LidarObstacle
									GPSL = TurnProcess();//AGV转弯
									if (GPSL == 0)
										SelfTestResult = 2;
									// printf("GPSL=%d SelfTestResult=%d", GPSL, SelfTestResult);
								}

								// 倒车
								if (task_status == 3)
								{
									arrive_flag=0;
									LidarRead();									// 前激光雷达检测 LidarObstacle
									//FB_IS_PAUSED = 0;//ZQY20260225
									GPSL = ReverseProcess(); // 倒车处理，是普通倒车还是固定长度倒车
									GPSL = 1;
									if (GPSL == 0)
										SelfTestResult = 2;
								}

								// 任务终点停车
								if (task_status == 4)
								{
									StopTask(); // EPBStop();AGV
									if(car_x<=100&&car_x>=-100&&car_y<=100&&car_y>=-100)
									{arrive_flag=0;printf("arrive_flag=0\r\n");}
									else{arrive_flag=1;printf("arrive_flag=1\r\n");}
								}

								// 倒车转弯
								if (task_status == 5)
								{
									arrive_flag=0;
									LidarRead();			 // 前激光雷达检测 LidarObstacle
									GPSL = ReverseTurnProcess(); // ReverseTurn();
									if (GPSL == 0)
										SelfTestResult = 2;
									printf("GPSL=%d SelfTestResult=%d", GPSL, SelfTestResult);
									;
								}

								// 获取下一任务 //停车，等待系统任务
								if (task_status == 9)
								{
									GetTaskParameter();
								}


							}
							else
							{
//								printf("ehbstop");
//								if (AGV_getVehicle > 0)
//									ehbstop();

								//////////////ZQY新增20260227
								Expect_speed=0;
								if(AGV_getVehicle != 0)
								{
									EHBControl(5);
									printf("EHBControl(5)");
								}
								else
								{
									EHBControl(0);
									printf("EHBControl(0)");	
								}
								//////////////
								if (OutTolerance == 1 || LidarObstacle == 1 || lidarLoss == 1)
								{
									LidarRead(); // 前激光雷达检测 LidarObstacle

//									Taskokreset();


									if (OutTolerance == 1)
										printf("##OutTolerance=1, Please check Lidar and speed!\r\n");
									else if (lidarLoss == 1)
										printf("##One line Lidar lose, waiting revovery!\r\n");
									else
									{
										printf("##Please move obstacle!\r\n");
									}
								}
							}
						
						// printf("LidarObstacle=%d  LidarObstacle2=%d CameraObstacle=%d  task_flag=%d  KEY0=%d\r\n", LidarObstacle, LidarObstacle2, CameraObstacle, task_flag, KEY0);
					
				}


			}
		}
		// 自检结果为零，需要自检
	}
}

int main(void)
{
	SCB->VTOR = FLASH_BASE | 0x8000;

	Run();
}

/*********************************************END OF FILE**********************/
