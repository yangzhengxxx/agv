#ifndef __RUN_H
#define __RUN_H
#include "stm32f4xx.h"
extern unsigned char ULtrasonicFlag; // 11.19 南奔洋改动，增加超声波
extern unsigned char Brake_Pedal_Travel;
extern unsigned char MCU1flag, MCU2flag, MCU3flag, EPSflag, EPBflag, EHBflag, EHB1flag, GPSflag, Lidarflag, Lidarflag1, GPS1flag, Cameraflag, Tyreflag; // 9-19 9-6
extern uint8_t MCU1canbuf[8];
extern uint8_t MCU2canbuf[8];
extern uint8_t MCU3canbuf[8];
extern uint8_t EPScanbuf[8];
extern uint8_t EPBcanbuf[8];
extern uint8_t EPBcanbuf1[8];//F系列车两个EPB
extern uint8_t EPBcanbuf2[8];

extern uint8_t EHBcanbuf[8];
extern uint8_t EHB1canbuf[8];
extern uint8_t GPScanbuf[8];
extern uint8_t Lidarcanbuf1[8];
extern uint8_t GPS1buf[8];
extern uint8_t Cameracanbuf[8]; // 9-6
extern uint8_t Tyrecanbuf[8];   // 9-19
extern int16_t now_task[50][12];    // AGV
extern unsigned char canbuf[8], next_task[8];
extern unsigned char epscanbuf[8];
extern unsigned char vcucanbuf[8], vcurevcanbuf[8];
extern unsigned char drvcanbuf[8], drvrevcanbuf[8];
extern uint8_t ULtracanbuf[8];
extern uint8_t check_mcu_data[8];
extern uint8_t newEPSrecvcanbuf[8];
extern int16_t newEPSreturn;
extern unsigned char Drv_CtrlMode, Drv_TgtGear;
extern unsigned short Drv_TgtPedpos,Drv_TgtPedpos1, MCU_ThrotAct;
extern unsigned int MCU_MtrSpd;
extern unsigned char check_mcupressure;


//liyuweigai
int NewReverse(void);
extern unsigned char newehbcanbuf[8];
extern uint16_t newBrakePressureReq;

extern u8 SelfTestResult, lidarLoss, Release_flag,DrivingSpeed, openmcuflag, remote_control_status, SOC,SelfTestResultposition,SelfTestResultlidar;//2025shan Release_flag,
extern u8 remote_release_flag;
extern	unsigned char QRR,GPSL;;
extern int pass_goal_x, pass_goal_y, Release_x, Release_y, goal_x, goal_y;

extern unsigned char Expect_Gear, Auto_Drive_Mod_Sta;
extern unsigned char Left_signal_control, Right_signal_control;
extern float VehicleSpeed;
extern unsigned char EPB_Park_Request, EPB_System_Status, BrakePressureReq, Expect_Pedal_Depth, ACU_Fault_Code;
extern unsigned char EPB1_System_Status,EPB2_System_Status,EPB_DTC;
extern unsigned short Expect_Steering_Angle;
extern unsigned char Reversing_light_control, Horn_control, Emergency_hazad_warning_control, Braking_control;
extern unsigned char Message_Counter_ACU, lidardatalose;

extern int car_x, car_y;
extern int line_count, Reverse_count, Expect_EPS_Angle;
extern unsigned char task_status, TyreStatus, VehicleState;

extern unsigned char HeadingAngleError, GPSError, SOCError, LidarError;
extern unsigned char EHB_Fault_Level, EPBError, CANError, GPSError, SOC, PositionError;
extern unsigned char LidarObstacle, LidarObstacle2, CameraObstacle;
extern unsigned char MotorDct, CameraTestSwitch;
extern unsigned char epbcount;
extern unsigned char MCU_errlevel;
extern unsigned char MCU_errlevelcount;
extern unsigned char lastExpect_Gear;
extern u8 remoteflag,rengongflag;

extern unsigned int EPS_Faulty;
extern unsigned char CameraError, LidarTestSwitch, LidarTestSwitch1;
extern u8 task_flag, wirelesscharging_flag;
extern u8 TaskCount, TaskTotal, OutTolerance;
extern u8 HeavyThro, heavy_flag, turn_num, lidar1Error;
extern   u8 key;                  //2024.11.16
extern   u8 carmode;
extern    u8 lastcarmode; //1为遥控驾驶2为人工驾驶
// 鏁呴殰鐮?

static u8 RemoteError = 0, EPSError = 0, EHBError = 0, OtherError = 0;
extern uint8_t uart_tx_flag;
extern uint8_t FB_IS_PAUSED;//暂停状态标志
void Init(void);

int GetTaskForBuf(void);
void GetTaskParameter(void);

int ReverseProcess(void);
int TurnProcess(void);
int LineProcess(void);
int ReverseTurnProcess(void);
int StartPositionTest(void);
void ZeroEPSControlCode(void);
void REndTurnJudge(void);
void LineTask(int x, int y, unsigned char PD);
void TurnTask(unsigned char id, int r, int enddis, unsigned char PD);
void ReverseTask(int x, int y, unsigned char PD);
void ReverseTurnTask(unsigned char id, int r, int enddis, unsigned char PD);
// void GetTask(void);
void check_mcu(void);
void StopTask(void);
// void DealAbnormal(void);
void Stop(void);

void SteerAngleCal(void);
void ehbstop(void);

void GetPosition(void);
void CarPositionCal(void);


void EPB12Trans(void);


void UWBGPSTrans(void);
void CANCreate(void);
void usart5Create(void);
void ExitAutopilotLoop(void);
void EHBControl(unsigned char BrakePressure);
void EPBControl(void);
void EPSControl(void);
void ReverseControl(void);

void VoidGear(void);
void CAN_Data_Receive(void);

extern uint8_t Chargecanbuf[8]; //2023.9.2南奔洋改
extern uint8_t lowercomputercanbuf[8];//2025
extern uint8_t remotemodecanbuf[8];
extern uint8_t AGV_VCU_canbuf[8];//AGV底盘反馈
extern uint8_t AGV_EPB_canbuf[8];//AGV epb状态反馈
extern uint8_t AGV_PAUSED_canbuf[8];//AGV epb状态反馈
extern uint8_t AGV_EPS_canbuf[8];//AGV EPS
extern uint8_t AGV_EHB_canbuf[8];//AGV EHB
extern u8 AGV_remote_status;
extern u8 AGV_pole_status;
extern float AGV_getVehicle;//AGV底盘反馈速度
extern u8 AGV_getGear;//agv底盘反馈档位
extern uint8_t DISPATCHING_STOP_SW;//急停标志位
extern uint8_t TASK_START;//启动标志位
extern uint8_t TASK_PAUSE;//暂停标志位
extern uint8_t FB_DISPATCHING_STOP_SW;//急停标志位
extern uint8_t FB_TASK_START;//启动标志位
extern uint8_t FB_TASK_PAUSE;//暂停标志位
extern uint8_t TO_AGV_PSUSE_canbuf[8];
extern u8 auto_start;
extern u8 AGVtaskover;
extern int16_t AGV_BMS_canbuf[8];//AGVBMS反馈
extern int16_t AGV_soc;
extern uint8_t brushSig;     /* 电刷信号：0/1 */
extern uint8_t chargeState;  /* 充电状态：0/1 */
extern u8 taskseq;
extern u8 AGVtaskovercount;
extern u8 newAGVtaskover;
extern u8 AGVwait;//AGV是否需要在等待点等待，1为等待，0为继续走 20251204
extern uint16_t arrive_flag; //2025921
extern uint8_t shield_flag; //2025921
extern u8 Homestate;     //归位开关状态 	为1时充电枪处于归位状态；为0时充电枪处于未归位状态。
extern u8 DCGunConnectStatus;
extern u8 ACGunConnectStatus;
extern u8 remote_start_status;
//extern u8 MCU_ErrLevel;//mcu故障等级
extern u8 CDCU_MCU_GearAct;//实际档位
extern u8 HVstopstate;   //高压急停开关 	为1时高压急停按钮按下，执行下高压操作；为0时高压急停按钮未按下，执行上高压操作。
extern u8 Chargestate;   //充电状态     	0x01:充电中  0x02:空闲、离线、故障等非充电状态
extern u8 Chargunstate;   //充电枪状态  	0x01:插枪    0x02:拔枪
extern u8 ChargeToSelfstate;  //给充电车自身充电  1：充电   0：未充电
extern u8 HighLowSpeedSwitch; //高低速模式开关  0:低速  1：高速
extern uint16_t lidarLossNum ; // 8-29
extern u8 Expect_speed;
void HomeTrans(void);

int Line(void);

int Turn(void);


int ReverseTurn(void);

int EndTaskConditions(void);
void LidarRead(void);

// void CANDataRead(unsigned char n);


void Yaw(void);

// void RemoteRun(unsigned char mode);
extern void RevokeControl(void);
void Taskokreset(void);
void CoordinateComple(u8 car_direct);
void EPBControl1(void);
void BMSTrans(void);
void MCUSend(void);
#endif
