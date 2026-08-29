#include "run.h"
#include "math.h"
#include "delay.h"
// #include "imu.h"
#include "stdio.h"

#include "bsp_can1.h"
#include "bsp_can2.h"
// #include "comnet.h"
#include "4g_comnet.h"
#include "timer.h"
// #include "stm32f4xx_it.h"

#include "stm32f4xx.h"
#include "bsp_debug_usart.h"
// #include "bsp_485.h"
#include "string.h"
#include "rs16_lidar.h"
#include "key.h"
#include "lidar.h"
// #include "gps.h"

#define PI 3.1415926
#define MaxPedalDepth 100
// 最大油门  40, 40/0.4=100
#define MaxRomotePedalDepth 200
// 最大遥控油门 80, 80/0.4=200
#define BasePedalDepth 16
// 基础油门
#define PedalDepthCof 8
// 油门系数，速度差乘油门系数=油门变化量
#define BaseSpeed 2
#define NearGoalLen 1000
// 接近目标距离
#define SafeSpeed 2 // 9-20  6km/h used Car 2, 3km/h used Car 3，Dakhin22.1.6改回2
// 安全车速 2km/h
#define NormalSpeed 4 // 9-22
// 正常车速 10km/h
#define TurnSpeed 2
// 转弯车速 3km/h
#define RampSpeed 2
// 爬坡车速 2km/h
#define ReverseSpeed 3
// 倒车车速 3km/h
#define SpeedCoefficient 1.8
// 速度系数，用于计算轮速，1号车MCU反馈有误差
#define CorDis1 10
// 纠偏距离，当车辆偏离准确位置达到CorDis1，开始纠偏，单位cm
#define CorDis2 80
// 纠偏距离，当车辆偏离准确位置达到CorDis2，开始最大转角纠偏
#define CorDis3 20
////纠偏距离，当车辆偏离准确位置达到CorDis3，开始纠偏
u8 MaxCorAng = 5;
// 最大纠偏角，单位度
#define FilterAngle 8
// 直线行驶滤波角度
#define CANDataReadNum 20
// CAN总线无信号故障计数值
#define SpeedCalCof 5.5
// 速度计算系数，通过轮速计算车辆下一时刻位置
u8 EndPointCorrection = 0;
// 路线终止条件，考虑调度系统延迟设立的提前量
#define Turndis 0 // 计算转弯半径减少cm
#define ReverseCorVal 10
// 倒车修偏计算基准值
#define ReverseCorAng 4
// 倒车修偏角
#define compensate_angle 0 // 4.6号2
// Turn补偿转角

// CanRxMsg RxMessage,*Rx;
////CAN 数据接收结构体

unsigned char task_status = 0, NextTaskStatus, task_line_status = 0, task_stack = 100, task_type, task_end = 0, task_turn_num = 0, TaskError; // increase task_line_status
// task_status 任务状态, 0-无任务, 1-直线任务, 2-转弯任务, 3-倒车, 4-停车
// 5-倒车转向, 6-, 7-停车等待固定时间, 8-EmengancyStop, 9-brake
// 10-waiting	//14-停车等待固定时间，始终用EHB
// task_line_status，0-平地，1-爬坡，2-重载爬坡，3-溜坡
// 6-平地节拍行驶状态，7-溜坡节拍行驶状态，8-爬坡节拍行驶状态，9-重载爬坡节拍行驶状态
// task_stack 任务堆栈，车辆异常时临时记录车辆任务状态, 100-堆栈为空时的值
// task_type 调度系统传输任务类型, 1-line, 2-arc
// task_end, 任务结束状态标识，0-结束, 1-执行中
// TaskError 判断是否有重复任务，0-无重复，1-重复

int turnCenterX, turnCenterY;
// 转弯圆心
	unsigned char QRR, GPSL;//2025
u8 Expect_speed=0;
int WaitingBeat = 0, WaitingNum = 0;
// 等待节拍计数器，等待数值

unsigned short turn_goal_angle, RemoteCount = 0;
// 转弯目标角度
// 遥控计数器
unsigned char TaskCount = 0, TaskTotal = 0,  waiting_sum = 0, ThrottleSize, DrivingSpeed, BaseThrottle = 10, ScaleThrottle, ThrottleGrade;
// TaskCount,任务计数器
// TaskTotal，任务总数
// now_task，当前任务数组
// wating_sum, 等待计数器
// 遥控状态下进空挡（即退出自动驾驶状态）标识，0-正常遥控，1-进空挡
// DrivingSpeed 行驶速度，由调度系统给出的预期速度
// BaseThrottle,基础油们	ScaleThrottle,比例系数	ThrottleGrade,油门等级

// unsigned char SelfTestResult=0;//,pcnum=0;
// 自检结果，0-未自检，1-正常, 2-发车位置错误, 3-控制错误

// unsigned char relaxDataOK = 0;
int relaxPoint[20][2];
// 释放点位置

unsigned char SelfTestResult = 0, openmcuflag = 1;
unsigned char SelfTestResultposition = 0;
unsigned char SelfTestResultlidar = 0;
// 自检结果，0-未自检，1-正常, 2-发车位置错误, 3-控制错误
//  openmcuflag:开机首次进入遥控标志

int car_x = 0, car_y = 0, car_angle = 0, goal_x, goal_y, radius, turn_id, goal_angle;
// car_x,car_y 和 car_angle 是车辆x、y坐标和角度
// goal_x and goal_y goal_angle是目标x、y坐标，目标方位角
// radius 车辆转弯半径
// turn_id 转弯类型标识，1-左转x+，2-左转x-，3-左转y+，4-左转y-，5-右转x+，6-右转x-，7-右转y+，8-右转y-
// radius 车辆目标角度
int Cal_x, Cal_y, now_x, now_y, pass_x = 0, pass_y = 0, pass_angle, now_angle, deta_X = 0, deta_Y = 0;
// Cal_x,Cal_y 轮速计算的x、y坐标
// now_x,now_y now_angle 当前x、y坐标，方位角?ppass_x,pass_y  pass_angle前一x、y坐标，方位角
int start_x, start_y, start_angle;
// start_x,start_y ,start_angle直线起点x、y坐标和车辆方位角

int CarAngleData[20], end_x, end_y, NowCarangle, CarAngleCorrect, BaseAngle, AngleVarient;
// CarAngleData，车辆方位角数据，用于进行基于GPS的惯导校准
// end_x,end_y,终点x、y坐标，用于进行基于GPS的惯导校准
// NowCarangle，当前车辆方位角，用于进行基于GPS的惯导校准
// CarAngleCorrect，车辆方位角校准值，用于进行基于GPS的惯导校准
// BaseAngle 基准角度，车间停车前惯导角度，AngleVarient，车间停车再起时惯导角度变化值
unsigned char CarAngleNum, LidarTestSwitch = 1, LidarTestSwitch1 = 1, AngleTrans, NowPosition, CameraTestSwitch; // 11-17
// CarAngleNum，车辆角度计数器，采集车辆方位角数据时使用
// AngleTrans停车角度记录和用于惯导校准，1-记录，2-惯导校准
// LidarTestSwitch, 前后激光雷达障碍处理状态
// LidarTestSwitch1,侧雷达障碍处理状态
// CameraTestSwitch  前相机障碍处理状态

//***********GPS定义
unsigned char GPSLoseCount;
// unsigned char GPS_qual,Headin_qual;
// float GPS_speed;
// 是否使用GPS定位标识，0，不使用，1-使用

//***********预瞄算法
int preview_point_x, preview_point_y;
// 预瞄点x、y坐标
int pass_goal_x, pass_goal_y;
// 预瞄算法：下一基准出发点，值为上一目标点
int Release_x, Release_y;
// 释放点坐标
u8 Release_flag = 1;

short cor_angle = 0;
// 直线纠偏角
unsigned char cor_count;
// 直线纠偏计数器

unsigned short turnradius;
// 转弯半径

unsigned char remote_control_status = 1, car_direct;
// remote_control_status, 0-程序控制状态,1-遥控状态
// car_direct,直线行驶方向，影响直线行驶终止判别 11-x轴正向,12- x 轴负向, 21- y 轴正向,22-y 轴负向

unsigned char line_start = 0, turn_start = 0, turn_num = 0; //
// line_start 直线开始标识，0-直线初始化，1-正常行驶，
// turn_start 转弯开始标识，0-转弯段，1-10-回正段
// turn_num 转向计数器
int line_count = 0, Reverse_count = 0, ReverTurn_count = 0;
// 直线计数器，正常行驶计数，节拍行驶累加速度;倒车计数器；倒车转弯计数器
unsigned char wheel_angle;
// 车轮转角，转弯时的转角
unsigned short a = 206, b = 6780;
// a ， b 计算转弯时转角的系数

// 体重系数
u8 HeavyThro = 0;  // 油门
u8 heavy_flag = 1; // 首次速度之后，记录体重油门标志位

int length;
// 距目标距离，用于提前减速
unsigned char BrakePressure;
// BrakePressure 刹车压力，EPBStopStatus停车状态，0-未拉EPB，1-拉起EPB，2-EPB异常（无法拉起，需使用EHB驻车）

// IO口无线充电与任务开关定义
u8 task_flag = 0;
u8 wirelesscharging_flag = 1;

// 故障码定义
unsigned char HeadingAngleError = 0, GPSError = 0, SOCError = 0, lidarLoss = 0, LidarError = 0, OutTolerance = 0; // 8-29
u8 lidar1LossNum = 0, lidar1Loss = 0, lidar1Error = 0;
uint16_t lidarLossNum = 0;
// HeadingAngleError, 车辆方位角异常标识，0-正常, 1-异常
// GPSError GPS故障标识 0-正常, 1-故障
// lidarLoss  激光雷达丢失标识，0-正常, 1-丢失
// lidarError 激光雷达故障标识，0-正常, 1-故障
// lidarLossNum, 激光雷达丢失计数器，达到一定数值则认为激光雷达故障
// SOCError 电量故障标识，0-正常, 1-故障
u8 Stop_OutTo_flag = 0; // 刹车超差标志位，防止刹车滑行导致超差

unsigned char CANError, EPBError, PositionError;
unsigned char MCU_errlevelcount;
unsigned char CameraObstacleNum = 0, CameraLossNum = 0, CameraError = 0; // 9-6
// CameraError 相机故障标识，0-正常, 1-故障
// CameraObstacleNum, 相机障碍计数器，达到一定数值则认为相机检到障碍

unsigned char TyreStatus = 0;
// TyreStatus 胎压状态标识，0-正常，1-异常

unsigned char LidarObstacle = 0, LidarObstacle2 = 0, CameraObstacle = 0, LidarEndCondition = 0;
// LidarObstacle 激光雷达检测障碍标识，0-检测距离内无障碍，1-检测距离内有障碍
// CameraObstacle 相机检测障碍标识，0-检测距离内无障碍，1-检测距离内有障碍
// LidarEndCondition （预留）以激光雷达判定是否结束直线行驶标识，0-不使用，1-使用

unsigned char VehicleState = 0; //,PositionReadState=0;
// VehicleState, 车辆状态，0-静止, 1-直线, 2-转弯
// PositionReadState 位置读取状态，0-正常，1-异常（通常GPS坐标为零）
unsigned char epbcount = 0;
int imuruncnt = 0;	  // 惯导运行计数器
u8 lidardatalose = 0; // lidar数据丢失（迭代次数多，变慢了）

// canbuf used to storage the information of CAN instructions
unsigned char canbuf[8] = {0x0c, 0x00, 0x00, 0x00, 0x1E, 0x78, 0x00, 0x3e};
unsigned char epscanbuf[8] = {0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char vcucanbuf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char vcurevcanbuf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char drvcanbuf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char drvrevcanbuf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char newehbcanbuf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t newEPSrecvcanbuf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int16_t newEPSreturn = 0;
// CAN数据缓存数组

unsigned char MCU1flag, MCU2flag, MCU3flag, EPSflag, EPBflag, EHBflag, EHB1flag;
unsigned char GPSflag, Lidarflag, Lidarflag1, GPS1flag, GPS1flag, BMSflag, Cameraflag, Tyreflag;
unsigned char ULtrasonicFlag = 0; // 11.19 南奔洋改动，增加超声波

// 各CAN信号标识位

// 各CAN信号缓存数组
uint8_t MCU1canbuf[8];
uint8_t MCU2canbuf[8];
uint8_t MCU3canbuf[8];
uint8_t EPScanbuf[8];
uint8_t EPBcanbuf[8];
uint8_t EPBcanbuf1[8]; // F系列车两个EPB
uint8_t EPBcanbuf2[8]; // F系列车两个EPB
uint8_t EHBcanbuf[8];
uint8_t EHB1canbuf[8];
uint8_t GPScanbuf[8];
uint8_t Lidarcanbuf1[8];
uint8_t GPS1canbuf[8];
uint8_t BMScanbuf[8];
uint8_t Cameracanbuf[8]; // 9-6
uint8_t Tyrecanbuf[8];	 // 9-19
uint8_t Feature_idbuf[8] = {0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a};
uint8_t ULtracanbuf[8];	   // 1.24 南奔洋改，增加超声波can接收数据
uint8_t check_mcu_data[8]; // 1.18 李雨微改，检查mcu欠压状态
u8 remoteflag = 0, rengongflag = 0;
uint8_t Chargecanbuf[8]; // 2023.9.2南奔洋改
uint8_t lowercomputercanbuf[8];//2025
u8 Homestate;			 // 归位开关状态 	为1时充电枪处于归位状态；为0时充电枪处于未归位状态。
u8 DCGunConnectStatus;	 // 直流补电座状态，0未连接1已连接
u8 ACGunConnectStatus;	 // 交流
u8 remote_start_status;//遥控器打开状态0关闭1打开

u8 CDCU_MCU_GearAct;//实际档位
//u8 MCU_ErrLevel;//故障等级

u8 HVstopstate;		   // 高压急停开关 	为1时高压急停按钮按下，执行下高压操作；为0时高压急停按钮未按下，执行上高压操作。
u8 Chargestate = 2;	   // 充电状态     	0x01:充电中  0x02:空闲、离线、故障等非充电状态
u8 Chargunstate;	   // 充电枪状态  	0x01:插枪    0x02:拔枪
u8 ChargeToSelfstate;  // 给充电车自身充电  1：充电   0：未充电
u8 HighLowSpeedSwitch; // 高低速模式开关  0:低速  1：高速

//***********MCU1参数
unsigned char Accelerator_Pedal, Motor_Tro, Motor_Speed_Plus, Energy_Gear_Sig, Shift_Change, MCU1_Counter, MCU_Control_Model;
float VehicleSpeed;
// VehicleSpeed 车速, 单位 km/h
// unsigned char MCU1_Updata,MCU2_Updata,MCU3_Updata,EPS_Updata,EHB_Updata,EHB1_Updata,GPS_Updata,Lidar_Updata,GPS1_Updata,BMS_Updata;
// 各CAN信号更新成功标识

//***********MCU2参数
unsigned char GearStatus = 0, BrakeSigna, MotorWorkModule, MotorControlStatus, PowerLimitStatus, MasterDisconnectRequest, MotorDct, ReducePowerTip;
unsigned int MotorSpeed, TripMeter;
// GearStatus 挡位状态, 1-倒挡, 4-前进挡 ,其他是空挡

//***********MCU3参数
unsigned int Battery_Voltage, Motor_Current, Motor_Temprature, Motor_Control_Temprature;

//***********EPS参数******充电车
unsigned int Address, Motor_Speed, Resever, EPS_Counter;
int Expect_EPS_Angle = 0;
// Address 地址
// Motor_Speed 电机速度设定
// Expect_EPS_Angle 转向角度设定 -50-0左      0-50右
// Resever 预留
// EPS_Counter 计数器

//***********VCU参数******充电车-动力参数
unsigned char VCU_Move = 0, VCU_Back = 0, VCU_Hand = 0, VCU_Auto = 1, VCU_Stop = 0;
unsigned short VCU_Moto_Speed = 0, VCU_Moto_Add = 0, VCU_Moto_Loss = 0;
// VCU_Move 前进：1有效，0取消
// VCU_Back 后退：1有效，0取消
// VCU_Hand 手动模式：1有效，0取消
// VCU_Auto 自动模式：1有效，0取消
// VCU_Stop 制动：1有效，0取消
// VCU_Moto_Speed 行走电机目标转速 0-5000r
// VCU_Moto_Add 行走电机加速率 0-300   精度0.1
// VCU_Moto_Loss 行走电机减速率

//********NEW MCU参数
unsigned char Drv_CtrlMode = 0, Drv_TgtGear = 0;
unsigned short Drv_TgtPedpos = 0, MCU_ThrotAct = 0, Drv_TgtPedpos1 = 0;
unsigned int MCU_MtrSpd = 0;
// Drv_CtrlMode 模式选择0x0:Throttle Control 0x1:Speed Control 0x2: Reserved 0x3: Reserved
// Drv_TgtGear 挡位0：N   1：D   2：R
// Drv_TgtPedpos 目标油门 偏移量-100，最大255

//********NEWEHB参数
uint16_t newBrakePressureReq = 0;

//***********EPB参数
unsigned char EPB_Park_Lamp_State, EPB_Warning_Lamp_State, EPB_Switch_State, Brake_Lights_Request, Deceleration_Request_Status, EPB_System_Status, EPB_Control_Model;
unsigned char EPB1_System_Status, EPB2_System_Status, EPB_DTC; // 故障码
unsigned int Deceleration_Request, Deceleration_Request_Check;
// EPB_System_Status EPB状态，0-未拉起，1-已拉起，4-拉起中，2-释放中

//***********EHB参数
unsigned char EHB_Status, Parking_Brake_Request, Actual_Pressure, EHB_Fault_Level, Brake_Condition, Brake_Pedal_Travel, EHB_Fault_Level;

//***********EHB1参数
unsigned char Braking_Request_Warning, OilIsfc_Warning, Current_Sensor_Fault, NTC_Fault, EHB_Over_Temp_Warning, EHB_Power_Supply_Warning, Pedal_Travel_Sensor_Fault_Lv1, Pressure_Sensor_Fault, Pedal_Travel_Sensor_Fault_Lv2, EHB_Power_Supply_Fault, ECU_Power_Supply_Fault, Power_Driver_Fault, Motor_Fault, Can_BusOff;

//***********BMS参数
unsigned char Battery_Fault, SOC; // SOC 电量
unsigned int BatteryVoltage;

//***********ACU参数
unsigned char Expect_Gear, Auto_Drive_Mod_Sta;
unsigned char lastExpect_Gear;
unsigned char Left_signal_control = 0, Right_signal_control = 0;
unsigned char EPB_Park_Request, BrakePressureReq, Expect_Pedal_Depth, ACU_Fault_Code = 0;
unsigned short Expect_Steering_Angle;
unsigned char Reversing_light_control = 0, Horn_control = 0, Emergency_hazad_warning_control = 0, Braking_control = 0;
unsigned char Message_Counter_ACU = 0;
unsigned char MCU_errlevel = 0;
// Expect_Gear 期望挡位，
// EPB_Park_Request EPB请求
// BrakePressureReq EHB刹车压力
// Expect_Pedal_Depth 期望油门
// unsigned char car_Fault_Code=0;
unsigned char check_mcupressure;

u8 key;
u8 carmode = 0;		// 1为遥控驾驶2为人工驾驶
u8 lastcarmode = 0; // 1为遥控驾驶2为人工驾驶







//五菱
u8 auto_start=1;//wuling1为开始0为关闭
int EPS_Angle = 0;//AGVEPS角度
int16_t AGVsetangle;//最终给底盘的角度
u8 counter_ACU;
u8 lifting_req = 0; // 举升电机请求：0=悬停，1=举升，2=下降
uint8_t emergency_release = 0;
uint8_t emergency_brake = 0;
uint8_t remotemodecanbuf[8];//2025 0驻车1遥控2自动
uint8_t AGV_VCU_canbuf[8];//AGV底盘反馈
uint8_t AGV_EPB_canbuf[8];//AGV epb状态反馈
uint8_t AGV_PAUSED_canbuf[8];//AGV epb状态反馈
uint8_t AGV_EPS_canbuf[8];//AGV EPS反馈
uint8_t AGV_EHB_canbuf[8];//AGV EHB反馈
u8 AGV_remote_status;//AGV遥控状态0驻车1遥控2自动
u8 AGV_pole_status;//AGV拨杆模式1遥控2自动3默认
float AGV_getVehicle;//AGV底盘反馈速度
u8 AGV_getGear;//agv底盘反馈档位
uint8_t DISPATCHING_STOP_SW=0;//急停标志位  AGV给控制板
uint8_t TASK_START=0;//启动标志位
uint8_t TASK_PAUSE =0;//暂停标志位
uint8_t selftestresult=0;
uint8_t FB_DISPATCHING_STOP_SW=0;//急停标志位   控制板返给AGV
uint8_t FB_TASK_START=0;//启动标志位
uint8_t FB_TASK_PAUSE =0;//暂停标志位
uint8_t TO_AGV_PSUSE_canbuf[8];//给AGV底盘反馈暂停和启动状态
u8 AGVtaskover=0;//单个编号任务完成标志1为完成，2为未完成
u8 newAGVtaskover=0;//任务几完成就为几
int16_t now_task[50][12];
static int8_t currentSpeed = 0;  //AGVs速度阶梯变化 范围：-128 ~ 127
const int8_t SPEED_STEP = 1;//AGV阶梯是2
uint8_t uart_tx_flag = 0;
uint8_t FB_IS_PAUSED;//暂停状态标志
int16_t AGV_BMS_canbuf[8];//AGVBMS反馈
int16_t AGV_soc;
uint8_t brushSig;     /* 电刷信号：0/1 */
uint8_t chargeState;  /* 充电状态：0/1 */
uint8_t overchargeState;  /* 请求结束充电0/1 */
u8 taskseq;
u8 AGVtaskovercount;
u8 AGVwait;//AGV是否需要在等待点等待，1为等待，0为继续走 20251204
uint16_t arrive_flag;//到达任务终点播报语音提示（除了充电点都被置1）2025921
uint8_t shield_flag;//屏蔽超声波雷达标志位0不处理1屏蔽2025921
void HomeTrans(void)
{
  AGV_remote_status=remotemodecanbuf[0]&0x07;//遥控模式状态
	AGV_pole_status=(remotemodecanbuf[0]>>3);//遥杆当前所处位置
	AGV_getVehicle = (float)((AGV_VCU_canbuf[4] << 8) + AGV_VCU_canbuf[5]) * 0.0071132;
	AGV_getGear = (AGV_VCU_canbuf[0] & 0x0F) >> 2;
	DISPATCHING_STOP_SW = (AGV_PAUSED_canbuf[1] >> 7) & 0x01; // 位15，最高位
  TASK_START = (AGV_PAUSED_canbuf[1] >> 5) & 0x01;          // 位13，次高位
  TASK_PAUSE = (AGV_PAUSED_canbuf[1] >> 6) & 0x01;          // 位14，第三高位   
	
	AGV_soc=((uint16_t)AGV_BMS_canbuf[4] << 8) | AGV_BMS_canbuf[5];//电量
	brushSig    = (AGV_BMS_canbuf[6] >> 0) & 0x01;//电刷状态
  chargeState = (AGV_BMS_canbuf[6] >> 1) & 0x01;//充电状态  
	FB_DISPATCHING_STOP_SW = DISPATCHING_STOP_SW;//返回给AGV
	FB_TASK_START = TASK_START;
	FB_TASK_PAUSE = TASK_PAUSE;
	
		// 根据 TASK_PAUSE / TASK_START 更新暂停标志
	if (FB_TASK_PAUSE == 1)//625
	{
			FB_IS_PAUSED = 1;
	}
	if (FB_TASK_START == 1)
	{
			FB_IS_PAUSED = 0;
	}
	printf("xin lu xian");
	printf("AGV_BMS_canbuf[4]: %d", AGV_BMS_canbuf[4]);
  printf("AGV_BMS_canbuf[5]: %d", AGV_BMS_canbuf[5]);
  printf("FB_IS_PAUSED=%d AGV_soc=%d  brushSig=%d chargeState=%d\r\n", FB_IS_PAUSED,AGV_soc,brushSig,chargeState);//调试用
  printf("AGV_remote_status=%d AGV_getVehicle=%f\r\n", AGV_remote_status,AGV_getVehicle);
  printf("auto_start=%d\r\n",auto_start);
//	printf("AGV_getVehicle=%f \r\n", AGV_getVehicle);
	
}
void ALLGPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_SetBits(GPIOE, GPIO_Pin_1);
	printf("GPIO_PE high!\r\n");
}

// 初始化函数
void Init(void)
{
	Debug_USART1_Config();							// Printf
	delay_init(168);								// 初始化延时函数
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 设置系统中断优先级分组2
	KEY_Init();
	ALLGPIO_Init();
	delay_ms(100);

	/*初始化串口*/
	// Debug_USART2_Config(); //GPS
	// Debug_USART3_Config(); //Wifi
//	Debug_UART4_Config();  // Remote
	Debug_UART5_Config();  // Lidar//调度

	// printf("USART Init\r\n");

	/*初始化CAN*/
	CAN1_Config();
	CAN2_Config();
	// printf("CAN Init\r\n");


	delay_ms(1000);

	/* 初始化定时器 */
	TIM2_Int_Init(1679, 999);
	TIM_Cmd(TIM2, ENABLE); // 使能定时器
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	TIM5_Int_Init(4199, 999);
	TIM_Cmd(TIM5, ENABLE); // 中断使能，开启中断发送CAN指令模式
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

	TIM4_Int_Init(839, 999);

	// TIM3_Int_Init(2099, 999);
	// TIM_Cmd(TIM3, ENABLE); // 中断使能，开启中断发送CAN指令模式
	// TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	VoidGear();

	//	BCMControl(11);
	delay_ms(1000);



	KEY_Init();
	//printf("Init finished");
	// TIM5_Int_Init(8399, 999);
}

// 变量初始化函数
//*******************************CAN 接收函数
// mCU1信号读取函数
// mCU2信号读取函数
// EPS信号读取函数
// 老EPB信号读取函数
// 五菱新EPB
void EPB12Trans(void)
{	      
	EPB_System_Status = AGV_EPB_canbuf[4] & 0x0F;
}

// EHB信号读取函数2025
// VCU信号读取函数
// VCU信号读取函数
// EHB1信号读取函数
// 喇叭控制函数
//*******************************CAN recieve funtion
//// CAN数据生成函数
//void CANCreate(void)//chenpuhe
//{
//	int Expect_EPS_Angle2 = 0;

//	if (carmode == 2)
//	{
//		canbuf[1] = 0x00;
//		canbuf[2] = 0x00;
//		canbuf[4] = 0x1E;
//		canbuf[5] = 0x78;
//		Auto_Drive_Mod_Sta = 0;
//		Expect_Gear = 0x07;
//		//		EPB_Park_Request=0x03;
//	}
//	else if (carmode == 1)
//	{
//		Auto_Drive_Mod_Sta = 1;
//		canbuf[1] = BrakePressureReq;
//		canbuf[2] = Expect_Pedal_Depth;
//		Expect_EPS_Angle2 = Expect_EPS_Angle * 10;
//		if (Expect_EPS_Angle2 > 450)
//			Expect_EPS_Angle2 = 450;
//		if (Expect_EPS_Angle2 < -450)
//			Expect_EPS_Angle2 = -450;

//		Expect_EPS_Angle2 = Expect_EPS_Angle2 * 10 + 7800;

//		canbuf[4] = Expect_EPS_Angle2 / 256;
//		canbuf[5] = Expect_EPS_Angle2 % 256;

//		if ((_channels[0] == 0) && (_channels[1] == 0) && (_channels[2] == 0) && (_channels[3] == 0)) // 遥控关闭或通讯消失
//		{
//			canbuf[1] = 0x00;
//			canbuf[2] = 0x00;
//			canbuf[4] = 0x1E;
//			canbuf[5] = 0x78;

//			Expect_Gear = 0x07;
//			//		EPB_Park_Request=0x03;
//			//		if(carmode==1)
//			//		{
//			if (EPB_System_Status == 0 && epbcount <= 3)
//			{
//				EPBControl();
//			}
//			if (EPB_System_Status == 1)
//			{
//				EPB_Park_Request = 0;
//				printf("remote mode loss EPBstatus  is apply\r\n");
//			}

//			//		}
//			printf("remote close send invalid value");
//		}
//	}

//	canbuf[0] = Expect_Gear + Auto_Drive_Mod_Sta * 8 + Left_signal_control * 16 + Right_signal_control * 32 + EPB_Park_Request * 64;
//	canbuf[3] = ACU_Fault_Code;
//	canbuf[6] = Message_Counter_ACU * 16 + Braking_control * 8 + Emergency_hazad_warning_control * 4 + Horn_control * 2 + Reversing_light_control;
//	canbuf[7] = canbuf[0] + canbuf[1] + canbuf[2] + canbuf[3] + canbuf[4] + canbuf[5] + canbuf[6];
//	Message_Counter_ACU = (Message_Counter_ACU + 1) % 4;
//}

static inline int8_t i8_abs(int8_t x) {
    return (x < 0) ? -x : x;
}

// CAN数据生成函数2025
void CANCreate(void)//chenpuhe五菱
{
		if(FB_IS_PAUSED==1||task_status == 4||task_status == 0)//暂停
		{Expect_EPS_Angle=0;}
    EPS_Angle =Expect_EPS_Angle*10 ;
		if (EPS_Angle > 450)     EPS_Angle = 450;
		else if (EPS_Angle < -450)  EPS_Angle = -450;	
	  AGVsetangle=(EPS_Angle+450)/0.1;//角度
	
		if (Expect_speed > 50)     Expect_speed = 50;
	  int8_t targetSpeed = (int8_t)Expect_speed;//速度阶梯
    int8_t speedDiff = targetSpeed - currentSpeed;
	  if (i8_abs(speedDiff) > SPEED_STEP) 
		{
        currentSpeed += (speedDiff > 0 ? SPEED_STEP : -SPEED_STEP);
    } 
		else 
		{
        currentSpeed = targetSpeed;
    }
	  if (currentSpeed > 50)     currentSpeed = 50;//速度
		
		
if (taskseq == 101 || (taskseq >= 120 && taskseq <= 125))//20251216
//if (taskseq == 101 && taskseq == 120 && taskseq == 121 && taskseq == 122 && taskseq == 123 && taskseq == 124 && taskseq == 125)
{
    overchargeState = 1;
//    if (task_status == 3)//ZQY20260225
//    {
//        FB_IS_PAUSED = 0;
//    }
}
		else overchargeState=0;
	canbuf[0] = (Expect_Gear & 0x07) |
            ((auto_start & 0x01) << 3) |
            ((lifting_req & 0x03) << 4) |
            ((EPB_Park_Request & 0x03) << 6);
	canbuf[1] = BrakePressureReq;
	canbuf[2] = (uint8_t)currentSpeed;  // 使用处理后的速度值;
	canbuf[3] = 0xff; // ACU错误代码;
	canbuf[4] = AGVsetangle;
	canbuf[5] = AGVsetangle>> 8;
	canbuf[6] = ((++counter_ACU & 0x0F) << 4) |
                0 << 3 | overchargeState << 2 | 0;
	canbuf[7] = canbuf[0] + canbuf[1] + canbuf[2] + canbuf[3] + canbuf[4] + canbuf[5] + canbuf[6];
	
	if(SelfTestResult==1) selftestresult=1;
	if(SelfTestResult==2) selftestresult=2;
	TO_AGV_PSUSE_canbuf[1]=(FB_DISPATCHING_STOP_SW << 0)|(FB_TASK_START << 1)| (FB_TASK_PAUSE << 2)|(selftestresult << 3)|(LidarObstacle << 5)|(arrive_flag << 6)|(shield_flag <<7 );//发送给AGV暂停,自检，遇障状态,到站语音标志，是否屏蔽超声波雷达
	if (task_status == 2) // 转向灯
    {
        if (Expect_EPS_Angle < 0)
        {
            TO_AGV_PSUSE_canbuf[0] = 0x04; //             
        }
        else if (Expect_EPS_Angle > 0)
        {
            TO_AGV_PSUSE_canbuf[0] = 0x08; // Light_TurnRight在bit3，设置为1（0x08 = 00001000
        }
    }

    else if (task_status == 1) // 日行灯
    {
				TO_AGV_PSUSE_canbuf[0]= 0x20; // 左日行灯 (bit5)
				TO_AGV_PSUSE_canbuf[0]|= 0x40; // 右日行灯 (bit6)
    }
    else // 关闭所有灯
    {
        TO_AGV_PSUSE_canbuf[0]= 0; // 

    }

}


void usart5Create(void)//AGV五菱
{
//	if(taskseq==1&&AGVtaskover==1&&NowPosition!=4){AGVtaskover=0;printf("one");}
//	if(taskseq==2&&AGVtaskover==1&&NowPosition!=1){AGVtaskover=0;printf("two");}
//	if(taskseq==3&&AGVtaskover==1&&NowPosition!=2){AGVtaskover=0;printf("three");}
//	if(taskseq==4&&AGVtaskover==1&&NowPosition!=4){AGVtaskover=0;printf("four");}
	
	AGVupsendbuf[1]=taskseq;
	AGVupsendbuf[2]=newAGVtaskover;
	AGVupsendbuf[3]=task_status;
  AGVupsendbuf[4]=AGV_remote_status;
	AGVupsendbuf[5]=AGV_soc;
	AGVupsendbuf[6]=brushSig;
	AGVupsendbuf[7]=chargeState;
	AGVupsendbuf[8]=AGVupsendbuf[1]+AGVupsendbuf[2]+AGVupsendbuf[3]+AGVupsendbuf[4]+AGVupsendbuf[5]+AGVupsendbuf[6]+AGVupsendbuf[7];


}






// EPS_CAN数据生成函数******充电车
// VCU_CAN数据生成函数******充电车
// MCU_CAN数据生成函数******充电车
// 空挡状态函数
void VoidGear(void)
{
	printf("VoidGear mode\n");
	Expect_Gear = 0;;
	EPB_Park_Request = 0;
	BrakePressureReq = 0;
	Expect_speed=0;//2025
	Expect_EPS_Angle=0;

}

// 退出自动驾驶状态函数
// EHB控制函数
void EHBControl(unsigned char BrakePressure)
{

	printf("***************using EHB to stop car with input brake pressure\n");
	//	Expect_Gear = 0;

	BrakePressureReq = BrakePressure;
	//Expect_speed=0;

}

void EPBGetup(void)
{
	Expect_Gear = 0;
	EPB_Park_Request = 2;
	BrakePressureReq = 0;
	Expect_speed=0;//2025
	Expect_EPS_Angle=0xffff;
	printf("epbgetup");
}

// EPB控制函数
void EPBControl(void) // AGV拉起EPB
{
	unsigned char i = 0, j = 0, k = 0; // i,j,k;
	printf("***************using EPB to stop car,EPBcanbuf[0]=%x\r\n", EPBcanbuf[0]);
	epbcount = epbcount + 1;
	EPBGetup();


	for (i = 0; i < 50; i++)
	{
		delay_ms(20);
		EPB12Trans();
		printf("EPB_System_Status=%d EPB_Park_Request=%d BrakePressureReq=%d\n", EPB_System_Status, EPB_Park_Request, BrakePressureReq);
		if (EPB_System_Status == 1)
		{
			i = 50;
			epbcount = 0;
		}
	}
	if (EPB_System_Status == 1)
	{
		printf("\r\n*****EPB has getup success!\r\n");
		epbcount = 0;
	} // 0:释放  1：应用
	else if (EPB_System_Status == 4)
	{
		printf("\r\n*****EPB is being engaged!\r\n");

	} // 0:释放  1：应用
	else
	{
		printf("\r\n*****EPB can not getup!    ");

		for (k = 0; k < 50; k++)
		{
			EPB12Trans();
			if (EPB_System_Status == 0)
				k = 50;
			printf("EPB_System_Status=%d ", EPB_System_Status);
		}
	}
}

// AGVEPB控制函数1
void EPBControl1(void)
{
	unsigned char i = 0, j = 0; //,j;
	printf("***************relax EPB,EPBcanbuf[0]=%x\r\n", EPBcanbuf[0]);

	Expect_Gear = 0;;
	EPB_Park_Request = 1;
	BrakePressureReq = 0;


	for (i = 0; i < 50; i++)
	{
		delay_ms(10);
		EPB12Trans();
		if (EPB_System_Status == 0)
			i = 50;
		printf("EPB_System_Status=%d ", EPB_System_Status);
	}
	if (EPB_System_Status == 0)
	{
		printf("EPB has release success!\r\n");
	}
	else
	{
		printf("EPB can not release!!! ");
	}
}

// EPS控制函数
void EPSControl(void)
{
	printf(" Expect_EPS_Angle=%d #@*\n", Expect_EPS_Angle);

	BrakePressureReq = 0;
	CANCreate();

}

// 倒车控制函数
void ReverseControl(void)
{
	// unsigned char i;
	if (LidarObstacle == 1)
	{
		// Expect_Pedal_Depth = 0;
		// Expect_Steering_Angle = 0xffff;
//		Drv_TgtPedpos = 0;
		Expect_speed=0;//2025
		Expect_EPS_Angle = 0;
	}

	printf("Expect_Pedal_Depth=%d Expect_Steering_Angle=%d \n", Expect_Pedal_Depth, Expect_Steering_Angle);
	// Expect_Gear = 1;
	Auto_Drive_Mod_Sta = 1;
	EPB_Park_Request = 0;
	BrakePressureReq = 0;
	newBrakePressureReq = 0;
}

// 等待控制函数
//*************************************************************************************************************

// 计算两点方位角的函数
int CalTwoPiontAngle(int x1, int y1, int x2, int y2)
{
	int deta_x, deta_y, angle;
	// deta_x is x coordinate difference between current location and target location
	// deta_y is y coordinate difference between current location and target location

	deta_x = x2 - x1;
	deta_y = y2 - y1;
	if (deta_x != 0)
	{
		angle = 1800 * atan((float)deta_y / (float)deta_x) / PI;

		if ((deta_x < 0) && (deta_y > 0))
			angle = 1800 + angle; // section 2
		if ((deta_x < 0) && (deta_y < 0))
			angle = 1800 + angle; // section 3
		if ((deta_x > 0) && (deta_y < 0))
			angle = 3600 + angle; // section 4
		printf("deta_x=%d deta_y=%d angle=%d  ", deta_x, deta_y, angle);
	}
	return angle;
}

// 转弯中心点计算
void TurnCenterCal(void)
{
	int angle, deta_x, deta_y;
	GetPosition();

	if (turn_id == 1 | turn_id == 5)
		angle = 0;
	if (turn_id == 2 | turn_id == 6)
		angle = 1800;
	if (turn_id == 3 | turn_id == 7)
		angle = 900;
	if (turn_id == 4 | turn_id == 8)
		angle = 2700;

	goal_x = car_x;
	goal_y = car_y;
	// turnCenterX = goal_x + (radius - 100) * cos(PI * angle / 1800);
	// turnCenterY = goal_y + (radius - 100) * sin(PI * angle / 1800);
	turnCenterX = goal_x + radius * cos(PI * angle / 1800);
	turnCenterY = goal_y + radius * sin(PI * angle / 1800);
	printf("$$radius=%d angle=%d turnCenterX=%d turnCenterY=%d ", radius, angle, turnCenterX, turnCenterY);
}

// 转弯时轮胎的期望转角计算
void TireDeflectionAngleCal(void) // 初始角度
{
	wheel_angle = b / (radius - a);
	// 修改添加倒车转弯的基础转角 LI
	if (task_status == 2)
	{
		// if (turn_id < 5)
		// 	BaseAngle = 0 - wheel_angle - compensate_angle;
		BaseAngle = turn_id < 5 ? -wheel_angle : wheel_angle;
	}
	else if (task_status == 5)
	{
		BaseAngle = turn_id < 5 ? wheel_angle : -wheel_angle;
	}
	printf("  **BaseAngle=%d**\n", BaseAngle);
	// Expect_Steering_Angle = BaseAngle; // 12.18加Dakhin
}

// 计算与转弯中心点的距离
int DistanceCenterCal(int x, int y, int xc, int yc)
{
	int Len;
	Len = sqrt((x - xc) * (x - xc) + (y - yc) * (y - yc));
	printf(" ##distance of turn center, Len=%d ", Len);
	return Len;
}

// 车辆位置读取 激光雷达位置
void GetPosition(void)
{
	int detax, detay;
	int Doublesum;
	int maxDoublesum;

	// 读取当前位置前，记录前次位置信息

	pass_x = car_x;
	pass_y = car_y;
	pass_angle = car_angle;

	LidarData_Deal(); // LidarS数据读取
	car_x = rslidar_x;
	car_y = rslidar_y;
	car_angle = Rs_Angle; // car_angle扩大10倍，应用于预瞄算法，11.29 Dakhin

	if (pass_x == car_x && pass_y == car_y && task_status == 1 && line_start == 1)
	{
		lidardatalose++;
		if(AGV_getVehicle==0)//
			lidardatalose=0;
		printf("lidardatalose cnt=%d\n", lidardatalose);
		if (lidardatalose >= 30)//原来是10AGV改为30
			lidarlossStop = 1; // 多次收不到数据
	}
	else
	{
		lidarlossStop = 0;
		lidardatalose = 0;
	}

	detax = car_x - pass_x;
	detay = car_y - pass_y;
	Doublesum = detax * detax + detay * detay; // 超差

	if (Stop_OutTo_flag == 0 && AGV_getVehicle != 0)
	{
		if (lidardatalose > 1 ||task_status == 2)
			maxDoublesum = 40000;
		else
			maxDoublesum = 20000;

		if (Doublesum > maxDoublesum)
		{
			matchingcnt++;
			printf("matchingcnt=%d!!! car_x=%d,car_y=%d,pass_x=%d,pass_y=%d,Doublesum=%d\r\n", matchingcnt, car_x, car_y, pass_x, pass_y, Doublesum);
			CoordinateComple(car_direct);
			if (matchingcnt > 2)
				MatchingLoss = 1; // 多次超差标志
		}
		else
		{
			MatchingLoss = 0;
			matchingcnt = 0;
		}
		if (Doublesum > maxDoublesum && MatchingLoss == 1)
		{
			OutTolerance = 1;
			printf("OutTolerance=%d,pass_x=%d,pass_y=%d,car_x=%d,car_y=%d,Doublesum=%d, max=%d\r\n", OutTolerance, pass_x, pass_y, car_x, car_y, Doublesum, maxDoublesum);
			Stop();

			auto_start=0;//超差关自动驾驶
		}
		else
			OutTolerance = 0;
	}
	else
	{
		OutTolerance = 0;
		Stop_OutTo_flag = 0;
	}
}

void CoordinateComple(u8 car_direct)
{
	switch (car_direct)
	{
	case 11:
		car_x = pass_x + 20;
		car_y = pass_y;
		break;
	case 12:
		car_x = pass_x - 20;
		car_y = pass_y;
		break;
	case 21:
		car_x = pass_x;
		car_y = pass_y + 20;
		break;
	case 22:
		car_x = pass_x;
		car_y = pass_y - 20;
		break;
	}
	printf("NEW: car_direct=%d  car_x=%d  car_y=%d\r\n", car_direct, car_x, car_y);
}

// AGV车辆出发位置检测2025
int StartPositionTest(void)
{
	const int32_t relaxPoint[8][2] = {
    {  113, -81 },//充电桩
		{ -449, -994},//上料缓冲
		{ 4453, 1582},//2-1
		{ 4467, 605},//2-2
		{ 4467, -154},//2-3
		{ 4481, -1060},//2-4
		{ 4483, -1701},//2-5
		{ 4493, -2225},//2-6
};
	int outloerat2 = 0;
	unsigned char j, SPTR;
	// SPTR 位置检测结果，2-无法匹配初始位置，1-匹配初始位置成功
	u8 Release_Test_flag = 1;
	GetPosition(); // 获取车辆Lidar位置
	printf("***NOW car_x=%d  car_y=%d\r\n", car_x, car_y);
	NowPosition = 0;
	for (j = 0; j < 8; j++)
	{
		if (car_x != 0 || car_y != 0)
		{
			if (relaxPoint[j][0] != 0 | relaxPoint[j][1] != 0)
			{
				outloerat2 = ((car_x - relaxPoint[j][0]) * (car_x - relaxPoint[j][0]) + (car_y - relaxPoint[j][1]) * (car_y - relaxPoint[j][1]));
				if (outloerat2 < 0)
					outloerat2 = -outloerat2;
				if (outloerat2 <= 40000)//AGV自检改为两米
				{
					NowPosition = j + 1;
					Release_x = relaxPoint[j][0];
					Release_y = relaxPoint[j][1];
					Release_flag = 1;//待改
					printf("NowPosition=%d   outloerat2=%d\r\n", NowPosition, outloerat2);
					printf("Release_x=%d  Release_y=%d\n", Release_x, Release_y);
//					UART5_SendArray(t8266sendbuf, 24);
					j = 8;
				}
			}
		}
		else
		{
			if (Release_Test_flag == 1)
			{
				printf("Lidar error ,no buf, please check!\r\n");
				Release_Test_flag = 0;
			}
			SPTR = 2;
		}
	}

	if (NowPosition == 0)
		SPTR = 2;
	else
		SPTR = 1;
	
	if (taskseq == 101 || (taskseq >= 120 && taskseq <= 125)) 
{ 
    // ?????1
    if(NowPosition != 1) { SelfTestResult = 0; SPTR = 0; printf("number error1 selftest again"); }
}
else if ((taskseq >= 102 && taskseq <= 107) || taskseq == 126) 
{ 
    // ?????2
    if(NowPosition != 2) { SelfTestResult = 0; SPTR = 0; printf("number error2 selftest again"); }
}
else if (taskseq == 108 || taskseq == 114 || (taskseq >= 127 && taskseq <= 131)) 
{ 
    // ?????3
    if(NowPosition != 3) { SelfTestResult = 0; SPTR = 0; printf("number error3 selftest again"); }
}
else if (taskseq == 109 || taskseq == 115 || (taskseq >= 132 && taskseq <= 135)) 
{ 
    // ?????4
    if(NowPosition != 4) { SelfTestResult = 0; SPTR = 0; printf("number error4 selftest again"); }
}
else if (taskseq == 110 || taskseq == 116 || (taskseq >= 136 && taskseq <= 138)) 
{ 
    // ?????5
    if(NowPosition != 5) { SelfTestResult = 0; SPTR = 0; printf("number error5 selftest again"); }
}
else if (taskseq == 111 || taskseq == 117 || taskseq == 139 || taskseq == 140) 
{ 
    // ?????6
    if(NowPosition != 6) { SelfTestResult = 0; SPTR = 0; printf("number error6 selftest again"); }
}
else if (taskseq == 112 || taskseq == 118 || taskseq == 141) 
{ 
    // ?????7
    if(NowPosition != 7) { SelfTestResult = 0; SPTR = 0; printf("number error7 selftest again"); }
}
else if (taskseq == 113 || taskseq == 119) 
{ 
    // ?????8
    if(NowPosition != 8) { SelfTestResult = 0; SPTR = 0; printf("number error8 selftest again"); }
}
	
	return SPTR;
}

// 自检结果处理
// 故障码处理函数
// 调度系统通迅函数
// 转弯任务数据处理，根据坐标计算转弯参数
// 停车任务，在停车完成后进入等待任务状态
void StopTask(void)
{
	LidarTestSwitch1 = 0; // 侧雷达不再检测
//	if (AGV_getVehicle >= 3)
//	{
//		Stop();
//		Expect_speed=0;//2025
//	}
//	else
//	{
		EHBControl(20);
		delay_ms(20);
		EHBControl(0);
		Expect_speed=0;//2025
		Expect_EPS_Angle=0;//2025819
		memset(cangpsbuf2, 0, sizeof cangpsbuf2); //清空数组  
		memset(now_task, 0, sizeof now_task);
		memset(pointbuf, 0, sizeof pointbuf);
		task_status = 0;//2025819改为0	
		Task_Count=0;
		SelfTestResult = 0;//任务切换要重新自检
		if (taskseq >= 101 && taskseq <= 141) {
    newAGVtaskover = taskseq;//20251205任务几完成就发几
}
		else {printf("taskseq error！！");newAGVtaskover=0;}
		printf("taskover  newAGVtaskover==%d!!!！！",newAGVtaskover);
//	}
	AGVtaskover=1;
	if (!((taskseq >= 114 && taskseq <= 119) || taskseq == 126)) {
    FB_IS_PAUSED = 1;
    printf("taskseq=%d ", taskseq);
    printf("taskover pause!!");
}
	GetPosition();
}

// AGV获取任务参数2025
void GetTaskParameter(void)
{
	unsigned char turnid;
	unsigned short enddis;
	int passx, passy; // 上一目标点坐标

	if (TaskCount > TaskTotal + 1)
	{
		task_status = 0;
		Task_Count = 0;
		TaskTotal = 0;
		AGVtaskover=1;
		printf("Task end!!! TaskCount=%d  Task_Count=%d TaskTotal=%d\n", TaskCount, Task_Count, TaskTotal);
	}
	else
	{
		if (TaskCount == TaskTotal + 1)
		{
			task_status = 4;
			printf("Stop!!! TaskCount=%d Task_Count=%d TaskTotal=%d\n", TaskCount, Task_Count, TaskTotal);
		}
		else
		{
			task_type = now_task[TaskCount][0];
			DrivingSpeed = now_task[TaskCount][4]*1;
			if (task_type == 1)
			{
				LidarTestSwitch = 3;//待改

				passx = goal_x;
				passy = goal_y;
				pass_goal_x = goal_x;
				pass_goal_y = goal_y;
				goal_x = now_task[TaskCount][1];
				goal_y = now_task[TaskCount][2];

				goal_angle = CalTwoPiontAngle(passx, passy, goal_x, goal_y);


				printf("Get line task! TaskCount=%d TaskTotal=%d task_type=%d \n", TaskCount, TaskTotal, task_type);
				printf(" DrivingSpeed=%d goal_x=%d goal_y=%d goal_angle=%d pass_goal_x=%d pass_goal_y=%d\n", DrivingSpeed, goal_x, goal_y, goal_angle, pass_goal_x, pass_goal_y);
				LineTask(goal_x, goal_y, 0); // 大直线，平路，普通油门，激光雷达开，相机开
			}

			if (task_type == 2)
			{
				LidarTestSwitch = 4;//待改

				turnid = now_task[TaskCount][5];
				turnradius = now_task[TaskCount][7]*10;
				enddis = now_task[TaskCount][6]*10;

				printf("Get turn task! TaskCount=%d TaskTotal=%d task_type=%d \n", TaskCount, TaskTotal, task_type);
				printf("LidarTestSwitch=%d DrivingSpeed=%d turnid=%d  turnradius=%d enddis=%d \n", LidarTestSwitch, DrivingSpeed, turnid, turnradius, enddis);
				TurnTask(turnid, turnradius, enddis, 0); // 左转，普通油门，激光雷达开
			}
			if (task_type == 3)
			{

				passx = goal_x;
				passy = goal_y;
				pass_goal_x = goal_x;
				pass_goal_y = goal_y;
				goal_x = now_task[TaskCount][1];
				goal_y = now_task[TaskCount][2];

				LidarTestSwitch = 15;//待改

				printf("Get reverse task! TaskCount=%d TaskTotal=%d task_type=%d  \n", TaskCount, TaskTotal, task_type);
				printf("LidarTestSwitch=%d DrivingSpeed=%d goal_x=%d  goal_y=%d \n", LidarTestSwitch, DrivingSpeed, goal_x, goal_y);

				ReverseTask(goal_x, goal_y, 0); // 大直线，平路，普通油门，激光雷达开，相机开
			}
			if (task_type == 5)
			{
				LidarTestSwitch =15;//待改

				turnid = now_task[TaskCount][5];
				turnradius = now_task[TaskCount][7]*10;
				enddis = now_task[TaskCount][6]*10;
				printf("Get reverseturn task! TaskCount=%d TaskTotal=%d task_type=%d  LidarTestSwitch=%d turnid=%d  turnradius=%d enddis=%d DrivingSpeed=%d\n", TaskCount, TaskTotal, task_type, LidarTestSwitch, turnid, turnradius, enddis,DrivingSpeed);
				ReverseTurnTask(turnid, turnradius, enddis, 0); // 左转，普通油门，激光雷达开
			}
			if ((task_type != 1) && (task_type != 2) && (task_type != 3) && (task_type != 5)&& (task_type != 4))
			{
				printf("Task data error! TaskCount=%d Task_Count=%d task_type=%d \n", TaskCount, Task_Count, task_type);
				ehbstop();
				EHBControl(10);
				auto_start=0;//退出自动驾驶
			}
			if (task_type == 4)
			{
				task_status = 4;
			}
			
			if (TaskCount == TaskTotal)
			{
				DrivingSpeed=10;
				printf("finial task");
			}
			

		}
		TaskCount = TaskCount + 1;
		// printf("CurpointX=%d CurpointY=%d\n", CurpointX, CurpointY); //可能计算出错，很大，22.3.4 Dakhin
	}
}

// 从缓存中获取任务数据AGV
int GetTaskForBuf(void)
{
	unsigned char i, j, repeatTask; //,checkSum;
	for (j = 1; j <= 50; j++)
		for (i = 0; i < 12; i++)
			now_task[j][i] = 0;

	if (Task_Count == 0)
		Checksum_Error = 1; // 如果没有收到任务，返回1
	else
	{
		TaskTotal = Task_Count;
		Checksum_Error = 0;
		printf("TaskTotal=%d \n", TaskTotal);
		for (j = 1; j <= TaskTotal; j++)
		{
			for (i = 0; i < 12; i++)
			{
				now_task[j][i] = cangpsbuf2[j-1][i];//AGV
				printf("%d ", now_task[j][i]);
			}
			printf("\n ");
		}
	}

//	printf("Before repeat:***relax buf[1]=%d buf[2]=%d  buf[3]=%d  buf[4]=%d\r\n", release_canbuf[0][1], release_canbuf[0][2], release_canbuf[0][3], release_canbuf[0][4]);

//	// 检测是否有重复任务
//	for (j = 2; j <= TaskTotal; j++)
//	{
//		repeatTask = 1;
//		for (i = 0; i < 12; i++)
//		{
//			if (now_task[j][i] != now_task[j - 1][i])
//			{
//				repeatTask = 0;
//				i = 12;
//			}
//		}
//		printf("repeatTask=%d  ", repeatTask);
//		if (repeatTask == 1)
//		{
//			printf("Repeat task error!!!\r\n");
//			j = TaskTotal + 1;
//			Checksum_Error = 1;
//			PointNum = 1;
//			Task_Count = 0;
//			memset(cangpsbuf2, 0, sizeof cangpsbuf2); // 清空数组  11.12Dakhin add
//			memset(now_task, 0, sizeof now_task);
//			memset(pointbuf, 0, sizeof pointbuf);
//		}
//	}

//	printf("After Repeat:***relax buf[1]=%d buf[2]=%d  buf[3]=%d  buf[4]=%d\r\n", release_canbuf[0][1], release_canbuf[0][2], release_canbuf[0][3], release_canbuf[0][4]);

	if (Checksum_Error == 0)
	{

		memset(cangpsbuf2, 0, sizeof cangpsbuf2); // 清空数组  11.12Dakhin add
		
		TaskCount = 1;//任务从一开始
	}

	printf("After error=0:***relax buf[1]=%d buf[2]=%d  buf[3]=%d  buf[4]=%d\r\n", release_canbuf[0][1], release_canbuf[0][2], release_canbuf[0][3], release_canbuf[0][4]);
	
	printf("Checksum_Error=%d\r\n", Checksum_Error);
	return Checksum_Error;
}

// 停车，等待系统任务,0-获取系统任务失败，1-获取系统任务成功
// 驻车等待
// 在等待任务过程中，如果超时，需要立刻停车
// 获取释放点数据
// 与调度系统通信，以获释放点取数据
// 按下遥控接管键后，进入遥控接管模式
// 遥控行驶
// 倒车处理，是普通倒车还是固定长度倒车
int ReverseProcess(void)
{
	unsigned char GPSL = 1;
	// GPSL 行驶时GPS丢失状态，0-丢失，1-正常
	LidarTestSwitch1 = 0; // 侧雷达不再检测
	GetPosition();		  // 获取车辆GPS位置
	printf("car_x=%d  car_y=%d  car_angle=%d\r\n", car_x, car_y, car_angle);
	if ((task_line_status == 0) || (task_line_status == 1) || (task_line_status == 2) || (task_line_status == 3))
	{
		if (lidarlossStop == 1 || MatchingLoss == 1 || lidarLoss == 1) // lidar多次数据丢失，多次超差......202592
		{
			printf("lidarlossStop=%d  MatchingLoss=%d lidarLoss=%d\r\n", lidarlossStop, MatchingLoss, lidarLoss);

			if (AGV_getVehicle > 0)
				Stop();
			

			return GPSL;
		}
		if (OutTolerance == 0 && MatchingLoss == 0)
		    	GPSL = NewReverse(); // 普通倒车

	}
	else
	{
		printf("Error Reverse ");

		//Expect_speed=0;//2025
	}
	GPSL = 1;
	return GPSL;
}

// 转向处理AGV
int TurnProcess(void)
{
	unsigned char GPSL = 1;
	// GPSL 行驶时GPS丢失状态，0-丢失，1-正常
//	LidarTestSwitch1 = 2; // 侧雷达开启检测
	GetPosition();		  // 获取车辆lidar位置
	if ((LidarObstacle == 0)  && (OutTolerance == 0))
		{
				if (lidarlossStop == 1 || MatchingLoss == 1 || lidarLoss == 1) // lidar多次数据丢失，多次超差....202592
			{
				printf("lidarlossStop=%d  MatchingLoss=%d lidarLoss=%d\r\n", lidarlossStop, MatchingLoss, lidarLoss);

				if (AGV_getVehicle > 0)
					Stop();
			

				return GPSL;
			}
			GPSL = Turn(); // 普通转弯
		}
	else
	{
		printf("Error turn ");

		//Expect_speed=0;//2025
	}
	return GPSL;
}

// 倒车转向处理
int ReverseTurnProcess(void)
{
	unsigned char GPSL = 1;
	// GPSL 行驶时GPS丢失状态，0-丢失，1-正常
	LidarTestSwitch1 = 0; // 侧雷达不再检测
	GetPosition();		  // 获取车辆GPS位置
	
	if ((task_line_status == 0) || (task_line_status == 1) || (task_line_status == 2) || (task_line_status == 3))
	{
		if (lidarlossStop == 1 || MatchingLoss == 1 || lidarLoss == 1) // lidar多次数据丢失，多次超差.....20259.2
		{
			printf("lidarlossStop=%d  MatchingLoss=%d  lidarLoss =%d\r\n", lidarlossStop, MatchingLoss, lidarLoss);

			if (AGV_getVehicle > 0)
				Stop();

			return GPSL;
		}
		if (OutTolerance == 0 && MatchingLoss == 0)
			GPSL = ReverseTurn(); // 倒车转向
	}
	else
	{
		printf("Error Reverse ");
//		Expect_Pedal_Depth = 0;
		Expect_speed=0;
	}
	// GPSL = 1;
	return GPSL;
}

// AGV直行处理，是普通直行还是固定长度直行
int LineProcess(void)
{
	unsigned char GPSL = 1;
	// GPSL 行驶时GPS丢失状态，0-丢失，1-正常
	LidarTestSwitch1 = 0; // 侧雷达不再检测
	if ((task_line_status == 0) || (task_line_status == 1) || (task_line_status == 2) || (task_line_status == 3))
	{
		if ((LidarObstacle == 0) &&  (OutTolerance == 0))
		{
			GetPosition();								// 获取车辆lidar位置 ，lidarlossStop
			printf("car_x=%d  car_y=%d  car_angle=%d\r\n", car_x, car_y, car_angle);
			if (lidarlossStop == 1 || MatchingLoss == 1 || lidarLoss == 1) // lidar多次数据丢失，多次超差....202592
			{
				printf("lidarlossStop=%d  MatchingLoss=%d lidarLoss=%d\r\n", lidarlossStop, MatchingLoss, lidarLoss);

				if (AGV_getVehicle > 0)
					Stop();
			

				return GPSL;
			}
			if (OutTolerance == 0 && MatchingLoss == 0)
				GPSL = Line(); // 普通直线行驶
		}
		else
		{
			printf("Error task_line_status %d", task_line_status);

			Expect_speed=0;
		}
	}
	return GPSL;
}

// 任务是否结束判别及处理
// 直行任务，x,y目标坐标，PD直行类型
//  Lk激光雷达开放标识,1-激光雷达开,2-激光雷达开（近距离）,3-激光雷达和相机开，4
//  ICK=1进行GPS惯导校准标识,=2是否y坐标特别处理标识
//  GB行驶节拍数，AT停车角度记录标识，BC惯导修正标识
void LineTask(int x, int y, unsigned char PD)
{
	task_status = 1;
	task_line_status = PD;
	goal_x = x;
	// The minimum unit of dispatching system is 0.1 meter, which needs to be converted to 0.01 meter
	goal_y = y;
	printf("goal_x=%d goal_y=%d \n", goal_x, goal_y);

	// 油门计算
	//  BaseThrottle = BasePedalDepth + ThrottleGrade * 4;
	//  ScaleThrottle = PedalDepthCof + ThrottleGrade * 2;
	//  ThrottleGrade = now_task[TaskCount][1] % 16;
	//  printf("ThrottleGrade=%d  ScaleThrottle=%d  BaseThrottle=%d\r\n", ThrottleGrade, ScaleThrottle, BaseThrottle);
	// 直线行驶状态参数和计数器置零
	line_start = 0;
	// line_count = 0;
	// VehicleState = 1;
	// waiting_sum=0;
	// task_end=1;
	printf("LineTask OK! ");
}

// 右转，普通油门，激光雷达开，相机开
void TurnTask(unsigned char id, int r, int enddis, unsigned char PD)
{
	task_status = 2;
	task_line_status = PD;
	turn_id = id;
	radius = r;

	TireDeflectionAngleCal(); // 转弯时轮胎的期望转角计算
	TurnCenterCal();		  // 计算转弯中心点坐标

	if (id == 1)
		goal_x = car_x + enddis;
	if (id == 2)
		goal_x = car_x - enddis;
	if (id == 3)
		goal_y = car_y + enddis;
	if (id == 4)
		goal_y = car_y - enddis;
	if (id == 5)
		goal_x = car_x + enddis;
	if (id == 6)
		goal_x = car_x - enddis;
	if (id == 7)
		goal_y = car_y + enddis;
	if (id == 8)
		goal_y = car_y - enddis;

	printf("Turn id=%d radius=%d goal_x=%d goal_y=%d \n", id, radius, goal_x, goal_y);

	// LidarTestSwitch=Lk;
	// CameraTestSwitch=2;//12-17
	turn_start = 0;
	// VehicleState = 2;
	waiting_sum = 0;
	// task_end=1;
}

// 倒车任务，x,y目标坐标，PD倒车类型
//  Lk后激光雷达开放标识,1-激光雷达开
//  ICK=1进行GPS惯导校准标识,=2是否y坐标特别处理标识
//  GB行驶节拍数，AT停车角度记录标识，BC惯导修正标识
void ReverseTask(int x, int y, unsigned char PD)
{
	task_status = 3;
	task_line_status = PD;
	goal_x = x;
	// The minimum unit of dispatching system is 0.1 meter, which needs to be converted to 0.01 meter
	goal_y = y;
	printf("goal_x=%d goal_y=%d \n", goal_x, goal_y);

	line_start = 0;
	// line_count = 0;
	// VehicleState = 1;
	waiting_sum = 0;
	// task_end=1;
	printf("ReverseTask OK! ");
}

void ReverseTurnTask(unsigned char id, int r, int enddis, unsigned char PD)
{
	task_status = 5;
	task_line_status = PD;
	turn_id = id;
	radius = r;
	TireDeflectionAngleCal(); // 转弯时轮胎的期望转角计算
	TurnCenterCal();		  // 计算转弯中心点坐标
	if (id == 1)
		goal_x = car_x + enddis;
	if (id == 2)
		goal_x = car_x - enddis;
	if (id == 3)
		goal_y = car_y + enddis;
	if (id == 4)
		goal_y = car_y - enddis;
	if (id == 5)
		goal_x = car_x + enddis;
	if (id == 6)
		goal_x = car_x - enddis;
	if (id == 7)
		goal_y = car_y + enddis;
	if (id == 8)
		goal_y = car_y - enddis;

	printf("ReverseTurn id=%d radius=%d goal_x=%d goal_y=%d \n", id, radius, goal_x, goal_y);

	// CameraTestSwitch=2;//12-17
	// VehicleState = 2;
	waiting_sum = 0;
	turn_start = 0;
	// task_end=1;
}
// 参数置零
// 刹车函数AGV2025
void Stop(void)
{
	unsigned char i;
	static u8 stop_flag = 1; // 只拉一次EPB

	Stop_OutTo_flag = 1;

	for (i = 0; i < 40; i++)
	{
		printf("Stop! AGV_getVehicle=%.1f\n", AGV_getVehicle);
		if (AGV_getVehicle > 0)
		{

			BrakePressure = 5 + i / 4;
			if (BrakePressure > 30)
				BrakePressure = 30;//3.14
			EHBControl(BrakePressure);
		}
		else
		{
			i = 40; // 11.8
		}

		delay_ms(15);
		stop_flag = 1;
	}

	VehicleState = 0;

	if (EPB_System_Status != 1 && stop_flag == 1)
	{
		printf("Meeting obstacle, using EPB!\r\n");
		EPBControl();
		stop_flag = 0;
	}
}

void ehbstop(void)//ZQY20251121
{
	int i;
//	for (i = 0; i < 1000; i++)
	for (i = 0; i < 80; i++)
	{
		HomeTrans();
		printf("Stop! AGV_getVehicle=%.1f\n", AGV_getVehicle);
		//if (AGV_getVehicle > 0)
		if (AGV_getVehicle != 0)
		{
			//BrakePressure = 20 + i / 4;
			BrakePressure = (20 + i) / 4;
			if (BrakePressure > 40)
				BrakePressure = 40;
			EHBControl(BrakePressure);
		}
		else
		{
			i = 80;
			//i = 1000;
		}


		delay_ms(10);
	}
	VehicleState = 0;
}

////紧急刹车函数
// EPB刹车函数
// 减速函数，减到goal_speed，EHB压力固定6
// 减速函数，减到goal_speed，EHB压力固定6
// 油门计算函数，根据task_line_status数值对应的行驶状态计算油门
// 计算当前期望方位角，即当前位置到预瞄点的角度
void CalNowAngle(void)
{
	int deta_x, deta_y;
	// deta_x x坐标差值，当前位置和预瞄点
	// deta_y y坐标差值，当前位置和预瞄点

	deta_x = preview_point_x - car_x;
	deta_y = preview_point_y - car_y;
	if (deta_x != 0)
	{
		now_angle = 1800 * atan((float)deta_y / (float)deta_x) / PI;

		if ((deta_x < 0) && (deta_y >= 0))
			now_angle = 3600 + now_angle; //?2?? 21
		if ((deta_x < 0) && (deta_y < 0))
			now_angle = 3600 + now_angle; //?3??
		if ((deta_x > 0) && (deta_y < 0))
			now_angle = 1800 + now_angle; //?4??
		if ((deta_x > 0) && (deta_y >= 0))
			now_angle = 1800 + now_angle; //?1??
	}
	 printf("***deta_x=%d deta_y=%d now_angle=%d  \n", deta_x, deta_y, now_angle);
}

// 计算直行修偏角度
void SteerAngleCal(void)
{
	int offset_angle; // offset_angle 当前方位角和期望方位角的差值
	cor_angle = 0;

	if (task_status == 3 || task_status == 5) // 偏差角 = 方位角 - 当前角度
	{
		int RearHeadingAngle;
		if (car_angle >= 1800)
			RearHeadingAngle = car_angle - 1800;
		else
			RearHeadingAngle = car_angle + 1800;
		offset_angle = RearHeadingAngle - now_angle;
	}
	else
	{
		offset_angle = car_angle - now_angle; // 偏差角 = 方位角 - 当前角度
	}

	// 将偏差角处理在-180到180区间
	if (offset_angle > 1800)
		offset_angle = offset_angle - 3600;
	if (offset_angle < -1800)
		offset_angle = offset_angle + 3600;

	if ((offset_angle > 10000) || (offset_angle < -10000))
		offset_angle = 0;

	if (offset_angle > 200 || offset_angle < -200)
		MaxCorAng = 10;
	else
		MaxCorAng = 7;

	// 偏差角大于滤波角，修偏角度为正
	if (offset_angle > FilterAngle)
	{
		cor_angle = (offset_angle - FilterAngle) / 15;
		if (cor_angle > MaxCorAng)
			cor_angle = MaxCorAng;
	}
	// 偏差角小于负滤波角，修偏角度为负
	if (offset_angle < -FilterAngle)
	{
		cor_angle = (offset_angle + FilterAngle) / 15;
		if (cor_angle < -MaxCorAng)
			cor_angle = -MaxCorAng;
	}

	printf("offset_angle=%d cor_angle=%d\n", offset_angle, cor_angle);
}

void SteerIMUAngleCal(void)
{
	int offset_angle = 0;
	offset_angle = Rs_Angle - imuinit_angle;

	// 将偏差角处理在-180到180区间
	if (offset_angle > 1800)
		offset_angle = offset_angle - 3600;
	if (offset_angle < -1800)
		offset_angle = offset_angle + 3600;

	if ((offset_angle > 10000) || (offset_angle < -10000))
		offset_angle = 0;

	if (offset_angle > 200 || offset_angle < -200)
		MaxCorAng = 6;
	else
		MaxCorAng = 3;

	// 偏差角大于滤波角，修偏角度为正
	if (offset_angle > FilterAngle)
	{
		cor_angle = (offset_angle - FilterAngle) / 15;
		if (cor_angle > MaxCorAng)
			cor_angle = MaxCorAng;
	}
	// 偏差角小于负滤波角，修偏角度为负
	if (offset_angle < -FilterAngle)
	{
		cor_angle = (offset_angle + FilterAngle) / 15;
		if (cor_angle < -MaxCorAng)
			cor_angle = -MaxCorAng;
	}
	printf("IMU offset_angle=%d cor_angle=%d\n", offset_angle, cor_angle);
}

// 计算预瞄点
void PreviewPointCal(unsigned char car_direct)
{
	// 分别按终止条件为x轴正向、x轴负向、y轴正向、y轴负向计算
	if (car_direct == 11)
	{
		preview_point_x = rslidar_x + 200; // 9-16
		preview_point_y = start_y + (float)(goal_y - start_y) * (float)(preview_point_x - start_x) / (float)(goal_x - start_x);
	}
	if (car_direct == 12)
	{
		preview_point_x = rslidar_x - 200; // 9-16
		preview_point_y = start_y + (float)(goal_y - start_y) * (float)(preview_point_x - start_x) / (float)(goal_x - start_x);
	}
	if (car_direct == 21)
	{
		preview_point_y = rslidar_y + 200; // 9-16
		preview_point_x = start_x + (float)(goal_x - start_x) * (float)(preview_point_y - start_y) / (float)(goal_y - start_y);
	}
	if (car_direct == 22)
	{
		preview_point_y = rslidar_y - 200; // 9-16
		preview_point_x = start_x + (float)(goal_x - start_x) * (float)(preview_point_y - start_y) / (float)(goal_y - start_y);
	}
	 printf("***preview_point_x=%d preview_point_y=%d ", preview_point_x, preview_point_y);
}

// 行驶故障检测函数，0-故障，1-正常
// 以当前车辆位姿信息作为起点位置和角度AGV2025
void StartPositionAssignment(void) // 确定直线的出发点
{
	int start_offset = 0;
	u8 start_flag = 0;
	if (Release_flag == 1 && TaskCount == 2)
	{
//		start_x = Release_x;
//		start_y = Release_y;
		start_x =car_x;
		start_y =car_y;
		Release_flag = 0;
		start_flag = 1;
		printf("start_x =car_x start_y =car_y ");
	}
	else
	{
		start_offset = (pass_goal_x - car_x) * (pass_goal_x - car_x) + (pass_goal_y - car_y) * (pass_goal_y - car_y);
		if (start_offset < 6000)
		{
			start_x = pass_goal_x;
			start_y = pass_goal_y;
			start_flag = 2;
		}
		else
		{
			start_x = car_x;
			start_y = car_y;
			start_flag = 3;
		}
	}
	start_angle = car_angle;

	printf("*start_x=%d start_y=%d start_flag=%d start_offset=%d\n", start_x, start_y, start_flag, start_offset);
}

// 发出空的EPS控制指令，防止车辆不确定运动
void ZeroEPSControlCode(void)
{

	Expect_EPS_Angle = 0;
	EPSControl();
}

// 行驶声音和部分设备检测,TT-任务类型，1直行，2转向
// 根据终止条件判别是否结束直行
void EndLineJudge(int length)
{
	u8 region = 30;
	static u8 flag = 1;
	if (length < 160000 && flag == 1)
	{
			EndPointCorrection = 5; // 刹车提前量

		flag = 0;
	}
	else
	{
		EndPointCorrection = 0;
		flag = 1;
	}

	if ((car_direct == 11) && (car_x >= goal_x - EndPointCorrection) && (car_y < goal_y + region | car_y > goal_y - region))
	{
		// task_status=0;
		task_status = 9;
		line_start = 0;
		arrive = 1;
		// task_end=0;
	}
	if ((car_direct == 12) && (car_x <= goal_x + EndPointCorrection) && (car_y < goal_y + region | car_y > goal_y - region))
	{
		// task_status=0;
		task_status = 9;
		line_start = 0;
		arrive = 1;
		// task_end=0;
	}
	if ((car_direct == 21) && (car_y >= goal_y - EndPointCorrection) && (car_x > goal_x - region | car_x < goal_x + region))
	{
		// task_status=0;
		task_status = 9;
		line_start = 0;
		arrive = 1;
		// task_end=0;
	}
	if ((car_direct == 22) && (car_y <= goal_y + EndPointCorrection) && (car_x > goal_x - region | car_x < goal_x + region))
	{
		// task_status=0;
		task_status = 9;
		line_start = 0;
		arrive = 1;
		// task_end=0;
	}
}

void EndIMUJudge(void)
{
	imuruncnt++;
	if (imuruncnt >= imucnt)
	{
		Stop();
		remote_control_status = 1;
		//		BCMControl(21); // 声音“遥控”
		printf("IMU have arrive, but lidar matching lose!\r\n");
	}
}

// AGV直行函数
int Line(void)
{
	unsigned char i, DriveFault = 1;
	// int ExpectSpeed;
	static u8 tim = 0;
	// DriveFault 行驶故障，0-故障，1-正常，包括：GPS丢失、激光雷达故障、相机故障等
	// printf("task line line_count=%d goal_x=%d goal_y=%d\r\n", line_count, goal_x, goal_y);
	// line_start=0，直行初始化

	if (line_start == 0)//AGV获取任务参数后就是0
	{
		StartPositionAssignment(); // 获取起点位置和角度     AGV可以

		car_direct = EndTaskConditions(); // 计算终止条件,得到 car_direct  重要！！！  AGV可以

		Expect_EPS_Angle = 0; // 初始给0度

		line_start = 1;
		if (VehicleState != 1)//AGV好像没用
			line_count = 1;
	}

	// 当line_start=1，开始直行
	else
	{

		HomeTrans();//AGV
		if (EPB_System_Status == 1)
		{
			epbcount = 0;
			EPBControl1();
		} // 如果EPB拉起，释放EPB
		length = (car_x - goal_x) * (car_x - goal_x) + (car_y - goal_y) * (car_y - goal_y); // 计算离目标点的距离
		if (task_status == 3 || task_status == 5)
		Expect_Gear = 0x1; // 后退档
		else if (task_status == 1 || task_status == 2)
		Expect_Gear = 0x2; // 前进挡

		if ((length < 40000) )
		{
			VehicleState = 1;
			printf("slow!!!!");
			Expect_speed=15;//2025即1.5公里每小时
			if(car_x<=200&&car_x>=-200&&car_y<=200&&car_y>=-200)//2025926
			{
			printf("slow  3!!");
			Expect_speed=3;
			//shield_flag=1;//2025921
			}
		}
		else
		{
			if (matchingcnt > 1 | lidardatalose > 10)//略微超差或者上个数据和下个数据一样
			{
				printf("##0chaocha  slow！！！: ");
       Expect_speed=20;//2025即1公里每小时
			}
			else
			{
				printf("##2: ");
				Expect_speed=20;//2025DrivingSpeedAGV
			}
		}

		VehicleState = 1; // 置车辆状态为1，直行状态

		if (imuflag == 0) //********lidar给坐标在走
		{
			imuruncnt = 0;
			PreviewPointCal(car_direct); // 预瞄点
			CalNowAngle();				 // 计算当前角度
			SteerAngleCal();			 // 计算偏差角=航向角-自身角  //AGV好像可以不用改
		}
		else //*******惯导给角度在走
		{
			SteerIMUAngleCal();
		}
		Yaw();		  // 计算期望车轮转角
		EPSControl(); // EPS指令//AGV好像可以不用改




		if (imuflag == 0)//此条件必成立
			EndLineJudge(length); // 根据终止条件判别是否结束直行
		else
			EndIMUJudge();
	}
	return DriveFault;
}

// 转弯结束判别
void EndTurnJudge(void)
{
	u8 pre_end_dis = 20;								   // 出弯点提前20cm
	if ((turn_id == 1) && (car_x >= goal_x - pre_end_dis)) // 左转x增加
	{
		turn_start = 1;
		printf("##1: outturn_x=%d  outturn_y=%d\r\n", goal_x, goal_y - turnradius + 200);
	}
	if ((turn_id == 2) && (car_x <= goal_x + pre_end_dis)) // 左转X减小
	{
		turn_start = 1;
		printf("##2: outturn_x=%d  outturn_y=%d\r\n", goal_x, goal_y + turnradius - 200);
	}
	if ((turn_id == 3) && (car_y >= goal_y - pre_end_dis)) // 左转y增加
	{
		turn_start = 1;
		printf("##3: outturn_x=%d  outturn_y=%d\r\n", goal_x + turnradius - 200, goal_y);
	}
	if ((turn_id == 4) && (car_y <= goal_y + pre_end_dis)) // 左转y减小
	{
		turn_start = 1;
		printf("##4: outturn_x=%d  outturn_y=%d\r\n", goal_x - turnradius + 200, goal_y);
	}
	if ((turn_id == 5) && (car_x >= goal_x - pre_end_dis))
	{
		turn_start = 1;
		printf("##5: outturn_x=%d  outturn_y=%d\r\n", goal_x, goal_y + turnradius - 200);
	}
	if ((turn_id == 6) && (car_x <= goal_x + pre_end_dis))
	{
		turn_start = 1;
		printf("##6: outturn_x=%d  outturn_y=%d\r\n", goal_x, goal_y - turnradius + 200);
	}
	if ((turn_id == 7) && (car_y >= goal_y - pre_end_dis))
	{
		turn_start = 1;
		printf("##7: outturn_x=%d  outturn_y=%d\r\n", goal_x - turnradius + 200, goal_y);
	}
	if ((turn_id == 8) && (car_y <= goal_y + pre_end_dis))
	{
		turn_start = 1;
		printf("##8: outturn_x=%d  outturn_y=%d\r\n", goal_x + turnradius - 200, goal_y);
	}
}

// 倒车转弯结束判别
// 转弯结束判别
void REndTurnJudge(void)
{
	u8 pre_end_dis = 0;									   // 出弯点提前20cm
	if ((turn_id == 1) && (car_x >= goal_x - pre_end_dis)) // 左转x增加
	{
		turn_start = 1;
		printf("##1: outturn_x=%d  outturn_y=%d\r\n", goal_x, goal_y - turnradius + 200);
	}
	if ((turn_id == 2) && (car_x <= goal_x + pre_end_dis)) // 左转X减小
	{
		turn_start = 1;
		printf("##2: outturn_x=%d  outturn_y=%d\r\n", goal_x, goal_y + turnradius - 200);
	}
	if ((turn_id == 3) && (car_y >= goal_y - pre_end_dis)) // 左转y增加
	{
		turn_start = 1;
		printf("##3: outturn_x=%d  outturn_y=%d\r\n", goal_x + turnradius - 200, goal_y);
	}
	if ((turn_id == 4) && (car_y <= goal_y + pre_end_dis)) // 左转y减小
	{
		turn_start = 1;
		printf("##4: outturn_x=%d  outturn_y=%d\r\n", goal_x - turnradius + 200, goal_y);
	}
	if ((turn_id == 5) && (car_x >= goal_x - pre_end_dis))
	{
		turn_start = 1;
		printf("##5: outturn_x=%d  outturn_y=%d\r\n", goal_x, goal_y + turnradius - 200);
	}
	if ((turn_id == 6) && (car_x <= goal_x + pre_end_dis))
	{
		turn_start = 1;
		printf("##6: outturn_x=%d  outturn_y=%d\r\n", goal_x, goal_y - turnradius + 200);
	}
	if ((turn_id == 7) && (car_y >= goal_y - pre_end_dis))
	{
		turn_start = 1;
		printf("##7: outturn_x=%d  outturn_y=%d\r\n", goal_x - turnradius + 200, goal_y);
	}
	if ((turn_id == 8) && (car_y <= goal_y + pre_end_dis))
	{
		turn_start = 1;
		printf("##8: outturn_x=%d  outturn_y=%d\r\n", goal_x + turnradius - 200, goal_y);
	}
}

// 转弯车轮回正段,tt任务类型，1-转向，2-倒车转向
void SectinReturn(unsigned char TT)
{
	Expect_EPS_Angle = 0;

	//ThrottleCal(VehicleSpeed, 1);
	Expect_speed=10;
	
	if (TT == 1)
		EPSControl();
	if (TT == 2)
		ReverseControl();
	turn_start = turn_start + 1;
	if (turn_start > 5)
	{
		GetPosition();
		task_status = 9;
		turn_start = 0;
		arrive = 1;
		goal_x = car_x;
		goal_y = car_y;
		// task_end=0;
	}
}

// 计算转弯修正角度
void CorAngCal(int ExpRadius, int RealRadius)
{
	int CorAngC;
	if ((turn_id <= 4) && (turn_id >= 1))
	{
		if (task_status == 2)
		{
			CorAngC = -(RealRadius - ExpRadius) / 10;
		}
		else
		{

			CorAngC = (RealRadius - ExpRadius) / 10;
		}
	}
	if ((turn_id <= 8) && (turn_id >= 5))
	{
		if (task_status == 2)
		{
			CorAngC = (RealRadius - ExpRadius) / 10;
		}
		else
		{

			CorAngC = -(RealRadius - ExpRadius) / 10;
		}
	}

	if (CorAngC > 8)
		CorAngC = 8;
	if (CorAngC < -8)
		CorAngC = -8;

	Expect_EPS_Angle = BaseAngle + CorAngC; // 固定转角后开始修改

	if (Expect_EPS_Angle > 45)
		Expect_EPS_Angle = 45;
	if (Expect_EPS_Angle < -45)
		Expect_EPS_Angle = -45;
	printf("  CorAngC=%d  Expect_EPS_Angle=%d\r\n", CorAngC, Expect_EPS_Angle);
}

// 转弯
int Turn(void)
{
	unsigned char GPSLose = 1, i, DriveFault = 1;
	// GPSLose GPS丢失情况，0-丢失，1-正常
	int detaAngleTurn, PedalDepthCal, LenC, CorAng, ExpectSpeed; // 10-1
	 HomeTrans();
	if (turn_start == 0)//获取完就是0
	{
		EPB12Trans();
		VehicleState = 2;
		if (EPB_System_Status == 1)
			EPBControl1();

		LenC = DistanceCenterCal(car_x, car_y, turnCenterX, turnCenterY); // 到转弯中心距离

//		Expect_EPS_Angle = 0;
		printf("##0:Expect_EPS_Angle=%d\r\n", Expect_EPS_Angle);

		CorAngCal(radius, LenC); // 修正角度CorAngC

		printf("wheel_angle=%d Expect_EPS_Angle=%d turn_id=%d\r\n", wheel_angle, Expect_EPS_Angle, turn_id);


		printf("##2: ");
		if (task_status == 3 || task_status == 5)
		Expect_Gear = 0x1; // 后退档
	  else if (task_status == 1 || task_status == 2)
		Expect_Gear = 0x2; // 前进挡
		Expect_speed=DrivingSpeed;//2025DrivingSpeed单位为km/h
		EPSControl();


		EndTurnJudge(); // 转弯结束判别
	}

	//********Steering return section
	if ((turn_start > 0) && (turn_start < 11))
		SectinReturn(1); // 转弯车轮回正段

	return DriveFault;
}

// 倒车
// 倒车转弯
int ReverseTurn(void) // 8-26
{
	unsigned char DriveFault = 1;
	int ExpectSpeed = 0, LenC = 0;
   HomeTrans();
	// GPSLose GPS丢失情况，0-丢失，1-正常
	if (turn_start == 0)
	{
		EPB12Trans();
		if (EPB_System_Status == 1)
		{
			epbcount = 0;
			EPBControl1();
		}

		LenC = DistanceCenterCal(car_x, car_y, turnCenterX, turnCenterY); // 到转弯中心距离
		Expect_EPS_Angle = 0;
		VehicleState = 4;
		CorAngCal(radius, LenC); // 修正角度CorAngC
		printf("RETURN:wheel_angle=%d Expect_EPS_Angle=%d turn_id=%d\r\n", wheel_angle, Expect_EPS_Angle, turn_id);

		if (task_status == 3 || task_status == 5)
		 Expect_Gear = 0x1; // 后退档
		else if (task_status == 1 || task_status == 2)
		 Expect_Gear = 0x2; // 前进?
		Expect_speed=DrivingSpeed;
		ReverseControl();

		REndTurnJudge(); // 转弯结束判别
	}
	if ((turn_start > 0) && (turn_start < 11))
		SectinReturn(2); // 转弯车轮回正段
	return DriveFault;
}

// Calculate the heading from the current position to the target position
// 结束条件判别，11-x轴正向，12-x轴负向，21-y轴正向，22-y轴负向
int EndTaskConditions(void)
{

	int deta_x, deta_y, redata;

	deta_x = goal_x - start_x;
	deta_y = goal_y - start_y;
	if (deta_x < 0)
		deta_x = -deta_x;
	if (deta_y < 0)
		deta_y = -deta_y;

	// printf("deta_x=%d  deta_y=%d\n",deta_x,deta_y);

	if (deta_x > deta_y)
	{
		if (goal_x - start_x >= 0)
			redata = 11;
		else
			redata = 12;
	}
	else
	{
		if (goal_y - start_y >= 0)
			redata = 21;
		else
			redata = 22;
	}
	printf("goal_x-start_x=%d goal_y-start_y=%d  car_direct=%d \n", goal_x - start_x, goal_y - start_y, redata);
	return redata;
}

// 从CAN读取MCU1数据
// 从CAN读取MCU2数据
// 从CAN读取Lidar数据
//void LidarRead(void)
//{
//	static u8 lidarreadcnt = 0;
//	// Lidar_Updata=0;
//	if (Lidarflag == 1)
//	{
//		LidarTrans();
//		lidarreadcnt++;
//		Lidarflag = 0;
//		lidarLossNum = 0; // 8-29
//		if (lidarreadcnt > 2)
//		{
//			if (lidarLoss == 1)
//				printf("##One line Lidar has recovery!\r\n");
//			lidarLoss = 0;
//		}
//	}
//	else if (LidarObstacle == 0)
//	{
//		lidarreadcnt = 0;
//		lidarLossNum++;
//		if (lidarLossNum > 30)
//		{
//			lidarLoss = 1;
//			//		  BCMControl(30);//F系列避障系统故障
//		}
//		else
//			lidarLoss = 0;
//		printf("lidarLossNum=%d  lidarLoss=%d  \r\n", lidarLossNum, lidarLoss);
//	}
//	if (lidarreadcnt > 100)
//		lidarreadcnt = 0;
//	if (lidarLossNum > 200)
//		lidarLossNum = 100;
//}

//void LidarRead(void)//新版
//{

//	if (USART5_RX_END == 1)
//	{
//		newLidarTrans();
//		
//	}
//  if (LidarObstacle == 0)
//	{
//	
//		lidarLossNum++;
//		if (lidarLossNum > 300)
//		{
//			lidarLoss = 1;
//			//		  BCMControl(30);//F系列避障系统故障
//		}
//		else
//			lidarLoss = 0;
//		printf("lidarLossNum=%d  lidarLoss=%d  \r\n", lidarLossNum, lidarLoss);
//	}

//	if (lidarLossNum > 400)
//		lidarLossNum = 100;
//}

void LidarRead(void) // 6.23版
{
//        newLidarTrans();
     newcanLidarTrans();
		if (Lidar_Status == 0)
		{
		  lidarLoss = 0;
			lidarLossNum=0;
		
		}
    else
    {
     
        //if (Lidarflag > 50)//202592
        //{
            lidarLoss = 1;     // MODIFIED: ??????,?????
        //}
        //else
        //{
          // lidarLoss = 0;     // MODIFIED: ????,?????
        //}
        printf("Lidar_Status=%d  lidarLoss=%d  \r\n", Lidar_Status, lidarLoss);
    }

    if (Lidarflag > 100)//2025926
    {
        lidarLoss = 1;
			Length_Obstacle=0;
			Length_Obstacle1=0;

			
			printf("Lidarflag=%d",Lidarflag);
    }

		
}



// 从CAN读取Lidar1数据
// 从CAN读取EHB数据
// 从CAN读取EPS数据
// 从CAN读取EPB数据
// 从CAN读取Camera数据
// Determine the steering according to the calculated deviation, 1-turn right, 2-turn left, 0-straight
// the function used to calculate the correction deviation angle
void Yaw(void)
{
if (task_status == 1)
{
	if (cor_angle > 0)
	{
		Expect_EPS_Angle = cor_angle * 2 + compensate_angle;
		printf("%%  right  cor_angle=%d %%", cor_angle);
	}
	else if (cor_angle < 0)
	{
		Expect_EPS_Angle = cor_angle * 2;
		printf("%%  left  cor_angle=%d %%", cor_angle);
	}
	else
	{
		Expect_EPS_Angle = compensate_angle;
	}
}
if (task_status == 3)
{
		if (cor_angle > 0)
	{
		Expect_EPS_Angle = -(cor_angle * 2 + compensate_angle);
		printf("%%  right  cor_angle=%d %%", cor_angle);
	}
	else if (cor_angle < 0)
	{
		Expect_EPS_Angle = -cor_angle * 2;
		printf("%%  left  cor_angle=%d %%", cor_angle);
	}
	else
	{
		Expect_EPS_Angle = 0;
	}
}
}

// the function used to calculate the correction deviation angle when reversing
// Remote turn funtion, 1-29 turn left, 31-60 turn right
// 激光雷达自检
void RevokeControl(void)
{
//if (remote_control_status == 0)//2025非遥控状态撤回订单才刹车
//	{
//		Stop();
//	}
	Taskokreset();
	Task_Count = 0;
	task_status = 0;  // 需要重新接收任务，当前任务类型
	TaskCount = 0;	  // 当前第几个任务
	TaskTotal = 0;	  // 任务总数
	task_flag = 0;	  // 任务开关

}

void Taskokreset(void)
{
	turn_num = 0;
	Reverse_count = 0;
	ReverTurn_count = 0;
	if (VehicleState != 1)
	{
		line_count = 0;
		heavy_flag = 1; // 重量油门标志位置
	}
}

void MCUSend(void)
{
	MCU_Control_Model = 2;
	MCU1_Counter = (MCU1_Counter + 1) % 4;
	MCU1canbuf[6] = MCU_Control_Model * 4 + MCU1_Counter * 16;
	MCU1canbuf[7] = MCU1canbuf[6];

	MotorControlStatus = 1;
	MCU2canbuf[0] = MotorControlStatus * 32;

	Can2_Send_Msg_Flag(0x0, 0x0CF008FB, canbuf, 8);
	//  Can2_Send_Msg_Flag(0x0, 0X18F900EF, MCU1canbuf, 8);
	//	Can2_Send_Msg_Flag(0x0, 0X18F560EF, MCU2canbuf, 8);
}

int16_t U1_distance, U2_distance, U3_distance, U4_distance;
int8_t ULtraObstacle;
int8_t firstObstacleFlag = 1;
int NewReverse(void)
{
	unsigned char DriveFault;
	HomeTrans();
	if (line_start == 0)
	{
		printf("task Reverse line_count=%d goal_x=%d goal_y=%d\n", line_count, goal_x, goal_y);
		StartPositionAssignment(); // 获取起点位置和角度
		car_direct = EndTaskConditions();
		ZeroEPSControlCode(); // 发出空的EPS控制指令，防止车辆不确定运动
		if (VehicleState != 3)
			Reverse_count = 1;
		DriveFault = 1;
		line_start = 1;
	}
	else
	{
		printf("Reverseing3");
		VehicleState = 3;

		if (EPB_System_Status == 1)
			EPBControl1();
		length = (car_x - goal_x) * (car_x - goal_x) + (car_y - goal_y) * (car_y - goal_y);
		if (task_status == 3 || task_status == 5)
		 Expect_Gear = 0x1; // 后退档
	  else if (task_status == 1 || task_status == 2)
		 Expect_Gear = 0x2; // 前进挡


		if (length < 40000)
		{
			printf("slow!!!!");
			Expect_speed=10;
		}
		else
			Expect_speed=DrivingSpeed;//2025DrivingSpeed单位为km/h
		printf("DrivingSpeed=%d",DrivingSpeed);
		PreviewPointCal(car_direct); // 预瞄点
		CalNowAngle();				 // 计算当前角度
		SteerAngleCal();			 // 计算偏差角=航向角-自身角


		if (cor_angle > 0)
		{
			Expect_EPS_Angle = -(cor_angle * 2 + compensate_angle);
			printf("%%  right  cor_angle=%d %%", cor_angle);
		}
		else if (cor_angle < 0)
		{
			Expect_EPS_Angle = -cor_angle * 2;
			printf("%%  left  cor_angle=%d %%", cor_angle);
		}
		else
		{
			Expect_EPS_Angle = 0;
		}
		EPSControl(); // EPS指令
		EndLineJudge(length);
	}
	return DriveFault;
}

