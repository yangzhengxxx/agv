#include "4g_comnet.h"
#include "bsp_debug_usart.h"
#include "run.h"
///////////////////////////网络接收发送数据变量//////////////////////////////////////////////////////////////////
/////路径点序列
s32 StartpointX; //起点坐标
s32 StartpointY;
s32 Startdir;		   //起点方向
s32 CurpointX = 10620; //当前点坐标
s32 CurpointY = 111480;
s32 Curdir;		///当前点方向
s32 NextpointX; //下个点坐标
s32 NextpointY;
s32 Nextdir;		   ///下个点方向
s32 CurPathNo = 1;	   ///当前路径编号
s32 NextPathNo = 1;	   ///下条路径编号
u8 arrive;			   ////到达目标位置标记，0：没到达 1：到达；在串口头文件中定义
static u8 temflag = 0; //小车当前状态标志
u8 Isgoalop = 0;	   //目标位置是否有操作0为没有，1为有
u8 PointNum = 1;
static u8 Release_PointNum = 1;
u8 Task_Count = 0; //任务点数量计数
u8 Checksum_Error = 0;
u8 release_flag = 0;
u8 remote_release_flag=0;
u8 Sysrelease_flag = 0;
u32 link_count = 0;	 //与调度失联次数
u8 task_conflag = 0; //调度订单任务是否连续标志
int k = 0x07;		 //小车状态标识，2为错误,3为空闲，4为执行,5为充电,7为遥控

extern char *str_imei; // 4G模块序列号
char atstr2[200];
int lastpointbuf1 = 0;
int lastpointbuf2 = 0;
int lastpointbuf3 = 0;
int lastReleasepointbuf1 = 0;
int lastReleasepointbuf2 = 0;
///////////////////
///////////////////***FLASH
//要写入到STM32 FLASH的字符串数组
u8 flashbuf[20][8];
#define TEXT_LENTH 320 //数组长度
#define SIZE TEXT_LENTH / 4 + ((TEXT_LENTH % 4) ? 1 : 0)
#define FLASH_SAVE_ADDR 0X0800C004 //设置FLASH 保存地址(必须为偶数，且所在扇区,要大于本代码所占用到的扇区. \
								   //否则,写操作的时候,可能会导致擦除整个扇区,从而引起部分程序丢失.引起死机.
u8 datatemp[SIZE];
///////////////////

u8 t8266revbuf[24] = {0x63, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

u8 oksendbuf1[24] = {'a', 'o', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 oksendbuf2[24] = {'b', 'p', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 oksendbuf3[24] = {'c', 'q', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 oksendbuf4[24] = {'s', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 oksendbuf5[24] = {'d', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 oksendbuf6[24] = {'n', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 oksendbuf7[24] = {'m', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 oksendbuf8[24] = {'r', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 oksendbuf9[24] = {'f', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 oksendbuf10[24] = {'g', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 oksendbuf11[24] = {'k', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

u8 cangpsbuf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

int16_t cangpsbuf2[100][12]; //存储整段任务点AGV
u8 release_canbuf[20][10];

int16_t pointbuf[100][12];
u8 release_pointbuf[20][24]; //释放点数组  4.17扩大
u16 agvport = 2001;			 ///车载端从2001开始，调度系统端从5001开始，两者之间相差3000；
trandata tmppt;
u8 fflag = 0;






//五菱
#define SEQ1_LEN    3
#define SEQ2_LEN    11
#define SEQ3_LEN    11
#define SEQ4_LEN    11
#define SEQ5_LEN    11  // ?10,?1
#define SEQ6_LEN    11  // ?10,?1
#define SEQ7_LEN    11  // ?14,?1
#define SEQ8_LEN    5  // ?14,?1
#define SEQ9_LEN    5  // ?14,?1
#define SEQ10_LEN   5  // ?15,?1
#define SEQ11_LEN   5  // ?14,?1
#define SEQ12_LEN   5  // ?14,?1
#define SEQ13_LEN   5  // ?14,?1
#define SEQ14_LEN   7  // ?14,?1
#define SEQ15_LEN   7
#define SEQ16_LEN   7
#define SEQ17_LEN   7
#define SEQ18_LEN   7
#define SEQ19_LEN   7
#define SEQ20_LEN   13
#define SEQ21_LEN   13
#define SEQ22_LEN   13
#define SEQ23_LEN   13
#define SEQ24_LEN   13
#define SEQ25_LEN   13
#define SEQ26_LEN   3
#define SEQ27_LEN   1  // ?10,?1
#define SEQ28_LEN   1  // ?10,?1
#define SEQ29_LEN   1  // ?14,?1
#define SEQ30_LEN   1  // ?14,?1
#define SEQ31_LEN   1  // ?14,?1
#define SEQ32_LEN   1  // ?14,?1
#define SEQ33_LEN   1  // ?14,?1
#define SEQ34_LEN   1
#define SEQ35_LEN   1
#define SEQ36_LEN   1
#define SEQ37_LEN   1   // ?8,?1
#define SEQ38_LEN   1   // ?8,?1
#define SEQ39_LEN   1  // ?12,?1
#define SEQ40_LEN   1  // ?12,?1
#define SEQ41_LEN   1  // ?12,?1

int16_t AGVpointbuf[100][12];
u8 AGVtasklimit=0;//防止任务一直赋值更新
u8 AGVupsendbuf[12] = {'a', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

const int16_t TASK_SEQUENCE_1[SEQ1_LEN][8] =  {
    {3, -126, 93, 0, 10, 0, 0, 0},
    {5, -457, 189, 0, 10, 7, 7, 25},
    {1, -449, -994,0, 20, 0, 0, 0},//充电-上料缓冲
};
/*
const int16_t TASK_SEQUENCE_1[SEQ1_LEN][8] =  {
    {3, -146, -78, 0, 10, 0, 0, 0},
    {5, -457, 18, 0, 10, 7, 11, 25},
    {1, -409, -2746,0, 20, 0, 0, 0},
    {2, -113, -2843, 0, 10, 1, 18, 25}, 
    {1, 2998, -2851, 0, 20, 0, 0, 0},//任务1直行终点
    {2, 3101, -3154, 0, 10, 8, 17, 25},
    {1, 3189, -7521, 0, 20, 0, 0, 0},
    {2, 3481, -7629, 0, 10, 1, 20, 25},
    {1, 4341, -7624, 0, 20, 0, 0, 0},
    {1, 5382, -7636, 0, 20, 0, 0, 0},
    {2, 5497,-7333 , 0, 10, 3, 20, 25},
    {3, 5508, -8234, 0, 15, 0, 0, 0}//充电-原材料上货 
};
*/
const int16_t TASK_SEQUENCE_2[SEQ2_LEN][8] = {
    {1, -429, -2706,0, 20, 0, 0, 0},
    {2, -113, -2843, 0, 10, 1, 18, 25}, 
    {1, 2948, -2851, 0, 20, 0, 0, 0},
		{1, 6295, -2829, 0, 20, 0, 0, 0},/*ZQY6295*/
    {2, 6432, -2522, 0, 10, 3, 20, 25},
		{1, 6487, 53, 0, 20, 0, 0, 0},//马超
    {1, 6424, 2727, 0, 20, 0, 0, 0},
		{2, 6105, 2864, 0, 10, 2, 21, 25},
		{1, 4595, 2875, 0, 20, 0, 0, 0},/*ZQY4525--4595*/
		{2, 4437, 2578, 0, 10, 4, 22, 25},/*ZQY20--22*/
		{1, 4453, 1582, 0, 20, 0, 0, 0}//上料缓冲-2-1
};

const int16_t TASK_SEQUENCE_3[SEQ3_LEN][8] = {
		{1, -429, -2706,0, 20, 0, 0, 0},
    {2, -113, -2843, 0, 10, 1, 18, 25}, 
    {1, 2948, -2851, 0, 20, 0, 0, 0},
		{1, 6295, -2829, 0, 20, 0, 0, 0},/*ZQY6295*/
    {2, 6432, -2522, 0, 10, 3, 20, 25},
		{1, 6487, 53, 0, 20, 0, 0, 0},//马超
    {1, 6424, 2727, 0, 20, 0, 0, 0},
		{2, 6105, 2864, 0, 10, 2, 21, 25},
		{1, 4595, 2875, 0, 20, 0, 0, 0},/*ZQY4525--4595*/
		{2, 4437, 2578, 0, 10, 4, 22, 25},/*ZQY20--22*/
		{1, 4467, 605, 0, 20, 0, 0, 0}//上料缓冲-2-2
};
const int16_t TASK_SEQUENCE_4[SEQ4_LEN][8] = {
		{1, -429, -2706,0, 20, 0, 0, 0},
    {2, -113, -2843, 0, 10, 1, 18, 25}, 
    {1, 2948, -2851, 0, 20, 0, 0, 0},
		{1, 6295, -2829, 0, 20, 0, 0, 0},/*ZQY6295*/
    {2, 6432, -2522, 0, 10, 3, 20, 25},
		{1, 6487, 53, 0, 20, 0, 0, 0},//马超
    {1, 6424, 2727, 0, 20, 0, 0, 0},
		{2, 6105, 2864, 0, 10, 2, 21, 25},
		{1, 4595, 2875, 0, 20, 0, 0, 0},/*ZQY4525--4595*/
		{2, 4437, 2578, 0, 10, 4, 22, 25},/*ZQY20--22*/
		{1, 4466, -153, 0, 20, 0, 0, 0}//上料缓冲-2-3
};
const int16_t TASK_SEQUENCE_5[SEQ5_LEN][8] = {
		{1, -429, -2706,0, 20, 0, 0, 0},
    {2, -113, -2843, 0, 10, 1, 18, 25}, 
    {1, 2948, -2851, 0, 20, 0, 0, 0},
		{1, 6295, -2829, 0, 20, 0, 0, 0},/*ZQY6295*/
    {2, 6432, -2522, 0, 10, 3, 20, 25},
		{1, 6487, 53, 0, 20, 0, 0, 0},//马超
    {1, 6424, 2727, 0, 20, 0, 0, 0},
		{2, 6105, 2864, 0, 10, 2, 21, 25},
		{1, 4595, 2875, 0, 20, 0, 0, 0},/*ZQY4525--4595*/
		{2, 4437, 2578, 0, 10, 4, 22, 25},/*ZQY20--22*/
		{1, 4481, -1060, 0, 20, 0, 0, 0}//上料缓冲-2-4
};
const int16_t TASK_SEQUENCE_6[SEQ6_LEN][8] = {
		{1, -429, -2706,0, 20, 0, 0, 0},
    {2, -113, -2843, 0, 10, 1, 18, 25}, 
    {1, 2948, -2851, 0, 20, 0, 0, 0},
		{1, 6295, -2829, 0, 20, 0, 0, 0},/*ZQY6295*/
    {2, 6432, -2522, 0, 10, 3, 20, 25},
		{1, 6487, 53, 0, 20, 0, 0, 0},//马超
    {1, 6424, 2727, 0, 20, 0, 0, 0},
		{2, 6105, 2864, 0, 10, 2, 21, 25},
		{1, 4595, 2875, 0, 20, 0, 0, 0},/*ZQY4525--4595*/
		{2, 4437, 2578, 0, 10, 4, 22, 25},/*ZQY20--22*/
		{1, 4483, -1701, 0, 20, 0, 0, 0}//上料缓冲-2-5
};
const int16_t TASK_SEQUENCE_7[SEQ7_LEN][8] = {
		{1, -429, -2706,0, 20, 0, 0, 0},
    {2, -113, -2843, 0, 10, 1, 18, 25}, 
    {1, 2948, -2851, 0, 20, 0, 0, 0},
		{1, 6295, -2829, 0, 20, 0, 0, 0},/*ZQY6295*/
    {2, 6432, -2522, 0, 10, 3, 20, 25},
		{1, 6487, 53, 0, 20, 0, 0, 0},//马超
    {1, 6424, 2727, 0, 20, 0, 0, 0},
		{2, 6105, 2864, 0, 10, 2, 21, 25},
		{1, 4595, 2875, 0, 20, 0, 0, 0},/*ZQY4525--4595*/
		{2, 4437, 2578, 0, 10, 4, 22, 25},/*ZQY20--22*/
		{1, 4493, -2225, 0, 20, 0, 0, 0}//上料缓冲-2-6
};
const int16_t TASK_SEQUENCE_8[SEQ8_LEN][8] = {
		{1, 4493, -2700, 0, 20, 0, 0, 0},
		{2, 4192, -2830, 0, 10, 6, 23, 25},
		{1, -265, -2873, 0, 20, 0, 0, 0},
    {2, -422, -2575, 0, 10, 7, 22, 25},
    {1, -449, -994, 0, 20, 0, 0, 0}//2-1-上料缓冲
};
const int16_t TASK_SEQUENCE_9[SEQ9_LEN][8] = {
		{1, 4493, -2700, 0, 20, 0, 0, 0},
		{2, 4192, -2830, 0, 10, 6, 23, 25},
		{1, -265, -2873, 0, 20, 0, 0, 0},
    {2, -422, -2575, 0, 10, 7, 22, 25},
    {1, -449, -994, 0, 20, 0, 0, 0}//2-2-上料缓冲
};
/*
const int16_t TASK_SEQUENCE_9[SEQ9_LEN][8] = {
		{1, 5523, -7817, 0, 10, 0, 0, 0},//需要再采一下
    {2, 5222, -7627, 0, 10, 2, 18, 26},
    {1, 4392, -7630, 0, 20, 0, 0, 0},
    {1, 3305, -7657, 0, 20, 0, 0, 0},
    {2, 3193, -7349, 0, 10, 7, 19, 25},
    {1, 3110, -2988, 0, 20, 0, 0, 0},
    {2, 3424, -2865, 0, 10, 5, 20, 25},//you
    {1, 6345, -2829, 0, 20, 0, 0, 0},
    {2, 6452, -2522, 0, 10, 3, 16, 25},
    {1, 6414, 2767, 0, 20, 0, 0, 0},
		{2, 6105, 2864, 0, 10, 2, 21, 25},
		{1, 2213, 2848, 0, 20, 0, 0, 0},
		{2, 2113, 2535, 0, 10, 4, 17, 25},
		{1, 2127, -235, 0, 20, 0, 0, 0},//原材料上货-1-7
};
*/
const int16_t TASK_SEQUENCE_10[SEQ10_LEN][8] = {
		{1, 4493, -2700, 0, 20, 0, 0, 0},
		{2, 4192, -2830, 0, 10, 6, 23, 25},
		{1, -265, -2873, 0, 20, 0, 0, 0},
    {2, -422, -2575, 0, 10, 7, 22, 25},
    {1, -449, -994, 0, 20, 0, 0, 0}//2-3-上料缓冲
};
const int16_t TASK_SEQUENCE_11[SEQ11_LEN][8] = {
		{1, 4493, -2700, 0, 20, 0, 0, 0},
		{2, 4192, -2830, 0, 10, 6, 23, 25},
		{1, -265, -2873, 0, 20, 0, 0, 0},
    {2, -422, -2575, 0, 10, 7, 22, 25},
    {1, -449, -994, 0, 20, 0, 0, 0}//2-4-上料缓冲
};
const int16_t TASK_SEQUENCE_12[SEQ12_LEN][8] = {
		{1, 4493, -2700, 0, 20, 0, 0, 0},
		{2, 4192, -2830, 0, 10, 6, 23, 25},
		{1, -265, -2873, 0, 20, 0, 0, 0},
    {2, -422, -2575, 0, 10, 7, 22, 25},
    {1, -449, -994, 0, 20, 0, 0, 0}//2-5-上料缓冲
};
const int16_t TASK_SEQUENCE_13[SEQ13_LEN][8] = {
		{1, 4493, -2700, 0, 20, 0, 0, 0},
		{2, 4192, -2830, 0, 10, 6, 23, 25},
		{1, -265, -2873, 0, 20, 0, 0, 0},
    {2, -422, -2575, 0, 10, 7, 22, 25},
    {1, -449, -994, 0, 20, 0, 0, 0}//2-6-上料缓冲
};
const int16_t TASK_SEQUENCE_14[SEQ14_LEN][8] = {
		{1, 4493, -2700, 0, 20, 0, 0, 0},
		{2, 4192, -2830, 0, 10, 6, 23, 25},
		{1, -265, -2873, 0, 20, 0, 0, 0},
    {2, -422, -2575, 0, 10, 7, 22, 25},
		{1, -474, -65, 0, 20, 0, 0, 0},
    {2, -168, 93, 0, 10, 5, 27, 25},
    {1, 115, 89, 0, 3, 0, 0, 0}//2-1-充电 
};
const int16_t TASK_SEQUENCE_15[SEQ15_LEN][8] = {
		{1, 4493, -2700, 0, 20, 0, 0, 0},
		{2, 4192, -2830, 0, 10, 6, 23, 25},
		{1, -265, -2873, 0, 20, 0, 0, 0},
    {2, -422, -2575, 0, 10, 7, 22, 25},
		{1, -474, -65, 0, 20, 0, 0, 0},
    {2, -168, 93, 0, 10, 5, 27, 25},
    {1, 115, 89, 0, 3, 0, 0, 0}//2-2-充电 
};
const int16_t TASK_SEQUENCE_16[SEQ16_LEN][8] = {
		{1, 4493, -2700, 0, 20, 0, 0, 0},
		{2, 4192, -2830, 0, 10, 6, 23, 25},
		{1, -265, -2873, 0, 20, 0, 0, 0},
    {2, -422, -2575, 0, 10, 7, 22, 25},
		{1, -474, -65, 0, 20, 0, 0, 0},
    {2, -168, 93, 0, 10, 5, 27, 25},
    {1, 115, 89, 0, 3, 0, 0, 0}//2-3-充电 
};
const int16_t TASK_SEQUENCE_17[SEQ17_LEN][8] = {
		{1, 4493, -2700, 0, 20, 0, 0, 0},
		{2, 4192, -2830, 0, 10, 6, 23, 25},
		{1, -265, -2873, 0, 20, 0, 0, 0},
    {2, -422, -2575, 0, 10, 7, 22, 25},
		{1, -474, -65, 0, 20, 0, 0, 0},
    {2, -168, 93, 0, 10, 5, 27, 25},
    {1, 115, 89, 0, 3, 0, 0, 0}//2-4-充电 
};
const int16_t TASK_SEQUENCE_18[SEQ18_LEN][8] = {
		{1, 4493, -2700, 0, 20, 0, 0, 0},
		{2, 4192, -2830, 0, 10, 6, 23, 25},
		{1, -265, -2873, 0, 20, 0, 0, 0},
    {2, -422, -2575, 0, 10, 7, 22, 25},
		{1, -474, -65, 0, 20, 0, 0, 0},
    {2, -168, 93, 0, 10, 5, 27, 25},
    {1, 115, 89, 0, 3, 0, 0, 0}//2-5-充电 
};
const int16_t TASK_SEQUENCE_19[SEQ19_LEN][8] = {
		{1, 4493, -2700, 0, 20, 0, 0, 0},
		{2, 4192, -2830, 0, 10, 6, 23, 25},
		{1, -265, -2873, 0, 20, 0, 0, 0},
    {2, -422, -2575, 0, 10, 7, 22, 25},
		{1, -474, -65, 0, 20, 0, 0, 0},
    {2, -168, 93, 0, 10, 5, 27, 25},
    {1, 115, 89, 0, 3, 0, 0, 0}//2-6-充电 
};
const int16_t TASK_SEQUENCE_20[SEQ20_LEN][8] = {
    {3, -126, 93, 0, 10, 0, 0, 0},
    {5, -457, 189, 0, 10, 7, 7, 25},
    {1, -439, -2706,0, 20, 0, 0, 0},
    {2, -113, -2843, 0, 10, 1, 18, 25}, 
    {1, 2948, -2851, 0, 20, 0, 0, 0},
		{1, 6295, -2829, 0, 20, 0, 0, 0},/*ZQY6295*/
    {2, 6432, -2522, 0, 10, 3, 20, 25},
		{1, 6487, 53, 0, 20, 0, 0, 0},//马超
    {1, 6424, 2727, 0, 20, 0, 0, 0},
		{2, 6105, 2864, 0, 10, 2, 21, 25},
		{1, 4595, 2875, 0, 20, 0, 0, 0},/*ZQY4525--4595*/
		{2, 4437, 2578, 0, 10, 4, 22, 25},/*ZQY20--22*/
		{1, 4453, 1582, 0, 20, 0, 0, 0}//充电-2-1
};
const int16_t TASK_SEQUENCE_21[SEQ21_LEN][8] = {
    {3, -126, 93, 0, 10, 0, 0, 0},
    {5, -457, 189, 0, 10, 7, 7, 25},
    {1, -439, -2706,0, 20, 0, 0, 0},
    {2, -113, -2843, 0, 10, 1, 18, 25}, 
    {1, 2948, -2851, 0, 20, 0, 0, 0},
		{1, 6295, -2829, 0, 20, 0, 0, 0},/*ZQY6295*/
    {2, 6432, -2522, 0, 10, 3, 20, 25},
		{1, 6487, 53, 0, 20, 0, 0, 0},//马超
    {1, 6424, 2727, 0, 20, 0, 0, 0},
		{2, 6105, 2864, 0, 10, 2, 21, 25},
		{1, 4595, 2875, 0, 20, 0, 0, 0},/*ZQY4525--4595*/
		{2, 4437, 2578, 0, 10, 4, 22, 25},/*ZQY20--22*/
		{1, 4467, 605, 0, 20, 0, 0, 0}//充电-2-2
};
const int16_t TASK_SEQUENCE_22[SEQ22_LEN][8] = {
    {3, -126, 93, 0, 10, 0, 0, 0},
    {5, -457, 189, 0, 10, 7, 7, 25},
    {1, -439, -2706,0, 20, 0, 0, 0},
    {2, -113, -2843, 0, 10, 1, 18, 25}, 
    {1, 2948, -2851, 0, 20, 0, 0, 0},
		{1, 6295, -2829, 0, 20, 0, 0, 0},/*ZQY6295*/
    {2, 6432, -2522, 0, 10, 3, 20, 25},
		{1, 6487, 53, 0, 20, 0, 0, 0},//马超
    {1, 6424, 2727, 0, 20, 0, 0, 0},
		{2, 6105, 2864, 0, 10, 2, 21, 25},
		{1, 4595, 2875, 0, 20, 0, 0, 0},/*ZQY4525--4595*/
		{2, 4437, 2578, 0, 10, 4, 22, 25},/*ZQY20--22*/
		{1, 4467, -154, 0, 20, 0, 0, 0}//充电-2-3
};
const int16_t TASK_SEQUENCE_23[SEQ23_LEN][8] = {
    {3, -126, 93, 0, 10, 0, 0, 0},
    {5, -457, 189, 0, 10, 7, 7, 25},
    {1, -439, -2706,0, 20, 0, 0, 0},
    {2, -113, -2843, 0, 10, 1, 18, 25}, 
    {1, 2948, -2851, 0, 20, 0, 0, 0},
		{1, 6295, -2829, 0, 20, 0, 0, 0},/*ZQY6295*/
    {2, 6432, -2522, 0, 10, 3, 20, 25},
		{1, 6487, 53, 0, 20, 0, 0, 0},//马超
    {1, 6424, 2727, 0, 20, 0, 0, 0},
		{2, 6105, 2864, 0, 10, 2, 21, 25},
		{1, 4595, 2875, 0, 20, 0, 0, 0},/*ZQY4525--4595*/
		{2, 4437, 2578, 0, 10, 4, 22, 25},/*ZQY20--22*/
		{1, 4481, -1060, 0, 20, 0, 0, 0}//充电-2-4
};
const int16_t TASK_SEQUENCE_24[SEQ24_LEN][8] = {
    {3, -126, 93, 0, 10, 0, 0, 0},
    {5, -457, 189, 0, 10, 7, 7, 25},
    {1, -439, -2706,0, 20, 0, 0, 0},
    {2, -113, -2843, 0, 10, 1, 18, 25}, 
    {1, 2948, -2851, 0, 20, 0, 0, 0},
		{1, 6295, -2829, 0, 20, 0, 0, 0},/*ZQY6295*/
    {2, 6432, -2522, 0, 10, 3, 20, 25},
		{1, 6487, 53, 0, 20, 0, 0, 0},//马超
    {1, 6424, 2727, 0, 20, 0, 0, 0},
		{2, 6105, 2864, 0, 10, 2, 21, 25},
		{1, 4595, 2875, 0, 20, 0, 0, 0},/*ZQY4525--4595*/
		{2, 4437, 2578, 0, 10, 4, 22, 25},/*ZQY20--22*/
		{1, 4483, -1701, 0, 20, 0, 0, 0}//充电-2-5
};
const int16_t TASK_SEQUENCE_25[SEQ25_LEN][8] = {
    {3, -126, 93, 0, 10, 0, 0, 0},
    {5, -457, 189, 0, 10, 7, 7, 25},
    {1, -439, -2706,0, 20, 0, 0, 0},
    {2, -113, -2843, 0, 10, 1, 18, 25}, 
    {1, 2948, -2851, 0, 20, 0, 0, 0},
		{1, 6295, -2829, 0, 20, 0, 0, 0},/*ZQY6295*/
    {2, 6432, -2522, 0, 10, 3, 20, 25},
		{1, 6487, 53, 0, 20, 0, 0, 0},//马超
    {1, 6424, 2727, 0, 20, 0, 0, 0},
		{2, 6105, 2864, 0, 10, 2, 21, 25},
		{1, 4595, 2875, 0, 20, 0, 0, 0},/*ZQY4525--4595*/
		{2, 4437, 2578, 0, 10, 4, 22, 25},/*ZQY20--22*/
		{1, 4493, -2225, 0, 20, 0, 0, 0}//充电-2-6
};
const int16_t TASK_SEQUENCE_26[SEQ26_LEN][8] = {
		{1, -474, -65, 0, 20, 0, 0, 0},
    {2, -168, 93, 0, 10, 5, 27, 25},
    {1, 115, 89, 0, 3, 0, 0, 0}//上料缓冲-充电 y-81==>93
};
const int16_t TASK_SEQUENCE_27[SEQ27_LEN][8] = {
		{1, 4467, 605, 0, 20, 0, 0, 0}//2-1-2-2
};
const int16_t TASK_SEQUENCE_28[SEQ28_LEN][8] = {
		{1, 4467, -154, 0, 20, 0, 0, 0}//2-1-2-3
};
const int16_t TASK_SEQUENCE_29[SEQ29_LEN][8] = {
		{1, 4481, -1060, 0, 20, 0, 0, 0}//2-1-2-4
};
const int16_t TASK_SEQUENCE_30[SEQ30_LEN][8] = {
		{1, 4483, -1701, 0, 20, 0, 0, 0}//2-1-2-5
};
const int16_t TASK_SEQUENCE_31[SEQ31_LEN][8] = {
		{1, 4493, -2225, 0, 20, 0, 0, 0}//2-1-2-6
};
const int16_t TASK_SEQUENCE_32[SEQ32_LEN][8] = {
		{1, 4467, -154, 0, 20, 0, 0, 0}//2-2-2-3
};
const int16_t TASK_SEQUENCE_33[SEQ33_LEN][8] = {
		{1, 4481, -1060, 0, 20, 0, 0, 0}//2-2-2-4
};
const int16_t TASK_SEQUENCE_34[SEQ34_LEN][8] = {
		{1, 4483, -1701, 0, 20, 0, 0, 0}//2-2-2-5
};
const int16_t TASK_SEQUENCE_35[SEQ35_LEN][8] = {
		{1, 4493, -2225, 0, 20, 0, 0, 0}//2-2-2-6
};
const int16_t TASK_SEQUENCE_36[SEQ36_LEN][8] = {
		{1, 4481, -1060, 0, 20, 0, 0, 0}//2-3-2-4
};
const int16_t TASK_SEQUENCE_37[SEQ37_LEN][8] = {
		{1, 4483, -1701, 0, 20, 0, 0, 0}//2-3-2-5
};
const int16_t TASK_SEQUENCE_38[SEQ38_LEN][8] = {
		{1, 4493, -2225, 0, 20, 0, 0, 0}//2-3-2-6
};
const int16_t TASK_SEQUENCE_39[SEQ39_LEN][8] = {
		{1, 4483, -1701, 0, 20, 0, 0, 0}//2-4-2-5
};
const int16_t TASK_SEQUENCE_40[SEQ40_LEN][8] = {
		{1, 4493, -2225, 0, 20, 0, 0, 0}//2-4-2-6
};
const int16_t TASK_SEQUENCE_41[SEQ41_LEN][8] = {
		{1, 4493, -2225, 0, 20, 0, 0, 0}//2-5-2-6
};


//////////////////////通信协议的实现,发送状态，获取任务AGV2025
void comm_path_trans(void)
{
	static const int i = 0x61;
	static u8 key, pointNo = 1; // pointNo为坐标点计数;
	static u8 recallflag = 0;	//小车订单撤回标志位



	////////////////////////////////////////////////////
	if (USART5_RX_END == 1) //接收到有数据//待改
	{
		printf("Receive Opreation code : %c  %d  %d\n", t8266revbuf[0], t8266revbuf[1], t8266revbuf[2]);
		

	 if (t8266revbuf[0] == 'd')
	 //if (t8266revbuf[0] == 0x64)
   {
    const int16_t (*seq)[8] = NULL;
    int seqLen = 0;

		if (t8266revbuf[1] == 101) { seq = TASK_SEQUENCE_1; seqLen = SEQ1_LEN; taskseq=101;}
		else if (t8266revbuf[1] == 102) { seq = TASK_SEQUENCE_2; seqLen = SEQ2_LEN; taskseq=102;}
		else if (t8266revbuf[1] == 103) { seq = TASK_SEQUENCE_3; seqLen = SEQ3_LEN; taskseq=103;}
		else if (t8266revbuf[1] == 104) { seq = TASK_SEQUENCE_4; seqLen = SEQ4_LEN; taskseq=104;}
		else if (t8266revbuf[1] == 105) { seq = TASK_SEQUENCE_5; seqLen = SEQ5_LEN; taskseq=105;}
		else if (t8266revbuf[1] == 106) { seq = TASK_SEQUENCE_6; seqLen = SEQ6_LEN; taskseq=106;}
		else if (t8266revbuf[1] == 107) { seq = TASK_SEQUENCE_7; seqLen = SEQ7_LEN; taskseq=107;}
		else if (t8266revbuf[1] == 108) { seq = TASK_SEQUENCE_8; seqLen = SEQ8_LEN; taskseq=108;}
		else if (t8266revbuf[1] == 109) { seq = TASK_SEQUENCE_9; seqLen = SEQ9_LEN; taskseq=109;}
		else if (t8266revbuf[1] == 110) { seq = TASK_SEQUENCE_10; seqLen = SEQ10_LEN; taskseq=110;}
		else if (t8266revbuf[1] == 111) { seq = TASK_SEQUENCE_11; seqLen = SEQ11_LEN; taskseq=111;}
		else if (t8266revbuf[1] == 112) { seq = TASK_SEQUENCE_12; seqLen = SEQ12_LEN; taskseq=112;}
		else if (t8266revbuf[1] == 113) { seq = TASK_SEQUENCE_13; seqLen = SEQ13_LEN; taskseq=113;}
		else if (t8266revbuf[1] == 114) { seq = TASK_SEQUENCE_14; seqLen = SEQ14_LEN; taskseq=114;}
		else if (t8266revbuf[1] == 115) { seq = TASK_SEQUENCE_15; seqLen = SEQ15_LEN; taskseq=115;}
		else if (t8266revbuf[1] == 116) { seq = TASK_SEQUENCE_16; seqLen = SEQ16_LEN; taskseq=116;}
		else if (t8266revbuf[1] == 117) { seq = TASK_SEQUENCE_17; seqLen = SEQ17_LEN; taskseq=117;}
		else if (t8266revbuf[1] == 118) { seq = TASK_SEQUENCE_18; seqLen = SEQ18_LEN; taskseq=118;}
		else if (t8266revbuf[1] == 119) { seq = TASK_SEQUENCE_19; seqLen = SEQ19_LEN; taskseq=119;}
		else if (t8266revbuf[1] == 120) { seq = TASK_SEQUENCE_20; seqLen = SEQ20_LEN; taskseq=120;}
		else if (t8266revbuf[1] == 121) { seq = TASK_SEQUENCE_21; seqLen = SEQ21_LEN; taskseq=121;}
		else if (t8266revbuf[1] == 122) { seq = TASK_SEQUENCE_22; seqLen = SEQ22_LEN; taskseq=122;}
		else if (t8266revbuf[1] == 123) { seq = TASK_SEQUENCE_23; seqLen = SEQ23_LEN; taskseq=123;}
		else if (t8266revbuf[1] == 124) { seq = TASK_SEQUENCE_24; seqLen = SEQ24_LEN; taskseq=124;}
		else if (t8266revbuf[1] == 125) { seq = TASK_SEQUENCE_25; seqLen = SEQ25_LEN; taskseq=125;}
		else if (t8266revbuf[1] == 126) { seq = TASK_SEQUENCE_26; seqLen = SEQ26_LEN; taskseq=126;}
		else if (t8266revbuf[1] == 127) { seq = TASK_SEQUENCE_27; seqLen = SEQ27_LEN; taskseq=127;}
		else if (t8266revbuf[1] == 128) { seq = TASK_SEQUENCE_28; seqLen = SEQ28_LEN; taskseq=128;}
		else if (t8266revbuf[1] == 129) { seq = TASK_SEQUENCE_29; seqLen = SEQ29_LEN; taskseq=129;}
		else if (t8266revbuf[1] == 130) { seq = TASK_SEQUENCE_30; seqLen = SEQ30_LEN; taskseq=130;}
		else if (t8266revbuf[1] == 131) { seq = TASK_SEQUENCE_31; seqLen = SEQ31_LEN; taskseq=131;}
		else if (t8266revbuf[1] == 132) { seq = TASK_SEQUENCE_32; seqLen = SEQ32_LEN; taskseq=132;}
		else if (t8266revbuf[1] == 133) { seq = TASK_SEQUENCE_33; seqLen = SEQ33_LEN; taskseq=133;}
		else if (t8266revbuf[1] == 134) { seq = TASK_SEQUENCE_34; seqLen = SEQ34_LEN; taskseq=134;}
		else if (t8266revbuf[1] == 135) { seq = TASK_SEQUENCE_35; seqLen = SEQ35_LEN; taskseq=135;}
		else if (t8266revbuf[1] == 136) { seq = TASK_SEQUENCE_36; seqLen = SEQ36_LEN; taskseq=136;}
		else if (t8266revbuf[1] == 137) { seq = TASK_SEQUENCE_37; seqLen = SEQ37_LEN; taskseq=137;}
		else if (t8266revbuf[1] == 138) { seq = TASK_SEQUENCE_38; seqLen = SEQ38_LEN; taskseq=138;}
		else if (t8266revbuf[1] == 139) { seq = TASK_SEQUENCE_39; seqLen = SEQ39_LEN; taskseq=139;}
		else if (t8266revbuf[1] == 140) { seq = TASK_SEQUENCE_40; seqLen = SEQ40_LEN; taskseq=140;}
		else if (t8266revbuf[1] == 141) { seq = TASK_SEQUENCE_41; seqLen = SEQ41_LEN; taskseq=141;}
    else { return; }        // 非法序列号，直接丢弃 
		
		if(t8266revbuf[2]==1)
		{
		
			memset(cangpsbuf2, 0, sizeof cangpsbuf2); //清空数组  
			memset(now_task, 0, sizeof now_task);
			memset(pointbuf, 0, sizeof pointbuf);
			AGVtasklimit=0;
			SelfTestResult = 0;//任务切换要重新自检
			printf("task change");
		}
     printf("AGVtasklimit=%d",AGVtasklimit);
		if(AGVtasklimit>=0&&Checksum_Error==1)
		{ 
			AGVtasklimit=0; 
			printf("accept again");
		}
		  if(AGVtasklimit==0) 
		  {
				AGVtaskover=0;//822
    /* 拷贝整条任务序列 */
			for (u8 taskIdx = 0; taskIdx < seqLen; taskIdx++)
				{
					for (u8 col = 0; col < 8; col++)
						 {
							pointbuf[taskIdx][col] = seq[taskIdx][col];
							
						 }
						 datadeal2(taskIdx);
				}
					 AGVtasklimit++;
					 PointNum = seqLen;
					 Task_Count=seqLen;
					 printf("Load seq %d done, Task_Count=%d\r\n", t8266revbuf[1], Task_Count);
		  }
			

		}
		
		//新增AGV是否需要在等待点等待
		if (t8266revbuf[0] == 'w')
		{
			if (t8266revbuf[1] == 1)
			{
				AGVwait = 1;
				printf("AGVwait = 1");
			}
			//else if (t8266revbuf[1] == 0)
			if (t8266revbuf[1] == 0)
			{
				AGVwait = 0;
				printf("AGVwait = 0");
			}
		}//20251204
		
		
		
		if (t8266revbuf[0] == 'm')
		{
			printf("Start Opreation code 'm' \n");
			oksendbuf7[1] = PointNum - 1;
			printf("Task_Count = %d\n", Task_Count);
			UART5_SendArray(oksendbuf7, 24);			
			PointNum = 1;
			// printf("Opreation finish!\n");
		}

		//////////////////// 撤回订单判断 //////////////////
		if (t8266revbuf[0] == 'r')
		{
			 printf("Start Opreation code 'r' \n");
			RevokeControl();
      UART5_SendArray(oksendbuf8, 24);
			PointNum = 1;
			memset(cangpsbuf2, 0, sizeof cangpsbuf2); //清空数组  11.12Dakhin add
			memset(now_task, 0, sizeof now_task);
			memset(pointbuf, 0, sizeof pointbuf);
			delay_ms(20);

			UART5_SendArray(t8266sendbuf, 24);
		}

	}


	if (arrive)
	{
		printf("arrive!\r\n");


		Taskokreset();	//AGV未用到
		arrive = 0;
	}

}


int count = 0; //计数
int count2 = 0; //计数
//void datadeal2(int pointnum)
//{
//	//直线行驶
//	if (pointbuf[pointnum][10] == 1)
//	{
//		cangpsbuf2[pointnum][0] = pointbuf[pointnum][10] & 0x0f; //点类型1直行2转弯
//		cangpsbuf2[pointnum][0] |= pointbuf[pointnum][14] << 4;	 //速度
//		cangpsbuf2[pointnum][1] = pointbuf[pointnum][15] & 0x0f; //油门
//		cangpsbuf2[pointnum][1] |= pointbuf[pointnum][16] << 4;	 //激光雷达开关
//		cangpsbuf2[pointnum][2] = pointbuf[pointnum][3];
//		cangpsbuf2[pointnum][3] = pointbuf[pointnum][4];
//		cangpsbuf2[pointnum][4] = pointbuf[pointnum][5];
//		cangpsbuf2[pointnum][5] = pointbuf[pointnum][6];
//		cangpsbuf2[pointnum][6] = pointbuf[pointnum][7];
//		cangpsbuf2[pointnum][7] = pointbuf[pointnum][8];
//		cangpsbuf2[pointnum][8] = 0;
//		cangpsbuf2[pointnum][9] = 0;
//		cangpsbuf2[pointnum][10] = pointbuf[pointnum][17]; //相机开关
//		cangpsbuf2[pointnum][10] |= ((count2 + 1) % 15) << 4;
//	}
//	//转向行驶
//	else if (pointbuf[pointnum][10] == 2)
//	{
//		cangpsbuf2[pointnum][0] = pointbuf[pointnum][10] & 0x0f;
//		cangpsbuf2[pointnum][0] |= pointbuf[pointnum][14] << 4;	 //速度
//		cangpsbuf2[pointnum][1] = pointbuf[pointnum][15] & 0x0f; //油门
//		cangpsbuf2[pointnum][1] |= pointbuf[pointnum][16] << 4;	 //激光雷达开关
//		cangpsbuf2[pointnum][2] = pointbuf[pointnum][2];
//		cangpsbuf2[pointnum][3] = pointbuf[pointnum][11];  //转弯类型id : 1 - 8
//		cangpsbuf2[pointnum][4] = pointbuf[pointnum][12];  //转弯半径 m
//		cangpsbuf2[pointnum][5] = pointbuf[pointnum][13];  //转弯距离
//		cangpsbuf2[pointnum][10] = pointbuf[pointnum][17]; //相机开关
//		cangpsbuf2[pointnum][10] |= ((count2 + 1) % 15) << 4;
//	}
//	//倒车直行
//	else if (pointbuf[pointnum][10] == 3)
//	{
//		cangpsbuf2[pointnum][0] = pointbuf[pointnum][10] & 0x0f;
//		cangpsbuf2[pointnum][0] |= pointbuf[pointnum][14] << 4;	 //速度
//		cangpsbuf2[pointnum][1] = pointbuf[pointnum][15] & 0x0f; //油门
//		cangpsbuf2[pointnum][1] |= pointbuf[pointnum][16] << 4;	 //激光雷达开关
//		cangpsbuf2[pointnum][2] = pointbuf[pointnum][3];
//		cangpsbuf2[pointnum][3] = pointbuf[pointnum][4];
//		cangpsbuf2[pointnum][4] = pointbuf[pointnum][5];
//		cangpsbuf2[pointnum][5] = pointbuf[pointnum][6];
//		cangpsbuf2[pointnum][6] = pointbuf[pointnum][7];
//		cangpsbuf2[pointnum][7] = pointbuf[pointnum][8];
//		cangpsbuf2[pointnum][8] = 0;
//		cangpsbuf2[pointnum][9] = 0;
//		cangpsbuf2[pointnum][10] = pointbuf[pointnum][17]; //相机开关
//		cangpsbuf2[pointnum][10] |= ((count2 + 1) % 15) << 4;
//	}
//	// 倒车转向
//	else if (pointbuf[pointnum][10] == 5)
//	{
//		cangpsbuf2[pointnum][0] = pointbuf[pointnum][10] & 0x0f;
//		cangpsbuf2[pointnum][0] |= pointbuf[pointnum][14] << 4;	 // 速度
//		cangpsbuf2[pointnum][1] = pointbuf[pointnum][15] & 0x0f; // 油门
//		cangpsbuf2[pointnum][1] |= pointbuf[pointnum][16] << 4;	 // 激光雷达开关
//		cangpsbuf2[pointnum][2] = pointbuf[pointnum][2];
//		cangpsbuf2[pointnum][3] = pointbuf[pointnum][11];  // 转弯类型id : 1 - 8
//		cangpsbuf2[pointnum][4] = pointbuf[pointnum][12];  // 转弯半径 m
//		cangpsbuf2[pointnum][5] = pointbuf[pointnum][13];  // 转弯距离
//		cangpsbuf2[pointnum][10] = pointbuf[pointnum][17]; // 相机开关
//		cangpsbuf2[pointnum][10] |= ((count2 + 1) % 15) << 4;
//	}

//	return;
//}



//五菱新AGV
void datadeal2(int pointnum)
{
	//直线行驶
	if (pointbuf[pointnum][0] ==1||pointbuf[pointnum][0] ==2||pointbuf[pointnum][0] ==3||pointbuf[pointnum][0] ==4||pointbuf[pointnum][0] ==5)
	{
		cangpsbuf2[pointnum][0] = pointbuf[pointnum][0] ; //头
		cangpsbuf2[pointnum][1] = pointbuf[pointnum][1]; //任务类型
		cangpsbuf2[pointnum][2] = pointbuf[pointnum][2];
		cangpsbuf2[pointnum][3] = pointbuf[pointnum][3];
		cangpsbuf2[pointnum][4] = pointbuf[pointnum][4];
		cangpsbuf2[pointnum][5] = pointbuf[pointnum][5];
		cangpsbuf2[pointnum][6] = pointbuf[pointnum][6];
		cangpsbuf2[pointnum][7] = pointbuf[pointnum][7];
		cangpsbuf2[pointnum][8] = 0;
		cangpsbuf2[pointnum][9] = 0;
		cangpsbuf2[pointnum][10] =0;
		cangpsbuf2[pointnum][11] =0;
	}
	return;
}


















