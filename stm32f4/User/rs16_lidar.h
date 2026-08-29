#include <stm32f4xx.h>
extern unsigned char RSLidarcanbuf[8];
extern int rslidar_x, rslidar_y, imucnt;
extern float Rs_Angle, imuinit_angle;
extern u16 UART5_RX_STA;
//extern u8 UART5_RX_END;
//extern u8 UART5_RX_BUF[400];
extern u8 MatchingLoss, matchingcnt, imuflag;

extern void LidarData_Deal(void);
extern unsigned int taillose;
