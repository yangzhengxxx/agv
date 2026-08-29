#ifndef __KEY_H
#define __KEY_H	 
#include "stm32f4xx.h" 


/*下面的方式是通过直接操作库函数方式读取IO*/
#define KEY0 		GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_6) //PE4


/*下面方式是通过位带操作方式读取IO*/
/*
#define KEY0 		PEin(4)   	//PE4
#define KEY1 		PEin(3)		//PE3 
#define KEY2 		PEin(2)		//P32
#define WK_UP      	PAin(0)		//PA0
*/

#define KEY0_PRES 	1

void KEY_Init(void);	//IO初始化

#endif
