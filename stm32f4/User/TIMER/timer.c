
#include "timer.h"
#include "run.h"
#include "bsp_can1.h"
#include "bsp_can2.h"
#include "bsp_debug_usart.h"
#include "4g_comnet.h"
int tim4_cnt = 0;
u8 ceshi[24] = {0x68, 0x32, 0x00, 0x00, 0x00, 0x01, 0x01, 0x5A, 0x16};
//五菱
#define SEND_INTERVAL 5  // 每5次定时器中断(100ms)发送一次
volatile uint8_t timer_count = 0;


void TIM2_Int_Init(u16 arr, u16 psc)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitstrue;
  NVIC_InitTypeDef NVIC_InitStrue;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // 时钟使能

  TIM_TimeBaseInitstrue.TIM_Period = arr;                     // 自动装载值
  TIM_TimeBaseInitstrue.TIM_Prescaler = psc;                  // 预分频系数
  TIM_TimeBaseInitstrue.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
  TIM_TimeBaseInitstrue.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitstrue); //***定时器初始化

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //***开启定时器中断：选定时器4，更新中断，使能

  NVIC_InitStrue.NVIC_IRQChannel = TIM2_IRQn;           //***设置中断分组
  NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority = 0; // 先占优先级0级
  NVIC_InitStrue.NVIC_IRQChannelSubPriority = 0;        // 从优先级3级
  NVIC_InitStrue.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStrue); // 初始化NVIC寄存器

  TIM_Cmd(TIM2, DISABLE); // 使能定时器
}

void TIM2_IRQHandler(void) // 
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    

		CANCreate();
		Can_Send_Msg_Flag(0x100, 0x0, 2, canbuf, 8); 
		Can_Send_Msg_Flag(0x101, 0x0, 2, TO_AGV_PSUSE_canbuf, 8);
		Can2_Send_Msg_Flag(0x100, 0x0, canbuf, 8);
		Lidarflag++;
		if(++timer_count >= SEND_INTERVAL) 
		{
		timer_count = 0;
		uart_tx_flag = 1;  // 触发主循环发送
		}

		
//		usart5Create();
//		UART5_SendArray(AGVupsendbuf, 12);//需要测试验证是否可行
		

//		Can_Send_Msg_Flag(0x0, 0x0CF008FB, 1, canbuf, 8);
//		Can2_Send_Msg_Flag(0x0, 0x0CF008FB, canbuf, 8);

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    
  }
}

void TIM5_Int_Init(u16 arr, u16 psc)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitstrue;
  NVIC_InitTypeDef NVIC_InitStrue;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); // 时钟使能

  TIM_TimeBaseInitstrue.TIM_Period = arr;                     // 自动装载值
  TIM_TimeBaseInitstrue.TIM_Prescaler = psc;                  // 预分频系数
  TIM_TimeBaseInitstrue.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
  TIM_TimeBaseInitstrue.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitstrue); //***定时器初始化

  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE); //***开启定时器中断：选定时器4，更新中断，使能

  NVIC_InitStrue.NVIC_IRQChannel = TIM5_IRQn;              //***设置中断分组
  NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority = 0x01; // 先占优先级0级
  NVIC_InitStrue.NVIC_IRQChannelSubPriority = 0;           // 从优先级3级
  NVIC_InitStrue.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStrue); // 初始化NVIC寄存器

  TIM_Cmd(TIM5, ENABLE); // 使能定时器
}

void TIM5_IRQHandler(void) // TIM5中断 - 50ms
{
  // static u8 EPB_flag2 = 0;
  if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) // 检查TIM5更新中断发生与否
  {
    MCUSend();
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
  }
}

void TIM4_Int_Init(u16 arr, u16 psc)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitstrue;
  NVIC_InitTypeDef NVIC_InitStrue;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // 时钟使能

  TIM_TimeBaseInitstrue.TIM_Period = arr;                     // 自动装载值
  TIM_TimeBaseInitstrue.TIM_Prescaler = psc;                  // 预分频系数
  TIM_TimeBaseInitstrue.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
  TIM_TimeBaseInitstrue.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitstrue); //***定时器初始化

  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); //***开启定时器中断：选定时器4，更新中断，使能

  NVIC_InitStrue.NVIC_IRQChannel = TIM4_IRQn;           //***设置中断分组
  NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority = 0; // 先占优先级0级
  NVIC_InitStrue.NVIC_IRQChannelSubPriority = 0;        // 从优先级3级
  NVIC_InitStrue.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStrue); // 初始化NVIC寄存器

  TIM_Cmd(TIM4, DISABLE); // 使能定时器
}

volatile uint8_t Sendcount=0;
uint8_t LowSpeedcanbuf[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t HighSpeedcanbuf[8]={0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
int BCMcount=0;
/*********************************************END OF FILE**********************/
