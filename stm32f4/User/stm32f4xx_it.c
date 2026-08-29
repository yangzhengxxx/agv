/**
 ******************************************************************************
 * @file    FMC_SDRAM/stm32f4xx_it.c
 * @author  MCD Application Team
 * @version V1.0.1
 * @date    11-November-2013
 * @brief   Main Interrupt Service Routines.
 *         This file provides template for all exceptions handler and
 *         peripherals interrupt service routine.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_it.h"
#include "bsp_can1.h"
#include "bsp_can2.h"
// #include "bsp_exti.h"
#include "bsp_debug_usart.h"
#include "delay.h"
#include "run.h"
#include "lidar.h"

/** @addtogroup STM32F429I_DISCOVERY_Examples
 * @{
 */

/** @addtogroup FMC_SDRAM
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief  This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler(void)
{
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void)
{
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
void SVC_Handler(void)
{
}

/**
 * @brief  This function handles PendSV_Handler exception.
 * @param  None
 * @retval None
 */
void PendSV_Handler(void)
{
}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f429_439xx.s).                         */
/******************************************************************************/

extern __IO uint32_t flag1; // 用于标志是否接收到数据，在中断函数中赋值
extern CanRxMsg RxMessage1; // 接收缓冲区
extern unsigned char MCU1flag, MCU2flag, MCU3flag, EPSflag, EPBflag, EHBflag, EHB1flag, GPSflag,   GPS1flag, BMSflag, Cameraflag;
u8 CameraLossCount;
extern uint8_t MCU1canbuf[8];
extern uint8_t MCU2canbuf[8];
extern uint8_t MCU3canbuf[8];
extern uint8_t EPScanbuf[8];
extern uint8_t EPBcanbuf[8];
extern uint8_t EHBcanbuf[8];
extern uint8_t EHB1canbuf[8];
extern uint8_t GPScanbuf[8];
extern uint8_t Lidarcanbuf1[8];
// extern uint8_t GPS1canbuf[8];
extern uint8_t BMScanbuf[8];
extern uint8_t Cameracanbuf[8];
extern uint8_t newEPSrecvcanbuf[8];
extern uint8_t Chargecanbuf[8]; //2023.9.2南奔洋改
extern uint8_t lowercomputercanbuf[8];//2025
void CAN1_RX_IRQHandler(void)
{
  uint8_t i;
  /*从邮箱中读出报文*/
  CAN_Receive(CANx1, CAN_FIFO0, &RxMessage1);


  if ((RxMessage1.StdId == 0x200) && (RxMessage1.IDE == CAN_ID_STD) && (RxMessage1.DLC == 1))
  {
    for (i = 0; i < 8; i++)
      remotemodecanbuf[i] = RxMessage1.Data[i];
  }
	if ((RxMessage1.StdId == 0x147) && (RxMessage1.IDE == CAN_ID_STD) && (RxMessage1.DLC == 8))
  {
    for (i = 0; i < 8; i++)
      AGV_VCU_canbuf[i] = RxMessage1.Data[i];
  }
	
	if ((RxMessage1.StdId == 0x18C) && (RxMessage1.IDE == CAN_ID_STD) && (RxMessage1.DLC == 8))
  {
    for (i = 0; i < 8; i++)
      AGV_EPB_canbuf[i] = RxMessage1.Data[i];
  }
	if ((RxMessage1.StdId == 0x400) && (RxMessage1.IDE == CAN_ID_STD) && (RxMessage1.DLC == 8))
  {
    for (i = 0; i < 8; i++)
      AGV_PAUSED_canbuf[i] = RxMessage1.Data[i];
  }
	if ((RxMessage1.StdId == 0x103) && (RxMessage1.IDE == CAN_ID_STD) && (RxMessage1.DLC == 8))
  {
    for (i = 0; i < 8; i++)
      AGV_BMS_canbuf[i] = RxMessage1.Data[i];
  }
//	if ((RxMessage1.StdId == 0x010) && (RxMessage1.IDE == CAN_ID_STD) && (RxMessage1.DLC == 8))//20250914
//  {
//    for (i = 0; i < 8; i++)
//      Lidarcanbuf[i] = RxMessage1.Data[i];
//		Lidarflag=0;
//  }
	
}

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

// can口接收数据查询
// Rx:CanRxMsg 结构体指针;
// 返回值:0,无数据被收到;
//		 其他,接收的数据长度;

extern __IO uint32_t flag2; // 用于标志是否接收到数据，在中断函数中赋值
extern CanRxMsg RxMessage2; // 接收缓冲区
void CAN2_RX_IRQHandler(void)
{
  uint8_t i;
  /*从邮箱中读出报文*/
  CAN_Receive(CAN2, CAN_FIFO1, &RxMessage2);

  /* 比较ID是否为0x1314 */
  if ((RxMessage2.ExtId == 0x18F53C0C) && (RxMessage2.IDE == CAN_ID_EXT) && (RxMessage2.DLC == 8))
  {
    for (i = 0; i < 8; i++)
      EPBcanbuf[i] = RxMessage2.Data[i];
    EPBflag = 1; // 接收成功
  }
	if ((RxMessage2.StdId == 0x010) && (RxMessage2.IDE == CAN_ID_STD) && (RxMessage2.DLC == 8))//20250914
  {
    for (i = 0; i < 8; i++)
      Lidarcanbuf[i] = RxMessage2.Data[i];
		Lidarflag=0;
  }
	
}
