#ifndef __CAN1_H
#define __CAN1_H

#include "stm32f4xx.h"
#include <stdio.h>

#define CANx1 CAN1
#define CAN1_CLK RCC_APB1Periph_CAN1
#define CAN1_RX_IRQ CAN1_RX0_IRQn
#define CAN1_RX_IRQHandler CAN1_RX0_IRQHandler

#define CAN1_RX_PIN GPIO_Pin_0
#define CAN1_TX_PIN GPIO_Pin_1
#define CAN1_TX_GPIO_PORT GPIOD
#define CAN1_RX_GPIO_PORT GPIOD
#define CAN1_TX_GPIO_CLK RCC_AHB1Periph_GPIOD
#define CAN1_RX_GPIO_CLK RCC_AHB1Periph_GPIOD
#define CAN1_AF_PORT GPIO_AF_CAN1
#define CAN1_RX_SOURCE GPIO_PinSource0
#define CAN1_TX_SOURCE GPIO_PinSource1

/*debug*/

#define CAN_DEBUG_ON 1
#define CAN_DEBUG_ARRAY_ON 1
#define CAN_DEBUG_FUNC_ON 1

// Log define
#define CAN_INFO(fmt, arg...) printf("<<-CAN-INFO->> " fmt "\n", ##arg)
#define CAN_ERROR(fmt, arg...) printf("<<-CAN-ERROR->> " fmt "\n", ##arg)
#define CAN_DEBUG(fmt, arg...)                                        \
    do                                                                \
    {                                                                 \
        if (CAN_DEBUG_ON)                                             \
            printf("<<-CAN-DEBUG->> [%d]" fmt "\n", __LINE__, ##arg); \
    } while (0)

#define CAN_DEBUG_ARRAY(array, num)            \
    do                                         \
    {                                          \
        int32_t i;                             \
        uint8_t *a = array;                    \
        if (CAN_DEBUG_ARRAY_ON)                \
        {                                      \
            printf("<<-CAN-DEBUG-ARRAY->>\n"); \
            for (i = 0; i < (num); i++)        \
            {                                  \
                printf("%02x   ", (a)[i]);     \
                if ((i + 1) % 10 == 0)         \
                {                              \
                    printf("\n");              \
                }                              \
            }                                  \
            printf("\n");                      \
        }                                      \
    } while (0)

#define CAN_DEBUG_FUNC()                                                    \
    do                                                                      \
    {                                                                       \
        if (CAN_DEBUG_FUNC_ON)                                              \
            printf("<<-CAN-FUNC->> Func:%s@Line:%d\n", __func__, __LINE__); \
    } while (0)

static void CAN1_GPIO_Config(void);
static void CAN1_NVIC_Config(void);
static void CAN1_Mode_Config(void);
static void CAN1_Filter_Config(void);
void CAN1_Config(void);
void Init_RxMes(CanRxMsg *RxMessage);
u8 Can_Send_Msg_Flag(uint32_t StdId, uint32_t ExtId, u8 sta, u8 *msg, u8 len);

#endif
