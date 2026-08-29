#include "bsp_debug_usart.h"
#include "rs16_lidar.h"
#include "4g_comnet.h"
#include "stmflash.h"
#include <stdbool.h>

/// USART6嵌套向量中断控制器NVIC配置
/// UART5嵌套向量中断控制器NVIC配置
static void UART5_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 嵌套向量中断控制器组选择

	/* 配置USART为中断源 */
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	/* 抢断优先级为1 */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	/* 子优先级为1 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	/* 使能中断 */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/* 初始化配置NVIC */
	NVIC_Init(&NVIC_InitStructure);
}

/// UART4嵌套向量中断控制器NVIC配置
/// USART3嵌套向量中断控制器NVIC配置
/// USART2嵌套向量中断控制器NVIC配置
/// USART1嵌套向量中断控制器NVIC配置
static void USART1_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 嵌套向量中断控制器组选择

	/* 配置USART为中断源 */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	/* 抢断优先级为1 */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	/* 子优先级为1 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	/* 使能中断 */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/* 初始化配置NVIC */
	NVIC_Init(&NVIC_InitStructure);
}

/*************************************************************
 * @brief  DEBUG_USART6 GPIO 配置,工作模式配置。921600 8-N-1
 * @param  无
 * @retval 无
 *************************************************************/
/***********************************************************
 * @brief  DEBUG_USART5 GPIO 配置,工作模式配置。115200 8-N-1
 * @param  无
 * @retval 无
 ***********************************************************/
void Debug_UART5_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHB1PeriphClockCmd(DEBUG_UART_RX5_GPIO_CLK | DEBUG_UART_TX5_GPIO_CLK, ENABLE);

	/* 使能 UART 时钟 */
	RCC_APB1PeriphClockCmd(DEBUG_UART5_CLK, ENABLE);

	/* 连接 PXx 到 USARTx_Tx*/
	GPIO_PinAFConfig(DEBUG_UART_RX5_GPIO_PORT, DEBUG_UART_RX5_SOURCE, DEBUG_UART_RX5_AF);

	/*  连接 PXx 到 USARTx__Rx*/
	GPIO_PinAFConfig(DEBUG_UART_TX5_GPIO_PORT, DEBUG_UART_TX5_SOURCE, DEBUG_UART_TX5_AF);

	/* 配置Tx引脚为复用功能  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

	GPIO_InitStructure.GPIO_Pin = DEBUG_UART_TX5_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_UART_TX5_GPIO_PORT, &GPIO_InitStructure);

	/* 配置Rx引脚为复用功能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = DEBUG_UART_RX5_PIN;
	GPIO_Init(DEBUG_UART_RX5_GPIO_PORT, &GPIO_InitStructure);

	/* 配置串DEBUG_UART5 模式 */
	/* 波特率设置:DEBUG_USART_BAUDRATE */
	USART_InitStructure.USART_BaudRate = 115200;
	/* 字长(数据+校验位):8 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	/* 停止位:1个停止位 */
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	/* 校验位选择:不使用校验位 */
	USART_InitStructure.USART_Parity = USART_Parity_No;
	/* 硬件流控制:不使用硬件流 */
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	/* USART模式控制:同时使能接收和发送 */
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	/* 完成USART初始化配置 */
	USART_Init(DEBUG_UART5, &USART_InitStructure);
	/* 使能串口接受中断 */
	USART_ITConfig(DEBUG_UART5, USART_IT_RXNE, ENABLE);
	//		/* 使能串口空闲中断 */
	// USART_ITConfig(DEBUG_UART5, USART_IT_IDLE, ENABLE);
	/* 使能串口 */
	USART_Cmd(DEBUG_UART5, ENABLE);
	UART5_NVIC_Configuration();
}

/***********************************************************
 * @brief  DEBUG_UART4 GPIO 配置,工作模式配置。115200 8-N-1
 * @param  无
 * @retval 无
 ***********************************************************/
/***********************************************************
 * @brief  DEBUG_USART3 GPIO 配置,工作模式配置。115200 8-N-1
 * @param  无
 * @retval 无
 ***********************************************************/
/***********************************************************
 * @brief  DEBUG_USART2 GPIO 配置,工作模式配置。115200 8-N-1
 * @param  无
 * @retval 无
 ***********************************************************/
/*************************************************************
 * @brief  DEBUG_USART1 GPIO 配置,工作模式配置。115200 8-N-1
 * @param  无
 * @retval 无
 *************************************************************/
void Debug_USART1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHB1PeriphClockCmd(DEBUG_USART_RX1_GPIO_CLK | DEBUG_USART_TX1_GPIO_CLK, ENABLE);

	/* 使能 UART 时钟 */
	RCC_APB2PeriphClockCmd(DEBUG_USART1_CLK, ENABLE);

	/* 连接 PXx 到 USARTx_Tx*/
	GPIO_PinAFConfig(DEBUG_USART_RX1_GPIO_PORT, DEBUG_USART_RX1_SOURCE, DEBUG_USART_RX1_AF);

	/*  连接 PXx 到 USARTx__Rx*/
	GPIO_PinAFConfig(DEBUG_USART_TX1_GPIO_PORT, DEBUG_USART_TX1_SOURCE, DEBUG_USART_TX1_AF);

	/* 配置Tx引脚为复用功能  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX1_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART_TX1_GPIO_PORT, &GPIO_InitStructure);

	/* 配置Rx引脚为复用功能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX1_PIN;
	GPIO_Init(DEBUG_USART_RX1_GPIO_PORT, &GPIO_InitStructure);

	/* 配置串DEBUG_USART1 模式 */
	/* 波特率设置:DEBUG_USART_BAUDRATE */
	USART_InitStructure.USART_BaudRate = 115200;
	// USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	/* 字长(数据+校验位):8 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	/* 停止位:1个停止位 */
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	/* 校验位选择:不使用校验位 */
	USART_InitStructure.USART_Parity = USART_Parity_No;
	/* 硬件流控制:不使用硬件流 */
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	/* USART模式控制:同时使能接收和发送 */
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	/* 完成USART初始化配置 */
	USART_Init(DEBUG_USART1, &USART_InitStructure);
	/* 使能串口接收中断 */
	USART_ITConfig(DEBUG_USART1, USART_IT_RXNE, ENABLE);
	/* 使能串口空闲中断 */
	USART_ITConfig(DEBUG_USART1, USART_IT_IDLE, ENABLE);
	/* 使能串口 */
	USART_Cmd(DEBUG_USART1, ENABLE);
	USART1_NVIC_Configuration();
}

u16 USART6_RX_STA = 0;
u16 USART5_RX_STA = 0;
u8 USART6_RX_END = 0;
u8 USART5_RX_END = 0;
u8 test_USART5_RX_END = 0;

u8 AtRxBuffer[400];
u8 AtRxBuffer1[400];
u8 u5RXbuf[24];
u8 u5RXlen;
uint16_t Rxcouter;
uint16_t UART5_RxCouter;
u8 recv_4g_flag = 0;

// #ifdef EN_USART6_RX //如果定义该宏名，串口6使能接收

// void USART6_IRQHandler(void)
//{
//	u8 res;

//	if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) //接收到数据
//	{
//		USART_ClearITPendingBit(USART6, USART_IT_RXNE); //清除标志位

//		res = USART_ReceiveData(USART6); //读取接收到的数据
//		AtRxBuffer[Rxcouter++] = res;	 //
//		USART6_RX_STA++;
//	}
//	if (USART_GetITStatus(USART6, USART_IT_IDLE) != RESET) //接收到数据
//	{
//		USART6->SR;
//		USART6->DR;
//		USART6_RX_END = 1;
//		recv_4g_flag = 1;
//	}
//}

//void DEBUG_UART5_IRQHandler(void)//原版
//{
//	u8 res;

//	if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) // 接收到数据
//	{
//		USART_ClearITPendingBit(UART5, USART_IT_RXNE); // 清除标志位
////    printf("jinzhongdaun\r\n");
//		
//		res = USART_ReceiveData(UART5); // 读取接收到的数据
//		AtRxBuffer[UART5_RX_STA++] = res;
//		if (UART5_RX_STA >= 24)
//		{
//			test_USART5_RX_END = 1;
//			
////		if ((AtRxBuffer[0] == 0xA5||AtRxBuffer[0]=='w'|| AtRxBuffer[0]=='s'||AtRxBuffer[0]=='f'||AtRxBuffer[0]=='g'||AtRxBuffer[0]=='n'||AtRxBuffer[0]=='m'||AtRxBuffer[0]=='r'||AtRxBuffer[0]=='a'||AtRxBuffer[0]=='c'
////			||AtRxBuffer[0]=='d'||AtRxBuffer[0]=='l'||AtRxBuffer[0]=='t')&& AtRxBuffer[23] == 0x5A )
//			if (AtRxBuffer[0] == 0xA5||AtRxBuffer[0]=='w'|| AtRxBuffer[0]=='s'||AtRxBuffer[0]=='f'||AtRxBuffer[0]=='g'||AtRxBuffer[0]=='n'||AtRxBuffer[0]=='m'||AtRxBuffer[0]=='r'||AtRxBuffer[0]=='a'||AtRxBuffer[0]=='c'
//			||AtRxBuffer[0]=='d'||AtRxBuffer[0]=='l'||AtRxBuffer[0]=='t')
//			{
//				for (int i = 0; i < UART5_RX_STA; i++)
//				{
//					t8266revbuf[i] = AtRxBuffer[i];
//					// printf(" 0x%02X ", buf[i]);
//				}
//				 USART5_RX_END=1;
//			}

//			UART5_RX_STA = 0;
//		}
//	}
//}


//void u5_Receive_Data(u8 *buf, u8 *len)
//{
//	u8 i = 0;
//	*len = 0; //
//	// printf("USART6_RX_END = %d  USART6_RX_STA = %d\r\n", USART6_RX_END, USART6_RX_STA);
//	if (USART5_RX_END == 1)
//	{

//		if (UART5_RX_STA >= 24)
//			UART5_RX_STA = 24;

//		for (i = 0; i < UART5_RX_STA; i++)
//		{
//			buf[i] = AtRxBuffer[i];
//			// printf(" 0x%02X ", buf[i]);
//		}
//		//				printf("\r\n");
//		*len = UART5_RX_STA; //

//		UART5_RX_STA = 0; //
//		USART5_RX_END = 0;
//	}
//	// Rxcouter=0;
//}

#define FRAME_LEN    12    // 一帧总字节数
#define USART_PORT   UART5 // 使用的 UART 口

// 这里定义所有可能的帧头字节
static inline bool is_header(uint8_t b) {
    return  (b == 0xA5)  ||  // 0xA5 二进制头
            (b == 'd')   ||
            (b == 'l')   ||
            (b == 't')   ||   // …按需添加更多
						(b == 'w');
}

// 中断内临时存帧缓冲
static uint8_t  _rx_buf[FRAME_LEN];
// 当前已接收字节数（写指针）
static uint8_t  _rx_idx   = 0;
// 状态机状态：0=等待帧头、1=正在收帧
static uint8_t  _rx_state = 0;

//// 供主循环读取的最终缓冲和标志
//extern uint8_t  t8266revbuf[FRAME_LEN];
//extern volatile uint8_t USART5_RX_END;
//extern volatile uint8_t test_USART5_RX_END;

// UART5 中断服务函数
void DEBUG_UART5_IRQHandler(void)
{
    // ----- 1）处理“接收到新字节”中断 -----
    if (USART_GetITStatus(USART_PORT, USART_IT_RXNE)) {
        // 清除 RXNE 中断标志
        USART_ClearITPendingBit(USART_PORT, USART_IT_RXNE);

        // 读取一个字节
        uint8_t b = (uint8_t)USART_ReceiveData(USART_PORT);

        if (_rx_state == 0) {
            // ---- 状态 WAIT_HEADER：还没看到合法帧头，丢弃无关字节 ----
            if (is_header(b)) {
                // 找到帧头，保存到缓冲首位，切换到“收帧”状态
                _rx_buf[0]  = b;
                _rx_idx     = 1;
                _rx_state   = 1;
            }
            // 否则继续丢弃，保持等待帧头状态
        }
        else {
            // ---- 状态 RECEIVING：正在收帧，直到收满 FRAME_LEN ----
            _rx_buf[_rx_idx++] = b;

            if (_rx_idx >= FRAME_LEN) {
                // 整帧收完：拷贝到全局缓冲，通知主循环
                for (uint8_t i = 0; i < FRAME_LEN; i++) {
                    t8266revbuf[i] = _rx_buf[i];
                }
                USART5_RX_END      = 1;  // 主循环打印使用
                test_USART5_RX_END = 1;  // 可选，保留之前测试标志

                // 重置状态机，准备下一帧
                _rx_state = 0;
                _rx_idx   = 0;
            }
        }
    }

    // ----- 2）处理“硬件过载”错误，出现丢帧就立刻同步 -----
    if (USART_GetFlagStatus(USART_PORT, USART_FLAG_ORE)) {
        // 清除 Overrun Error 标志
        USART_ClearFlag(USART_PORT, USART_FLAG_ORE);
        // 丢弃当前半帧，重新从下一个字节开始找帧头
        _rx_state = 0;
        _rx_idx   = 0;
    }
}





//void UART5_SendArray(u8 *array, u8 length)
//{
//	for (u8 i = 0; i < length; i++)
//	{
//		// 等待发送数据寄存器为空
//		while (USART_GetFlagStatus(DEBUG_UART5, USART_FLAG_TC) == RESET)
//			;
//		// 发送一个字节数据
//		USART_SendData(DEBUG_UART5, array[i]);
//	}
//}

void UART5_SendArray(u8 *array, u8 length)
{
    for (u8 i = 0; i < length; i++)
    {
        // 等待发送数据寄存器为空（TXE标志可能更合适）
        while (USART_GetFlagStatus(DEBUG_UART5, USART_FLAG_TXE) == RESET)
            ;
        
        // 发送一个字节数据
        USART_SendData(DEBUG_UART5, array[i]);
    }
    
    // 可选：等待最后一位数据发送完成
    while (USART_GetFlagStatus(DEBUG_UART5, USART_FLAG_TC) == RESET)
        ;
}





volatile uint8_t UART5_TxBusy = 0;



// #endif
// 串口6的发送函数
// 串口3发送缓存区
// 串口1发送缓存区
__align(8) u8 USART1_TX_BUF[USART1_MAX_SEND_LEN]; // 发送缓冲,最大USART3_MAX_SEND_LEN字节
#ifdef USART1_RX_EN								  // 如果使能了接收
// 串口3接收缓存区
u8 USART1_RX_BUF[USART1_MAX_RECV_LEN]; // 接收缓冲,最大USART3_MAX_RECV_LEN个字节.

////通过判断接收连续2个字符之间的时间差不大于100ms来决定是不是一次连续的数据.
////如果2个字符接收间隔超过100ms,则认为不是1次连续数据.也就是超过100ms没有接收到
////任何数据,则表示此次接收完毕.
////接收到的数据状态
////[10]:0,没有接收到数据;1,接收到了一批数据.
////[9:0]:接收到的数据长度
u16 USART1_RX_STA = 0;
// u8 arrive=0;
void USART1_IRQHandler(void)
{
	u8 res;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) // 接收到数据
	{
		res = USART_ReceiveData(USART1);
		USART1_RX_BUF[USART1_RX_STA++] = res; // 记录接收到的值
	}
	if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) // 接收到数据
	{
		(void)USART1->SR;
		(void)USART1->DR;
		USART1_RX_BUF[USART1_RX_STA++] = 0;
		if (!strcmp((char *)USART1_RX_BUF, "reset"))
		{
			__disable_irq();
			NVIC_SystemReset();
		}
		if (!strcmp((char *)USART1_RX_BUF, "update"))
		{
			u32 temp_flag = 0x1111;
			STMFLASH_Write(0x08004000, &temp_flag, 1);
			__disable_irq();
			NVIC_SystemReset();
		}
		USART1_RX_STA = 0;
	}
}

#endif

/****************** 发送数据 ********************/
/***************  发送一个字符  ***************/
/***************  发送一个字符串  ***************/
/// 重定向c库函数printf到串口DEBUG_USART1，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
	/* 发送一个字节数据到串口DEBUG_USART */
	USART_SendData(USART1, (uint8_t)ch);

	/* 等待发送完毕 */
	while (USART_GetFlagStatus(DEBUG_USART1, USART_FLAG_TXE) == RESET)
		;
	return (ch);
}

/// 重定向c库函数scanf到串口DEBUG_USART1，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
	/* 等待串口输入数据 */
	while (USART_GetFlagStatus(DEBUG_USART1, USART_FLAG_RXNE) == RESET)
		;

	return (int)USART_ReceiveData(DEBUG_USART1);
}

/*********************************************END OF FILE**********************/
