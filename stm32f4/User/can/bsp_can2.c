#include "./can/bsp_can2.h"



/*
 * 函数名：CAN2_GPIO_Config
 * 描述  ：CAN的GPIO 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN2_GPIO_Config(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
   	

  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(CAN2_TX_GPIO_CLK|CAN2_RX_GPIO_CLK, ENABLE);
	
	  /* Connect CAN pins to AF9 */
  GPIO_PinAFConfig(CAN2_TX_GPIO_PORT, CAN2_RX_SOURCE, CAN2_AF_PORT);
  GPIO_PinAFConfig(CAN2_RX_GPIO_PORT, CAN2_TX_SOURCE, CAN2_AF_PORT); 

	  /* Configure CAN TX pins */
  GPIO_InitStructure.GPIO_Pin = CAN2_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(CAN2_TX_GPIO_PORT, &GPIO_InitStructure);
	
	/* Configure CAN RX  pins */
  GPIO_InitStructure.GPIO_Pin = CAN2_RX_PIN ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(CAN2_RX_GPIO_PORT, &GPIO_InitStructure);


}

/*
 * 函数名：CAN2_NVIC_Config
 * 描述  ：CAN的NVIC 配置,第1优先级组，0，0优先级
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN2_NVIC_Config(void)
{
   	NVIC_InitTypeDef NVIC_InitStructure;
		/* Configure one bit for preemption priority */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	 	/*中断设置*/
		NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX_IRQ;	   //CAN RX1中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		   //抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			   //子优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*
 * 函数名：CAN2_Mode_Config
 * 描述  ：CAN的模式 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN2_Mode_Config(void)
{
	CAN_InitTypeDef        CAN_InitStructure;
	/************************CAN通信参数设置**********************************/
	/* Enable CAN clock */
  RCC_APB1PeriphClockCmd(CAN2_CLK, ENABLE);

	/*CAN寄存器初始化*/
	CAN_DeInit(CAN2);
	CAN_StructInit(&CAN_InitStructure);

	/*CAN单元初始化*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  关闭时间触发通信模式使能
	CAN_InitStructure.CAN_ABOM=ENABLE;			   //MCR-ABOM  自动离线管理 
	CAN_InitStructure.CAN_AWUM=ENABLE;			   //MCR-AWUM  使用自动唤醒模式
	CAN_InitStructure.CAN_NART=DISABLE;			   //MCR-NART  禁止报文自动重传	  DISABLE-自动重传
	CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  接收FIFO 锁定模式  DISABLE-溢出时新报文会覆盖原有报文  
	CAN_InitStructure.CAN_TXFP=DISABLE;			   //MCR-TXFP  发送FIFO优先级 DISABLE-优先级取决于报文标示符 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //正常工作模式
	CAN_InitStructure.CAN_SJW=CAN_SJW_2tq;		   //BTR-SJW 重新同步跳跃宽度 2个时间单元
	 
	/* ss=1 bs1=4 bs2=2 位时间宽度为(1+4+2) 波特率即为时钟周期tq*(1+4+2)  */
	CAN_InitStructure.CAN_BS1=CAN_BS1_4tq;		   //BTR-TS1 时间段1 占用了4个时间单元
	CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;		   //BTR-TS1 时间段2 占用了2个时间单元	
	
	/* CAN Baudrate = 1 MBps (1MBps已为stm32的CAN最高速率) (CAN 时钟频率为 APB 1 = 42 MHz) */
	CAN_InitStructure.CAN_Prescaler =24;		   ////BTR-BRP 波特率分频器  定义了时间单元的时间长度 42/(1+4+2)/6=1 Mbps
	CAN_Init(CAN2, &CAN_InitStructure);
}

/*
 * 函数名：CAN2_Filter_Config
 * 描述  ：CAN的过滤器 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN2_Filter_Config(void)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/*CAN筛选器初始化*/
	CAN_FilterInitStructure.CAN_FilterNumber=14;						//筛选器组14
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//工作在掩码模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//筛选器位宽为单个32位。
	/* 使能筛选器，按照标志的内容进行比对筛选，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */

//	CAN_FilterInitStructure.CAN_FilterIdHigh= ((((u32)0x18050011<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF0000)>>16;		//要筛选的ID高位 
//	CAN_FilterInitStructure.CAN_FilterIdLow= (((u32)0x18050011<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; //要筛选的ID低位 
//	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0xFFFF;			//筛选器高16位每位必须匹配
//	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0xFFFF;			//筛选器低16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterIdHigh= 0x00;		//要筛选的ID高位 
	CAN_FilterInitStructure.CAN_FilterIdLow= 0x00; //要筛选的ID低位 
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x00;			//筛选器高16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x00;			//筛选器低16位每位必须匹配
//	CAN_FilterInitStructure.CAN_FilterIdHigh= 0x00;		//要筛选的ID高位 
//	CAN_FilterInitStructure.CAN_FilterIdLow= 0x00; //要筛选的ID低位 
//	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x00;			//筛选器高16位每位必须匹配
//	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x00;			//筛选器低16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO1 ;				//筛选器被关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//使能筛选器
	CAN_FilterInit(&CAN_FilterInitStructure);
	/*CAN通信中断使能*/
	CAN_ITConfig(CAN2, CAN_IT_FMP1, ENABLE);
}


/*
 * 函数名：CAN2_Config
 * 描述  ：完整配置CAN的功能
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void CAN2_Config(void)
{
  CAN2_GPIO_Config();
  CAN2_NVIC_Config();
  CAN2_Mode_Config();
  CAN2_Filter_Config();   
}


/**
  * @brief  初始化 Rx Message2数据结构体
  * @param  RxMessage: 指向要初始化的数据结构体
  * @retval None
  */
/*
 * 函数名：CAN2_SetMsg
 * 描述  ：CAN通信报文内容设置,设置一个数据内容为0-7的数据包
 * 输入  ：发送报文结构体
 * 输出  : 无
 * 调用  ：外部调用
 */	 
//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
u8 Can2_Send_Msg_Flag(uint32_t StdId,uint32_t ExtId,u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=StdId;					 // 标准标识符 
  TxMessage.ExtId=ExtId;				   // 设置扩展标示符 
  TxMessage.IDE=CAN_Id_Extended; // 扩展帧
  TxMessage.RTR=CAN_RTR_Data;		 // 数据帧
  TxMessage.DLC=len;						// 要发送的数据长度
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];			          
  mbox= CAN_Transmit(CAN2, &TxMessage);  

	//printf("\n  std=%d,ext=%d  \n",TxMessage.StdId,TxMessage.ExtId);
  i=0;
  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 1;
  return 0;		

}


/**************************END OF FILE************************************/











