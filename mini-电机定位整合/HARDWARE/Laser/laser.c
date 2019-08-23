#include "laser.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK Mini STM32开发板
//按键输入 驱动代码		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/3/06
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									   
//////////////////////////////////////////////////////////////////////////////////	 

u8 USART_RX_BUF_3[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA_3=0;       //接收状态标记	
u16 Distance;
 void LaserInit(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);///////////////////////////???????????
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	 
	USART_InitStructure.USART_BaudRate = 19200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure); 
	
	USART_Cmd(USART3, ENABLE);
	
	USART_ClearFlag(USART3,USART_FLAG_TC);	
		
	USART_ITConfig(USART3, USART_IT_TXE, DISABLE);    
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//#define Max_BUFF_Len 200
//unsigned char Uart3_Buffer[Max_BUFF_Len];
//unsigned int Uart3_Rx=0;
//void USART3_IRQHandler()
//{
//	if(USART_GetITStatus(USART3,USART_IT_RXNE) != RESET) //中断产生 
//	{
//		USART_ClearITPendingBit(USART3,USART_IT_RXNE); //清除中断标志
//			 
//		Uart3_Buffer[Uart3_Rx] = USART_ReceiveData(USART3);     //接收串口1数据到buff缓冲区
//		Uart3_Rx++; 
//     		 
//		if(Uart3_Buffer[Uart3_Rx-1] == 0x0a)    //如果接收到尾标识是换行符（或者等于最大接受数就清空重新接收）
//		{
//			if(Uart3_Buffer[0] == ':'&&Uart3_Buffer[1]==' ')                      //检测到头标识是我们需要的 
//			{
//				int t=1;
//				Distance=(Uart3_Buffer[t+2]-0x30)*1000+(Uart3_Buffer[t+4]-0x30)*100+(Uart3_Buffer[t+5]-0x30)*10+(Uart3_Buffer[t+6]-0x30)*1.0;
//				Uart3_Rx=0;
//			} 
//		}
//	}
//}

void Laser(u16 date)
{
  USART_SendData(USART3,date);
	delay_ms(20);
}

u16 Laser_Distant(void)
{
	
return Distance;
}