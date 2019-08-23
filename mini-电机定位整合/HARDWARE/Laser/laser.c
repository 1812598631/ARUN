#include "laser.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK Mini STM32������
//�������� ��������		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/3/06
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									   
//////////////////////////////////////////////////////////////////////////////////	 

u8 USART_RX_BUF_3[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA_3=0;       //����״̬���	
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
//	if(USART_GetITStatus(USART3,USART_IT_RXNE) != RESET) //�жϲ��� 
//	{
//		USART_ClearITPendingBit(USART3,USART_IT_RXNE); //����жϱ�־
//			 
//		Uart3_Buffer[Uart3_Rx] = USART_ReceiveData(USART3);     //���մ���1���ݵ�buff������
//		Uart3_Rx++; 
//     		 
//		if(Uart3_Buffer[Uart3_Rx-1] == 0x0a)    //������յ�β��ʶ�ǻ��з������ߵ�������������������½��գ�
//		{
//			if(Uart3_Buffer[0] == ':'&&Uart3_Buffer[1]==' ')                      //��⵽ͷ��ʶ��������Ҫ�� 
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