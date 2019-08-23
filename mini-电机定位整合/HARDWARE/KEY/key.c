#include "key.h"
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
u16 USART_RX_STA2=0;       //接收状态标记	 
int r=0;
float PosX=0,PosY,PosAngle;
u8 USART_RX_BUF2[USART_REC_LEN_2];     //接收缓冲,最大USART_REC_LEN个字节.
u8 USART_RX_BUF3[USART_REC_LEN_3];     //接收缓冲,最大USART_REC_LEN个字节.
 u16 Distance;
 void MyusartInit2(u16 bound)
{
	 USART_InitTypeDef USART_InitStruct1;                        //创建串口1结构体对象
	 GPIO_InitTypeDef  GPIO_InitStruct1;                         //创建GPIO1结构体对象
	 NVIC_InitTypeDef  NVIC_InitStruct1;                         //创建中断1结构体对象

	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);     //串口1时钟使能
	
	
	
	 GPIO_InitStruct1.GPIO_Mode=  GPIO_Mode_AF_PP;             //1.不同的模式分开初始化
	 GPIO_InitStruct1.GPIO_Pin= GPIO_Pin_2;   //TX
	 GPIO_InitStruct1.GPIO_Speed=GPIO_Speed_50MHz ;
	 GPIO_Init(GPIOA,&GPIO_InitStruct1);
	
	 GPIO_InitStruct1.GPIO_Mode=  GPIO_Mode_IN_FLOATING;
	 GPIO_InitStruct1.GPIO_Pin= GPIO_Pin_3;   //RX
	 GPIO_Init(GPIOA,&GPIO_InitStruct1);                        //2.上拉浮空输入不需要输出频率
	
	 USART_InitStruct1.USART_BaudRate=bound;
	 USART_InitStruct1.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	 USART_InitStruct1.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	 USART_InitStruct1.USART_Parity=USART_Parity_No;
	 USART_InitStruct1.USART_StopBits=USART_StopBits_1;
   USART_InitStruct1.USART_WordLength=USART_WordLength_8b;	
		 
	 USART_Init(USART2,&USART_InitStruct1);
	 USART_Cmd(USART2,ENABLE);
	 USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
//	 USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);				//开启空闲中断
	 USART_ClearFlag(USART2, USART_FLAG_TC);		//清楚发送标志
	 
	 
	 NVIC_InitStruct1.NVIC_IRQChannel=USART2_IRQn;
	 NVIC_InitStruct1.NVIC_IRQChannelCmd=ENABLE;
	 NVIC_InitStruct1.NVIC_IRQChannelPreemptionPriority=1;
	 NVIC_InitStruct1.NVIC_IRQChannelSubPriority=1;
	 NVIC_Init(& NVIC_InitStruct1);

}
u16 USART_RX_STA3=0;       //接收状态标记	 
int x=0,y=0,p=0;
u8 USART_RX_BUF3[USART_REC_LEN_3];     //接收缓冲,最大USART_REC_LEN个字节.
 void MyusartInit3(u16 bound)
{
	 USART_InitTypeDef USART_InitStruct1;                        //创建串口1结构体对象
	 GPIO_InitTypeDef  GPIO_InitStruct1;                         //创建GPIO1结构体对象
	 NVIC_InitTypeDef  NVIC_InitStruct1;                         //创建中断1结构体对象

	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);      //GPIOA时钟使能
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);     //串口1时钟使能
	
	
	
	 GPIO_InitStruct1.GPIO_Mode=  GPIO_Mode_AF_PP;             //1.不同的模式分开初始化
	 GPIO_InitStruct1.GPIO_Pin= GPIO_Pin_10;   //TX
	 GPIO_InitStruct1.GPIO_Speed=GPIO_Speed_50MHz ;
	 GPIO_Init(GPIOB,&GPIO_InitStruct1);
	
	 GPIO_InitStruct1.GPIO_Mode=  GPIO_Mode_IN_FLOATING;
	 GPIO_InitStruct1.GPIO_Pin= GPIO_Pin_11;   //RX
	 GPIO_Init(GPIOB,&GPIO_InitStruct1);                        //2.上拉浮空输入不需要输出频率
	
	 USART_InitStruct1.USART_BaudRate=bound;
	 USART_InitStruct1.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	 USART_InitStruct1.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	 USART_InitStruct1.USART_Parity=USART_Parity_No;
	 USART_InitStruct1.USART_StopBits=USART_StopBits_1;
     USART_InitStruct1.USART_WordLength=USART_WordLength_8b;	
		 
	 USART_Init(USART3,&USART_InitStruct1);
	 USART_Cmd(USART3,ENABLE);
	 USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);

	 NVIC_InitStruct1.NVIC_IRQChannel=USART3_IRQn;
	 NVIC_InitStruct1.NVIC_IRQChannelCmd=ENABLE;
	 NVIC_InitStruct1.NVIC_IRQChannelPreemptionPriority=1;
	 NVIC_InitStruct1.NVIC_IRQChannelSubPriority=0;
	 NVIC_Init(& NVIC_InitStruct1);
}
u8 USART2_RECEIVE_DATA[520];
u8 USART2_RX_Finish=1; // USART2接收完成标志量
//void DMA_Configuration(void)
//{

//DMA_InitTypeDef DMA_InitStructure;
///* DMA clock enable */	
//NVIC_InitTypeDef NVIC_InitStructure;                         //创建中断1结构体对象
//	
//	
//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//DMA1

///* DMA1 Channel5 (triggered by USART2 Tx event) Config */
//DMA_DeInit(DMA1_Channel7);
//DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(0x40013804);//0x40013804 &USART2->DR
//DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART2_SEND_DATA;	
//DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;				//方向：外设->内存
//DMA_InitStructure.DMA_BufferSize = 512;
//DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//DMA_InitStructure.DMA_Priority = DMA_Priority_High;		//优先级为高 非常高为VeryHigh
//DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;					//没有设置为内存->内存
//DMA_Init(DMA1_Channel7, &DMA_InitStructure);					//初始化DMA
//DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);
//DMA_ITConfig(DMA1_Channel7, DMA_IT_TE, ENABLE);
///* Enable USART1 DMA TX request */
//USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
//DMA_Cmd(DMA1_Channel7, DISABLE);



/* DMA1 Channel6 (triggered by USART1 Rx event) Config */
//DMA_DeInit(DMA1_Channel6);
//DMA_InitStructure.DMA_PeripheralBaseAddr = 0x40004404;
//DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART2_RECEIVE_DATA;
//DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//DMA_InitStructure.DMA_BufferSize = 512;
//DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Word;
//DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
//DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
//DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//DMA_Init(DMA1_Channel6, &DMA_InitStructure);
//DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);
//DMA_ITConfig(DMA1_Channel6, DMA_IT_TE, ENABLE);
///* Enable USART2 DMA RX request */
//USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
//DMA_Cmd(DMA1_Channel6, ENABLE);


//NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//NVIC_Init(&NVIC_InitStructure);


//}


//串口通讯方式接收激光
void USART2_IRQHandler(void)
{
//	static uint8_t k=0,rebuf[9]={0};
//	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断
//	{
//		rebuf[k++] =USART_ReceiveData(USART2);//(USART1->DR);	//读取接收到的数据
//		if(!(rebuf[0]=='D'))//如果帧头错误，清缓存
//		 {
//		 	k=0;
//			rebuf[0]=0;
//		 }
//		  if(k==9)//数据接收完毕
//		 {
//			if(rebuf[8]=='m')//判断帧尾，正确则解析姿态角
//			{
//			Distance=(rebuf[3]-30)*1000+(rebuf[5]-30)*100+(rebuf[6]-30)*10+(rebuf[7]-30)-20000;
//					LCD_ShowString(120,90,200,16,16,"d=");	
//					LCD_ShowxNum(120,110,r,6,16,0X80);
//			}
//			k=0;//清缓存
////		USART_SendData(USART2,r);
////		while(USART_GetFlagStatus(USART2,USART_FLAG_TC) != SET);
//	   } 
////	USART_ClearFlag(USART2,USART_FLAG_TC);

//	}


	static uint8_t k=0,rebuf[9]={0};
	
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断
	{
		rebuf[k++] =USART_ReceiveData(USART2);//(USART1->DR);	//读取接收到的数据
		if(!(rebuf[0]=='D'))//如果帧头错误，清缓存
		 {
		 	k=0;
			rebuf[0]=0;
		 }
		  if(k==9)//数据接收完毕
		 {
			if(rebuf[8]=='m')//判断帧尾，正确则解析姿态角
			{
			Distance=(rebuf[3]-30)*1000+(rebuf[5]-30)*100+(rebuf[6]-30)*10+(rebuf[7]-30)-20000;
					LCD_ShowString(120,90,200,16,16,"the d=");	
					LCD_ShowxNum(120,110,r,6,16,0X80);
			}
			k=0;//清缓存
//		USART_SendData(USART2,r);
//		while(USART_GetFlagStatus(USART2,USART_FLAG_TC) != SET);
	   } 
//	USART_ClearFlag(USART2,USART_FLAG_TC);

	}
	
	

}









//void USART2_IRQHandler(void)
//{
//    unsigned char num=0,i=0;
//		u16 Distance;
//    if(USART_GetITStatus(USART2,USART_IT_IDLE) == SET)
//    {
//			 DMA_Cmd(DMA1_Channel6,DISABLE);    //关闭DMA
//       num = USART2->SR;
//       num = USART2->DR; //清USART_IT_IDLE标志
//			DMA_ClearFlag( DMA1_FLAG_TC5 );  

//       num = 512 -  DMA_GetCurrDataCounter(DMA1_Channel6);      //得到真正接收数据个数
//					LCD_ShowString(60,130,200,16,16,"num=");	
//					LCD_ShowxNum(60,150,USART2_RECEIVE_DATA[i],9,16,0X80);			
////			for(i=0;i<num;i++)
////			{
//				if(USART2_RECEIVE_DATA[i]==0x3a)
//				{
//					Distance=(USART2_RECEIVE_DATA[i+2]-0x30)*1000+(USART2_RECEIVE_DATA[i+4]-0x30)*100+(USART2_RECEIVE_DATA[i+5]-0x30)*10+(USART2_RECEIVE_DATA[i+6]-0x30)*1.0;
//					LCD_ShowString(60,90,200,16,16,"d=");	
//					LCD_ShowxNum(60,110,Distance,9,16,0X80);
//				}
////				break;
////			}
//       USART2_RECEIVE_DATA[num] = '\0';
//			 DMA_ClearFlag(DMA1_FLAG_GL6 | DMA1_FLAG_TC6 | DMA1_FLAG_TE6 | DMA1_FLAG_HT6);//清标志
//       DMA1_Channel6->CNDTR=512;       //重新设置接收数据个数        
//       DMA_Cmd(DMA1_Channel6,ENABLE);  //开启DMA
//				USART_ClearITPendingBit(USART2, USART_IT_TC);
//        USART_ClearITPendingBit(USART2, USART_IT_IDLE);
//		}
//	}
int Rx_Counter=0,Rx_Fin;
void USART3_IRQHandler(void)                	//串口3中断服务程序
	{
	static uint8_t k=0,rebuf[9]={0};
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
		{
		 rebuf[k++] =USART_ReceiveData(USART3);	//读取接收到的数据
		 LCD_ShowString(60,170,200,16,16,"ok");	
			LCD_ShowxNum(60,220,rebuf[0],6,16,0X80);
		 if(!(rebuf[0]==0xA5))//如果帧头错误，清缓存
		 {
			k=0;
		 }
		 if(k==8)//数据接收完毕
		 {
			if(rebuf[8]==0x0D)//判断帧尾，正确则解析姿态角
			{
				PosX=rebuf[1]*100+rebuf[2];
				PosY=rebuf[3]*100+rebuf[4];
				PosAngle=rebuf[5]*100+rebuf[6];
				LCD_ShowString(60,210,200,16,16,"OK");	
			}
			k=0;//清缓存
		 }
    }
}
//	void USART3_IRQHandler(void)                	//串口2中断服务程序
//	{
//		u8 Res;
//if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
//		{
//		Res =USART_ReceiveData(USART1);	//读取接收到的数据
//		
//		if((USART_RX_STA3&0x8000)==0)//接收未完成
//			{
//			if(USART_RX_STA3&0x4000)//接收到了0x0d
//				{
//				if(Res!=0x0a)USART_RX_STA3=0;//接收错误,重新开始
//				else USART_RX_STA3|=0x8000;	//接收完成了 
//				}
//			else //还没收到0X0D
//				{
//				if(Res==0x0d)USART_RX_STA3|=0x4000;
//				else
//					{
//					USART_RX_BUF3[USART_RX_STA3&0X3FFF]=Res ;
//					USART_RX_STA3++;
//					if(USART_RX_STA3>(USART_REC_LEN_3-1))USART_RX_STA3=0;//接收数据错误,重新开始接收	  
//					}		 
//				}
//			}
//LCD_ShowString(60,170,200,16,16,"ok");	
//LCD_ShowxNum(60,220,USART_RX_BUF3[0],6,16,0X80);
//		}
//	}
//void DMA1_Channel6_IRQHandler(void)
//{
//DMA_ClearITPendingBit(DMA1_IT_TC6);
//DMA_ClearITPendingBit(DMA1_IT_TE6);
//DMA_Cmd(DMA1_Channel6, DISABLE);//关闭DMA,防止处理其间有数据
//DMA1_Channel6->CNDTR = 512;//重装填
//DMA_Cmd(DMA1_Channel6, ENABLE);//处理完,重开DMA
//}




//u16 Laser(u16 date)
//{
//	u16 Distance;
//	u16 t=0;
//	USART_SendData(USART2,date);
//	delay_ms(20);
//	Distance=(USART_RX_BUF2[t+2]-0x30)*1000+(USART_RX_BUF2[t+4]-0x30)*100+(USART_RX_BUF2[t+5]-0x30)*10+(USART_RX_BUF2[t+6]-0x30)*1.0;
//	LCD_ShowString(60,90,200,16,16,"d=");	
//	LCD_ShowxNum(60,110,Distance,6,16,0X80);
//	delay_ms(1000);
//	USART_RX_STA2=0;
//	return Distance;
//}
u16 Laser(u16 date)
{
	u8 len;
	u8 t=0;
  USART_SendData(USART2,date);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC) != SET);
//		if(USART_RX_STA2&0x8000)
//		{
//		len=USART_RX_STA2&0x3fff;//得到此次接收到的数据长度
//				Distance=(USART_RX_BUF2[t+2]-0x30)*1000+(USART_RX_BUF2[t+4]-0x30)*100+(USART_RX_BUF2[t+5]-0x30)*10+(USART_RX_BUF2[t+6]-0x30)*1.0;
//					LCD_ShowString(60,90,200,16,16,"d=");	
//					LCD_ShowxNum(60,110,Distance,6,16,0X80);
//					delay_ms(1000);	
//					USART_RX_STA2=0;
					return Distance;
//		}
}
//u16 Laser_Distance(void)
//{
//	u16 Distance;
//	u16 t=0;
//	Distance=(USART_RX_BUF2[t+2]-0x30)*1000+(USART_RX_BUF2[t+4]-0x30)*100+(USART_RX_BUF2[t+5]-0x30)*10+(USART_RX_BUF2[t+6]-0x30)*1.0;
//return Distance;
//}

float GetPosX(void)
	{
		return PosX;
}
float GetPosY(void)
	
{
		return PosY;
}
float GetAngle(void)
{
		return PosAngle;
}

