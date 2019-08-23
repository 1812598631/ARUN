#include "key.h"
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
u16 USART_RX_STA2=0;       //����״̬���	 
int r=0;
float PosX=0,PosY,PosAngle;
u8 USART_RX_BUF2[USART_REC_LEN_2];     //���ջ���,���USART_REC_LEN���ֽ�.
u8 USART_RX_BUF3[USART_REC_LEN_3];     //���ջ���,���USART_REC_LEN���ֽ�.
 u16 Distance;
 void MyusartInit2(u16 bound)
{
	 USART_InitTypeDef USART_InitStruct1;                        //��������1�ṹ�����
	 GPIO_InitTypeDef  GPIO_InitStruct1;                         //����GPIO1�ṹ�����
	 NVIC_InitTypeDef  NVIC_InitStruct1;                         //�����ж�1�ṹ�����

	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);     //����1ʱ��ʹ��
	
	
	
	 GPIO_InitStruct1.GPIO_Mode=  GPIO_Mode_AF_PP;             //1.��ͬ��ģʽ�ֿ���ʼ��
	 GPIO_InitStruct1.GPIO_Pin= GPIO_Pin_2;   //TX
	 GPIO_InitStruct1.GPIO_Speed=GPIO_Speed_50MHz ;
	 GPIO_Init(GPIOA,&GPIO_InitStruct1);
	
	 GPIO_InitStruct1.GPIO_Mode=  GPIO_Mode_IN_FLOATING;
	 GPIO_InitStruct1.GPIO_Pin= GPIO_Pin_3;   //RX
	 GPIO_Init(GPIOA,&GPIO_InitStruct1);                        //2.�����������벻��Ҫ���Ƶ��
	
	 USART_InitStruct1.USART_BaudRate=bound;
	 USART_InitStruct1.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	 USART_InitStruct1.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	 USART_InitStruct1.USART_Parity=USART_Parity_No;
	 USART_InitStruct1.USART_StopBits=USART_StopBits_1;
   USART_InitStruct1.USART_WordLength=USART_WordLength_8b;	
		 
	 USART_Init(USART2,&USART_InitStruct1);
	 USART_Cmd(USART2,ENABLE);
	 USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
//	 USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);				//���������ж�
	 USART_ClearFlag(USART2, USART_FLAG_TC);		//������ͱ�־
	 
	 
	 NVIC_InitStruct1.NVIC_IRQChannel=USART2_IRQn;
	 NVIC_InitStruct1.NVIC_IRQChannelCmd=ENABLE;
	 NVIC_InitStruct1.NVIC_IRQChannelPreemptionPriority=1;
	 NVIC_InitStruct1.NVIC_IRQChannelSubPriority=1;
	 NVIC_Init(& NVIC_InitStruct1);

}
u16 USART_RX_STA3=0;       //����״̬���	 
int x=0,y=0,p=0;
u8 USART_RX_BUF3[USART_REC_LEN_3];     //���ջ���,���USART_REC_LEN���ֽ�.
 void MyusartInit3(u16 bound)
{
	 USART_InitTypeDef USART_InitStruct1;                        //��������1�ṹ�����
	 GPIO_InitTypeDef  GPIO_InitStruct1;                         //����GPIO1�ṹ�����
	 NVIC_InitTypeDef  NVIC_InitStruct1;                         //�����ж�1�ṹ�����

	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);      //GPIOAʱ��ʹ��
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);     //����1ʱ��ʹ��
	
	
	
	 GPIO_InitStruct1.GPIO_Mode=  GPIO_Mode_AF_PP;             //1.��ͬ��ģʽ�ֿ���ʼ��
	 GPIO_InitStruct1.GPIO_Pin= GPIO_Pin_10;   //TX
	 GPIO_InitStruct1.GPIO_Speed=GPIO_Speed_50MHz ;
	 GPIO_Init(GPIOB,&GPIO_InitStruct1);
	
	 GPIO_InitStruct1.GPIO_Mode=  GPIO_Mode_IN_FLOATING;
	 GPIO_InitStruct1.GPIO_Pin= GPIO_Pin_11;   //RX
	 GPIO_Init(GPIOB,&GPIO_InitStruct1);                        //2.�����������벻��Ҫ���Ƶ��
	
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
u8 USART2_RX_Finish=1; // USART2������ɱ�־��
//void DMA_Configuration(void)
//{

//DMA_InitTypeDef DMA_InitStructure;
///* DMA clock enable */	
//NVIC_InitTypeDef NVIC_InitStructure;                         //�����ж�1�ṹ�����
//	
//	
//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//DMA1

///* DMA1 Channel5 (triggered by USART2 Tx event) Config */
//DMA_DeInit(DMA1_Channel7);
//DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(0x40013804);//0x40013804 &USART2->DR
//DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART2_SEND_DATA;	
//DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;				//��������->�ڴ�
//DMA_InitStructure.DMA_BufferSize = 512;
//DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//DMA_InitStructure.DMA_Priority = DMA_Priority_High;		//���ȼ�Ϊ�� �ǳ���ΪVeryHigh
//DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;					//û������Ϊ�ڴ�->�ڴ�
//DMA_Init(DMA1_Channel7, &DMA_InitStructure);					//��ʼ��DMA
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


//����ͨѶ��ʽ���ռ���
void USART2_IRQHandler(void)
{
//	static uint8_t k=0,rebuf[9]={0};
//	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�
//	{
//		rebuf[k++] =USART_ReceiveData(USART2);//(USART1->DR);	//��ȡ���յ�������
//		if(!(rebuf[0]=='D'))//���֡ͷ�����建��
//		 {
//		 	k=0;
//			rebuf[0]=0;
//		 }
//		  if(k==9)//���ݽ������
//		 {
//			if(rebuf[8]=='m')//�ж�֡β����ȷ�������̬��
//			{
//			Distance=(rebuf[3]-30)*1000+(rebuf[5]-30)*100+(rebuf[6]-30)*10+(rebuf[7]-30)-20000;
//					LCD_ShowString(120,90,200,16,16,"d=");	
//					LCD_ShowxNum(120,110,r,6,16,0X80);
//			}
//			k=0;//�建��
////		USART_SendData(USART2,r);
////		while(USART_GetFlagStatus(USART2,USART_FLAG_TC) != SET);
//	   } 
////	USART_ClearFlag(USART2,USART_FLAG_TC);

//	}


	static uint8_t k=0,rebuf[9]={0};
	
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�
	{
		rebuf[k++] =USART_ReceiveData(USART2);//(USART1->DR);	//��ȡ���յ�������
		if(!(rebuf[0]=='D'))//���֡ͷ�����建��
		 {
		 	k=0;
			rebuf[0]=0;
		 }
		  if(k==9)//���ݽ������
		 {
			if(rebuf[8]=='m')//�ж�֡β����ȷ�������̬��
			{
			Distance=(rebuf[3]-30)*1000+(rebuf[5]-30)*100+(rebuf[6]-30)*10+(rebuf[7]-30)-20000;
					LCD_ShowString(120,90,200,16,16,"the d=");	
					LCD_ShowxNum(120,110,r,6,16,0X80);
			}
			k=0;//�建��
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
//			 DMA_Cmd(DMA1_Channel6,DISABLE);    //�ر�DMA
//       num = USART2->SR;
//       num = USART2->DR; //��USART_IT_IDLE��־
//			DMA_ClearFlag( DMA1_FLAG_TC5 );  

//       num = 512 -  DMA_GetCurrDataCounter(DMA1_Channel6);      //�õ������������ݸ���
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
//			 DMA_ClearFlag(DMA1_FLAG_GL6 | DMA1_FLAG_TC6 | DMA1_FLAG_TE6 | DMA1_FLAG_HT6);//���־
//       DMA1_Channel6->CNDTR=512;       //�������ý������ݸ���        
//       DMA_Cmd(DMA1_Channel6,ENABLE);  //����DMA
//				USART_ClearITPendingBit(USART2, USART_IT_TC);
//        USART_ClearITPendingBit(USART2, USART_IT_IDLE);
//		}
//	}
int Rx_Counter=0,Rx_Fin;
void USART3_IRQHandler(void)                	//����3�жϷ������
	{
	static uint8_t k=0,rebuf[9]={0};
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
		{
		 rebuf[k++] =USART_ReceiveData(USART3);	//��ȡ���յ�������
		 LCD_ShowString(60,170,200,16,16,"ok");	
			LCD_ShowxNum(60,220,rebuf[0],6,16,0X80);
		 if(!(rebuf[0]==0xA5))//���֡ͷ�����建��
		 {
			k=0;
		 }
		 if(k==8)//���ݽ������
		 {
			if(rebuf[8]==0x0D)//�ж�֡β����ȷ�������̬��
			{
				PosX=rebuf[1]*100+rebuf[2];
				PosY=rebuf[3]*100+rebuf[4];
				PosAngle=rebuf[5]*100+rebuf[6];
				LCD_ShowString(60,210,200,16,16,"OK");	
			}
			k=0;//�建��
		 }
    }
}
//	void USART3_IRQHandler(void)                	//����2�жϷ������
//	{
//		u8 Res;
//if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
//		{
//		Res =USART_ReceiveData(USART1);	//��ȡ���յ�������
//		
//		if((USART_RX_STA3&0x8000)==0)//����δ���
//			{
//			if(USART_RX_STA3&0x4000)//���յ���0x0d
//				{
//				if(Res!=0x0a)USART_RX_STA3=0;//���մ���,���¿�ʼ
//				else USART_RX_STA3|=0x8000;	//��������� 
//				}
//			else //��û�յ�0X0D
//				{
//				if(Res==0x0d)USART_RX_STA3|=0x4000;
//				else
//					{
//					USART_RX_BUF3[USART_RX_STA3&0X3FFF]=Res ;
//					USART_RX_STA3++;
//					if(USART_RX_STA3>(USART_REC_LEN_3-1))USART_RX_STA3=0;//�������ݴ���,���¿�ʼ����	  
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
//DMA_Cmd(DMA1_Channel6, DISABLE);//�ر�DMA,��ֹ�������������
//DMA1_Channel6->CNDTR = 512;//��װ��
//DMA_Cmd(DMA1_Channel6, ENABLE);//������,�ؿ�DMA
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
//		len=USART_RX_STA2&0x3fff;//�õ��˴ν��յ������ݳ���
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

