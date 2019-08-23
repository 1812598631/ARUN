#include "can.h"

void CAN1_Mode_Init(u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef 				CAN_InitStructure;
	CAN_FilterInitTypeDef 	CAN_FilterInitStructure;

#if CAN_RX0_INT_ENABLE 
	NVIC_InitTypeDef 				NVIC_InitStructure;
#endif
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);		//��CAN1ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//PA�˿�ʱ�Ӵ�

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;		//PA11	   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 	 //��������ģʽ
	GPIO_Init(GPIOA, &GPIO_InitStructure);	  

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;		//PA12	   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 	 //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);	


//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_CAN1);
//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_CAN1);


//CAN��Ԫ����
	CAN_InitStructure.CAN_TTCM = DISABLE;//��ʱ�䴥��ͨ��ģʽ   
	CAN_InitStructure.CAN_ABOM = ENABLE;	//����Զ����߹���	  
	CAN_InitStructure.CAN_AWUM = DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART = ENABLE;//ʹ�ñ����Զ����� 
	CAN_InitStructure.CAN_RFLM = DISABLE;//���Ĳ�����,�µĸ��Ǿɵ�  
	CAN_InitStructure.CAN_TXFP = DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode = mode; //ģʽ���� 
	CAN_InitStructure.CAN_SJW = tsjw;//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1 = tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2 = tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler = brp;//��Ƶϵ��(Fdiv)Ϊbrp+1	
	CAN_Init(CAN1, &CAN_InitStructure);// ��ʼ��CAN1

//���ù�����
	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit; //16λ 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;//16λID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//16λMASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��

#if CAN_RX0_INT_ENABLE 
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    

	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     // �����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
}
