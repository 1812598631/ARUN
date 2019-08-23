
#include "CAN_Receive.h"
#include "stm32f10x.h"


//���̵�����ݶ�ȡ

#define get_motor_measure(ptr, RxMessage)                                                      \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((RxMessage)->Data[0] << 8 | (RxMessage)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((RxMessage)->Data[2] << 8 | (RxMessage)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((RxMessage)->Data[4] << 8 | (RxMessage)->Data[5]); \
        (ptr)->temperate = (RxMessage)->Data[6];																							\
    }


//ͳһ����can���պ���
static void CAN_hook(CanRxMsg *RxMessage);
//�����������
static motor_measure_t  motor_chassis[2];

//static CanTxMsg GIMBAL_TxMessage;


//can1�ж�
#if CAN_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	static CanRxMsg RxMessage;
	CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	CAN_hook(&RxMessage);
}
#endif

////CAN ���� 0x700��ID�����ݣ�������M3508�����������IDģʽ
//void CAN_CMD_CHASSIS_RESET_ID(void)
//{

//	u8 mbox;
//	u16 i;
//	CanTxMsg TxMessage;
//	TxMessage.StdId = 0x700;
//	TxMessage.IDE = CAN_ID_STD;
//	TxMessage.RTR = CAN_RTR_DATA;
//	TxMessage.DLC = 0x08;
//	for(i=0;i<8;i++)	
//		TxMessage.Data[i] = 0;

//	mbox=CAN_Transmit(CAN1, &TxMessage);
//	i=0;
//	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0xFFF))i++;	//�ȴ����ͽ���
//}

//���͵��̵����������
void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2)
{
	u8 mbox;
	u16 i;
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_CHASSIS_ALL_ID;// ��׼��ʶ��Ϊ0x200
	TxMessage.IDE = CAN_ID_STD; // ������չ��ʾ����29λ��
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = motor1 >> 8;
	TxMessage.Data[1] = motor1;
	TxMessage.Data[2] = motor2 >> 8;
	TxMessage.Data[3] = motor2;
	TxMessage.Data[4] = 0;
	TxMessage.Data[5] = 0;
	TxMessage.Data[6] = 0;
	TxMessage.Data[7] = 0;

	mbox=CAN_Transmit(CHASSIS_CAN, &TxMessage);
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0xFFF))i++;	//�ȴ����ͽ���
}

//void CAN_Receive_Msg(void)
//{
//	static CanRxMsg RxMessage;

////	if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
////	{
////		LCD_ShowString(10,80,tftlcd_data.width,tftlcd_data.height,16,"ok");
//		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
//		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
//		CAN_hook(&RxMessage);
////	}
//}

//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
   return &motor_chassis[(i & 0x01)];
}

//ͳһ����can�жϺ��������Ҽ�¼�������ݵ�ʱ�䣬��Ϊ�����ж�����
static void CAN_hook(CanRxMsg *RxMessage)
{
//	s16 spd;
	switch (RxMessage->StdId)
	{
		case CAN_3508_M1_ID:
		case CAN_3508_M2_ID:
		{
			static uint8_t i = 0;
			//������ID��
			i = RxMessage->StdId - CAN_3508_M1_ID;
			//���������ݺ꺯��
			get_motor_measure(&motor_chassis[i], RxMessage);
//			spd=motor_chassis[i].speed_rpm;
//			if(spd<0)
//				spd=~spd+1;
//			Oled_Printf_U16(10+30*i,5,spd,1);		
			break;
		}
		default:
		{
			break;
		}		
	}
}		
