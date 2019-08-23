
#ifndef CANTASK_H
#define CANTASK_H
#include "main.h"

#define CAN_RX0_INT_ENABLE 1   //ʹ���ж�
#define CHASSIS_CAN CAN1

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,

} can_msg_id_e;

//rm���ͳһ���ݽṹ��
typedef struct
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	int16_t last_ecd;
} motor_measure_t;

//void CAN_Receive_Msg(void);

extern void CAN_CMD_CHASSIS_RESET_ID(void);

//���͵��̵����������
extern void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2);
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����,i�ķ�Χ��0-3����Ӧ0x201-0x204,
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);


#endif
