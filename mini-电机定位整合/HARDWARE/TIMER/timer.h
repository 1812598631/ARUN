#ifndef _timer_H
#define _timer_H

#include "stm32f10x.h"
#include "can.h"
#include "led.h"
#include "CAN_Receive.h"
#include "move_base.h"
#include "math.h"
#include "pid.h"
#include "delay.h"
#include "stdlib.h"
#include "usart.h"


//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f

//m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������
//#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
//#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//�����������Ƶ�ʣ���δʹ�������
//#define CHASSIS_CONTROL_FREQUENCE 500.0f
//���̵������ٶ�
//#define MAX_WHEEL_SPEED 4.0f
//�����˶��������ǰ���ٶ�
//#define NORMAL_MAX_CHASSIS_SPEED_X 3.0f

//���̵���ٶȻ�PID
#define M3505_MOTOR1_SPEED_PID_KP 2.05f
#define M3505_MOTOR1_SPEED_PID_KI 0.01f
#define M3505_MOTOR1_SPEED_PID_KD 0.08f

#define M3505_MOTOR2_SPEED_PID_KP 2.10f
#define M3505_MOTOR2_SPEED_PID_KI 0.01f
#define M3505_MOTOR2_SPEED_PID_KD 0.08f

#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f


typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} Chassis_Motor_t;		//motor_measure_t ������"CAN_Receive.h"

typedef struct
{
  Chassis_Motor_t motor_chassis[2];          //���̵������
  PidTypeDef motor_speed_pid[2];             //���̵���ٶ�pid
} chassis_move_t;

//�����˶�����
extern chassis_move_t chassis_move;
//���̳�ʼ������Ҫ��pid��ʼ��
extern void chassis_init(chassis_move_t *chassis_move_init);
void chassis_feedback_update(chassis_move_t *chassis_move_update);
void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
void PID_Move_Clear(chassis_move_t *chassis_move_clear);
void give_motor1(int32_t wheel_1);
void give_motor2(int32_t wheel_2);
void TIM2_Init(u16 per,u16 psc);
void TIM3_Init(u16 per,u16 psc);
void TIM4_Init(u16 per,u16 psc);
#endif


