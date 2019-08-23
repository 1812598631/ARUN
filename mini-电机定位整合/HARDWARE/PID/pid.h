/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
typedef float fp32;
typedef double fp64;
#ifndef NULL
#define NULL 0
#endif

#ifdef  __PID_GLOBALS   //�䶨������Լ���c�ļ���
#define __PID_EXT
#else
#define __PID_EXT extern
#endif

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    char mode;
    //PID ������
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //������
    fp32 max_iout; //���������

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3]; //����� 0���� 1��һ�� 2���ϴ�

} PidTypeDef;
extern void PID_Init(PidTypeDef *pid, char mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
extern fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set);
extern fp32 PID_motor_Calc(PidTypeDef *pid, fp32 ref, fp32 set, char sign);

extern void PID_clear(PidTypeDef *pid);
extern void PID_para_init(PidTypeDef *pid,const fp32 PID[3]);


//float pid_control_gyro_left(PidTypeDef *pid,float ref,float set);
//float pid_control_gyro_right(PidTypeDef *pid,float ref,float set);
//float pid_control_distance_left(PidTypeDef *pid,float ref,float set);
//float pid_control_distance_right(PidTypeDef *pid,float ref,float set);
//float pid_control_distance_all(PidTypeDef *pid,float ref,float set);
//void PID_Init_distance(void);  

//__PID_EXT   PidTypeDef distance_left,distance_right,distance_all,gyro_left,gyro_right;

#endif
