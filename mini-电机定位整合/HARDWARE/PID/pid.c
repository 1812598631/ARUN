/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#define  __PID_GLOBALS
#include "pid.h"
#include "lcd.h"




#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void PID_Init(PidTypeDef *pid, char mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}
void PID_para_init(PidTypeDef *pid,const fp32 PID[3])
{
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
}
fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set)
{
//    if (pid == NULL)
//    {
//        return 0.0f;
//    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = ref-set;
		
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);

    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
				LCD_ShowString(150,190,200,16,16,"error=");	
		LCD_ShowxNum(150,210,pid->out,6,16,0X80);
    return pid->out;
}

fp32 PID_motor_Calc(PidTypeDef *pid, fp32 ref, fp32 set, char sign)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
			if(ref>=set*0.85||sign==1||ref<=set*1.15)
        pid->Iout = pid->Ki * pid->error[0];
			else 
				pid->Iout = 0;
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}


//float pid_control_gyro_left(PidTypeDef *pid,float ref,float set)
//{
//    pid->set = set;
//    pid->fdb = ref;
//    pid->error[0] = set - ref;  
//	  if(pid->error[0]<95&&pid->error[0]>45)  pid->out=-4;
//	  if(pid->error[0]<45&&pid->error[0]>20)  pid->out=-3.5;
//	  if(pid->error[0]<20&&pid->error[0]>10)  pid->out=-2.5;
//	  if(pid->error[0]>-95&&pid->error[0]<-45)  pid->out=4;
//	  if(pid->error[0]>-45&&pid->error[0]<-20)  pid->out=3.5;
//	  if(pid->error[0]>-20&&pid->error[0]<-10)  pid->out=2.5;
//	  if(pid->error[0]<5&&pid->error[0]>-5)  pid->out=0;
//	  return pid->out;
//}
//float pid_control_gyro_right(PidTypeDef *pid,float ref,float set)
//{
//	  pid->set = set;
//    pid->fdb = ref;
//    pid->error[0] = set - ref;  
//	  if(pid->error[0]<95&&pid->error[0]>45)  pid->out=4;
//	  if(pid->error[0]<45&&pid->error[0]>20)  pid->out=3.5;
//	  if(pid->error[0]<20&&pid->error[0]>10)  pid->out=2.5;
//	  if(pid->error[0]>-95&&pid->error[0]<-45)  pid->out=-4;
//	  if(pid->error[0]>-45&&pid->error[0]<-20)  pid->out=-3.5;
//	  if(pid->error[0]>-20&&pid->error[0]<-10)  pid->out=-2.5;
//	  if(pid->error[0]<5&&pid->error[0]>-5)  pid->out=0;
//	  return pid->out;
//}


//float pid_control_distance_left(PidTypeDef *pid,float ref,float set)
//{
//    pid->error[1] = pid->error[0];
//    pid->set = set;
//    pid->fdb = ref;
//    pid->error[0] = set - ref;  
//		if(pid->error[0]<500&&pid->error[0]>30)  pid->out=1;
//	  if(pid->error[0]>-500&&pid->error[0]<-30)  pid->out=2;
//		if(pid->error[0]<30&&pid->error[0]>-30)  pid->out=0;
//	  return pid->out;
//}
//float pid_control_distance_right(PidTypeDef *pid,float ref,float set)
//{
//	  pid->error[1] = pid->error[0];
//    pid->set = set;
//    pid->fdb = ref;
//    pid->error[0] = set - ref;  
//		if(pid->error[0]<500&&pid->error[0]>30)  pid->out=2;
//	  if(pid->error[0]>-500&&pid->error[0]<-30)  pid->out=1;
//		if(pid->error[0]<30&&pid->error[0]>-30)  pid->out=0;
//	  return pid->out;
//}
//float pid_control_distance_all(PidTypeDef *pid,float ref,float set)
//{
//	  pid->error[1] = pid->error[0];
//    pid->set = set;
//    pid->fdb = ref;
//    pid->error[0] = set - ref;  
//	  if(pid->error[0]<1000&&pid->error[0]>500)  pid->out=0.6;
//		if(pid->error[0]<500&&pid->error[0]>100)  pid->out=0.2;
//	  if(pid->error[0]>-1000&&pid->error[0]<500)  pid->out=-0.6;
//	  if(pid->error[0]>-500&& pid->error[0]<-100)pid->out=-0.2;
//	  if(pid->error[0]>-50&&pid->error[0]<50)pid->out=0;
//	  return pid->out;
//}

//void PID_Init_distance(void)           //初始化变量
//{
//	distance_left.Kd=5;
//	distance_left.out=0;
//}
	








