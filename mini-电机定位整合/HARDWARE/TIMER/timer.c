
#include "timer.h"


u8 flag[2]={0,0};

int32_t speed1,speed2;
//底盘运动数据
static int32_t wheel_speed[2];

 chassis_move_t chassis_move;
//PID变量清零函数
void PID_Move_Clear(chassis_move_t *chassis_move_clear);
//底盘初始化，主要是pid初始化 
void chassis_init(chassis_move_t *chassis_move_init);
//底盘数据更新
void chassis_feedback_update(chassis_move_t *chassis_move_update);
//底盘PID计算以及运动分解
void chassis_control_loop(chassis_move_t *chassis_move_control_loop);



void give_motor1(int32_t wheel_1)
{
wheel_speed[0]=wheel_1;
}
void give_motor2(int32_t wheel_2)
{
wheel_speed[1]=wheel_2;
}
void TIM2_Init(u16 per,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//使能TIM2时钟
	
	TIM_TimeBaseInitStructure.TIM_Period=per;   //自动装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc; //分频系数
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //设置向上计数模式
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //开启定时器中断
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;//定时器中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	
	
}

void TIM3_Init(u16 per,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//使能TIM3时钟
	
	TIM_TimeBaseInitStructure.TIM_Period=per;   //自动装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc; //分频系数
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //设置向上计数模式
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //开启定时器中断
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;//定时器中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	
	
}

void TIM4_Init(u16 per,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//使能TIM4时钟
	
	TIM_TimeBaseInitStructure.TIM_Period=per;   //自动装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc; //分频系数
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //设置向上计数模式
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //开启定时器中断
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;//定时器中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =5;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	
	
	TIM_Cmd(TIM4,ENABLE); //使能定时器	
}

//堵转计时中断
//堵转计时中断
//左轮
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update))
	{
		if(abs(chassis_move.motor_chassis[0].speed)>abs(chassis_move.motor_chassis[0].speed_set*0.6)||
			abs(chassis_move.motor_chassis[0].speed)<abs(chassis_move.motor_chassis[0].speed_set*1.4))
			flag[0]=0;
		else 
			flag[0]=1;
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
}
//右轮
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update))
	{
		LED0=!LED1;
		if(abs(chassis_move.motor_chassis[1].speed)>abs(chassis_move.motor_chassis[1].speed_set*0.6)||
			abs(chassis_move.motor_chassis[1].speed)<abs(chassis_move.motor_chassis[1].speed_set*1.4))
			flag[1]=0;
		else 
			flag[1]=1;
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
}

void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update))
	{
		LED1=!LED1;
////			wheel_speed[0]=6000;
////			wheel_speed[1]=6000;		//底盘数据更新
		chassis_feedback_update(&chassis_move);
//		//底盘控制PID计算
		chassis_control_loop(&chassis_move);
//		
		CAN_CMD_CHASSIS(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current);
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
}


///**
//* @brief 距离PID
//* @note	PID类型：位置型PID
//* @param valueSet：距离设定值
//* @param valueNow：当前距离值
//* @retval 电机前进基础速度
//* @attention
//*/

//float DistancePid(float distance)
//{
//    fp32 valueOut;
//    PID_param_dis[0]=55;
//    PID_param_dis[1]=0;
//    PID_param_dis[2]=0;

//    PID_Init(&PID_distance,PID_POSITION,PID_param_dis,9000.0f,1000.0f);//PID初始化
//    valueOut=PID_Calc(&PID_distance,distance,0);
//    return valueOut;
//}




///**
//* @brief 距离对角度的增量PID
//* @note	PID类型：位置型PID
//* @param valueSet：距离设定值
//* @param valueNow：当前距离值
//* @retval 电机前进基础速度
//* @attention
//*/
//float Distance_Arc_Pid(float distance)
//{
//    float valueOut;
//    PID_Init(&PID_DisArc,PID_POSITION,PID_param_DisArc,90.0f,10.0f);//位置型PID初始化
//    valueOut=PID_Calc(&PID_DisArc,distance,0);
//    return valueOut;
//}



///**
//* @brief 角度PID
//* @note	PID类型：位置型PID
//* @param valueSet：角度设定值
//* @param valueNow：当前角度值
//* @retval 电机速度差值
//* @attention
//*/
//float AnglePid(float valueSet,float valueNow)
//{
//    float err=0;
//    float valueOut=0;
//    err=valueSet-valueNow;
//    //角度突变处理
//    if(err > 180)
//    {
//        err=err-360;
//    }
//    else if(err < -180)
//    {
//        err=360+err;
//    }
//    PID_Init(&PID_angle,PID_POSITION,PID_param_angle,10000.0f,180.0f);//PID初始化
//    valueOut=PID_Calc(&PID_angle,err,0);	//PID计算转弯差值
//    return valueOut;
//}





void PID_Move_Clear(chassis_move_t *chassis_move_clear)
{
	u8 i=0;
	for(i=0;i<2;i++)
		PID_clear(&chassis_move_clear->motor_speed_pid[i]);
}

void chassis_init(chassis_move_t *chassis_move_init)
{
	uint8_t i;
	//底盘速度环pid值
	const static fp32 motor1_speed_pid[3] = {M3505_MOTOR1_SPEED_PID_KP, M3505_MOTOR1_SPEED_PID_KI, M3505_MOTOR1_SPEED_PID_KD};
	const static fp32 motor2_speed_pid[3] = {M3505_MOTOR2_SPEED_PID_KP, M3505_MOTOR2_SPEED_PID_KI, M3505_MOTOR2_SPEED_PID_KD};
	if (chassis_move_init == NULL)
	{
			return;
	}
	//初始化PID 
	for (i = 0; i < 2; i++)
	{
		chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);		
		PID_clear(&chassis_move_init->motor_speed_pid[i]);
	}
	PID_Init(&chassis_move_init->motor_speed_pid[0], PID_DELTA, motor1_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
	PID_Init(&chassis_move_init->motor_speed_pid[1], PID_DELTA, motor2_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
	//更新一下数据
	chassis_feedback_update(chassis_move_init);
}
void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	uint8_t i = 0;
	if (chassis_move_update == NULL)
	{
		return;
	}
	for (i = 0; i < 2; i++)
	{
			//更新电机速度，加速度是速度的PID微分
		chassis_move_update->motor_chassis[i].speed = chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
		chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0];
	}
}

void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	uint8_t i = 0;
/*************************************目标速度******************************************/
//	 wheel_speed[0]=0;
//	 wheel_speed[1]=0;
//	printf(" %5d %5d\r\n",wheel_speed[0],wheel_speed[1]);
/**************************************************************************************/		
	//计算轮子控制最大速度，并限制其最大速度
	for (i = 0; i < 2; i++)
	{
		chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
//		temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
//		if (max_vector < temp)
//		{
//			max_vector = temp;
//		}
//	}
//	if (max_vector > MAX_WHEEL_SPEED)
//	{
//		vector_rate = MAX_WHEEL_SPEED / max_vector;
//		for (i = 0; i < 2; i++)
//		{
//			chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
//		}
	}
	/*****************堵转定时器状态判断********************/
		if(abs(chassis_move_control_loop->motor_chassis[0].speed)>abs(chassis_move_control_loop->motor_chassis[0].speed_set)*0.6||
			abs(chassis_move_control_loop->motor_chassis[0].speed)<abs(chassis_move_control_loop->motor_chassis[0].speed_set)*1.4)
		{
			TIM_Cmd(TIM2,DISABLE); //关闭定时器2	
			flag[0]=0;
		}
		else
			TIM_Cmd(TIM2,ENABLE); //使能定时器2
		if(abs(chassis_move_control_loop->motor_chassis[1].speed)>abs(chassis_move_control_loop->motor_chassis[1].speed_set)*0.6||
			abs(chassis_move_control_loop->motor_chassis[1].speed)<abs(chassis_move_control_loop->motor_chassis[1].speed_set)*1.4)
		{
			TIM_Cmd(TIM3,DISABLE); //关闭定时器3	
			flag[1]=0;
		}
		else
			TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	//计算pid
	for (i = 0; i < 2; i++)
	{
		PID_motor_Calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set,flag[i]);
	}
	//赋值电流值
	for (i = 0; i < 2; i++)
	{
		chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
	}
}


