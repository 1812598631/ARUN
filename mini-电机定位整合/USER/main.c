#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "can.h"
#include "point2point.h"
#include "pid.h"
#include "lcd.h"
#include "led.h"
#include "timer.h"
#include "math.h"
#include "move_base.h"
#include "main.h"
#include "ps2.h"


#define PI (3.141593f)
static float laser_distance;
fp32 Px,Py;
float Pp;
fp32 v;
/************************************************
所有变量P均为角度
距离PID及角度PID
两种行走方式：
POINT2POINT
距离角度差速
************************************************/
/************************************************
8-13更新：
目前遇到的问题：
PID的参数应该在外部定义 不应该是PID计算中定义
下次更新修改
以及将函数打包为库文件
************************************************/

/************************************************
8-14更新：
目前遇到的问题：
PID参数定义完成 每次调用PID前函数内赋值
定时器取样未完成
想到改进方向：
将函数打包为库文件未完成
************************************************/

/************************************************
8-15更新：
目前解决问题：
底盘运动函数完成
底盘运动函数打包为库文件已完成
PID取样定时器配置完成
中断函数部分待与全局定位整合后完成
可能遇到的问题：
PID参数繁多 难调
定位不精准 路径无法用坐标规划
想到改进方向：
************************************************/

/************************************************
8-16凌晨更新：
目前待解决问题：
move_to_pos()函数有关键性错误：
目标点与当前点距离过大时
当角度PID修正后，当前前进方向与目标方向平行
距离PID无法调节与目标点的距离形成闭环
可能遇到的问题：
PID参数繁多 难调
定位不精准 路径无法用坐标规划
想到改进方向：
分段PID  move_to_pos()函数计算方式修改
改为距离过大时 用直线——角度PID+角度PID
当底盘运行到 原move_to_pos()调节范围内
可使得小车在目标点停止
************************************************/

/************************************************
8-18傍晚更新：
目前待解决问题：

电机底盘整合成功
串口中断无法更新激光数据
motorCMD函数重写  整个底盘动作逻辑需要重新写
定时器PID取样未解决
PS2自动挡/手动挡未解决

可能遇到的问题：
PID参数繁多 难调
定位不精准 路径无法用坐标规划
上位机通讯无法传输数据
定时器中断影响主函数运行
************************************************/


/************************************************
8-24凌晨更新：
目前待解决问题：
定位底盘整合
激光接受数据已成功
失败原因：未将激光与单片机共地，导致相对低电平并不相同
串口通讯未
LCD显示屏不能显示负值
电机赋值时两种方式  数值取反+1	/   直接添-号
未明确区别，暂时无影响
串口发送float为32位数据 分四个字节 前两个字节为整数部分  后两个字节为小数部分

motorCMD函数重写  整个底盘动作逻辑需要重新写
定时器PID取样未解决
PS2初步加入，通讯不成功，自动挡/手动挡未解决

分球：OPENMV寻找矩形中心调整步进电机旋转  从而使分球结构回原点  未调试
颜色变化已解决，原因为初始上电时白平衡未关闭  对后面关闭时起对比作用
PID取值未解决
撞边程序：   直线撞中间储藏室后倒退至限位开关触发 射光弹仓后再次出发

分球可改为 循迹模块 循白线使得步进电机归原点

可能遇到的问题：
PID参数繁多 难调
定位不精准 路径无法用坐标规划
上位机通讯无法传输数据
定时器中断影响主函数运行
************************************************/


/************************************************
8-24
目前待解决问题：
串口接收到的字符数组无法转化为字符串
则无法使用字符串函数对float xyp进行处理

串口中断部分已经写完，待理清逻辑
未调分球转盘


可能遇到的问题：
PID参数繁多 难调
定位不精准 路径无法用坐标规划
上位机通讯无法传输数据
定时器中断影响主函数运行
************************************************/

/************************************************
8-26
目前待解决问题：


串口3引脚与LCD冲突

改为串口5

未调分球转盘
串口接受已完成

减少了串口中断中的程序，使得串口得以正确接收

总体逻辑未写

各部分模块程序基本完成
射球模块每次射球后需要5秒将射球电机恢复

各部分程序未对复数 角度转换进行处理
PID定时器计算
各部分PID的使用种类 增量式1/位置式0

可能遇到的问题：
PID参数繁多 难调
定位不精准 路径无法用坐标规划

************************************************/




/************************************************
作者：阿RUN @SAU——科协
************************************************/


int main(void)
{
//	 int32_t motor_v1=3000;
//int32_t motor_v2=3000;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
    uart_init(115200);//串口初始化为115200


    delay_init();//延时函数初始化
    MyusartInit2(19200);//串口2
    MyusartInit5(115200);
    LCD_Init();

    CAN1_Mode_Init(CAN_SJW_1tq, CAN_BS2_3tq, CAN_BS1_8tq, 3, CAN_Mode_Normal);  //CAN初始化模式,波特率1Mbps
    chassis_init(&chassis_move);//底盘初始化
    PS2_Init();					//PS2管脚初始化
    PS2_SetInit();		 //PS2配置初始化,配置“红绿灯模式”，并选择是否可以修改
    //开启震动模式
    delay_ms(50);
    TIM2_Init(3000,36000-1);//1.5秒定时
    TIM3_Init(3000,36000-1);//1.5秒定时
    TIM4_Init(2000,72-1);
    delay_ms(20);
//	 		motorCMD(1,0,v);
//		motorCMD(2,0,v);
//		give_motor1(motor_v1);
//give_motor2(motor_v2);
    delay_ms(5000);
    clear();
    while(1)
    {
        Px=GetPosX();
        Py=GetPosY();
        Pp=GetAngle();
        if(Px<0)
        {
            LCD_ShowString(60,50,200,16,16,"x=");
            LCD_ShowString(55,70,200,16,16,"-");
            LCD_ShowxNum(60,70,-Px,6,16,0X80);
        }
        else
        {
            LCD_ShowString(60,50,200,16,16,"x=");
					            LCD_ShowString(55,70,200,16,16," ");

            LCD_ShowxNum(60,70,Px,6,16,0X80);
        }
        if(Py<0)
        {
            LCD_ShowString(60,90,200,16,16,"y=");
            LCD_ShowString(55,110,200,16,16,"-");
            LCD_ShowxNum(60,110,-Py,6,16,0X80);
        }
        else
        {
            LCD_ShowString(60,90,200,16,16,"y=");
					            LCD_ShowString(55,110,200,16,16," ");

            LCD_ShowxNum(60,110,Py,6,16,0X80);
        }
        if(Pp<0)
        {   
					  LCD_ShowString(60,130,200,16,16,"Angle=");
            LCD_ShowString(55,150,200,16,16,"-");
            LCD_ShowxNum(60,150,-Pp,6,16,0X80);
        }
        else
        {
            LCD_ShowString(60,130,200,16,16,"Angle=");
            LCD_ShowString(55,150,200,16,16," ");
            LCD_ShowxNum(60,150,Pp,6,16,0X80);
        }
				printf("%d,",Pp);
//LCD_ShowString(60,50,200,16,16,"x=");
//LCD_ShowxNum(60,70,Px,6,16,0X80);
//LCD_ShowString(60,90,200,16,16,"y=");
//LCD_ShowxNum(60,110,Py,6,16,0X80);
//LCD_ShowString(60,130,200,16,16,"Angle=");
//LCD_ShowxNum(60,150,Pp,6,16,0X80);
//		walk_point();		//点到点测试
closeRound(0,2400,1500,1,1500);
//		laser_distance=Laser(0x44);
//		LCD_ShowString(60,50,200,16,16,"distance=");
//		LCD_ShowxNum(60,70,laser_distance,3,16,0X80);


            //delay_ms(20);
//		ps2_move();
//		v=DistancePid(laser_distance);
//		LCD_ShowString(60,90,200,16,16,"v=");
//		LCD_ShowxNum(60,110,v,5,16,0X80);
//	 		motorCMD(v,v);

        }
    }
