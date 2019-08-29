#ifndef _move_base_H
#define _move_base_H
#include <math.h>
#include "pid.h"
#include "sys.h"
#include "point2point.h"
#include "timer.h"
#include "key.h"
#include "lcd.h"
#include "ps2.h"


#define PI (3.141593f)


//函数声明

void set_point(u16 x,u16 y,u16 p);

void ps2_move(void);
void walk_point(int x,int y, int p);
void move_to_pos(float x,float y,float angle);
void minimum_Turn(float angle);
void forward_Turn(float angle,float gospeed);
void back_Turn(float angle,float gospeed);
float DistancePid(float distance);
float Distance_Arc_Pid(float distance);
float AnglePid(float valueSet,float valueNow);
uint8_t straightLine(float A1,float B1,float C1,uint8_t dir,float setSpeed);
void closeRound(float x,float y,float R,float clock,float forwardspeed,u8 Roundsize);
void motorCMD(int32_t motor1,int32_t motor2);
void motor_back_CMD(int32_t motor1,int32_t motor2);//电机控制 非点到点电机控制函数


#endif
