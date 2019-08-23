#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
#include "delay.h"
#include "LCD.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK miniSTM32开发板
//按键驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/3
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   	

#define USART_REC_LEN_2  			200  	//定义最大接收字节数 200
#define USART_REC_LEN_3  			200  	//定义最大接收字节数 200

	  	extern int r;
extern u8  USART_RX_BUF2[USART_REC_LEN_2]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA2;         		//接收状态标记	

extern u8  USART_RX_BUF3[USART_REC_LEN_3]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA3;         		//接收状态标记	

u16 Laser(u16 date);
float GetPosX(void);
float GetPosY(void);
float GetPosAngle(void);
void MyusartInit2(u16);
void MyusartInit3(u16);
//void DMA_Configuration(void);
u16 Laser_Distance(void);
#endif
