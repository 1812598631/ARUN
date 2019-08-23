#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
#include "delay.h"
#include "LCD.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK miniSTM32������
//������������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/3
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   	

#define USART_REC_LEN_2  			200  	//�����������ֽ��� 200
#define USART_REC_LEN_3  			200  	//�����������ֽ��� 200

	  	extern int r;
extern u8  USART_RX_BUF2[USART_REC_LEN_2]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA2;         		//����״̬���	

extern u8  USART_RX_BUF3[USART_REC_LEN_3]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA3;         		//����״̬���	

u16 Laser(u16 date);
float GetPosX(void);
float GetPosY(void);
float GetPosAngle(void);
void MyusartInit2(u16);
void MyusartInit3(u16);
//void DMA_Configuration(void);
u16 Laser_Distance(void);
#endif
