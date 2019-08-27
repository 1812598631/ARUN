#ifndef __KEY_H
#define __KEY_H
#include "sys.h"
#include "delay.h"
#include "LCD.h"
#include "stdio.h"
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

extern u8  USART_RX_BUF2[USART_REC_LEN_2]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�
extern u16 USART_RX_STA2;         		//����״̬���

extern u8  USART_RX_BUF3[USART_REC_LEN_3]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�
extern u16 USART_RX_STA3;         		//����״̬���

u16 Laser(u16 date);
void getXYP(void);
float GetPosX(void);
float GetPosY(void);
float GetAngle(void);
void clear(void);
void UART5_Send_Byte(u8 Data);//����һ���ֽڣ�
void UART5_Send_String(u8 *Data); //�����ַ�����
void MyusartInit2(u16);
void MyusartInit5(u32);
//void DMA_Configuration(void);
unsigned long strtou32(char *str);
u16 Laser_Distance(void);
float ConvertTo32(int la, int lb, int lc, int ld);
#endif
