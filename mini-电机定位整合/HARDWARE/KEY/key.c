#include "key.h"
/*
串口2：激光1
串口3：全局定位模块



*/
u16 USART_RX_STA2 = 0; //����״̬���
//float PosX = 0, PosY;
float PosX = 0, PosY;
s32 PosAngle;
u16 laser_left=0,laser_right=0;
s16 PosAngle_temp = 0;
u16 Distance;
u8 red,blue;

u8 tub_1,tub_2;		//tub1=1,2,3,4中的一个 则为红队,5,6,7,8则为蓝队。tub2=1,2,3,4中的一个则为红队,5,6,7,8则为蓝队。

void button_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);//使能PA,B,C端时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;                // HWBZ-->PE1 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;        // 上拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;         //IO口速度为50MHz
    GPIO_Init(GPIOB,&GPIO_InitStructure);                        //根据参数设定初始化GPIOE1

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;                // HWBZ-->PE1 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;        // 上拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;         //IO口速度为50MHz
    GPIO_Init(GPIOC,&GPIO_InitStructure);                        //根据参数设定初始化GPIOE1

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;                // PC2端口配置 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;        // 上拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;         //IO口速度为50MHz
    GPIO_Init(GPIOC,&GPIO_InitStructure);                        //根据参数设定初始化GPIOE1
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;                // PC3端口配置 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;        // 上拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;         //IO口速度为50MHz
    GPIO_Init(GPIOC,&GPIO_InitStructure);                        //根据参数设定初始化GPIOE1
		
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;                // PC0端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;        // 上拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;         //IO口速度为50MHz
    GPIO_Init(GPIOC,&GPIO_InitStructure);                        //根据参数设定初始化GPIOE1
		

}
void MyusartInit2(u32 bound)
{
	//A2TX  A3RX
    USART_InitTypeDef USART_InitStruct1; 
    GPIO_InitTypeDef GPIO_InitStruct1;   
    NVIC_InitTypeDef NVIC_InitStruct1;   

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 

    GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_InitStruct1.GPIO_Pin = GPIO_Pin_2;		  //TX
    GPIO_InitStruct1.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct1);

    GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStruct1.GPIO_Pin = GPIO_Pin_3; //RX
    GPIO_Init(GPIOA, &GPIO_InitStruct1);	

    USART_InitStruct1.USART_BaudRate = bound;
    USART_InitStruct1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct1.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct1.USART_Parity = USART_Parity_No;
    USART_InitStruct1.USART_StopBits = USART_StopBits_1;
    USART_InitStruct1.USART_WordLength = USART_WordLength_8b;

    USART_Init(USART2, &USART_InitStruct1);
    USART_Cmd(USART2, ENABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ClearFlag(USART2, USART_FLAG_TC); 

    NVIC_InitStruct1.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct1.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct1.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct1.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStruct1);
}
u16 USART_RX_STA3 = 0; 
int x = 0, y = 0, p = 0;
u8 USART_RX_BUF3[USART_REC_LEN_3];


void MyusartInit5(u32 bound)
{

    		//	 PC12	TX
    		//	 PD2 	RX

    USART_InitTypeDef USART_InitStruct1;
    GPIO_InitTypeDef GPIO_InitStruct1;
    NVIC_InitTypeDef NVIC_InitStruct1;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

    GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_InitStruct1.GPIO_Pin = GPIO_Pin_12;	  //TX
    GPIO_InitStruct1.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStruct1);

    GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStruct1.GPIO_Pin = GPIO_Pin_2; //RX
    GPIO_Init(GPIOD, &GPIO_InitStruct1);	

    USART_InitStruct1.USART_BaudRate = bound;
    USART_InitStruct1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct1.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct1.USART_Parity = USART_Parity_No;
    USART_InitStruct1.USART_StopBits = USART_StopBits_1;
    USART_InitStruct1.USART_WordLength = USART_WordLength_8b;

    USART_Init(UART5, &USART_InitStruct1);
    USART_Cmd(UART5, ENABLE);
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);

    NVIC_InitStruct1.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStruct1.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct1.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct1.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStruct1);
}


//void USART2_IRQHandler(void)
//{
//    static uint8_t k = 0, rebuf[9] = {0};

//    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) 
//    {
//        rebuf[k++] = USART_ReceiveData(USART2); //(USART1->DR);	
//        if (!(rebuf[0] == 'D'))					
//        {
//            k = 0;
//            rebuf[0] = 0;
//        }
//        if (k == 9) //���ݽ������
//        {
//            if (rebuf[8] == 'm') //�ж�֡β����ȷ�������̬��
//            {
//                Distance = (rebuf[3] - 30) * 1000 + (rebuf[5] - 30) * 100 + (rebuf[6] - 30) * 10 + (rebuf[7] - 30) - 20000;
//                //					LCD_ShowString(120,90,200,16,16,"the d=");
//                //					LCD_ShowxNum(120,110,r,6,16,0X80);
//            }
//            k = 0;
//        }
//    }
//}
void USART2_IRQHandler(void)
{
    static uint8_t n = 0,rebuf_laser[8] = {0};
    if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
    {
        rebuf_laser[n] = USART_ReceiveData(UART5);
        n++;
			if (rebuf_laser[0]!= 0x55)	//帧头不为0x55则清零
        {
            n = 0;
            rebuf_laser[0] = 0;
        }
				else											//帧头为0x55
				{
        if (rebuf_laser[n-1]==0xaa&&n==8)	//帧尾为0xaa且接收整整一帧数据
        {
								laser_left=rebuf_laser[1];
								laser_left<<8;
								laser_left|=rebuf_laser[2];
							
								laser_right=rebuf_laser[3];
								laser_right<<8;
								laser_right|=rebuf_laser[4];
								tub_1=rebuf_laser[5];	
								tub_2=rebuf_laser[6];
								n=0;
								rebuf_laser[0] = 0;
        }
    }
	}
}

//void UART5_IRQHandler(void)
//{
//    static uint8_t m = 0,rebuf_3[9] = {0};
//    if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
//    {
//        rebuf_3[m] = USART_ReceiveData(UART5);
//        m++;
//        if (rebuf_3[0]!= 0xaa)
//        {
//            m = 0;
//            rebuf_3[0] = 0;
//        }
//        if (rebuf_3[m-1]==0x55)
//        {
//        }
//        else {
//            if(m==8)
//            {
//                PosX = rebuf_3[1];
//								PosX <<=8;
//								PosX|=rebuf_3[2];
//                PosY = rebuf_3[3];
//								PosY <<=8;
//								PosY|=rebuf_3[4];
//                PosAngle_temp = rebuf_3[5];
//                PosAngle_temp <<= 8;//高八位
//                PosAngle_temp |= rebuf_3[6];
//								PosAngle=-(PosAngle_temp);
//                m = 0;
//                rebuf_3[0]=0;
//            }
//        }
//    }
//}



void UART5_IRQHandler(void)
{
    static uint8_t m = 0,rebuf_3[12] = {0};
    if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
    {
        rebuf_3[m] = USART_ReceiveData(UART5);
        m++;
        if (rebuf_3[0]!= 0x55)
        {
            m = 0;
            rebuf_3[0] = 0;
        }
        if (rebuf_3[m-1]==0xaa)	//检测帧尾是否为0xaa
        {
        }
        else {
            if(m==11)
            {
                PosX = ConvertTo32(rebuf_3[4], rebuf_3[3], rebuf_3[2], rebuf_3[1]);
                PosY = ConvertTo32(rebuf_3[8], rebuf_3[7], rebuf_3[6], rebuf_3[5]);

                PosAngle_temp = rebuf_3[10];
                PosAngle_temp <<= 8;//高八位
                PosAngle_temp |= rebuf_3[9];
                PosAngle=-(PosAngle_temp);
                m = 0;
                rebuf_3[0]=0;
            }
        }
    }
}


//u16 Laser(u16 date)
//{
//USART_SendData(USART2, date);
//while (USART_GetFlagStatus(USART2, USART_FLAG_TC) != SET);
//return Distance;
//}

u8 aim_tub1(void)
{
	return tub_1;
}

u8 aim_tub2(void)
{
	return tub_2;
}

/*
u8 color_judge()
{
	switch (tub_1)
	{
		case 1:return 1;
		case 2:return 2;
		case 3:return 3;
		case 4:return 4;
	}
	switch (tub_2)
	{
		case 5:return 5;
		case 6:return 6;
		case 7:return 7;
		case 8:return 8;
	}
}
*/

float GetPosX(void)
{
    return PosX/100;
}
float GetPosY(void)

{
    return PosY/100;
}

float GetAngle(void)
{
    float angle_templ;
    angle_templ=(float)PosAngle;
    return angle_templ/100;
}

void clear(void)
{
    u8 clear1[8]="clearall";
    UART5_Send_String(clear1);
    UART5_Send_Byte(0x0d);

    UART5_Send_Byte(0x0a);
}
void UART2_Send_Byte(u8 Data)
{
    USART_SendData(USART2,Data);
    while( USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET );
}

void UART5_Send_Byte(u8 Data) //发送一个字节；
{
    USART_SendData(UART5,Data);
    while( USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET );
}

void UART5_Send_String(u8 *Data) //发送字符串；

{

    while(*Data)
        UART5_Send_Byte(*Data++);
}

/*
void location_refresh()
{
	

}
*/


float ConvertTo32(int la, int lb, int lc, int ld)
{
    la = 0XFFFFFF | (la << 24);
    lb = 0XFF00FFFF | (lb << 16);
    lc = 0XFFFF00FF | (lc << 8);
    ld = 0XFFFFFF00 | ld;
    return la & lb & lc & ld;
}
