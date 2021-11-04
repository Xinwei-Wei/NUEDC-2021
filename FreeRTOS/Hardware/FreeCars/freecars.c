#include "freecars.h"
#include "pid.h"
#include "includes.h"
char uSendBuf[ScopeChaNum * 2]; //待发送给上位机的数据

#if EN_USART2_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误
//u8 USART2_RX_BUF[USART2_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
//u16 USART2_RX_STA = 0;     //接收状态标记

#define UartRxBufferLen  100
#define UartRxDataLen    41           //上位机发送浮点数据MCU接收，不要改
#define UartRxCmdLen     7	      //上位机接收命令数据长度，不要改
//float a1, a2, b1, b2, c1, c2;
typedef struct
{
    int Stack;   //堆栈记录 总是指向下一个数据位
    u8 Data;     //记录从本次接收到的数据
    u8 PreData;
    u8 Buffer[UartRxBufferLen];  //从上位机接收到的数据
    u8 Enable;
    u8 Check;
} SerialPortType;

SerialPortType SerialPortRx;
double UartData[9];          //从上位机接收到的数据


void Page0_debug(void)
{
}

void Page1_debug(void)
{
}

void Page2_debug(void)
{
}

void Page3_debug(void)
{
}

void Page4_debug(void)
{
    //    CarSpeedCtr.Ki=UartData[4];
}

void Page5_debug(void)
{
    //  CarSpeedCtr.Kd=UartData[5];
}

void Page6_debug(void)
{
    //   k=UartData[6];
}

void Page7_debug(void)
{
    //  CarSpeedCtr.SpeedSet=UartData[7];
}

void UartDebug(void)
{
    switch((u16)UartData[8])
    {
    case 0:
        Page0_debug();
        break;
    case 1:
        Page1_debug();
        break;
    case 2:
        Page2_debug();
        break;
    case 3:
        Page3_debug();
        break;
    case 4:
        Page4_debug();
        break;
    case 5:
        Page5_debug();
        break;
    case 6:
        Page6_debug();
        break;
    case 7:
        Page7_debug();
        break;
    }

}

void Initial_USART2(u32 baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure;
	
    /* ?? UART2 ?????  ?? UART2???????PA???*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//??GPIOA??2,3(TX,RX)

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); //GPIOA2???USART2     TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); //GPIOA3???USART2     RX
    //USART2????
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2?GPIOA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//????
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//??50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //??????
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //??
    GPIO_Init(GPIOA, &GPIO_InitStructure); //???PA2,PA3
    //USART2?????
    USART_InitStructure.USART_BaudRate = baudrate;//?????
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//???8?????
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//?????
    USART_InitStructure.USART_Parity = USART_Parity_No;//??????
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//????????
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//????
    USART_Init(USART2, &USART_InitStructure); //?????2

    USART_Cmd(USART2, ENABLE);  //????2

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//??????
    //Usart2 NVIC ??
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//??2????
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //?????0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//????0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ????
    NVIC_Init(&NVIC_InitStructure);	//??????????VIC????

}

/******************************************************************
*???????????,
*4bytes????
*4*9bytes???(????1--9???,?????4?)
*1byte???
*******************************************************************/
void USART2_IRQHandler(void)
{
    u32 i = 0, b = 0, d;
    u32 dat_temp;
//    Buzzer_GetOffSetTmOver();//?????????
	
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //??????
    {
		PAout(6) = !PAout(6);
        SerialPortRx.Data = USART_ReceiveData(USART2);
        if( SerialPortRx.Stack < UartRxBufferLen )  //Stack?(0~Len-1)
            SerialPortRx.Buffer[SerialPortRx.Stack++] = SerialPortRx.Data;//stack??????????????
        //UartRxDataLen 41?????   41=4(???4bytes)+9*4(9???double?)+1(byte???)
        if( SerialPortRx.Stack >= UartRxDataLen
                && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxDataLen]  == 0xff //????
                && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxDataLen + 1] == 0x55
                && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxDataLen + 2] == 0xaa
                && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxDataLen + 3] == 0x10 )
        {   //double data 9???????   (?????????32?)
            SerialPortRx.Check = 0;
            b = SerialPortRx.Stack - UartRxDataLen; //???,????????

            for(i = b; i < SerialPortRx.Stack - 1; i++) //????????????
            {
                SerialPortRx.Check += SerialPortRx.Buffer[i];//??
            }

            //????????????????????
            if( SerialPortRx.Check == SerialPortRx.Buffer[SerialPortRx.Stack - 1] )
            {
                for(i = 0; i < 9; i++)
                {
                    //32? ?16??????,?16??????
                    dat_temp = SerialPortRx.Buffer[b + i * 4 + 4] * 0x1000000L
                               + SerialPortRx.Buffer[b + i * 4 + 5] * 0x10000L
                               + SerialPortRx.Buffer[b + i * 4 + 6] * 0x100L
                               + SerialPortRx.Buffer[b + i * 4 + 7];

                    if(dat_temp > 0x7FFFFFFF)  d = 0x7FFFFFFF- dat_temp  ; //??
                    else       d = dat_temp  ;

                    UartData[i] = d;
                    UartData[i] /= 65536.0;
										

                }
								Page0_debug();
								Page1_debug();
								Page2_debug();
								Page3_debug();
								Page4_debug();
								Page5_debug();
								Page6_debug();
								Page7_debug();
            }
            SerialPortRx.Stack = 0;
        }
        //????????,?????
        else if(   SerialPortRx.Stack >= UartRxCmdLen //UartRxCmdLen = 7?????
                   && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxCmdLen]  == 0xff
                   && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxCmdLen + 1] == 0x55
                   && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxCmdLen + 2] == 0xaa
                   && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxCmdLen + 3] == 0x77 ) //cmd
        {
            SerialPortRx.Check = 0;
            b = SerialPortRx.Stack - UartRxCmdLen; //???
            for(i = b; i < SerialPortRx.Stack - 1; i++) //???????????
            {
                SerialPortRx.Check += SerialPortRx.Buffer[i];//??
            }
            if( SerialPortRx.Check == SerialPortRx.Buffer[SerialPortRx.Stack - 1] )
            {   //????
                //UartCmd(UartCmdNum,UartCmdData);//????????,??MCU????
            }
            SerialPortRx.Stack = 0;
        }
    }

    else        //?????????????????
    {
        SerialPortRx.Stack = 0;
    }
}

#endif

//将要发送上位机的数据压入缓存
void push(char chanel, u16 data)
{
    uSendBuf[chanel * 2] = data / 256;
    uSendBuf[chanel * 2 + 1] = data % 256;
}


/*--------------------------单片机发送数据------------------------*/
//发送一个字符给串口
void uartSendChar(USART_TypeDef* USARTx, u8 sendData)
{
    USART_SendData(USARTx, sendData);         //向串口1发送数据
    while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) != SET); //等待发送结束
}

//将缓冲数据发送给上位机
void sendDataToScope(void)
{
    u8 i, sum = 0;

    //使用轮询的方式发送数据，当数据未发送，程序停在此处直到发送完成
    uartSendChar(MYUART, 251);
    uartSendChar(MYUART, 109);
    uartSendChar(MYUART, 37); //通知上位机要发送数据了

    sum += (251);    //全部数据加入校验
    sum += (109);
    sum += (37);

    for(i = 0; i < ScopeChaNum * 2; i++) //把压入缓存中的数据一次给通道0--16
    {
        uartSendChar(MYUART, uSendBuf[i]);
        sum += uSendBuf[i];       //全部数据加入校验
    }

    uartSendChar(MYUART, sum); //最后一位数据是校验位
}

///*-------------------------------------------------------------------------------*/

////void pushLineData(Int16 ccd,Byte *data)
////{
////	Int16 i;
////	Int16 head = ScopeChaNum*2;
////	uSendBuf[ScopeChaNum*2] = ccd;//CCD选择，选择显示发送CCD的数据
////	for(i=0;i<LineDataNum;i++) //转移到发送缓冲区
////	{
////		uSendBuf[head + i + 2] = data[i];
////	}
////}

///*-------------------------------------------------------------------------------*/
