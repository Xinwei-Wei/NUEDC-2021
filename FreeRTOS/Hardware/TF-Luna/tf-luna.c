#include "tf-luna.h"
#include "stm32f4xx.h"
#include "softiic.h"
#include "sys.h"

u32 dis_buff[9];
u8 flag_finish=0;



void uart6_init(u32 bound){
   //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOC时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART6时钟
 
	//串口6对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_USART1); //GPIOC6复用为USART6
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_USART1); //GPIOC7复用为USART6
	
	//USART6端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOC6与GPIOC7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PC6，PC7

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART6, &USART_InitStructure); //初始化串口6
	
	USART_Cmd(USART6, ENABLE);  //使能串口6
	
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart6 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//串口6中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、	
}


void USART6_IRQHandler(void)                	//串口6中断服务程序
{
	u8 Res;
	static u8 i=0;
	static u8 Flag=0;
	
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART6);//(USART2->DR);	//读取接收到的数据
		
		if(Res==0x59  &&  i==0 && flag_finish==0)
		{
			dis_buff[0]=Res;
			i++;			
		}
		else if(Res==0x59  && i==1 && flag_finish==0)
		{
			dis_buff[1]=Res;
			Flag=1;
			i++;	
		}
		else if(Flag && flag_finish==0)
		{
			dis_buff[i]=Res;
			i++;			
		}
		if(i==9)
		{
			Flag=0;
			i=0;
			flag_finish=1;	
		}		
  } 
}


//开关模式初始化
void Pin_6(void)
{

	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOE时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //KEY 对应引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//浮空
	GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIOE8
	 
} 


u32 distance()
{
	u32 dis;
	u32 amp;
	if(flag_finish)
	{
		dis=dis_buff[2]+dis_buff[3]*128;
		amp=dis_buff[4]+dis_buff[5]*128;
		flag_finish=0;
		if(amp>100)
			return dis;
		else
			return 0;
	}
	return 0;
}



