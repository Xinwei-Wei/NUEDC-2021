#include "tf-luna.h"
#include "stm32f4xx.h"
#include "softiic.h"
#include "sys.h"

u32 dis_buff[9];
u8 flag_finish=0;



void uart6_init(u32 bound){
   //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��GPIOCʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//ʹ��USART6ʱ��
 
	//����6��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_USART1); //GPIOC6����ΪUSART6
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_USART1); //GPIOC7����ΪUSART6
	
	//USART6�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOC6��GPIOC7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��PC6��PC7

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART6, &USART_InitStructure); //��ʼ������6
	
	USART_Cmd(USART6, ENABLE);  //ʹ�ܴ���6
	
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart6 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//����6�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����	
}


void USART6_IRQHandler(void)                	//����6�жϷ������
{
	u8 Res;
	static u8 i=0;
	static u8 Flag=0;
	
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART6);//(USART2->DR);	//��ȡ���յ�������
		
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


//����ģʽ��ʼ��
void Pin_6(void)
{

	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOEʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //KEY ��Ӧ����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����
	GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIOE8
	 
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



