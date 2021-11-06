#include "ccd.h"
#include "stm32f4xx.h"
#include "usart.h"
#include "adc.h"
#include "pid.h"
#include "includes.h"
#include "filter_mathod.h"

#define	delay_ms(x) vTaskDelay(x)

#define ture true
#define CCD1_CLK  	PAout(11)
#define CCD1_SI		PAout(12)
#define CCD2_CLK	PCout(11)
#define CCD2_SI		PCout(12)
#define threshold1  2500
#define line3_wide  30
#define line5_wide  70
#define threshold_Delta  300


u8 ccd_finish_flag;
u16 ccd1_data[128];
u16 ccd2_data[128];
u8 stop_line = line5_wide;
int EN_stop = 0, EN_EN_stop = 0;
int longest = 1000;
int is_find_line = -1;
extern int pharmacy_position[10];
extern int Target_pharmacy;
int en_find = 1;
int is_half_line = 0;


void CCD_Init(void)
{
	Adc_Init();
	CCD_IO();
}

//PA11:作为CLK
//PA12:作为SI
void CCD_IO(void)
{    	 
	GPIO_InitTypeDef  GPIO_InitStructure;	

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	//使能GPIOA时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	//使能GPIOD时钟

	//GPIOA初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;//CCD1对应IO口
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;			//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//50MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//初始化GPIOA
	GPIO_Init(GPIOC, &GPIO_InitStructure);					//初始化GPIOD
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      TSL1401线阵CCD数据采集
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               在isr.c里面先创建对应的中断函数，然后调用该函数(之后别忘记清除中断标志位)
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
//  @brief      TSL1401线阵CCD数据采集
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               在isr.c里面先创建对应的中断函数，然后调用该函数(之后别忘记清除中断标志位)
//-------------------------------------------------------------------------------------------------------------------
void CCD_Collect(void)
{
    u8 i = 0;

    CCD1_CLK=1;
    CCD1_SI=0;
    CCD1_CLK=0;
    CCD1_SI=1;
    CCD1_CLK=1;
    CCD1_SI=0;

    for(i=0;i<128;i++)
    {
        CCD1_CLK=0;
        //这里可以同时采集两个CCD数据
        ccd1_data[i] = Get_Adc1();
        CCD1_CLK=1;
    }

    //采集完成标志位置1
    ccd_finish_flag = 1;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      TSL1401线阵CCD图像发送至上位机查看图像
//  @param      uart_n          串口号
//  @param      uart_n          线性CCD数据指针
//  @return     void
//  @since      v1.0
//  Sample usage:               调用该函数前请先初始化串口
//-------------------------------------------------------------------------------------------------------------------
void ccd_send_data(USART_TypeDef* uart_n ,u16 *dat)
{
   u8 i = 0;
   uart_putchar(uart_n, 0x00); 
   uart_putchar(uart_n, 0xff);
   uart_putchar(uart_n, 0x01);
   uart_putchar(uart_n, 0x00);
   
    for(i=0; i<128; i++)
    {
        uart_putchar(uart_n, dat[i]>>8);   //发送高8位
        uart_putchar(uart_n, dat[i]&0XFF); //发送低8位
    }
}

//定时器7中断服务函数
void TIM7_IRQHandler(void)
{

}


//寻黑单线，返回起始位置
int find_line(void)
{
	u8 i;
	for(i=0;i<128;i++)
	{
		if(ccd1_data[i] > 2700 && ccd1_data[i+1]>2700 && ccd1_data[i+2] > 2700)
		{
			return i;			
		}
	}
	return -1;	
}


//int CCD_find_Line(int center, int threshold)
//{
//	int i, emergency_flag = 0, edge_count = 0, edge_left = 0, edge_right = 127;
//	int emergency_count = 0, emergency_max = 0, emergency_right = 0;
//	
//	threshold = OTSU(ccd1_data);
//	
//	for(i=center-2; i<=center+2; i++)
//	{
//		if(ccd1_data[i] > threshold || ccd1_data[i]< threshold_black)
//		{
//			emergency_flag = 1;
//			break;
//		}
//	}
//	
//	if(emergency_flag == 0)
//	{
//		for(i=center-3; i>=0; i--)
//		{
//			if(ccd1_data[i] > threshold || ccd1_data[i]< threshold_black)
//				edge_count++;
//			if(edge_count == 3)
//				break;
//		}
//		edge_left = i+3;
//		edge_count = 0;
//		
//		for(i=center+3; i<=127; i++)
//		{
//			if(ccd1_data[i] > threshold || ccd1_data[i]< threshold_black)
//				edge_count++;
//			if(edge_count == 3)
//				break;
//		}
//		
//			
//		edge_right = i-3;
//		edge_count = 0;
//		
//		if(edge_left < 5)
//		{
//			return 66;
//		}
//		
//		center = (edge_left + edge_right) / 2 + 0.5;
//		center = LIMIT(3, center, 124);
//		return center;
//	}
//	else
//	{
//		emergency_max = 0;
//		emergency_flag = 0;
//		emergency_count = 0;
//		for(i=0; i<=127; i++)
//		{
//			if(ccd1_data[i] <= threshold && ccd1_data[i] > threshold_black)
//			{
//				emergency_count++;
//			}
//			else
//			{				
//				if(emergency_count > emergency_max)
//				{
//					emergency_max = emergency_count;
//					emergency_right = i-1;
//				}
//				emergency_count = 0;
//			}
//		}
//		if(emergency_max >= 3)
//			center = emergency_right - emergency_max/2 - 0.5;
//		center = LIMIT(3, center, 124);
//		return center;
//	}
//}

int gray_avr(int gray0, int gray1, int gray2)
{
//	return (0.2*gray0 + gray1 + 0.2*gray2)/1.4;
	return gray1;
}

int LXS_find_Line(int center, u16* ccd_data)
{
	int emergency_flag = 1, edge_count = 0, edge_left = 0, edge_right = 127, gray_left = 0, gray_right = 0, gray_left_last = 0, gray_right_last = 0;
	int black_num = 0, emergency_count = 1, emergency_num = 0, emergency_num_last = 0, emergency_left = 0, emergency_right = 127, emergency_left_last = 0;
	float black_perc = 0;
	int edge_left_flag = 0, edge_right_flag = 1;	//左右边界重捕获指示
	
//	int threshold = OTSU(ccd_data);
	int threshold = 1500;
	
//	for(int i = 0; i < 128; i++)
//	{
//		if(ccd_data[i] < threshold)
//			black_num++;
//	}
//	black_perc = black_num*100/128.0;
//	
//	if(black_perc >= 80)	//长黑线屏蔽
//		emergency_flag = 2;
//	
//	for(int i=center-2; i<=center+2; i++)	//丢线检测
//	{
//		if(ccd_data[i] > threshold)
//		{
//			emergency_flag = 1;	//丢线的优先级高于长黑线
//			break;
//		}
//	}
	
//	if(emergency_flag == 0)
//	{
//		gray_left_last = ccd_data[center];
//		gray_right_last = ccd_data[center];
//		
//		for(int i = center; i >= 1; i--)
//		{
//			gray_left = gray_avr(ccd_data[i-1], ccd_data[i], ccd_data[i+1]);
//			if(gray_left - gray_left_last > threshold_Delta)
//			{
//				edge_left = i;
//				break;
//			}
//		}
//		
//		for(int i = center; i <= 126; i++)
//		{
//			gray_right = gray_avr(ccd_data[i-1], ccd_data[i], ccd_data[i+1]);
//			if(gray_right - gray_right_last > threshold_Delta)
//			{
//				edge_right = i;
//				break;
//			}
//		}
//		return (edge_left + edge_right) / 2;
//	}
	
	if(emergency_flag == 1)
	{
		ccd_data[126] = ccd_data[127] = 4095;
		gray_left_last = gray_avr(ccd_data[0], ccd_data[1], ccd_data[2]);
		for(int i = 1; i <= 125; i++)
		{
			gray_left = gray_avr(ccd_data[i-1], ccd_data[i], ccd_data[i+1]);
			if(gray_left_last - gray_left > threshold_Delta)	//下降沿
			{
				edge_left_flag = 1;
				edge_right_flag = 0;
				gray_right_last = gray_left;
				emergency_left_last = i;
				break;
			}
			if(gray_left - gray_left_last > threshold_Delta)	//上升沿
			{
				edge_left_flag = 0;
				edge_right_flag = 1;
				emergency_right = i;
				emergency_num_last = emergency_num;
				gray_left_last = gray_left;
				break;
			}
			emergency_num++;
			emergency_count++;
			gray_left_last = gray_left;
		}
		emergency_num = 0;
		for(int i = emergency_count; i <= 126; i++)
		{
			if(edge_right_flag == 1)
			{
				gray_left = gray_avr(ccd_data[i-1], ccd_data[i], ccd_data[i+1]);
				if(gray_left_last - gray_left > threshold_Delta)
				{
					edge_left_flag = 1;
					edge_right_flag = 0;
					gray_right_last = gray_left;
					emergency_left_last = i;
				}
				else
					gray_left_last = gray_left;
			}
			
			if(edge_left_flag == 1)
			{
				gray_right = gray_avr(ccd_data[i-1], ccd_data[i], ccd_data[i+1]);
				if(gray_right - gray_right_last > threshold_Delta)
				{
					if(emergency_num > emergency_num_last)
					{
						emergency_left = emergency_left_last;
						emergency_right = i;
						emergency_num_last = emergency_num;
					}
					emergency_num = 0;
					edge_left_flag = 0;
					edge_right_flag = 1;
					gray_left_last = gray_right;
					
				}
				else
				{
					gray_right_last = gray_right;
					emergency_num++;
				}
			}
		}
		printf("left:%d    right:%d    middle:%d\r\n",emergency_left, emergency_right,emergency_num_last);
		if(emergency_right - emergency_left >= 100 || emergency_right - emergency_left<5){
			if(en_find){
				if(is_find_line == -1){
					is_find_line = 0;
				}
				else if(is_find_line == 0){
					is_find_line = 1;
					en_find = 0;
				}
			}
		}
		else{
			en_find = 1;
			is_find_line = -1;
			if(emergency_right - emergency_left >= 50 && (emergency_right > 120 || emergency_left < 5)){
				is_half_line = 1;
			}
		}
		if(emergency_num_last >= 5)
			return (emergency_left + emergency_right) / 2;
		
//		KalmanFilter(center, 0.8, 0.2);
		
		return 64;
	}
	
//	if(emergency_flag == 2)
//		return 64;
	
	return 64;
}


	
//	if(emergency_flag == 0)
//	{
//		gray_left_last = ccd_data[127];
//		gray_right_last = ccd_data[1];
//		
//		for(int i = 1; i < 128; i++)
//		{
//			start_left = 127-i;
//			if(start_left <= 2)
//			{
//				start_left = 2;
//			}
//			start_right = i;
//			if(start_right >= 126)
//			{
//				start_right = 126;
//			}
//			gray_left = ccd_data[start_left];
//			gray_right = ccd_data[start_right];
//			
//			if(gray_left - gray_left_last > threshold_Delta  && edge_left == 0)
//				edge_left = start_left;

//			else
//				gray_left_last = gray_left;
//			
//			if(gray_right - gray_right_last > threshold_Delta  && edge_right == 127)
//				edge_right = start_right;
//			else
//				gray_right_last = gray_right;
//			
//			if(edge_left != 0 && edge_right != 127)
//				break;
//		}
		
		
//		for(int i = 1; i < 128; i++)
//		{
//			start_left = center-i;
//			if(start_left <= 2)
//			{
//				start_left = 2;
//			}
//			start_right = center+i;
//			if(start_right >= 125)
//			{
//				start_right = 125;
//			}
////			gray_left = gray_avr(ccd_data[center-i-1], ccd_data[center-i], ccd_data[center-i+1]);
////			gray_right = gray_avr(ccd_data[center+i-1], ccd_data[center+i], ccd_data[center+i+1]);
//			gray_left = ccd_data[start_left];
//			gray_right = ccd_data[start_right];
//			
//			if(gray_left - gray_left_last > threshold_Delta)
//				edge_left = center-i;

//			else
//				gray_left_last = gray_left;
//			
//			if(gray_right - gray_right_last > threshold_Delta)
//				edge_right = center+i;
//			else
//				gray_right_last = gray_right;
//			
//			if(edge_left != 0 && edge_right != 127)
//				break;
//		}
//	}
	
//	printf("left:%d    right:%d    middle:%d\r\n",edge_left, edge_right,(edge_left + edge_right) / 2);
//	return (edge_left + edge_right) / 2;
//}
//			
			

//	if(emergency_flag == 0)
//	{
//		for(i=center-3; i>=0; i--)
//		{
//			if(ccd1_data[i] > threshold || ccd1_data[i]< threshold_black)
//				edge_count++;
//			if(edge_count == 3)
//				break;
//		}
//		edge_left = i+3;
//		edge_count = 0;
//		
//		for(i=center+3; i<=127; i++)
//		{
//			if(ccd1_data[i] > threshold || ccd1_data[i]< threshold_black)
//				edge_count++;
//			if(edge_count == 3)
//				break;
//		}
//		
//			
//		edge_right = i-3;
//		edge_count = 0;
//		
//		if(edge_left < 5)
//		{
//			return 66;
//		}
//		
//		center = (edge_left + edge_right) / 2 + 0.5;
//		center = LIMIT(3, center, 124);
//		return center;
//	}
//	else
//	{
//		emergency_max = 0;
//		emergency_flag = 0;
//		emergency_count = 0;
//		for(i=0; i<=127; i++)
//		{
//			if(ccd1_data[i] <= threshold && ccd1_data[i] > threshold_black)
//			{
//				emergency_count++;
//			}
//			else
//			{				
//				if(emergency_count > emergency_max)
//				{
//					emergency_max = emergency_count;
//					emergency_right = i-1;
//				}
//				emergency_count = 0;
//			}
//		}
//		if(emergency_max >= 3)
//			center = emergency_right - emergency_max/2 - 0.5;
//		center = LIMIT(3, center, 124);
//		return center;
//	}
//}

int OTSU(u16* array)
{
	int n0 = 0, n1 = 0;
	float n = 0, w0 = 0, w1 = 0, u0 = 0, u1 = 0, g = 0, gmax = 0, k = 0;
	for(int i = 10; i <= 40; i++)
	{
		for(int j = 0; j < 128; j++)
		{
			n = array[j] / 100;
			if(n < i)
			{
				n0++;
				u0 += n;
			}
			else
			{
				n1++;
				u1 += n;
			}
		}
		w0 = n0/128.0;
		w1 = n1/128.0;
		u0 /= n0;
		u1 /= n1;
		g = w0 * w1 * pow((u0 - u1), 2);
		if(g > gmax)
		{
			gmax = g;
			k = i * 100;
		}
		n0 = 0, n1 = 0, u0 = 0, u1 = 0;
	}
	return k;
}

//int CCD2_find_Line(int center, int threshold)
//{
//	int i, emergency_flag = 0, edge_count = 0, edge_left = 0, edge_right = 127;
//	int emergency_count = 0, emergency_max = 0, emergency_right = 0;
//	static int time = 0;
//	
//	for(i=center-2; i<=center+2; i++)
//	{
//		if(ccd2_data[i] > threshold)
//		{
//			emergency_flag = 1;
//			break;
//		}
//	}
//	
//	if(emergency_flag == 0)
//	{
//		for(i=center-3; i>=0; i--)
//		{
//			if(ccd2_data[i] > threshold)
//				edge_count++;
//			if(edge_count == 3)
//				break;
//		}
//		
//		edge_left = i+3;
//		edge_count = 0;
//		
//		for(i=center+3; i<=127; i++)
//		{
//			if(ccd2_data[i] > threshold)
//				edge_count++;
//			if(edge_count == 3)
//				break;
//		}
//		
//			
//		edge_right = i-3;
//		edge_count = 0;
//		
//		if(edge_right - edge_left > stop_line)
//		{
//			if(EN_EN_stop){	
//				if(targetSpeedY > 40){
//					if(edge_right - edge_left > 50){
//						//EN_EN_stop = 0;
//						//targetSpeedY = 20;
//						slow_down_judge = 1;
//					}
//				}
//				else
//				{	
//					EN_EN_stop = 0;					
//					EN_stop = 1;
//				}
//			}	
//			time = 0;
//		}
//		else{
//			if(time>2){
//				if(EN_stop){
//					//targetSpeedY = 0;
//					EN_stop=0;
//					stop_line = line3_wide;
//					stop_judge=1;
//				}
//				EN_EN_stop = 1;
//			}
//			time++;
//		}
//		
//		center = (edge_left + edge_right) / 2 + 0.5;
//		center = LIMIT(3, center, 124);
//		if(edge_left<4)
//			center = edge_right-15;
//		return center;
//	}
//	else
//	{
//		emergency_max = 0;
//		emergency_flag = 0;
//		emergency_count = 0;
//		for(i=0; i<=127; i++)
//		{
//			if(ccd2_data[i] <= threshold)
//			{
//				emergency_count++;
//			}
//			else
//			{				
//				if(emergency_count > emergency_max)
//				{
//					emergency_max = emergency_count;
//					emergency_right = i-1;
//				}
//				emergency_count = 0;
//			}
//		}
//		if(emergency_max >= 3)
//			center = emergency_right - emergency_max/2 - 0.5;
//		center = LIMIT(3, center, 124);
//		return center;
//	}
//}

int Find_Line_first(u16 *data, int threshold)
{
	int i;
	for(i=62; i<=66; i++)
	{
		if(data[i] > threshold)
		{
			return 0;
		}
	}
	return 1;
}
