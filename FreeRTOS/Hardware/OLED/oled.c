#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"
#include "math.h"
#include "usart.h"
#include "includes.h"

#define	delay_ms(x) vTaskDelay(x)

//OLED���Դ�
//��Ÿ�ʽ����.
//[0]0 1 2 3 ... 127
//[1]0 1 2 3 ... 127
//[2]0 1 2 3 ... 127
//[3]0 1 2 3 ... 127
//[4]0 1 2 3 ... 127
//[5]0 1 2 3 ... 127
//[6]0 1 2 3 ... 127
//[7]0 1 2 3 ... 127

//��ʼ��IIC1


void delay_us(u32 count)
{
	int i;
	for(i=0;i<count*14;i++)
	{
		;
	}
}


void test (void)
{
	delay_us(10);
	LED0=!LED0;	
}

void IIC1_Init(void)
{			
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��

  //GPIOB6,B7��ʼ������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	IIC1_SCL=1;
	IIC1_SDA=1;
}



//����IIC1��ʼ�ź�
void IIC1_Start(void)
{
	SDA1_OUT();     //sda�����
	IIC1_SDA=1;	  	  
	IIC1_SCL=1;
	delay_us(4);
 	IIC1_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC1_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC1_Stop(void)
{
	SDA1_OUT();//sda�����
	IIC1_SCL=0;
	IIC1_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC1_SCL=1; 
	IIC1_SDA=1;//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC1_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA1_IN();      //SDA����Ϊ����  
	IIC1_SDA=1;delay_us(1);	   
	IIC1_SCL=1;delay_us(1);	 
	while(READ1_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC1_Stop();
			return 1;
		}
	}
	IIC1_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC1_Ack(void)
{
	IIC1_SCL=0;
	SDA1_OUT();
	IIC1_SDA=0;
	delay_us(2);
	IIC1_SCL=1;
	delay_us(2);
	IIC1_SCL=0;
}
//������ACKӦ��		    
void IIC1_NAck(void)
{
	IIC1_SCL=0;
	SDA1_OUT();
	IIC1_SDA=1;
	delay_us(2);
	IIC1_SCL=1;
	delay_us(2);
	IIC1_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC1_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA1_OUT(); 	    
    IIC1_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC1_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC1_SCL=1;
		delay_us(2); 
		IIC1_SCL=0;	
		delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC1_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA1_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC1_SCL=0; 
        delay_us(2);
		IIC1_SCL=1;
        receive<<=1;
        if(READ1_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC1_NAck();//����nACK
    else
        IIC1_Ack(); //����ACK   
    return receive;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*��������:	    ������ֽ�д��ָ���豸 ָ���Ĵ���
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫд���ֽ���
		*data  ��Ҫд�����ݵ��׵�ַ
����   �����Ƿ�ɹ�
*******************************************************************************/ 

u8 IIC1writeBytes(u8 dev, u8 reg, u8 length, u8* data){
  
 	u8 count = 0;
	IIC1_Start();
	IIC1_Send_Byte(dev<<1);	   //����д����
	IIC1_Wait_Ack();
	IIC1_Send_Byte(reg);   //���͵�ַ
	IIC1_Wait_Ack();	  
	for(count=0;count<length;count++){
		IIC1_Send_Byte(data[count]); 
		IIC1_Wait_Ack(); 
 }
	IIC1_Stop();//����һ��ֹͣ����

    return 1; //status == 0;
	
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ����� length��ֵ
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫ�����ֽ���
		*data  ���������ݽ�Ҫ��ŵ�ָ��
����   ���������ֽ�����
*******************************************************************************/ 
u8 IIC1readBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
	IIC1_Start();
	IIC1_Send_Byte(dev<<1);	   //����д����
	IIC1_Wait_Ack();
	IIC1_Send_Byte(reg);   //���͵�ַ
	IIC1_Wait_Ack();	  
	IIC1_Start();
	IIC1_Send_Byte((dev<<1)+1);  //�������ģʽ	
	IIC1_Wait_Ack();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)data[count]=IIC1_Read_Byte(1);  //��ACK�Ķ�����
		 	else  data[count]=IIC1_Read_Byte(0);	 //���һ���ֽ�NACK
	}
    IIC1_Stop();//����һ��ֹͣ����
    return count;
}



void Write_IIC1_Command(unsigned char IIC_Command)
{
	IIC1_Start();
	IIC1_Send_Byte(0x78);            //Slave address,SA0=0
	IIC1_Wait_Ack();	
	IIC1_Send_Byte(0x00);			//write command
	IIC1_Wait_Ack();	
	IIC1_Send_Byte(IIC_Command); 
	IIC1_Wait_Ack();	
	IIC1_Stop();
}
/**********************************************
// IIC Write Data
**********************************************/
void Write_IIC1_Data(unsigned char IIC_Data)
{
	IIC1_Start();
	IIC1_Send_Byte(0x78);			//D/C#=0; R/W#=0
	IIC1_Wait_Ack();	
	IIC1_Send_Byte(0x40);			//write data
	IIC1_Wait_Ack();	
	IIC1_Send_Byte(IIC_Data);
	IIC1_Wait_Ack();	
	IIC1_Stop();
}

void OLED_WR_Byte(unsigned dat,unsigned cmd)
{
	if(cmd)
	{
		Write_IIC1_Data(dat);   
	}
	else 
	{
		Write_IIC1_Command(dat);		
	}
}


/********************************************
// fill_Picture
********************************************/
void fill_picture(unsigned char fill_Data)
{
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		OLED_WR_Byte(0xb0+m,0);		//page0-page1
		OLED_WR_Byte(0x00,0);		//low column start address
		OLED_WR_Byte(0x10,0);		//high column start address
		for(n=0;n<128;n++)
			{
				OLED_WR_Byte(fill_Data,1);
			}
	}
}


/***********************Delay****************************************/
void Delay_50ms(unsigned int Del_50ms)
{
	unsigned int m;
	for(;Del_50ms>0;Del_50ms--)
		for(m=6245;m>0;m--);
}

void Delay_1ms(unsigned int Del_1ms)
{
	unsigned char j;
	while(Del_1ms--)
	{	
		for(j=0;j<123;j++);
	}
}

//��������

void OLED_Set_Pos(unsigned char x, unsigned char y) 
{ 	OLED_WR_Byte(0xb0+y,OLED_CMD);
	OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte((x&0x0f),OLED_CMD); 
}   	  
//����OLED��ʾ    
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//�ر�OLED��ʾ     
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//��������,������,������Ļ�Ǻ�ɫ��!��û����һ��!!!	  
void OLED_Clear(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
		OLED_WR_Byte (0x00,OLED_CMD);      //������ʾλ�á��е͵�ַ
		OLED_WR_Byte (0x10,OLED_CMD);      //������ʾλ�á��иߵ�ַ   
		for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA); 
	} //������ʾ
}
void OLED_On(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
		OLED_WR_Byte (0x00,OLED_CMD);      //������ʾλ�á��е͵�ַ
		OLED_WR_Byte (0x10,OLED_CMD);      //������ʾλ�á��иߵ�ַ   
		for(n=0;n<128;n++)OLED_WR_Byte(1,OLED_DATA); 
	} //������ʾ
}
//��ָ��λ����ʾһ���ַ�,���������ַ�
//x:0~127
//y:0~63
//mode:0,������ʾ;1,������ʾ				 
//size:ѡ������ 16/12 
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size)
{      	
	unsigned char c=0,i=0;	
	c=chr-' ';//�õ�ƫ�ƺ��ֵ			
	if(x>Max_Column-1){x=0;y=y+2;}
	if(Char_Size ==16)
	{
		OLED_Set_Pos(x,y);	
		for(i=0;i<8;i++)
		OLED_WR_Byte(F8X16[c*16+i],OLED_DATA);
		OLED_Set_Pos(x,y+1);
		for(i=0;i<8;i++)
		OLED_WR_Byte(F8X16[c*16+i+8],OLED_DATA);
	}
	else 
	{	
		OLED_Set_Pos(x,y);
		for(i=0;i<6;i++)
		OLED_WR_Byte(F6x8[c][i],OLED_DATA);
		
	}
}
//m^n����
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}				  
//��ʾ2������
//x,y :�������	 
//len :���ֵ�λ��
//size:�����С
//mode:ģʽ	0,���ģʽ;1,����ģʽ
//num:��ֵ(0~4294967295);	 		  
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size2)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size2/2)*t,y,' ',size2);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size2/2)*t,y,temp+'0',size2); 
	}
} 
//��ʾһ���ַ��Ŵ�
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 Char_Size)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{		
		OLED_ShowChar(x,y,chr[j],Char_Size);
		x+=8;
		if(x>120){x=0;y+=2;}
		j++;
	}
}



void OLED_Init(void)
{ 	
 	IIC1_Init();

	OLED_WR_Byte(0xAE,OLED_CMD);//--display off
	OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
	OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
	OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  
	OLED_WR_Byte(0xB0,OLED_CMD);//--set page address
	OLED_WR_Byte(0x81,OLED_CMD); // contract control
	OLED_WR_Byte(0xFF,OLED_CMD);//--128   
	OLED_WR_Byte(0xA1,OLED_CMD);//set segment remap 
	OLED_WR_Byte(0xA6,OLED_CMD);//--normal / reverse
	OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
	OLED_WR_Byte(0x3F,OLED_CMD);//--1/32 duty
	OLED_WR_Byte(0xC8,OLED_CMD);//Com scan direction
	OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset
	OLED_WR_Byte(0x00,OLED_CMD);//
	
	OLED_WR_Byte(0xD5,OLED_CMD);//set osc division
	OLED_WR_Byte(0x80,OLED_CMD);//
	
	OLED_WR_Byte(0xD8,OLED_CMD);//set area color mode off
	OLED_WR_Byte(0x05,OLED_CMD);//
	
	OLED_WR_Byte(0xD9,OLED_CMD);//Set Pre-Charge Period
	OLED_WR_Byte(0xF1,OLED_CMD);//
	
	OLED_WR_Byte(0xDA,OLED_CMD);//set com pin configuartion
	OLED_WR_Byte(0x12,OLED_CMD);//
	
	OLED_WR_Byte(0xDB,OLED_CMD);//set Vcomh
	OLED_WR_Byte(0x30,OLED_CMD);//
	
	OLED_WR_Byte(0x8D,OLED_CMD);//set charge pump enable
	OLED_WR_Byte(0x14,OLED_CMD);//
	
	OLED_WR_Byte(0xAF,OLED_CMD);//--turn on oled panel
	
  	OLED_Clear(); 
}

