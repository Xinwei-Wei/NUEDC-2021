//#ifndef __OLED_H
//#define __OLED_H
//#include "sys.h"
//#include "stdlib.h"

////OLEDģʽ����
////0: 4�ߴ���ģʽ  ��ģ���BS1��BS2����GND��
////1: ����8080ģʽ ��ģ���BS1��BS2����VCC��
//#define OLED_MODE 	0

////-----------------OLED�˿ڶ���----------------
//#define OLED_CS 	PEout(15)          //CS
//#define OLED_RST  PAout(15)
//#define OLED_RS 	PCout(13)          //DC
////#define OLED_WR 	PAout(4)
////#define OLED_RD 	PDout(7)

////ʹ��4�ߴ��нӿ�ʱʹ��
//#define OLED_SCLK 	PCout(6)     //D0
//#define OLED_SDIN 	PCout(7)       //D1

//#define OLED_CMD  	0		//д����
//#define OLED_DATA 	1		//д����
////OLED�����ú���
//void OLED_WR_Byte(u8 dat, u8 cmd);
//void OLED_Display_On(void);
//void OLED_Display_Off(void);
//void OLED_Refresh_Gram(void);

//void OLED_Init(void);
//void OLED_Clear(void);
//void OLED_DrawPoint(u8 x, u8 y, u8 t);
//void OLED_Fill(u8 x1, u8 y1, u8 x2, u8 y2, u8 dot);
//void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size, u8 mode);
//void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size);
//void OLED_ShowString(u8 x, u8 y, const u8 *p, u8 size);
//void OLED_ShowDecimal(float value, u8 count0, u8 count1, u8 x, u8 y , u8 size);
//void OLED_P128x32Ch(u8 x, u8 y, u16 N);
//#endif




#ifndef __OLED_H
#define __OLED_H
#include "sys.h"
#include "stdlib.h"


#define OLED_MODE 0
#define SIZE 8
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	

//-----------------OLED�˿ڶ���----------------
//IO��������	 
#define IIC1_SCL    PBout(6) //SCL
#define IIC1_SDA    PBout(7) //SDA	 
#define READ1_SDA   PBin(7)  //����SDA 


//IO��������
#define SDA1_IN()  {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<7*2;}	//PB7����ģʽ
#define SDA1_OUT() {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<7*2;} //PB7���ģʽ

#define OLED_CMD  	0		//д����
#define OLED_DATA 	1		//д����


//OLED�����ú���
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 Char_Size);

void OLED_Init(void);
void test (void);
void delay_us(u32 count);

#endif

