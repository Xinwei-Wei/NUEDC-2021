#ifndef __CCD_H
#define __CCD_H
#include "sys.h"

#define THRESHOLD 3000

void  Adc_Init(void);
void  ccd_Init(void);
u16 Get_Adc_Average(u8 ch,u8 times);

void ccd_init(void);
void ccd_send_data(USART_TypeDef* uart_n ,u16 *dat);
int find_line(void);
int OTSU(u16* array);
void CCD_IO(void);
void CCD_Collect(void);
void CCD_Init(void);
int Find_Line(u16 *data, int center, int threshold);
int CCD_find_Line(int center, int threshold);
int Find_Line_first(u16 *data, int threshold);
int gray_avr(int gray0, int gray1, int gray2);
int LXS_find_Line(int center, u16* ccd_data);

#endif

