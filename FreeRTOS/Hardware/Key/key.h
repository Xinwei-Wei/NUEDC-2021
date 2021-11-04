#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h" 
 

/*下面方式是通过位带操作方式读取IO*/

#define KEY0 		PEin(4)   	//PE4
#define KEY1 		PEin(3)		//PE3 
#define WK_UP 	PAin(0)		//PA0

void KEY_Init(void);

u8 KEY_Scan(u8 mode);

#endif
