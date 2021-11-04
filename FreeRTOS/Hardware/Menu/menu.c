#include "menu.h"
#include "oled.h"

void ShowMenu(){
	OLED_ShowString(10,0,"durian1 ",12);
	OLED_ShowString(10,1,"durian2 ",12);
	OLED_ShowString(10,2,"durian3 ",12);
	OLED_ShowString(10,3,"durian4 ",12);
	OLED_ShowString(10,4,"durian5 ",12);
	OLED_ShowString(10,5,"back",12);
}

void ShowTask(u8 a){
	switch(a+1){
		case 1:{ OLED_ShowString(10,0,"durian11",12);
						 OLED_ShowString(10,1,"        ",12);
						 OLED_ShowString(10,2,"        ",12);
						 OLED_ShowString(10,3,"        ",12);
						 OLED_ShowString(10,4,"        ",12);
			       OLED_ShowString(10,5,"back",12);
						 break;	}
		case 2:{ OLED_ShowString(10,0,"durian21",12);
			       OLED_ShowString(10,1,"        ",12);
						 OLED_ShowString(10,2,"        ",12);
						 OLED_ShowString(10,3,"        ",12);
						 OLED_ShowString(10,4,"        ",12);
			       OLED_ShowString(10,5,"back",12);
						 break;	}
		case 3:{ OLED_ShowString(10,0,"durian31",12);
						 OLED_ShowString(10,1,"        ",12);
						 OLED_ShowString(10,2,"        ",12);
						 OLED_ShowString(10,3,"        ",12);
						 OLED_ShowString(10,4,"        ",12);
			       OLED_ShowString(10,5,"back",12);
						 break;	}
		case 4:{ OLED_ShowString(10,0,"durian41",12);
			       OLED_ShowString(10,1,"        ",12);
						 OLED_ShowString(10,2,"        ",12);
						 OLED_ShowString(10,3,"        ",12);
						 OLED_ShowString(10,4,"        ",12);
			       OLED_ShowString(10,5,"back",12);
						 break;	}
		case 5:{ OLED_ShowString(10,0,"durian51",12);
			       OLED_ShowString(10,1,"        ",12);
						 OLED_ShowString(10,2,"        ",12);
						 OLED_ShowString(10,3,"        ",12);
						 OLED_ShowString(10,4,"        ",12);
			       OLED_ShowString(10,5,"back",12);
						 break;	}
	}
}

void ShowSelect(char select){
	switch(select+1){
		case 1 :{ OLED_ShowChar(0,0,'*',12,1);
							OLED_ShowChar(0,1,' ',12,1);
							OLED_ShowChar(0,2,' ',12,1);
							OLED_ShowChar(0,3,' ',12,1);
							OLED_ShowChar(0,4,' ',12,1);
							OLED_ShowChar(0,5,' ',12,1);
							break;}
		case 2 :{ OLED_ShowChar(0,0,' ',12,1);
							OLED_ShowChar(0,1,'*',12,1);
							OLED_ShowChar(0,2,' ',12,1);
							OLED_ShowChar(0,3,' ',12,1);
							OLED_ShowChar(0,4,' ',12,1);
							OLED_ShowChar(0,5,' ',12,1);
							break;}
		case 3 :{ OLED_ShowChar(0,0,' ',12,1);
							OLED_ShowChar(0,1,' ',12,1);
							OLED_ShowChar(0,2,'*',12,1);
							OLED_ShowChar(0,3,' ',12,1);
							OLED_ShowChar(0,4,' ',12,1);
							OLED_ShowChar(0,5,' ',12,1);
							break;}
		case 4 :{ OLED_ShowChar(0,0,' ',12,1);
							OLED_ShowChar(0,1,' ',12,1);
							OLED_ShowChar(0,2,' ',12,1);
							OLED_ShowChar(0,3,'*',12,1);
							OLED_ShowChar(0,4,' ',12,1);
							OLED_ShowChar(0,5,' ',12,1);
							break;}
		case 5 :{ OLED_ShowChar(0,0,' ',12,1);
							OLED_ShowChar(0,1,' ',12,1);
							OLED_ShowChar(0,2,' ',12,1);
							OLED_ShowChar(0,3,' ',12,1);
							OLED_ShowChar(0,4,'*',12,1);
							OLED_ShowChar(0,5,' ',12,1);
							break;}
		case 6 :{ OLED_ShowChar(0,0,' ',12,1);
							OLED_ShowChar(0,1,' ',12,1);
							OLED_ShowChar(0,2,' ',12,1);
							OLED_ShowChar(0,3,' ',12,1);
							OLED_ShowChar(0,4,' ',12,1);
							OLED_ShowChar(0,5,'*',12,1);
							break;}
	}
}

