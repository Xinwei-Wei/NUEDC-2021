#include "includes.h"

/*
**********************************************************************************************************
											�������ȼ�����
**********************************************************************************************************
*/

#define		vTask_USART_PRIO			7
#define		vTask_Wheel_PRIO			6
#define		vTask_CCD_PRIO		 		5
#define		vTask_Control_PRIO		 	2
#define		vTask_Key_PRIO		 		3

/*
**********************************************************************************************************
											����������
**********************************************************************************************************
*/

static	void	AppTaskCreate		(void);
static	void	vTask_USART			(void *pvParameters);
static	void	vTask_Wheel			(void *pvParameters);
static	void	vTask_Control		(void *pvParameters);
static	void	vTask_CCD			(void *pvParameters);
static	void	vTask_Key			(void *pvParameters);

/*
**********************************************************************************************************
											�����������
**********************************************************************************************************
*/

static	TaskHandle_t	xHandleTask_USART			= NULL;
static	TaskHandle_t	xHandleTask_Wheel			= NULL;
static	TaskHandle_t	xHandleTask_Control			= NULL;
static	TaskHandle_t	xHandleTask_CCD 			= NULL;
static	TaskHandle_t	xHandleTask_Key			= NULL;

/*
**********************************************************************************************************
											�û���������
**********************************************************************************************************
*/

void	Periph_Init	(void);
static	void	TestLED(void);
static	void	turn_left(void);
static	void	turn_right(void);

/*
**********************************************************************************************************
											�û���������
**********************************************************************************************************
*/
struct IncrementalPID left_pid, right_pid;
double left_pwm, right_pwm;
double *target_v;
extern u16 ccd1_data[128];
int ccd1_center = 64;
int CCD1_p = 5;
int targetSpeedW = 0, targetSpeedY = 0;
extern int is_car2;
extern int Target_pharmacy;
extern int is_find_line;
int is_drugs;
extern int pharmacy_position[10];
extern int go_judge;


/*
*********************************************************************************************************
*	�� �� ��: main
*	����˵��: ��׼c������ڡ�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/

int main(void)
{
	/* 
	  ����������ǰ��Ϊ�˷�ֹ��ʼ��STM32����ʱ���жϷ������ִ�У������ֹȫ���ж�(����NMI��HardFault)��
	  �������ĺô��ǣ�
	  1. ��ִֹ�е��жϷ����������FreeRTOS��API������
	  2. ��֤ϵͳ�������������ܱ���ж�Ӱ�졣
	  3. �����Ƿ�ر�ȫ���жϣ���Ҹ����Լ���ʵ��������ü��ɡ�
	  ����ֲ�ļ�port.c�еĺ���prvStartFirstTask�л����¿���ȫ���жϡ�ͨ��ָ��cpsie i������__set_PRIMASK(1)
	  ��cpsie i�ǵ�Ч�ġ�
     */
	__set_PRIMASK(1);
	
	/* Ӳ����ʼ�� */
	Periph_Init();
	
	/* �������� */
	AppTaskCreate();
	
    /* �������ȣ���ʼִ������ */
    vTaskStartScheduler();

	/* 
	  ���ϵͳ���������ǲ������е�����ģ����е����Ｋ�п��������ڶ�ʱ��������߿��������
	  heap�ռ䲻����ɴ���ʧ�ܣ���Ҫ�Ӵ�FreeRTOSConfig.h�ļ��ж����heap��С��
	  #define configTOTAL_HEAP_SIZE	      ( ( size_t ) ( 30 * 1024 ) )
	*/
	while(1);
}

void Periph_Init()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);		//����ϵͳ�ж����ȼ�����4
	
	taskENTER_CRITICAL();
	
	uart_init(115200);
	TIM8_PWM_Init(500-1, 33-1);
	Encoder_Init_TIM2();
	Encoder_Init_TIM4();
	Motor_IO_Init();
	Servo_Init();
	LED_Init();
	CCD_Init();
	KEY_Init();
	
	taskEXIT_CRITICAL();
}

/*
**********************************************************************************************************
											    �û�����
**********************************************************************************************************
*/

static void vTask_USART(void *pvParameters)
{
	TickType_t xLastWakeTime;
	
	vTaskDelay(1000);
	
	for(;;)
	{
//		TestLED();
//		vTaskDelayUntil(&xLastWakeTime, 2000);
//		LED1=!LED1;
//		USART_SendData(USART2,1);
		vTaskDelay(1000);
	}
}

static void vTask_Control(void *pvParameters)
{
	TickType_t xLastWakeTime;
	int i;
	vTaskDelay(3000);
	
	for(;;)
	{
		//ѯ�ʵڶ������Ƿ�����
		for(i=0;i<3;i++){
			USART_SendData(USART1, 0xff);
			vTaskDelay(500);
		}
		//˫������
		if(is_car2){
			
		}
		//��������
		else{
			//�ȴ�ʶ��			
			while(!go_judge){
				USART_SendData(USART2, 'a');
				vTaskDelay(1000);
			}
			printf("%d\r\n",pharmacy_position[0]);
			vTaskResume(xHandleTask_CCD);
			vTaskDelay(3000);
			//�ȴ�����ҩƷ
//			vTaskResume(xHandleTask_Key);
//			while(!is_drugs){
//				vTaskDelay(20);
//			}
			//����		
			targetSpeedY = 700;
			is_find_line = -1;
			//�ȴ�CCDʶ��
			while(is_find_line != 1){
				vTaskDelay(5);
			}
			//��һ��·��
			//��ת
			if(pharmacy_position[0] == pharmacy_position[1]){
				turn_left();						
			}
			//��ת
			else if(pharmacy_position[0] == pharmacy_position[2]){
				turn_right();							
			}
			//ֱ��
			else{
				printf("GO\r\n");
				vTaskDelay(1000);
				is_find_line = -1;
				vTaskDelay(500);
				printf("STOP\r\n");
				targetSpeedY = 0;
				go_judge = 0;
				while(!go_judge){
					USART_SendData(USART2, 'a');
					vTaskDelay(1000);
				}
				targetSpeedY = 500;
				//�����������λ������
				/*
				//��η���
				*/
				//�ȴ�CCDʶ��
				while(is_find_line != 1){
					vTaskDelay(5);
				}
				//�ڶ���·��
				//��ת
				if(pharmacy_position[0] == pharmacy_position[3]){
					turn_left();						
				}
				//��ת
				else if(pharmacy_position[0] == pharmacy_position[4]){
					turn_right();							
				}
				//ֱ��
				else{
					//�����������λ������
					printf("GO\r\n");
					vTaskDelay(1000);
					is_find_line = -1;
					vTaskDelay(1000);
					printf("STOP\r\n");
					targetSpeedY = 0;
					go_judge = 0;
					while(!go_judge){
						USART_SendData(USART2, 'a');
						vTaskDelay(1000);
					}
					targetSpeedY = 500;
					//�ȴ�CCDʶ��
					while(is_find_line != 1){
						vTaskDelay(5);
					}
					//������·��
					//��ת
					if(pharmacy_position[0] == pharmacy_position[5] || pharmacy_position[0] == pharmacy_position[6]){
						turn_left();
						targetSpeedY = 700;
						is_find_line = -1;
						vTaskDelay(1000);
						//�����������λ������
						/*
						//��η���
						*/
						//�ȴ�CCDʶ��	
						while(is_find_line != 1){
							vTaskDelay(5);
						}		
						is_find_line = -1;
						if(pharmacy_position[0] == pharmacy_position[5]){
							turn_left();						
						}
						//��ת
						else if(pharmacy_position[0] == pharmacy_position[6]){
							turn_right();							
						}
					}
					//��ת
					else{
						turn_right();
//						while(1){
//							targetSpeedY = 0;
//						}
						targetSpeedY = 700;
						is_find_line = -1;
						//vTaskDelay(1000);
						//�����������λ������
						/*
						//��η���
						*/
						//�ȴ�CCDʶ��
						while(is_find_line != 1){
							vTaskDelay(5);
						}
						if(pharmacy_position[0] == pharmacy_position[7]){
							turn_left();						
						}
						//��ת
						else if(pharmacy_position[0] == pharmacy_position[8]){
							turn_right();							
						}
					}
				}
			}
			is_find_line = -1;
			while(is_find_line != 1){
				vTaskDelay(5);
			}			
			vTaskSuspend(xHandleTask_CCD);
			targetSpeedW = 0;
			targetSpeedY = 0;
			while(1){
				vTaskDelay(20);
			}
		}
		vTaskDelay(20);
	}
}

static void vTask_CCD(void *pvParameters)
{
	TickType_t xLastWakeTime;
	vTaskSuspend(xHandleTask_CCD);
	vTaskDelay(3000);
	
	for(;;)
	{
		taskENTER_CRITICAL();
		CCD_Collect();
		taskEXIT_CRITICAL();
		ccd1_center = LXS_find_Line(ccd1_center, ccd1_data);
//		ccd_send_data(USART1, ccd1_data);
//			printf("%d\r\n",ccd1_center);
		if(ccd1_center > 66 || ccd1_center < 62)
			targetSpeedW = (ccd1_center - 64) * CCD1_p;
		else targetSpeedW = 0;
		vTaskDelay(10);
	}
}

static void vTask_Wheel(void *pvParameters)
{
	TickType_t xLastWakeTime;
	static int time = 5;
	static int encoder_speed_l, encoder_speed_r;
	
	vTaskDelay(1000);
	incremental_pid_init(&right_pid, 0.03, 0.01, 0.006);
	incremental_pid_init(&left_pid,  0.03, 0.01, 0.006);
	for(;;)
	{
		//targetSpeedY = 500;
		//printf("speed_y:%d speed_w:%d\r\n",targetSpeedY, targetSpeedW);
		target_v = moto_caculate(targetSpeedY, targetSpeedW);
		
		encoder_speed_l = TIM2->CNT ;
		TIM2->CNT = 0;
		
		if(TIM4->CNT < 0x8000){
			encoder_speed_r = TIM4->CNT;
		}
		else{
			encoder_speed_r = TIM4->CNT - 0xffff;
		}
		//printf("TIM4:%d\r\n",TIM4->CNT);
		TIM4->CNT = 0;
		
		left_pid.error = target_v[0] - encoder_speed_l;
		right_pid.error = target_v[1] - encoder_speed_r;
		
		left_pwm  += incremental_pid(&left_pid);
		right_pwm += incremental_pid(&right_pid);
		
		Control_Dir(2, LIMIT(-99, left_pwm,  99));
		Control_Dir(3, LIMIT(-99, right_pwm, 99));
	
		
		vTaskDelay(time);
	}
}

static void vTask_Key(void *pvParameters)
{
	TickType_t xLastWakeTime;
	
	vTaskSuspend(xHandleTask_Key);
	for(;;)
	{
		if(!PEin(3)){
			vTaskDelay(20);
			if(!PEin(3)){
				is_drugs = 1;
				vTaskSuspend(xHandleTask_Key);
			}
		}
		vTaskDelay(20);
	}
}
/*
**********************************************************************************************************
											    �û�����
**********************************************************************************************************
*/

static void TestLED(void)
{
	vTaskDelay(500);
	LED1 = 0;

	vTaskDelay(500);
	LED1 = 1;
}

static void turn_left(void){
	printf("left turn\r\n");
	targetSpeedY = 300;				
	vTaskSuspend(xHandleTask_CCD);
	targetSpeedW = -250;
	vTaskDelay(700);
	targetSpeedW = 0;
	vTaskResume(xHandleTask_CCD);
}

static void turn_right(void){
	printf("right turn\r\n");
	targetSpeedY = 300;				
	vTaskSuspend(xHandleTask_CCD);
	targetSpeedW = 250;
	vTaskDelay(700);
	targetSpeedW = 0;
	vTaskResume(xHandleTask_CCD);
}

/*
*********************************************************************************************************
*	�� �� ��: AppTaskCreate
*	����˵��: ����Ӧ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void AppTaskCreate (void)
{
    xTaskCreate( vTask_USART,				/* ������  */
                 "vTask USART",				/* ������    */
                 512,						/* ����ջ��С����λword��4�ֽ� */
                 NULL,						/* �������  */
                 vTask_USART_PRIO,			/* �������ȼ�*/
                 &xHandleTask_USART );		/* ������  */
	
	xTaskCreate( vTask_Wheel,				/* ������  */
                 "vTask Wheel",				/* ������    */
                 512,						/* ����ջ��С����λword��4�ֽ� */
                 NULL,						/* �������  */
                 vTask_Wheel_PRIO,			/* �������ȼ�*/
                 &xHandleTask_Wheel );		/* ������  */
	
	xTaskCreate( vTask_Control,				/* ������  */
                 "vTask Control",			/* ������    */
                 512,						/* ����ջ��С����λword��4�ֽ� */
                 NULL,						/* �������  */
                 vTask_Control_PRIO,		/* �������ȼ�*/
                 &xHandleTask_Control );	/* ������  */
	
	xTaskCreate( vTask_CCD,					/* ������  */
                 "vTask CCD",				/* ������    */
                 512,						/* ����ջ��С����λword��4�ֽ� */
                 NULL,						/* �������  */
                 vTask_CCD_PRIO,			/* �������ȼ�*/
                 &xHandleTask_CCD );		/* ������  */
				 
	xTaskCreate( vTask_Key,					/* ������  */
                 "vTask Key",				/* ������    */
                 512,						/* ����ջ��С����λword��4�ֽ� */
                 NULL,						/* �������  */
                 vTask_Key_PRIO,			/* �������ȼ�*/
                 &xHandleTask_Key );		/* ������  */
	
}
/***************************** (END OF FILE) *********************************/
