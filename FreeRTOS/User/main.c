#include "includes.h"

/*
**********************************************************************************************************
											�������ȼ�����
**********************************************************************************************************
*/

#define		vTask_USART_PRIO			1
#define		vTask_Wheel_PRIO			6
#define		vTask_CCD_PRIO		 		5
#define		vTask_Control_PRIO		 	2

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

/*
**********************************************************************************************************
											�����������
**********************************************************************************************************
*/

static	TaskHandle_t	xHandleTask_USART			= NULL;
static	TaskHandle_t	xHandleTask_Wheel			= NULL;
static	TaskHandle_t	xHandleTask_Control			= NULL;
static	TaskHandle_t	xHandleTask_CCD 			= NULL;

/*
**********************************************************************************************************
											�û���������
**********************************************************************************************************
*/

void	Periph_Init	(void);
static	void	TestLED(void);

/*
**********************************************************************************************************
											�û���������
**********************************************************************************************************
*/
struct IncrementalPID left_pid, right_pid;
double left_pwm, right_pwm;
double *target_v;
extern u16 ccd1_data[128];
int ccd1_center;
int CCD1_p = 5;
int targetSpeedW, targetSpeedY;
extern int is_car2;
extern int Target_pharmacy;
extern int is_find_line;


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
	
//	LED_Init();
	uart_init(115200);
	//Initial_USART2(115200);
	TIM8_PWM_Init(500-1, 33-1);
	Encoder_Init_TIM2();
	Encoder_Init_TIM4();
	Motor_IO_Init();
	Servo_Init();
	LED_Init();
	CCD_Init();
	
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
		
		vTaskDelay(1000);
	}
}

static void vTask_Control(void *pvParameters)
{
	TickType_t xLastWakeTime;
	int i;
	vTaskDelay(1000);
	
	for(;;)
	{
//		for(i=0;i<3;i++){
//			USART_SendData(USART1, 0xff);
//			vTaskDelay(500);
//		}
//		if(is_car2){
//			
//		}
//		else{
//			while(Target_pharmacy){
//				vTaskDelay(20);
//			}
//			targetSpeedY = 100;
//			while(!is_find_line){
//				vTaskDelay(20);
//			}
//		}
		vTaskDelay(20);
	}
}

static void vTask_CCD(void *pvParameters)
{
	TickType_t xLastWakeTime;
	
	vTaskDelay(1000);
	
	for(;;)
	{
		CCD_Collect();
		ccd1_center = LXS_find_Line(ccd1_center, ccd1_data);
		//ccd_send_data(USART1, ccd1_data);
		//printf("%d\r\n",ccd1_center);
		if(ccd1_center > 66 || ccd1_center < 62)
			targetSpeedW = (ccd1_center - 64) * CCD1_p;
		else targetSpeedW = 0;
		vTaskDelay(20);
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
		targetSpeedY = 500;
		target_v = moto_caculate(targetSpeedY, targetSpeedW);
		
		encoder_speed_l = TIM2->CNT ;
		TIM2->CNT = 0;
		
		if(TIM4->CNT < 0x8000){
			encoder_speed_r = TIM4->CNT;
		}
		else{
			encoder_speed_r = TIM4->CNT - 0xffff;
		}
		printf("TIM4:%d\r\n",TIM4->CNT);
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
	
}
/***************************** (END OF FILE) *********************************/
