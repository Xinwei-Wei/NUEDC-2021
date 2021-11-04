#include "includes.h"

/*
**********************************************************************************************************
											�������ȼ�����
**********************************************************************************************************
*/

#define		vTask_USART_PRIO			1
#define		vTask_Wheel_PRIO			4
#define		vTask_CCD_PRIO		 		5
#define		vTask_Servo_PRIO		 	2

/*
**********************************************************************************************************
											����������
**********************************************************************************************************
*/

static	void	AppTaskCreate		(void);
static	void	vTask_USART			(void *pvParameters);
static	void	vTask_Wheel			(void *pvParameters);
static	void	vTask_Servo			(void *pvParameters);
static	void	vTask_CCD			(void *pvParameters);

/*
**********************************************************************************************************
											�����������
**********************************************************************************************************
*/

static	TaskHandle_t	xHandleTask_USART			= NULL;
static	TaskHandle_t	xHandleTask_Wheel			= NULL;
static	TaskHandle_t	xHandleTask_Servo			= NULL;
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

static void vTask_Servo(void *pvParameters)
{
	TickType_t xLastWakeTime;
	
	vTaskDelay(1000);
	
	for(;;)
	{
		vTaskDelay(1000);
	}
}

static void vTask_CCD(void *pvParameters)
{
	TickType_t xLastWakeTime;
	
	vTaskDelay(1000);
	
	for(;;)
	{
		CCD_Collect();
		ccd1_center = CCD_find_Line(ccd1_center, THRESHOLD);
		//ccd_send_data(USART1, ccd1_data);
		printf("%d\r\n",ccd1_center);
		if(ccd1_center > 66 || ccd1_center < 62)
			targetSpeedW = (ccd1_center - 64) * CCD1_p;
		else targetSpeedW = 0;
		vTaskDelay(20);
	}
}

static void vTask_Wheel(void *pvParameters)
{
	TickType_t xLastWakeTime;
	static int time = 20;
	
	vTaskDelay(1000);
	incremental_pid_init(&right_pid, 0.06, 0.1, 0.06);
	incremental_pid_init(&left_pid,  0.06, 0.1, 0.06);
	
	for(;;)
	{
		targetSpeedY = 50;
		target_v = moto_caculate(targetSpeedY, targetSpeedW);
		if(left_pwm < 0)
			left_pid.error = (target_v[0])+((TIM2->CNT<0xffffffff-TIM2->CNT) ? TIM2->CNT : 0xffffffff-TIM2->CNT)*4.08/time;
		else
			left_pid.error = (target_v[0])-((TIM2->CNT<0xffffffff-TIM2->CNT) ? TIM2->CNT : 0xffffffff-TIM2->CNT)*4.08/time;	
		TIM2->CNT = 0;
		if(right_pwm < 0)
			right_pid.error = (target_v[1])+((TIM4->CNT<0xffff-TIM4->CNT) ? TIM4->CNT : 0xffff-TIM4->CNT)*4.08/time;
		else
			right_pid.error = (target_v[1])-((TIM4->CNT<0xffff-TIM4->CNT) ? TIM4->CNT : 0xffff-TIM4->CNT)*4.08/time;	
		TIM4->CNT = 0;
		
		left_pwm  += incremental_pid(&left_pid);
		if((left_pwm > 0 && target_v[0] < 0) || (left_pwm < 0 && target_v[0] > 0)){
			left_pwm = 0;
		}
		right_pwm += incremental_pid(&right_pid);
		if((right_pwm > 0 && target_v[1] < 0) || (right_pwm < 0 && target_v[1] > 0)){
			right_pwm = 0;
		}
		//left_pwm=0;
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
	
	xTaskCreate( vTask_Servo,				/* ������  */
                 "vTask Servo",				/* ������    */
                 512,						/* ����ջ��С����λword��4�ֽ� */
                 NULL,						/* �������  */
                 vTask_Servo_PRIO,			/* �������ȼ�*/
                 &xHandleTask_Servo );		/* ������  */
	
	xTaskCreate( vTask_CCD,					/* ������  */
                 "vTask CCD",				/* ������    */
                 512,						/* ����ջ��С����λword��4�ֽ� */
                 NULL,						/* �������  */
                 vTask_CCD_PRIO,			/* �������ȼ�*/
                 &xHandleTask_CCD );		/* ������  */
	
}
/***************************** (END OF FILE) *********************************/
