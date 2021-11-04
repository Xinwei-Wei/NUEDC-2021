#include "includes.h"

/*
**********************************************************************************************************
											任务优先级声明
**********************************************************************************************************
*/

#define		vTask_USART_PRIO			1
#define		vTask_Wheel_PRIO			4
#define		vTask_CCD_PRIO		 		5
#define		vTask_Servo_PRIO		 	2

/*
**********************************************************************************************************
											任务函数声明
**********************************************************************************************************
*/

static	void	AppTaskCreate		(void);
static	void	vTask_USART			(void *pvParameters);
static	void	vTask_Wheel			(void *pvParameters);
static	void	vTask_Servo			(void *pvParameters);
static	void	vTask_CCD			(void *pvParameters);

/*
**********************************************************************************************************
											任务变量声明
**********************************************************************************************************
*/

static	TaskHandle_t	xHandleTask_USART			= NULL;
static	TaskHandle_t	xHandleTask_Wheel			= NULL;
static	TaskHandle_t	xHandleTask_Servo			= NULL;
static	TaskHandle_t	xHandleTask_CCD 			= NULL;

/*
**********************************************************************************************************
											用户函数声明
**********************************************************************************************************
*/

void	Periph_Init	(void);
static	void	TestLED(void);

/*
**********************************************************************************************************
											用户变量声明
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
*	函 数 名: main
*	功能说明: 标准c程序入口。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/

int main(void)
{
	/* 
	  在启动调度前，为了防止初始化STM32外设时有中断服务程序执行，这里禁止全局中断(除了NMI和HardFault)。
	  这样做的好处是：
	  1. 防止执行的中断服务程序中有FreeRTOS的API函数。
	  2. 保证系统正常启动，不受别的中断影响。
	  3. 关于是否关闭全局中断，大家根据自己的实际情况设置即可。
	  在移植文件port.c中的函数prvStartFirstTask中会重新开启全局中断。通过指令cpsie i开启，__set_PRIMASK(1)
	  和cpsie i是等效的。
     */
	__set_PRIMASK(1);
	
	/* 硬件初始化 */
	Periph_Init();
	
	/* 创建任务 */
	AppTaskCreate();
	
    /* 启动调度，开始执行任务 */
    vTaskStartScheduler();

	/* 
	  如果系统正常启动是不会运行到这里的，运行到这里极有可能是用于定时器任务或者空闲任务的
	  heap空间不足造成创建失败，此要加大FreeRTOSConfig.h文件中定义的heap大小：
	  #define configTOTAL_HEAP_SIZE	      ( ( size_t ) ( 30 * 1024 ) )
	*/
	while(1);
}

void Periph_Init()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);		//设置系统中断优先级分组4
	
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
											    用户任务
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
											    用户函数
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
*	函 数 名: AppTaskCreate
*	功能说明: 创建应用任务
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void AppTaskCreate (void)
{
    xTaskCreate( vTask_USART,				/* 任务函数  */
                 "vTask USART",				/* 任务名    */
                 512,						/* 任务栈大小，单位word，4字节 */
                 NULL,						/* 任务参数  */
                 vTask_USART_PRIO,			/* 任务优先级*/
                 &xHandleTask_USART );		/* 任务句柄  */
	
	xTaskCreate( vTask_Wheel,				/* 任务函数  */
                 "vTask Wheel",				/* 任务名    */
                 512,						/* 任务栈大小，单位word，4字节 */
                 NULL,						/* 任务参数  */
                 vTask_Wheel_PRIO,			/* 任务优先级*/
                 &xHandleTask_Wheel );		/* 任务句柄  */
	
	xTaskCreate( vTask_Servo,				/* 任务函数  */
                 "vTask Servo",				/* 任务名    */
                 512,						/* 任务栈大小，单位word，4字节 */
                 NULL,						/* 任务参数  */
                 vTask_Servo_PRIO,			/* 任务优先级*/
                 &xHandleTask_Servo );		/* 任务句柄  */
	
	xTaskCreate( vTask_CCD,					/* 任务函数  */
                 "vTask CCD",				/* 任务名    */
                 512,						/* 任务栈大小，单位word，4字节 */
                 NULL,						/* 任务参数  */
                 vTask_CCD_PRIO,			/* 任务优先级*/
                 &xHandleTask_CCD );		/* 任务句柄  */
	
}
/***************************** (END OF FILE) *********************************/
