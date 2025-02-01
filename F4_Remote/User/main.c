#include "system.h"
#include "adc.h"
#include "FreeRTOS.h"
#include "task.h"

#define CONTROL_TASK_PRIO			3
#define CONTROL_STK_SIZE			120 
void Control_task(void * pvParameters);
TaskHandle_t ControlTask_Handler;		//������

#define RECEIVE_TASK_PRIO			2
#define RECEIVE_STK_SIZE			120
void Receive_task(void * pvParameters);
TaskHandle_t RecviceTask_Handler;		//������	

#define START_TASK_PRIO			1
#define START_STK_SIZE			120
void start_task(void * pvParameters);  //������
TaskHandle_t StartTask_Handler;		//������	 


int main(void)
{
	xTaskCreate((TaskFunction_t	) start_task,
				(char*			) "start_task",
				(uint16_t		) START_STK_SIZE,
				(void * 		) NULL,
				(UBaseType_t	) START_TASK_PRIO,
				(TaskHandle_t*	) &StartTask_Handler);
	vTaskStartScheduler();          //�����������
}

void start_task(void * pvParameters)
{
	
	taskENTER_CRITICAL();               /* �����ٽ��� */
	
	// ����pid���ƺ���
	xTaskCreate((TaskFunction_t ) Control_task,
			(char*			) "Control_task",
			(uint16_t		) CONTROL_STK_SIZE,
			(void * 		) NULL,
			(UBaseType_t	) CONTROL_TASK_PRIO,
			(TaskHandle_t*	) &ControlTask_Handler);
			

	// ��������ң���ź�����	
	xTaskCreate((TaskFunction_t ) Receive_task,
			(char*			) "Receive_task",
			(uint16_t		) RECEIVE_STK_SIZE,
			(void * 		) NULL,
			(UBaseType_t	) RECEIVE_TASK_PRIO,
			(TaskHandle_t*	) &RecviceTask_Handler);
				
	vTaskDelete(StartTask_Handler); //NULL
			
	taskEXIT_CRITICAL();                /* �˳��ٽ��� */	
}


void Control_task(void * pvParameters)
{
	while(1)
	{
		
		vTaskDelay(20);
	}
}


void Receive_task(void * pvParameters)
{
	
	while(1)
	{
		vTaskDelay(7);
	}
}

