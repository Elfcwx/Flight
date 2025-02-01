#include "task_manager.h"

void Control_task(void * pvParameters);
TaskHandle_t ControlTask_Handler;		//任务句柄

void LightFlow_task(void * pvParameters);
TaskHandle_t LightFlowTask_Handler;		//任务句柄

void Send_task(void * pvParameters);
TaskHandle_t SendTask_Handler;		//任务句柄

float battery;
float battery_level;

// 任务入口
void Flight_task(void)
{
	LED2_ON;
	// 创建控制函数
	xTaskCreate((TaskFunction_t ) Control_task,
			(char*			) "Control_task",
			(uint16_t		) CONTROL_STK_SIZE,
			(void * 		) NULL,
			(UBaseType_t	) CONTROL_TASK_PRIO,
			(TaskHandle_t*	) &ControlTask_Handler);
			
	// 创建光流任务	
	xTaskCreate((TaskFunction_t ) LightFlow_task,
			(char*			) "LightFlow_task",
			(uint16_t		) LIGHTFLOW_STK_SIZE,
			(void * 		) NULL,
			(UBaseType_t	) LIGHTFLOW_TASK_PRIO,
			(TaskHandle_t*	) &LightFlowTask_Handler);
			
	// 创建接收遥控信号任务	
	xTaskCreate((TaskFunction_t ) Send_task,
			(char*			) "Send_task",
			(uint16_t		) SEND_STK_SIZE,
			(void * 		) NULL,
			(UBaseType_t	) SEND_TASK_PRIO,
			(TaskHandle_t*	) &SendTask_Handler);	
	vTaskStartScheduler();          //开启任务调度
}

// 执行频率为 250 Hz
void Control_task(void * pvParameters)
{
	TickType_t xLastTime = xTaskGetTickCount();
	float frequency;
	static uint8_t outerEN = 1;
	static uint8_t angle_update = 1;
	while(1)
	{
		Remote_Receive();
		
		IMU_Data(angle_update);
		
		Control(outerEN);
		
		angle_update++;
		outerEN++;
		if(angle_update>1){
			angle_update=0;
		}
		if(outerEN > 1){
			outerEN = 0;
		}
		
		// 计算时间差
        TickType_t xCurrentTime = xTaskGetTickCount();
        TickType_t xElapsedTicks = xCurrentTime - xLastTime;
        xLastTime = xCurrentTime;

        // 转换为频率（Hz）
        frequency = (float)configTICK_RATE_HZ / (float)xElapsedTicks;

        // 输出频率（如通过串口）
        printf("ControlFrequency: %.2f Hz\r\n", frequency);
		
		vTaskDelay(1);
	}
}

void LightFlow_task(void * pvParameters)
{
	
	while(1)
	{
//		if(OpticFlow_Update())
//		{
//			printf("height: %d\r\n", _opticflow_Data.height);
//			if(_opticflow_Data.valid == 245){
//				printf("x_offset: %f\r\n", x_offset);
//				printf("y_offset: %f\r\n", y_offset);
//			}
//			
//		}

//		//vTaskDelay(2);
	}
}


void Send_task(void * pvParameters)
{
	
	while(1)
	{	
//		battery = (float)Get_ADC_Value(ADC_Channel_9);
//		battery = battery/4096 * 3.3f * 126 /75;
//		printf("battary: %f\r\n", battery);
//		if(battery > 3.6f)
//		{
//			battery_level = (battery-3.6)*100 / (4.2f - 3.6f);
//		}else{
//			battery_level = 0;
//		}
//		printf("battary: %f\r\n", battery_level);
	}
}


