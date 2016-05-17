#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "EPW_behavior.h"
#include "uart.h"
#include "PWM.h"
#include "MPU6050.h"

void vApplicationTickHook(void) {
}
/* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created.  It is also called by various parts of the
   demo application.  If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
void vApplicationMallocFailedHook(void) {
        taskDISABLE_INTERRUPTS();
        for(;;);
}

/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
   task.  It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()).  If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
void vApplicationIdleHook(void) {
}

void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName) {
        (void) pcTaskName;
        (void) pxTask;
        /* Run time stack overflow checking is performed if
           configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
           function is called if a stack overflow is detected. */
        taskDISABLE_INTERRUPTS();
        for(;;);
}

void Usart3_Printf(char *string){
    while(*string){
        // send string to USART3 
        USART_SendData(USART3, (unsigned short int) *string++);//unsigned short int

        // wait for sending string finished 
        while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
    }
}


u16 readADC1(u8 channel) { //u16 = unsigned char
	 ADC_SoftwareStartConv(ADC1);//Start the conversion
	 while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//Processing the conversion
	 return ADC_GetConversionValue(ADC1); //Return the converted data
}


int main(void) {

		uint8_t ret = pdFALSE;
    int i;
    int count;
    char buf [] = "";
    int Working_Time = 50000;
    int Work_Count = 0;

    float ADCMaxVal = 4095;
    float mVMaxVal = 5000;
    float supplyMidPointmV = 2950/2;
    float mVperg = 295;
    float mVPerADC = mVMaxVal / ADCMaxVal;
    
    /*init.*/
		//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
		init_USART3(9600);
    init_USART2(9600);

        //init_linear_actuator();
        //init_ADC1();
        //ADC_SoftwareStartConv(ADC1);
        init_Timer();
        init_Wheels();
        init_PWM();
        reset_Wheels();
    //MPU6050_I2C_Init();
    //MPU6050_Initialize_1();
    //MPU6050_Initialize_2();

    //USART_puts(USART3, "apple");
    

    //PLX-DAQ
    /*USART_puts(USART3, "CLEARDATA"); //clears up any data left from previous projects
    USART_puts(USART3, "\r\n");
    USART_puts(USART3, "LABEL,Time,JOY_x,JOY_y,JOY_xx,JOY_yy,ACC1_x,ACC1_y,ACC1_z,ANG1_x,ANG1_y,ANG1_z,ACC2_x,ACC2_y,ACC2_z,ANG2_x,ANG2_y,ANG2_z"); //always write LABEL, so excel knows the next things will be the names of the columns (instead of Acolumn you could write Time for instance)
    USART_puts(USART3, "\r\n");
    USART_puts(USART3, "RESETTIMER"); //resets timer to 0
    USART_puts(USART3, "\r\n");*/

        /*
        if( MPU6050_TestConnection() == TRUE)
  {
     USART_puts(USART3, "connection success\r\n");
  }else {
     USART_puts(USART3, "connection failed\r\n");
  }*/
        //init_DMA();
//when running while(1), it'll stuck in the while loop instead of running FreeRTOS's task
        //while (1)
        //{
          
        //}
        /*unit testing.*/
        /*if(unit_tests_task()){ //unit tests not pass.
           GPIO_WriteBit(GPIOD,GPIO_Pin_14,SET); 
           return 0;
        }else{ //unit tests passed
           //response the success state to user.
           GPIO_WriteBit(GPIOD,GPIO_Pin_12,SET);
           delay(1000);
           GPIO_WriteBit(GPIOD,GPIO_Pin_12,RESET);
        }

		/*create the task. */         
        //printf("Task creating...........\r\n");
		//ret = xTaskCreate(neural_task, "neural PID update task", 8192 /*configMINIMAL_STACK_SIZE*/, NULL, 2, NULL);
        //ret &= xTaskCreate(receive_task, "receive command task", 1024 /*configMINIMAL_STACK_SIZE*/, NULL, 3, NULL);
        //ret &= xTaskCreate(send_data_task, "send data task", 1024 /*configMINIMAL_STACK_SIZE*/, NULL, 1, NULL);
        ret = xTaskCreate(parse_Joystick_dir, ( signed portCHAR * ) "parse Joystick direction", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
        //ret = xTaskCreate(send_Joystick_data, (signed portCHAR *) "send Joystick data", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
        //ret = xTaskCreate(send_MPU6050_data, (signed portCHAR *) "send MPU6050 data", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
        //ret &= xTaskCreate(send_Tremor_Warning, (signed portCHAR *) "send_Tremor_Warning", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
		//ret &= xTaskCreate(send_out_task, "send out information task", 1024 /*configMINIMAL_STACK_SIZE*/, NULL, 1, NULL);
		//if (ret == pdTRUE) {
				//printf("All tasks are created.\r\n");
                //printf("System Started!\r\n");
				vTaskStartScheduler();  // should never return
		//} else {
				//printf("System Error!\r\n");
				// --TODO blink some LEDs to indicates fatal system error
		//}

		for (;;);

        //return 0;
}





