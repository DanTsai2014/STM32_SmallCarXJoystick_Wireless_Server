#include "FreeRTOS.h"
#include "timers.h"
#include "stm32f4xx_syscfg.h"
#include "EPW_behavior.h"
#include "timers.h"
#include "stm32f4xx_usart.h"
#include "uart.h"
#include "PWM.h"

#define B_phase 1
#define NEURAL_IDENTIFIER 1
#define PID_ADJUST 1
#define RECORD_SIZE 750

/*============================================================================*/
/*============================================================================*
 ** function : parse_Joystick_dir
 ** brief : parse Joystick direction from the uart siganl
 ** param : Joystick_cmd
 ** retval :  None
 **============================================================================*/
/*============================================================================*/

void parse_Joystick_dir(void *pvParameters)
{
	while(1)
	{
		/*if(received_tmp[0] == 'f'){  //forward
			//PWM_Control
			TIM_SetCompare1(TIM1, 256 - 1);
			TIM_SetCompare2(TIM1, 256 - 1);
			TIM_SetCompare1(TIM3, 0);
			//TIM_SetCompare2(TIM3, 127); //minimum: 95
			TIM_SetCompare3(TIM3, 0);
			//TIM_SetCompare4(TIM3, 127);

			if(received_tmp[1] == '1'){ //min_speed
				TIM_SetCompare2(TIM3, 100);//min_speed
				TIM_SetCompare4(TIM3, 100);
			}
			if(received_tmp[1] == '2'){ 
				TIM_SetCompare2(TIM3, 178);//mid_speed
				TIM_SetCompare4(TIM3, 178);
			}
			if(received_tmp[1] == '3'){ 
				TIM_SetCompare2(TIM3, 255);//max_speed
			    TIM_SetCompare4(TIM3, 255);
		    }
		    vTaskDelay(100);
		}

		else if(received_tmp[0] == 's' && received_tmp[1] == 's'){  //stop
			TIM_SetCompare1(TIM1, 0);
			TIM_SetCompare2(TIM1, 0);
			TIM_SetCompare1(TIM3, 0);
			TIM_SetCompare2(TIM3, 0);
			TIM_SetCompare3(TIM3, 0);
			TIM_SetCompare4(TIM3, 0);
			vTaskDelay(100);
		}
	
		else if(received_tmp[0] == 'b'){
			TIM_SetCompare1(TIM1, 256 - 1);
			TIM_SetCompare2(TIM1, 256 - 1);
			//TIM_SetCompare1(TIM3, 127);
			TIM_SetCompare2(TIM3, 0);
			//TIM_SetCompare3(TIM3, 127);
			TIM_SetCompare4(TIM3, 0);

			if(received_tmp[1] == '1'){
				TIM_SetCompare1(TIM3, 100);
				TIM_SetCompare3(TIM3, 100);
			}
			if(received_tmp[1] == '2'){
				TIM_SetCompare1(TIM3, 178);
				TIM_SetCompare3(TIM3, 178);
			}
			if(received_tmp[1] == '3'){
				TIM_SetCompare1(TIM3, 255);
				TIM_SetCompare3(TIM3, 255);
			}
				vTaskDelay(100);
		}

		else if(received_tmp[0] == 'l'){
			TIM_SetCompare1(TIM1, 256 - 1);
			TIM_SetCompare2(TIM1, 256 - 1);
            //TIM_SetCompare1(TIM3, 127);
            TIM_SetCompare2(TIM3, 0);
            TIM_SetCompare3(TIM3, 0);
            //TIM_SetCompare4(TIM3, 127);

			if(received_tmp[1] == '1'){ //min_speed
				TIM_SetCompare1(TIM3, 100);
				TIM_SetCompare4(TIM3, 100);
			}
			if(received_tmp[1] == '2'){ //mid_speed
				TIM_SetCompare1(TIM3, 178);
				TIM_SetCompare4(TIM3, 178);
			}
			if(received_tmp[1] == '3'){ //max_speed
				TIM_SetCompare1(TIM3, 255);
				TIM_SetCompare4(TIM3, 255);
		    }
		    vTaskDelay(100);
		}
        
		else if(received_tmp[0] == 'r'){
			TIM_SetCompare1(TIM1, 256 - 1);
			TIM_SetCompare2(TIM1, 256 - 1);
            TIM_SetCompare1(TIM3, 0);
            //TIM_SetCompare2(TIM3, 127);
            //TIM_SetCompare3(TIM3, 127);
            TIM_SetCompare4(TIM3, 0);

			if(received_tmp[1] == '1'){
				TIM_SetCompare2(TIM3, 100);
				TIM_SetCompare3(TIM3, 100);
			}
			if(received_tmp[1] == '2'){
				TIM_SetCompare2(TIM3, 178);
				TIM_SetCompare3(TIM3, 178);
			}
			if(received_tmp[2] == '3'){
				TIM_SetCompare2(TIM3, 255);
				TIM_SetCompare3(TIM3, 255);
			}
			vTaskDelay(100);
		}
		else{
		}*/

        //only one speed
		if(received_tmp == 'f'){
			//PWM_Control
			TIM_SetCompare1(TIM1, 256 - 1);
			TIM_SetCompare2(TIM1, 256 - 1);
			TIM_SetCompare1(TIM3, 0);
			TIM_SetCompare2(TIM3, 127); //minimum: 95
			TIM_SetCompare3(TIM3, 0);
			TIM_SetCompare4(TIM3, 127);
		    vTaskDelay(100);
		}
		if(received_tmp == 's'){
			TIM_SetCompare1(TIM1, 0);
			TIM_SetCompare2(TIM1, 0);
			TIM_SetCompare1(TIM3, 0);
			TIM_SetCompare2(TIM3, 0); //minimum: 95
			TIM_SetCompare3(TIM3, 0);
			TIM_SetCompare4(TIM3, 0);
		    vTaskDelay(100);
		}
		if(received_tmp == 'b'){
			TIM_SetCompare1(TIM1, 256 - 1);
			TIM_SetCompare2(TIM1, 256 - 1);
			TIM_SetCompare1(TIM3, 127);
			TIM_SetCompare2(TIM3, 0);
			TIM_SetCompare3(TIM3, 127);
			TIM_SetCompare4(TIM3, 0);
			vTaskDelay(100);
		}
		if(received_tmp == 'l'){
			TIM_SetCompare1(TIM1, 256 - 1);
			TIM_SetCompare2(TIM1, 256 - 1);
            TIM_SetCompare1(TIM3, 127);
            TIM_SetCompare2(TIM3, 0);
            TIM_SetCompare3(TIM3, 0);
            TIM_SetCompare4(TIM3, 127);
		    vTaskDelay(100);
		}
		if(received_tmp == 'r'){
			TIM_SetCompare1(TIM1, 256 - 1);
			TIM_SetCompare2(TIM1, 256 - 1);
            TIM_SetCompare1(TIM3, 0);
            TIM_SetCompare2(TIM3, 127);
            TIM_SetCompare3(TIM3, 127);
            TIM_SetCompare4(TIM3, 0);
            vTaskDelay(100);
		}		
    }
}