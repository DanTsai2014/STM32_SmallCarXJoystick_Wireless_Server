#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "uart.h"

/* Scheduler includes. */
#include "task.h"
#include "queue.h"
#include "semphr.h"


/*The USART3 potr and pin setting , it's fixed var. cannot change*/
#define USART3_PORT                       GPIOB
#define USART3_TX_PIN                     GPIO_Pin_10
#define USART3_RX_PIN                     GPIO_Pin_11

#define USART2_PORT                       GPIOA
#define USART2_TX_PIN                     GPIO_Pin_2
#define USART2_RX_PIN                     GPIO_Pin_3

uint8_t Receive_String_Ready=0;
uint8_t Receive_String_Ready_2 = 0;


volatile xSemaphoreHandle serial_tx_wait_sem = NULL;
volatile xQueueHandle serial_rx_queue = NULL ; 
volatile xQueueHandle serial_str_queue = NULL ; 


/* Queue structure used for passing messages. */
typedef struct {
	char str[100];
} serial_str_msg;

/* Queue structure used for passing characters. */
typedef struct {
    char ch;
} serial_ch_msg;


void init_USART3(uint32_t baurate){

	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to 
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 * 
	 * They make our life easier because we don't have to mess around with 
	 * the low level stuff of setting bits in the correct registers
	 */
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART3 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* enable APB1 peripheral clock for USART3
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* enable the peripheral clock for the pins used by 
	 * USART3, PB10 for TX and PB11 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* This sequence sets up the TX and RX pins 
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = USART3_TX_PIN | USART3_RX_PIN; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;                         // the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;                // this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;                        // this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;                        // this activates the pullup resistors on the IO pins
	GPIO_Init(USART3_PORT, &GPIO_InitStruct);                                        // now all the values are passed to the GPIO_Init() function which sets the GPIO registers



	/* The RX and TX pins are now connected to their AF
	 * so that the USART3 can take over control of the 
	 * pins
	 */
	GPIO_PinAFConfig(USART3_PORT, GPIO_PinSource10, GPIO_AF_USART3); 
	GPIO_PinAFConfig(USART3_PORT, GPIO_PinSource11, GPIO_AF_USART3);

	/* Now the USART_InitStruct is used to define the 
	 * properties of USART3
	 */

	USART_InitStruct.USART_BaudRate = baurate;                                // the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;                // we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;                // we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART3, &USART_InitStruct);                                        // again all the properties are passed to the USART_Init function which takes care of all the bit setting

	/* Here the USART3 receive interrupt is enabled
	 * and the interrupt controller is configured 
	 * to jump to the USART3_IRQHandler() function
	 * if the USART3 receive interrupt occurs
	 */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // enable the USART3 receive interrupt 

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;                 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                         // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);                                                         // the properties are passed to the NVIC_Init function which takes care of the low level stuff        

	// finally this enables the complete USART3 peripheral
	USART_Cmd(USART3, ENABLE);
}

void init_USART2(uint32_t baurate){

	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to 
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 * 
	 * They make our life easier because we don't have to mess around with 
	 * the low level stuff of setting bits in the correct registers
	 */
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART3 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* enable APB1 peripheral clock for USART2
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* enable the peripheral clock for the pins used by 
	 * USART3, PB10 for TX and PB11 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* This sequence sets up the TX and RX pins 
	 * so they work correctly with the USART2 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = USART2_TX_PIN | USART2_RX_PIN; // Pins 2 (TX) and 3 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;                         // the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;                // this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;                        // this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;                        // this activates the pullup resistors on the IO pins
	GPIO_Init(USART2_PORT, &GPIO_InitStruct);                                        // now all the values are passed to the GPIO_Init() function which sets the GPIO registers



	/* The RX and TX pins are now connected to their AF
	 * so that the USART2 can take over control of the 
	 * pins
	 */
	GPIO_PinAFConfig(USART2_PORT, GPIO_PinSource2, GPIO_AF_USART2); 
	GPIO_PinAFConfig(USART2_PORT, GPIO_PinSource3, GPIO_AF_USART2);

	/* Now the USART_InitStruct is used to define the 
	 * properties of USART3
	 */

	USART_InitStruct.USART_BaudRate = baurate;                                // the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;                // we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;                // we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART2, &USART_InitStruct);                                        // again all the properties are passed to the USART_Init function which takes care of all the bit setting

	/* Here the USART3 receive interrupt is enabled
	 * and the interrupt controller is configured 
	 * to jump to the USART3_IRQHandler() function
	 * if the USART3 receive interrupt occurs
	 */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // enable the USART3 receive interrupt 

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;                 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                         // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);                                                         // the properties are passed to the NVIC_Init function which takes care of the low level stuff        

	// finally this enables the complete USART2 peripheral
	USART_Cmd(USART2, ENABLE);
}

// this is the interrupt request handler (IRQ) for ALL USART3 interrupts
/*void USART3_IRQHandler(void){

	// check if the USART3 receive interrupt flag was set
	if( USART_GetITStatus(USART3, USART_IT_RXNE) ){

		//check the uart RX have accept the char
		GPIO_ToggleBits(GPIOD,GPIO_Pin_14);


		static uint8_t cnt = 0; // this counter is used to determine the uart receive string length

		//Receive_data = USART3->DR; // the character from the USART3 data register is saved in t
		Receive_data = USART_ReceiveData(USART3);;

		// check if the received character is not the LF character (used to determine end of string) 
	    //or the if the maximum string length has been been reached 
		 

		if( cnt < MAX_STRLEN){
			received_string[cnt] = Receive_data;
            if(Receive_data=='0') GPIO_ToggleBits(GPIOD,GPIO_Pin_15);

            //start determine the period of command.
            if(received_string[cnt]=='\r'){
                Receive_String_Ready = 1; //Ready to parse the command 
                cnt=0; //restart to accept next stream message.
            }
            else{
                cnt++;
            }
		}
		else{ // over the max string length, cnt return to zero.
			Receive_String_Ready=1;
			cnt = 0;  
		}
		if(Receive_String_Ready){
			//print the content of the received string
			USART_puts(USART3, received_string);
			USART_puts(USART3,"\r\n");
			//receive_task();
			//clear the received string and the flag
			Receive_String_Ready = 0;
			int i;
			for( i = 0 ; i< MAX_STRLEN ; i++){
				received_string[i]= 0;
			}
		}
	}
}*/
/*
void USART2_IRQHandler(void){

	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART2, USART_IT_RXNE) ){

		static uint8_t cnt = 0; // this counter is used to determine the string length
		char t = USART2->DR; // the character from the USART1 data register is saved in t

		// check if the received character is not the LF character (used to determine end of string) 
		// or the if the maximum string length has been been reached 
		//
		if( (t != '\n') && (cnt < MAX_STRLEN) ){ 
			received_string_2[cnt] = t;
			cnt++;
		}
		else if(t == 'z'){
			Receive_String_Ready_2 = 1;
			cnt = 0;
		}

	    if(Receive_String_Ready_2){
	    	if(cnt == 0)
	    	Receive_data_2 = received_string_2[cnt-1];
	    	int i;
			for( i = 0 ; i< MAX_STRLEN ; i++){
				received_string_2[i]= 0;
	        }
        }
    }
}*/

void USART2_IRQHandler(void){
	if(USART_GetITStatus(USART2, USART_IT_RXNE)){
		static uint8_t cnt = 0;
	    received_tmp[cnt] = USART_ReceiveData(USART2);
	    cnt++;
	    if(cnt > 1){
	    	cnt = 0;
	    	USART_puts(USART3, received_tmp);
	    	int i;
	    	for(i = 0; i < 1; i++){
	    		received_tmp[i] = 0;
	    	}
        }
	}
}
/*
void USART2_IRQHandler(void){
	if(USART_GetITStatus(USART2, USART_IT_RXNE)){
		static uint8_t cnt = 0;
		//char t = USART2->DR;
		if(cnt < MAX_STRLEN){
			received_string[cnt] = USART_ReceiveData(USART2);
		    if(received_string[cnt] == 'f' || received_string[cnt] == 's' || received_string[cnt] == 'b' || received_string[cnt] == 'l' || received_string[cnt] == 'r'){
		    	received_strings = received_string[cnt];
		    	//USART_puts(USART3, received_strings); 
		    }
		    cnt++;
		}
		//USART_puts(USART3, t);
		else{
			//USART_puts(USART3, received_string);
			cnt = 0;
			int i;
			for( i = 0 ; i< MAX_STRLEN ; i++){
				received_string[i]= 0;
			}
		}
	}
}
*/
/*
void USART2_IRQHandler(void){

	// check if the USART3 receive interrupt flag was set
	if( USART_GetITStatus(USART2, USART_IT_RXNE) ){

		//check the uart RX have accept the char
		GPIO_ToggleBits(GPIOD,GPIO_Pin_14);


		static uint8_t cnt = 0; // this counter is used to determine the uart receive string length

		//Receive_data = USART3->DR; // the character from the USART3 data register is saved in t
		Receive_data = USART_ReceiveData(USART2);;

		// check if the received character is not the LF character (used to determine end of string) 
		 // or the if the maximum string length has been been reached 
		 //

		if( cnt < MAX_STRLEN){ 
			received_string[cnt] = Receive_data;
            if(Receive_data=='0') GPIO_ToggleBits(GPIOD,GPIO_Pin_15);

            //start determine the period of command.
            if(received_string[cnt]=='z'){
            	USART_puts(USART3, received_string[cnt]);
                Receive_String_Ready = 1; //Ready to parse the command 
                received_strings = received_string[cnt-1];
                //USART_puts(USART3, received_strings);
                cnt=0; //restart to accept next stream message.
            }
            else{
                cnt++;
            }
		}
		else{ // over the max string length, cnt return to zero.
			Receive_String_Ready=1;
			cnt = 0;  
		}
		if(Receive_String_Ready){
			//print the content of the received string
			USART_puts(USART3, received_string);
			USART_puts(USART3,"\r\n");
			//receive_task();
			//clear the received string and the flag
			Receive_String_Ready = 0;
			int i;
			for( i = 0 ; i< MAX_STRLEN ; i++){
				received_string[i]= 0;
			}
		}
	}
}*/

void USART_puts(USART_TypeDef* USARTx, volatile uint8_t *s) //uint8_t = unsigned char
{
	while(*s){
		// wait until data register is empty
		//while( !(USARTx->SR & 0x00000040) );
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
		USART_SendData(USARTx, *s);
		*s++;
		//s++;
	}
}

void USART_putd(USART_TypeDef* USARTx, uint32_t number) //uint32_t = unsigned int
{
	static uint32_t temp; //uint32_t
	static uint8_t cnt = 0; //uint8_t = unsigned char
	volatile uint8_t tmp_num[10]; //uint8_t
	volatile uint8_t num[10]; //uint8_t

	if(number == 0){
		tmp_num[cnt++] = '0';
	}
	while(number != 0){
		temp = number % 10;
		number -= temp;
		number /= 10;
		tmp_num[cnt] = temp+'0';
		cnt++;
	}
	
	int j = 0;
	while(cnt){
		num[j++] = tmp_num[--cnt];
	}
	num[j] = '\0';

	USART_puts(USART3, num);
	cnt = 0;
	int i;
	for( i = 0 ; i< 10 ; i++){
		tmp_num[i]= 0;
		num[i]= 0;
	}
}

void send_byte(char ch)
{
	/* Wait until the RS232 port can receive another byte (this semaphore
	 * is "given" by the RS232 port interrupt when the buffer has room for
	 * another byte.
	 */
	while (!xSemaphoreTake(serial_tx_wait_sem, portMAX_DELAY));

	/* Send the byte and enable the transmit interrupt (it is disabled by
	 * the interrupt).
	 */
	USART_SendData(USART3, ch);
	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
}
char receive_byte()
{
    serial_ch_msg msg ; 

    /* Wait for a byte to be queued by the receive interrupts handler. */
    while (!xQueueReceive(serial_rx_queue, &msg, portMAX_DELAY));
    return msg.ch ; 

}

#if 0
void receive_task(void *p)
{
    int i , j;
	struct  receive_cmd_list * receive_cmd_type;
    
	while (1) {
		if(Receive_String_Ready ){
			//GPIO_ToggleBits(GPIOD,GPIO_Pin_14);
            /*load the accept command string to the command list structure*/
            receive_cmd_type = received_string;
            /*identifier the command's format, if yes, analyze the command list and perform it. */
            if(receive_cmd_type->Identifier[0] =='c' && receive_cmd_type->Identifier[1] =='m' && receive_cmd_type->Identifier[2] =='d'){
                PerformCommand(receive_cmd_type->group,receive_cmd_type->control_id, receive_cmd_type->value);
                
            }
            
			/*clear the received string and the flag*/
			Receive_String_Ready = 0;
			for( i = 0 ; i< MAX_STRLEN ; i++){
				received_string[i]= 0;
			}
		} 
	}
}
#endif