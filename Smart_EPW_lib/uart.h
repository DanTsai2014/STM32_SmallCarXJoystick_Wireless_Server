/*=============================================================================
  *
  * @file     : uart.h
  * @author        : JackABK
  * @data       : 2014/2/24
  * @brief   : uart.c header file
  *
  *============================================================================*/
#ifndef __UART_H__
#define __UART_H__



/*the usart acept the command from RX when RX interrupt is trigger*/
uint8_t Receive_data ;
unsigned char Receive_data_2;

/*Setting the USART MAX string lenth */
#define MAX_STRLEN 50 // this is the maximum string length of our string in characters

volatile uint8_t received_string[MAX_STRLEN]; // this will hold the recieved string
volatile unsigned char received_string_2[MAX_STRLEN];
volatile unsigned char received_tmp[2];
volatile uint8_t received_strings;
volatile unsigned char t;

extern void init_USART3(uint32_t baurate);
extern void USART3_IRQHandler(void);
extern void init_USART2(uint32_t baurate);
extern void USART2_IRQHandler(void);
extern void USART_puts(USART_TypeDef* USARTx, volatile uint8_t *s); //uint8_t
extern void USART_putd(USART_TypeDef* USARTx, uint32_t number); //uint32_t

#endif /* __UART_H__ */