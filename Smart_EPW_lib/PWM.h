#ifndef __EPW_BEHAVIOR_H__ //avoid multiple definition
#define __EPW_BEHAVIOR_H__ //avoid multiple definition

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"

//smallcar: forward=IN2&IN4, backward=IN1&IN3, right=IN2(左輪前進)&IN3(右輪後退), left=IN1(左輪後退)&IN4(右輪前進)
uint16_t Wheels = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; //PC6 = IN1, PC7 = IN2, PC8 = IN3, PC9 = IN4
uint16_t EnableWheels = GPIO_Pin_8 | GPIO_Pin_9; //ENA = PA8, ENB = PA9
void init_Wheels();
void reset_Wheels();
void init_Timer();
void init_PWM();

#endif /*__PWM_H__*/