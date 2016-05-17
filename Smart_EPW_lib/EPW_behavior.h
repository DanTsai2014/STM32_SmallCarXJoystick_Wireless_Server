/*=============================================================================
 *
 * @file     : EPW_behavior.h
 * @author        : JackABK
 * @data       : 2014/2/3
 * @brief   : car_behavior.c header file
 *
 *============================================================================*/
#ifndef __EPW_BEHAVIOR_H__
#define __EPW_BEHAVIOR_H__


/*=================Re-define the all by pins=========================*/
/****Joystick****/
#define JOYSTICK_PORT                                             GPIOC
#define JOYSTICK_X_AXIS_PIN                                       GPIO_Pin_0
#define JOYSTICK_Y_AXIS_PIN                                       GPIO_Pin_1

 #define ACC_PORT                                                 GPIOA
 #define ACC_X_AXIS_PIN                                           GPIO_Pin_0
 #define ACC_Y_AXIS_PIN                                           GPIO_Pin_1
 #define ACC_Z_AXIS_PIN                                           GPIO_Pin_2
//__IO uint16_t ADC1ConvertedVoltage[2];
int16_t ADC1ConvertedVoltage[2]; //or array? /*Joystick's ADC*/
int16_t Joystick_x_Filter;
int16_t Joystick_y_Filter;
int16_t tremor_count;
int16_t Joy_dir;


/*===============end of define  the all by pins========================*/

extern void parse_Joystick_dir();
extern void send_Joystick_data();
extern void send_Tremor_Warning();
extern void detect_x_Tremor();
extern void detect_y_Tremor();
extern void send_MPU6050_data();
#endif /* __CAR_BEHAVIOR_H__ */
