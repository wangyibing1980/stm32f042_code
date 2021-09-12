#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f0xx.h"
#define LED_ON GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET)
#define LED_OFF GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET)

void led_gpio_init(void);
void motor_gpio_init(void);
void motor1_forward(void);
void motor1_back(void);
void motor1_stop(void);
void motor2_forward(void);
void motor2_back(void);
void motor2_stop(void);
#endif

