/*----------------------------------------- systick.h -----------------------------------------*/
/**
  *********************************  STM32F0xx  ********************************
  * @文件名     ： systick.h
  * @作者       ： JayYang
  * @库版本     ： V1.5.0
  * @文件版本   ： V1.0.0
  * @日期       ： 2020年07月02日
  * @摘要       ： 系统定时器头文件
  ******************************************************************************/

/* 定义防止递归包含 ----------------------------------------------------------*/
#ifndef _SYSTICK_H
#define _SYSTICK_H

/* 包含的头文件 --------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "core_cm0.h"
#include "motor.h"

/* 宏定义 --------------------------------------------------------------------*/

/* 变量定义 --------------------------------------------------------------------*/

/* 函数申明 ------------------------------------------------------------------*/
void delay_us(__IO uint32_t nUS);   //微秒延时函数
void delay_ms(__IO uint32_t nMS);   //毫秒延时函数最大300MS


#endif /* _SYSTICK_H */
