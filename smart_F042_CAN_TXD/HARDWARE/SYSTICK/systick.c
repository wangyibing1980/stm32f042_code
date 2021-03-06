/*----------------------------------------- systick.c -----------------------------------------*/
/**
  ********************************  STM32F0xx  *********************************
  * @文件名     ： systick.c
  * @作者       ： JayYang
  * @库版本     ： V1.5.0
  * @文件版本   ： V1.0.0
  * @日期       ： 2020年07月02日
  * @摘要       ： 系统定时器源文件
  ******************************************************************************/
/*----------------------------------------------------------------------------
  更新日志:
  2020-07-02 V1.0.0:初始版本
  ----------------------------------------------------------------------------*/
/* 包含的头文件 --------------------------------------------------------------*/
#include "systick.h"

/* 变量定义 --------------------------------------------------------------------*/

/************************************************
函数名称 ： Delay_us
功    能 ： 微秒延时函数
参    数 ： nUS：延时的微秒数
            48MHz时钟可取值为 1~349525us，
            6MHz时钟可取值为 1~2796202us 
返 回 值 ： 无
作    者 ： JayYang
*************************************************/
void delay_us(__IO uint32_t nUS){
    
	uint32_t temp;
    
	SysTick->LOAD = SystemCoreClock/8000000 * nUS;    //当SysTick嘀嗒定时器的时钟源采用HCLK时，设置延时的微妙数
    
	SysTick->VAL=0x00;      //清空计数器
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;       //使能嘀嗒定时器
    do
    {
        temp=SysTick->CTRL;//读取当前倒计数值
    }while((temp&0x01)&&(!(temp&(1<<16))));         //等待时间到达

    SysTick->CTRL &= ~SysTick_CTRL_COUNTFLAG_Msk;   //计数比较标志清0
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;      //关闭计数器
    SysTick->VAL=0x00;      //清空计数器
}


/************************************************
函数名称 ： Delay_ms
功    能 ： 毫秒延时函数
参    数 ： nMS：延时的毫秒数，
            48MHz时钟可取值为 1~300ms，
            6MHz时钟可取值为 1~2000ms 
返 回 值 ： 无
作    者 ： JayYang
*************************************************/
void delay_ms(__IO uint32_t nMS){
    
	uint32_t temp;
    
	SysTick->LOAD = SystemCoreClock/8000 * nMS;    //当SysTick嘀嗒定时器的时钟源采用HCLK时，设置延时的毫妙数
    
	SysTick->VAL=0x00;      //清空计数器
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;       //使能嘀嗒定时器
    do
    {
        temp=SysTick->CTRL;//读取当前倒计数值
    }while((temp&0x01)&&(!(temp&(1<<16))));         //等待时间到达

    SysTick->CTRL &= ~SysTick_CTRL_COUNTFLAG_Msk;   //计数比较标志清0
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;      //关闭计数器
    SysTick->VAL=0x00;      //清空计数器
}
