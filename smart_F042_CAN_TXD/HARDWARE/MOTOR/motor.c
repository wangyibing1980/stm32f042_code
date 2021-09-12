#include "motor.h"

void led_gpio_init(void)
{
  
  GPIO_InitTypeDef        GPIO_InitStructure;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  /* Configure PC10 and PC11 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_SET);

}
void motor_gpio_init(void)
{
  
  GPIO_InitTypeDef        GPIO_InitStructure;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  /* Configure PC10 and PC11 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_WriteBit(GPIOA, GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6, Bit_RESET);
}

void motor1_forward(void)
{
  GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_RESET);
  GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
}

void motor1_back(void)
{
  GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_SET);
  GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
}

void motor1_stop(void)
{
  GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_RESET);  
  GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);  
}

void motor2_forward(void)
{
 GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
 GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_SET);
}

void motor2_back(void)
{
  GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
  GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_RESET);  
}

void motor2_stop(void)
{
  GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);  
  GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_RESET);  
}

