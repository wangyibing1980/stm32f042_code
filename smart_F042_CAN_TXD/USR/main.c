

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "systick.h"
#include "can.h"
#include "sys.h"
#include "motor.h"
#include "stm32f0xx_tim.h"
void PWM2_Config(uint32_t MOTOR_COUNT);


/** @addtogroup STM32F0xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BSRR_VAL 0x0C00

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef        GPIO_InitStructure;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */


u8 mbox;  
u16 i=0;
u16 j=0;
volatile uint8_t can_recv_flag=0;
int main(void)
{

  led_gpio_init();
  motor_gpio_init();
  CAN_Config();
  PWM2_Config(9599);
  TIM_SetCompare2(TIM2,100);
  TIM_CtrlPWMOutputs(TIM2, ENABLE);
  while (1)
  {
	//   GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_RESET);
	//   delay_ms(300); 
	//   GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_SET);
	//	 delay_ms(300); 


    delay_ms(1);
   j++;
	 if(j==1)
		{
			GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_RESET);		
		}
		if(j==500)
		{
			GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_SET);
		}	
   if(j==1000)
  {
	
    TxMessage.StdId = 2;//标准标识符1
    TxMessage.ExtId = 2;//扩展标识符1
    TxMessage.IDE = CAN_ID_STD;//
    TxMessage.RTR = CAN_RTR_DATA;//数据帧
    TxMessage.DLC = 8;//数据长度
    for(i=0; i<8; i++) //写入数据
    TxMessage.Data[i] = 1;
    mbox = CAN_Transmit(CAN,&TxMessage);//发送数据
    while(CAN_TransmitStatus(CAN,mbox) == CAN_TxStatus_Ok);//等待发送完成
      j=0;
    }
      if(can_recv_flag==1)
      {
        switch (ReceiveBuffer[0])                   //小车移动
            {
               case 0x01:
                switch (ReceiveBuffer[1])
                  {
                    case 0x01:
                       motor1_stop();
                      break;
                    case 0x02:
                        motor1_forward();
                      break;
                    case 0x03:
                        motor1_back();
                      break;
                    default:
                      break;
                  } 
                break;
              case 0x02:
                switch (ReceiveBuffer[1])
                  {
                    case 0x01:
                       motor2_stop();
                      break;
                    case 0x02:
                        motor2_forward();  
                      break;
                    case 0x03:
                        motor2_back();
                      break;
                    default:
                      break;
                  } 
                case 0x03:
                switch (ReceiveBuffer[1])              //LED调光
                  {
                   case 0x00:
                       TIM_Cmd(TIM2, DISABLE);                    
                      break;
                    case 0x01:
                       TIM_Cmd(TIM2, ENABLE);
                       TIM_SetCompare2(TIM2,100);
                       TIM_CtrlPWMOutputs(TIM2, ENABLE);
                      
                      break;
                    case 0x02:
                       TIM_Cmd(TIM2, ENABLE);
                       TIM_SetCompare2(TIM2,200);
                       TIM_CtrlPWMOutputs(TIM2, ENABLE);
                    
                      break;
                    case 0x03:
                       TIM_Cmd(TIM2, ENABLE);
                       TIM_SetCompare2(TIM2,800);
                       TIM_CtrlPWMOutputs(TIM2, ENABLE);
                    
                      break;
                    case 0x04:
                       TIM_Cmd(TIM2, ENABLE);
                       TIM_SetCompare2(TIM2,1000);
                       TIM_CtrlPWMOutputs(TIM2, ENABLE);
                      
                      break;
                    case 0x05:
                       TIM_Cmd(TIM2, ENABLE);
                       TIM_SetCompare2(TIM2,8000);
                       TIM_CtrlPWMOutputs(TIM2, ENABLE);
                    
                      break;
                    default:
                      break;
                  } 
            
                break;
              default:
                break;
            } 
    
        can_recv_flag=0;
      }

    

  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


void PWM2_Config(uint32_t MOTOR_COUNT)
{
  
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);//使能时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    /* GPIOA Configuration: Channel 1, 2, 3 and 4 as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_2);//开启复用
   
    
    TIM_TimeBaseInitStruct.TIM_Prescaler = 0;//5k
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period = MOTOR_COUNT;//这里设置的是5k的频率，value =(48000000/你想要的频率)-1 
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
    
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStruct);
    
    /* 频道1，2，3，4的PWM 模式设置 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//输出极性
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;//输出极性
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    
    TIM_OCInitStructure.TIM_Pulse = (MOTOR_COUNT>>1);//使能频道1配置
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    
//    TIM_OCInitStructure.TIM_Pulse = (MOTOR_COUNT>>1);//使能频道1配置
//    TIM_OC4Init(TIM1, &TIM_OCInitStructure);    
    //值为0~MOTOR_COUNT，这里MOTOR_COUNT的值已经减一了
    TIM_SetCompare2(TIM2,(MOTOR_COUNT>>1));//输出波形的1/2
//   TIM_SetCompare4(TIM1,(MOTOR_COUNT>>1));//输出波形的1/2
    /* TIM1 计算器使能*/
    TIM_Cmd(TIM2, ENABLE);
    /* TIM1 主输出使能 */
    TIM_CtrlPWMOutputs(TIM2, ENABLE);
}

