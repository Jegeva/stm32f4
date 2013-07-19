/**
  ******************************************************************************
  * @file    SysTick/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h" // again, added because ST didn't put it here ?
#include <math.h>
#include <arm_math.h>
#include <stdlib.h>

/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup SysTick_Example
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PORTLED GPIOC
#define RCC_AHB1Periph_PORTLED RCC_AHB1Periph_GPIOC

#define PIN_LED_R GPIO_Pin_6
#define PIN_LED_G GPIO_Pin_8
#define PIN_LED_B GPIO_Pin_9

#define PIN_LED_R2 GPIO_Pin_5

#define PIN_LED_R_Source GPIO_PinSource6
#define PIN_LED_G_Source GPIO_PinSource8
#define PIN_LED_B_Source GPIO_PinSource9


#define PIN_LED_R2 GPIO_Pin_5
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

volatile static __IO uint32_t TimingDelay;

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nTime);
void TIM3PWM_SetDC(uint16_t channel,uint16_t dutycycle);
/* Private functions ---------------------------------------------------------*/

 /**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f4xx.c file
     */     
  float32_t x = 0;
 
  GPIO_InitTypeDef GPIO_InitStructure;
 
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  uint16_t CCR1_Val = 333;
  uint16_t CCR2_Val = 249;
  uint16_t CCR3_Val = 166;
  uint16_t CCR4_Val = 83;
  uint16_t PrescalerValue = 0;
  uint16_t period = 665;

   if (SysTick_Config(SystemCoreClock / 1000))
  { 
    /* Capture error */ 
    while (1);
  }

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_PORTLED, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);


  /* Initialize Leds mounted on PC6,8,9 */
 
  GPIO_InitStructure.GPIO_Pin = PIN_LED_R2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =  PIN_LED_R|PIN_LED_B| PIN_LED_G;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(PORTLED, &GPIO_InitStructure);
  
  
 /* Alternate functions*/

  GPIO_PinAFConfig(PORTLED, PIN_LED_R_Source, GPIO_AF_TIM3);
  GPIO_PinAFConfig(PORTLED, PIN_LED_G_Source, GPIO_AF_TIM3);
  GPIO_PinAFConfig(PORTLED, PIN_LED_B_Source, GPIO_AF_TIM3);

  
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 28000000) - 1;
 
 TIM_TimeBaseStructure.TIM_Period = period;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);


  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* Not chan 2*/
  /* TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; */
  /* TIM_OCInitStructure.TIM_Pulse = CCR2_Val; */
  /* TIM_OC2Init(TIM3, &TIM_OCInitStructure); */
  /* TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); */


  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
  
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

   TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);





  /* /\* Setup SysTick Timer for 1 msec interrupts.   *\/ */
  /* if (SysTick_Config(SystemCoreClock / 1000)) */
  /* {  */
  /*   /\* Capture error *\/  */
  /*   while (1); */
  /* } */

  /* while (1) */
  /* { */
  /*   /\* Toggle LED3 and LED6 *\/ */
  /*   GPIO_ToggleBits(PORTLED,PIN_LED_R|PIN_LED_B); */
  /*   Delay(100*arm_cos_f32(x)); */
  /*   GPIO_ToggleBits(PORTLED,PIN_LED_R|PIN_LED_B); */
  /*   Delay(100*arm_cos_f32(x)); */
  /*   x+=0.1; */
  /* } */

  unsigned char prescal = 10;
  while(1){
    prescal--;
    Delay(50);
    TIM3PWM_SetDC(1,period*(arm_cos_f32(x)));

    TIM3PWM_SetDC(3,period*(arm_cos_f32(x+(PI/3))));
    TIM3PWM_SetDC(4,period*(arm_cos_f32(x+(2*PI/3))));
    if(!prescal){
      GPIO_ToggleBits(GPIOD,PIN_LED_R2);
      prescal =10;
    }
    x+=0.1;
  }
}


void TIM3PWM_SetDC(uint16_t channel,uint16_t dutycycle)
{
  if (channel == 1)
  {
    TIM3->CCR1 = dutycycle;
  }
  else if (channel == 2)
  {
    TIM3->CCR2 = dutycycle;
  }
  else if (channel == 3)
  {
    TIM3->CCR3 = dutycycle;
  }
  else 
  {
    TIM3->CCR4 = dutycycle;
  }
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
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

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
