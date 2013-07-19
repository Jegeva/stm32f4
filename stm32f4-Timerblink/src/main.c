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

/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup SysTick_Example
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PORTLED GPIOD
#define RCC_AHB1Periph_PORTLED RCC_AHB1Periph_GPIOD
#define PIN_LED_R GPIO_Pin_14
#define PIN_LED_G GPIO_Pin_12
#define PIN_LED_B GPIO_Pin_15
#define PIN_LED_O GPIO_Pin_13
#define PIN_LED_R2 GPIO_Pin_5
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
volatile static __IO uint32_t TimingDelay;

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nTime);

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
       
  /* Initialize Leds mounted on STM32F4-Discovery board */
  GPIO_InitStructure.GPIO_Pin = PIN_LED_R2| PIN_LED_O|PIN_LED_B| PIN_LED_G|PIN_LED_R;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(PORTLED, &GPIO_InitStructure);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_PORTLED, ENABLE);


  GPIO_SetBits(PORTLED, PIN_LED_R2);//| PIN_LED_O|PIN_LED_B| PIN_LED_G|PIN_LED_R);
  GPIO_ResetBits(PORTLED, PIN_LED_G);
  GPIO_SetBits(PORTLED, PIN_LED_O);

  /* Setup SysTick Timer for 1 msec interrupts.   */
  if (SysTick_Config(SystemCoreClock / 1000))
  { 
    /* Capture error */ 
    while (1);
  }

  while (1)
  {
    /* Toggle LED3 and LED6 */
    GPIO_ToggleBits(PORTLED,PIN_LED_R|PIN_LED_B);
    Delay(100*arm_cos_f32(x));
    GPIO_ToggleBits(PORTLED,PIN_LED_R|PIN_LED_B);
    Delay(100*arm_cos_f32(x));
    x+=0.1;
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
