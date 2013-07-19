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
#include "stm32f4_discovery_lis302dl.h"
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

#define PIN_LED_R_Source GPIO_PinSource14
#define PIN_LED_G_Source GPIO_PinSource12
#define PIN_LED_B_Source GPIO_PinSource15
#define PIN_LED_O_Source GPIO_PinSource13

#define PORTRGB GPIOC
#define RCC_AHB1Periph_PORTRGB RCC_AHB1Periph_GPIOC
#define PIN_RGB_R GPIO_Pin_6
#define PIN_RGB_G GPIO_Pin_8
#define PIN_RGB_B GPIO_Pin_9
#define PIN_RGB_R_Source GPIO_PinSource6
#define PIN_RGB_G_Source GPIO_PinSource8
#define PIN_RGB_B_Source GPIO_PinSource9


#define PIN_LED_R2 GPIO_Pin_5
/* Private macro -------------------------------------------------------------*/
#define ABS(x)                           (x < 0) ? (-x) : x
#define MAX(a,b)                         (a < b) ? (b) : a

/* Private variables ---------------------------------------------------------*/
__IO int8_t XOffset;
__IO int8_t YOffset;
__IO int8_t ZOffset;
volatile static __IO uint32_t TimingDelay;
uint8_t Buffer[6];
int8_t Gravity[3];
int8_t Offset[3];
/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nTime);

void TIM3PWM_SetDC(uint16_t channel,uint16_t dutycycle); 
void TIM4PWM_SetDC(uint16_t channel,uint16_t dutycycle);
void TIM_Setup(uint16_t period);
void PORTLED_AFTIM4_Setup();
void PORTRGB_AFTIM3_Setup();
void PORTLED_IO_R2_Setup();
void Accel_Setup();
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
  float32_t y = 0;
  float alpha = 0.6;
 uint16_t period = 665;
 


   if (SysTick_Config(SystemCoreClock / 1000))
  { 
    /* Capture error */ 
    while (1);
  }

   /*init both ports with af functions + R2*/
   PORTLED_AFTIM4_Setup();
   PORTRGB_AFTIM3_Setup();
   PORTLED_IO_R2_Setup();
   Accel_Setup();
   TIM_Setup(period);
  /* while (1) */
  /* { */
  /*   /\* Toggle LED3 and LED6 *\/ */
  /*   GPIO_ToggleBits(PORTLED,PIN_LED_R2); */
  /*   Delay(1000); */
  /* } */

  unsigned char prescal = 10;
  while(1){
    prescal--;
    Delay(30);

    LIS302DL_Read(Buffer, LIS302DL_OUT_X_ADDR, 6);
    Offset[0] = (int8_t)Buffer[0];
    Offset[1] = (int8_t)Buffer[2];
    Offset[2] = (int8_t)Buffer[4];
    Gravity[0] =  Gravity[0]*alpha + Offset[0]*(1-alpha);
    Gravity[1] =  Gravity[1]*alpha + Offset[1]*(1-alpha);
    Gravity[2] =  Gravity[2]*alpha + Offset[2]*(1-alpha);
    Offset[0] -= Gravity[0];
    Offset[1] -= Gravity[1];
    Offset[2] -= Gravity[2];

    TIM3PWM_SetDC(1,period*(arm_cos_f32(x)));
    TIM3PWM_SetDC(3,period*(arm_cos_f32(x+(2*PI/3))));
    TIM3PWM_SetDC(4,period*(arm_cos_f32(x+(4*PI/3))));

    TIM4PWM_SetDC(1,period*  (0.9*( (1+arm_cos_f32( y))          /2)));
    TIM4PWM_SetDC(2,period*  (0.9*( (1+arm_cos_f32( y+(PI/2)))   /2)));
    TIM4PWM_SetDC(3,period*  (0.9*( (1+arm_cos_f32( y+(PI)))     /2)));
    TIM4PWM_SetDC(4,period*  (0.9*( (1+arm_cos_f32( y+(3*PI/2))) /2)));
    if(!prescal){
      GPIO_ToggleBits(GPIOD,PIN_LED_R2);
      prescal =10;
    }
    if( (ABS(Offset[0])+ABS(Offset[1])+ABS(Offset[2]) )>2){
    x+=0.1;
    y+=0.4;
    }
  }
}

void Accel_Setup(){
  LIS302DL_InitTypeDef  LIS302DL_InitStruct;
  LIS302DL_InterruptConfigTypeDef LIS302DL_InterruptStruct;  
  uint8_t ctrl = 0;
  /* Set configuration of LIS302DL*/
  LIS302DL_InitStruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
  LIS302DL_InitStruct.Output_DataRate = LIS302DL_DATARATE_400;
  LIS302DL_InitStruct.Axes_Enable = LIS302DL_X_ENABLE | LIS302DL_Y_ENABLE | LIS302DL_Z_ENABLE;
  LIS302DL_InitStruct.Full_Scale = LIS302DL_FULLSCALE_2_3;
  LIS302DL_InitStruct.Self_Test = LIS302DL_SELFTEST_NORMAL;
  LIS302DL_Init(&LIS302DL_InitStruct);
    
  /* Set configuration of Internal High Pass Filter of LIS302DL*/
  LIS302DL_InterruptStruct.Latch_Request = LIS302DL_INTERRUPTREQUEST_LATCHED;
  LIS302DL_InterruptStruct.SingleClick_Axes = LIS302DL_CLICKINTERRUPT_Z_ENABLE; 
  LIS302DL_InterruptStruct.DoubleClick_Axes = LIS302DL_DOUBLECLICKINTERRUPT_Z_ENABLE; 
  LIS302DL_InterruptConfig(&LIS302DL_InterruptStruct);
  Delay(35);

   /* Configure Interrupt control register: enable Click interrupt1 */
  ctrl = 0x07;
  LIS302DL_Write(&ctrl, LIS302DL_CTRL_REG3_ADDR, 1);
  
  /* Enable Interrupt generation on click/double click on Z axis */
  ctrl = 0x70;
  LIS302DL_Write(&ctrl, LIS302DL_CLICK_CFG_REG_ADDR, 1);
  
  /* Configure Click Threshold on X/Y axis (10 x 0.5g) */
  ctrl = 0xAA;
  LIS302DL_Write(&ctrl, LIS302DL_CLICK_THSY_X_REG_ADDR, 1);
  
  /* Configure Click Threshold on Z axis (10 x 0.5g) */
  ctrl = 0x0A;
  LIS302DL_Write(&ctrl, LIS302DL_CLICK_THSZ_REG_ADDR, 1);
  
  /* Configure Time Limit */
  ctrl = 0x03;
  LIS302DL_Write(&ctrl, LIS302DL_CLICK_TIMELIMIT_REG_ADDR, 1);
    
  /* Configure Latency */
  ctrl = 0x7F;
  LIS302DL_Write(&ctrl, LIS302DL_CLICK_LATENCY_REG_ADDR, 1);
  
  /* Configure Click Window */
  ctrl = 0x7F;
  LIS302DL_Write(&ctrl, LIS302DL_CLICK_WINDOW_REG_ADDR, 1);
  
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

void TIM4PWM_SetDC(uint16_t channel,uint16_t dutycycle)
{
  if (channel == 1)
  {
    TIM4->CCR1 = dutycycle;
  }
  else if (channel == 2)
  {
    TIM4->CCR2 = dutycycle;
  }
  else if (channel == 3)
  {
    TIM4->CCR3 = dutycycle;
  }
  else 
  {
    TIM4->CCR4 = dutycycle;
  }
}

void TIM3PWM_Setup(TIM_TimeBaseInitTypeDef*  TIM_TimeBaseStructure){
TIM_OCInitTypeDef  TIM_OCInitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  TIM_TimeBaseInit(TIM3, TIM_TimeBaseStructure);
  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* Not chan 2*/
  /* TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; */
  /* TIM_OCInitStructure.TIM_Pulse = 0; */
  /* TIM_OC2Init(TIM3, &TIM_OCInitStructure); */
  /* TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); */


  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
  
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

   TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}
void TIM4PWM_Setup(TIM_TimeBaseInitTypeDef*  TIM_TimeBaseStructure){
TIM_OCInitTypeDef  TIM_OCInitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  TIM_TimeBaseInit(TIM4, TIM_TimeBaseStructure);
/* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);


  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
  
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

   TIM_ARRPreloadConfig(TIM4, ENABLE);

  /* TIM4 enable counter */
  TIM_Cmd(TIM4, ENABLE);
}

void TIM_Setup(uint16_t period){
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  
  uint16_t PrescalerValue = 0;
  
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 28000000) - 1;
  TIM_TimeBaseStructure.TIM_Period = period;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  
  TIM3PWM_Setup(&TIM_TimeBaseStructure);
  TIM4PWM_Setup(&TIM_TimeBaseStructure);
}

void PORTLED_IO_R2_Setup(){
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_PORTLED, ENABLE);
  GPIO_InitStructure.GPIO_Pin = PIN_LED_R2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(PORTLED, &GPIO_InitStructure);
}
void PORTLED_AFTIM4_Setup(){
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_PORTLED, ENABLE);
  GPIO_InitStructure.GPIO_Pin =  PIN_LED_O|PIN_LED_R|PIN_LED_B| PIN_LED_G;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(PORTLED, &GPIO_InitStructure);
  /* Alternate functions*/
  GPIO_PinAFConfig(PORTLED, PIN_LED_R_Source, GPIO_AF_TIM4);
  GPIO_PinAFConfig(PORTLED, PIN_LED_G_Source, GPIO_AF_TIM4);
  GPIO_PinAFConfig(PORTLED, PIN_LED_B_Source, GPIO_AF_TIM4);
  GPIO_PinAFConfig(PORTLED, PIN_LED_O_Source, GPIO_AF_TIM4);
}

void PORTRGB_AFTIM3_Setup(){
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_PORTRGB, ENABLE);
  GPIO_InitStructure.GPIO_Pin =  PIN_RGB_R|PIN_RGB_B| PIN_RGB_G;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(PORTRGB, &GPIO_InitStructure);
 /* Alternate functions*/
  GPIO_PinAFConfig(PORTRGB, PIN_RGB_R_Source, GPIO_AF_TIM3);
  GPIO_PinAFConfig(PORTRGB, PIN_RGB_G_Source, GPIO_AF_TIM3);
  GPIO_PinAFConfig(PORTRGB, PIN_RGB_B_Source, GPIO_AF_TIM3);
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


uint32_t LIS302DL_TIMEOUT_UserCallback(void)
{
  /* MEMS Accelerometer Timeout error occured */
  while (1)
  {   
  }
}











/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
