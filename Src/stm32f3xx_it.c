#include "main.h"
#include "stm32f3xx_it.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim6;

/* STM32Cube Autogenerated handlers for various interrupts & errors */

/* Non maskable interrupt */
void NMI_Handler(void)
{
}

/* Hard fault interrupt */
void HardFault_Handler(void)
{
  while (1)
  {

  }
}

/* Memory management fault */
void MemManage_Handler(void)
{
  while (1)
  {

  }
}

/* Pre-fetch fault, memory access fault */
void BusFault_Handler(void)
{
  while (1)
  {

  }
}

/* Undefined instruction or illegal state */
void UsageFault_Handler(void)
{
  while (1)
  {

  }
}

/* System service call */
void SVC_Handler(void)
{
}

/* Debug monitor */
void DebugMon_Handler(void)
{
}

/* Pendable request for system service */
void PendSV_Handler(void)
{
}

/* System tick timer */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/* Autogenerated peripheral interrupt handlers */

/* ADC1 and ADC2 interrupts */
void ADC1_2_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&hadc1);
}

/* TIM6 interrupt */
void TIM6_DAC1_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim6);
}
