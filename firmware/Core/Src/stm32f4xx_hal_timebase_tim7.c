/* HAL timebase on TIM7; FreeRTOS keeps ownership of SysTick. */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

#include "app_events.h"

TIM_HandleTypeDef htim7;

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef clock_config = {0};
  uint32_t flash_latency = 0;
  uint32_t timer_clock;
  uint32_t prescaler;

  HAL_RCC_GetClockConfig(&clock_config, &flash_latency);
  timer_clock = HAL_RCC_GetPCLK1Freq();
  if (clock_config.APB1CLKDivider != RCC_HCLK_DIV1)
  {
    timer_clock *= 2U;
  }

  prescaler = (timer_clock / 1000000U) - 1U;
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = prescaler;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  /* TIM7 is a 16-bit timer: 1 MHz / (1000 * uwTickFreq) gives the
   * requested HAL tick (1 ms for the default 1 kHz setting). */
  htim7.Init.Period = (1000U * (uint32_t)uwTickFreq) - 1U;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  __HAL_RCC_TIM7_CLK_ENABLE();
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    return HAL_ERROR;
  }

  HAL_NVIC_SetPriority(TIM7_IRQn, TickPriority, 0U);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
  return HAL_TIM_Base_Start_IT(&htim7);
}

void HAL_SuspendTick(void)
{
  __HAL_TIM_DISABLE_IT(&htim7, TIM_IT_UPDATE);
}

void HAL_ResumeTick(void)
{
  __HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  else if (htim->Instance == TIM6)
  {
    /* 100 Hz sample tick.  Keep the ISR tiny: latch the timestamp and wake
     * the acquisition task; the I2C transfer runs in task context. */
    App_OnSampleTickFromIsr();
  }
}
