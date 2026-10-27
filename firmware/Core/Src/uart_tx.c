/**
 * @file    uart_tx.c
 * @brief   USART1 TX ring buffer drained by DMA2 Stream 7 (channel 4).
 *
 * Why not HAL_UART_Transmit()?  A 45-byte CSV line takes ~3.9 ms at 115200.
 * Blocking the 100 Hz sampling task for that long shifts the sample instants
 * and is the main source of timing jitter in the original firmware.
 *
 * Concurrency model:
 *   - Any task may call UartTx_Write(); the ring index update is protected by
 *     a short interrupt-free section (messages are small, < 128 bytes).
 *   - The DMA completion callback advances the tail and starts the next chunk.
 *   - UartTx_Kick() itself is protected, so the "empty ring" race between a
 *     producer and the DMA completion ISR cannot lose a wake-up.
 */
#include "uart_tx.h"

#include <string.h>

#include "app_config.h"
#include "main.h"

#if ((APP_UART_TX_BUFFER_SIZE & (APP_UART_TX_BUFFER_SIZE - 1U)) != 0U)
#error "APP_UART_TX_BUFFER_SIZE must be a power of two"
#endif

#define TX_RING_MASK ((uint16_t)(APP_UART_TX_BUFFER_SIZE - 1U))

static UART_HandleTypeDef *s_uart;
static DMA_HandleTypeDef s_dma;

static uint8_t s_ring[APP_UART_TX_BUFFER_SIZE];
static volatile uint16_t s_head;   /* written by producers */
static volatile uint16_t s_tail;   /* written by the DMA completion ISR */
static volatile uint16_t s_in_flight;
static volatile uint8_t s_active;
static volatile uint32_t s_dropped;
static volatile uint32_t s_errors;
static volatile uint32_t s_last_activity;

static uint16_t tx_used(void)
{
  return (uint16_t)((uint16_t)(s_head - s_tail) & TX_RING_MASK);
}

static uint16_t tx_contiguous(void)
{
  uint16_t tail = s_tail;
  uint16_t head = s_head;

  if (head >= tail)
  {
    return (uint16_t)(head - tail);
  }
  return (uint16_t)(APP_UART_TX_BUFFER_SIZE - tail);
}

void UartTx_Init(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  s_uart = huart;
  s_head = 0U;
  s_tail = 0U;
  s_in_flight = 0U;
  s_active = 0U;
  s_dropped = 0U;
  s_errors = 0U;
  s_last_activity = HAL_GetTick();

  __HAL_RCC_DMA2_CLK_ENABLE();

  s_dma.Instance = DMA2_Stream7;
  s_dma.Init.Channel = DMA_CHANNEL_4;
  s_dma.Init.Direction = DMA_MEMORY_TO_PERIPH;
  s_dma.Init.PeriphInc = DMA_PINC_DISABLE;
  s_dma.Init.MemInc = DMA_MINC_ENABLE;
  s_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  s_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  s_dma.Init.Mode = DMA_NORMAL;
  s_dma.Init.Priority = DMA_PRIORITY_HIGH;
  s_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

  if (HAL_DMA_Init(&s_dma) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_LINKDMA(huart, hdmatx, s_dma);

  /* Numerically >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5), so this
   * ISR may call FreeRTOS APIs if the transmitter is ever extended to do so. */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 6U, 0U);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

uint16_t UartTx_Write(const void *data, uint16_t len)
{
  const uint8_t *src = (const uint8_t *)data;
  uint32_t primask;
  uint16_t first;

  if ((src == NULL) || (len == 0U))
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  if ((uint16_t)(APP_UART_TX_BUFFER_SIZE - 1U - tx_used()) < len)
  {
    ++s_dropped;
    __set_PRIMASK(primask);
    return 0U;
  }

  first = (uint16_t)(APP_UART_TX_BUFFER_SIZE - s_head);
  if (first > len)
  {
    first = len;
  }
  memcpy(&s_ring[s_head], src, first);
  if (first < len)
  {
    memcpy(&s_ring[0], src + first, (size_t)(len - first));
  }
  s_head = (uint16_t)((s_head + len) & TX_RING_MASK);

  __set_PRIMASK(primask);

  UartTx_Kick();
  return len;
}

uint16_t UartTx_WriteString(const char *text)
{
  size_t length;

  if (text == NULL)
  {
    return 0U;
  }
  length = strlen(text);
  if (length > 0xFFFFU)
  {
    length = 0xFFFFU;
  }
  return UartTx_Write(text, (uint16_t)length);
}

void UartTx_Kick(void)
{
  uint32_t primask;
  uint16_t chunk = 0U;
  uint8_t start = 0U;
  HAL_StatusTypeDef status;

  if (s_uart == NULL)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  /* Recovery for a lost DMA-complete callback: the UART is idle but we still
   * think a chunk is in flight.  1 s is well above the worst-case 4 KiB
   * transfer time at 115200 baud (~355 ms). */
  if ((s_active != 0U) &&
      ((HAL_GetTick() - s_last_activity) > 1000U) &&
      (s_uart->gState == HAL_UART_STATE_READY))
  {
    s_tail = (uint16_t)((s_tail + s_in_flight) & TX_RING_MASK);
    s_in_flight = 0U;
    s_active = 0U;
    ++s_errors;
  }

  if ((s_active == 0U) && (s_head != s_tail))
  {
    chunk = tx_contiguous();
    if (chunk != 0U)
    {
      s_in_flight = chunk;
      s_active = 1U;
      s_last_activity = HAL_GetTick();
      start = 1U;
    }
  }
  __set_PRIMASK(primask);

  if (start == 0U)
  {
    return;
  }

  status = HAL_UART_Transmit_DMA(s_uart, &s_ring[s_tail], chunk);
  if (status != HAL_OK)
  {
    /* Leave the data queued; the 100 Hz service loop retries the kick. */
    __disable_irq();
    s_active = 0U;
    s_in_flight = 0U;
    ++s_errors;
    __set_PRIMASK(primask);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART1))
  {
    s_tail = (uint16_t)((s_tail + s_in_flight) & TX_RING_MASK);
    s_in_flight = 0U;
    s_active = 0U;
    s_last_activity = HAL_GetTick();
    UartTx_Kick();
  }
}

void DMA2_Stream7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&s_dma);
}

uint32_t UartTx_GetDropped(void)
{
  return s_dropped;
}

uint32_t UartTx_GetErrors(void)
{
  return s_errors;
}
