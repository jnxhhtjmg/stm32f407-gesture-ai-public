/**
 * @file    uart_tx.h
 * @brief   Non-blocking UART transmitter backed by a ring buffer and DMA.
 *
 * Producers call UartTx_Write()/UartTx_WriteString() from task context and
 * return immediately.  A single DMA stream drains the ring in the background.
 */
#ifndef UART_TX_H
#define UART_TX_H

#include "stm32f4xx_hal.h"

/** Configure the DMA stream used by USART1 TX. Call once after MX_USART1_UART_Init(). */
void UartTx_Init(UART_HandleTypeDef *huart);

/**
 * @brief Queue one complete message.
 * @return Number of bytes queued, or 0 if the message did not fit.
 *         A message is never split: either all bytes are queued or none.
 */
uint16_t UartTx_Write(const void *data, uint16_t len);

/** Queue a NUL-terminated string (the NUL itself is not sent). */
uint16_t UartTx_WriteString(const char *text);

/** Start the next DMA chunk if the transmitter is idle. Safe from ISR context. */
void UartTx_Kick(void);

/** Number of messages dropped because the ring was full. */
uint32_t UartTx_GetDropped(void);

/** Number of times the DMA stream could not be restarted. */
uint32_t UartTx_GetErrors(void);

#endif /* UART_TX_H */
