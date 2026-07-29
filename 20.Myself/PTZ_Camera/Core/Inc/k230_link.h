#ifndef K230_LINK_H
#define K230_LINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint16_t x;
  uint16_t y;
  uint16_t width;
  uint16_t height;
  uint16_t center_x;
  uint16_t center_y;
  uint32_t update_ms;
  uint32_t sequence;
  bool valid;
} K230Target_t;

typedef struct
{
  uint32_t valid_frames;
  uint32_t invalid_frames;
  uint32_t dropped_frames;
  uint32_t uart_errors;
} K230LinkStats_t;

HAL_StatusTypeDef K230Link_Init(UART_HandleTypeDef *uart);
void K230Link_Task(void);

bool K230Link_GetTarget(K230Target_t *target, uint32_t now_ms,
                        uint32_t timeout_ms);
void K230Link_GetStats(K230LinkStats_t *stats);

void K230Link_UartRxCpltCallback(UART_HandleTypeDef *uart);
void K230Link_UartErrorCallback(UART_HandleTypeDef *uart);

#ifdef __cplusplus
}
#endif

#endif /* K230_LINK_H */
