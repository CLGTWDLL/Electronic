#include "k230_link.h"

#include "app_config.h"

#include <stddef.h>
#include <string.h>

static UART_HandleTypeDef *s_uart;
static uint8_t s_rx_byte;
static uint8_t s_work_frame[K230_FRAME_MAX_LENGTH];
static uint8_t s_pending_frame[K230_FRAME_MAX_LENGTH];
static volatile uint16_t s_work_length;
static volatile uint16_t s_pending_length;
static volatile bool s_collecting;
static volatile bool s_frame_ready;
static K230Target_t s_target;
static volatile K230LinkStats_t s_stats;

static bool K230Link_ParseUnsigned(const uint8_t *frame, uint16_t length,
                                   uint16_t *index, uint8_t delimiter,
                                   uint32_t *value)
{
  uint32_t result = 0U;
  bool has_digit = false;

  if ((frame == NULL) || (index == NULL) || (value == NULL))
  {
    return false;
  }

  while ((*index < length) && (frame[*index] != delimiter))
  {
    uint8_t byte = frame[*index];
    if ((byte < (uint8_t)'0') || (byte > (uint8_t)'9'))
    {
      return false;
    }
    has_digit = true;
    result = result * 10U + (uint32_t)(byte - (uint8_t)'0');
    if (result > 100000U)
    {
      return false;
    }
    (*index)++;
  }

  if (!has_digit || (*index >= length))
  {
    return false;
  }

  (*index)++;
  *value = result;
  return true;
}

static bool K230Link_ParseFrame(const uint8_t *frame, uint16_t length,
                                K230Target_t *target)
{
  uint16_t index = 1U;
  uint32_t declared_length;
  uint32_t function_id;
  uint32_t x;
  uint32_t y;
  uint32_t width;
  uint32_t height;
  uint16_t class_start;
  size_t expected_class_length = sizeof(K230_EXPECTED_CLASS_NAME) - 1U;

  if ((frame == NULL) || (target == NULL) || (length < 15U) ||
      (frame[0] != (uint8_t)'$') ||
      (frame[length - 1U] != (uint8_t)'#'))
  {
    return false;
  }

  if (!K230Link_ParseUnsigned(frame, length, &index, (uint8_t)',',
                              &declared_length) ||
      !K230Link_ParseUnsigned(frame, length, &index, (uint8_t)',',
                              &function_id) ||
      !K230Link_ParseUnsigned(frame, length, &index, (uint8_t)',', &x) ||
      !K230Link_ParseUnsigned(frame, length, &index, (uint8_t)',', &y) ||
      !K230Link_ParseUnsigned(frame, length, &index, (uint8_t)',', &width) ||
      !K230Link_ParseUnsigned(frame, length, &index, (uint8_t)',', &height))
  {
    return false;
  }

  class_start = index;
  while (index < (length - 1U))
  {
    if ((frame[index] < 0x20U) || (frame[index] > 0x7EU) ||
        (frame[index] == (uint8_t)','))
    {
      return false;
    }
    index++;
  }

  if ((declared_length != length) ||
      (function_id != K230_FUNCTION_ID) ||
      (class_start == index) ||
      (((size_t)index - class_start) != expected_class_length) ||
      (memcmp(&frame[class_start], K230_EXPECTED_CLASS_NAME,
              expected_class_length) != 0) ||
      (x >= K230_IMAGE_WIDTH_PX) ||
      (y >= K230_IMAGE_HEIGHT_PX) ||
      (width < K230_BOX_MIN_WIDTH_PX) ||
      (width > K230_BOX_MAX_WIDTH_PX) ||
      (height < K230_BOX_MIN_HEIGHT_PX) ||
      (height > K230_BOX_MAX_HEIGHT_PX) ||
      ((x + width) > K230_IMAGE_WIDTH_PX) ||
      ((y + height) > K230_IMAGE_HEIGHT_PX))
  {
    return false;
  }

  target->x = (uint16_t)x;
  target->y = (uint16_t)y;
  target->width = (uint16_t)width;
  target->height = (uint16_t)height;
  target->center_x = (uint16_t)(x + width / 2U);
  target->center_y = (uint16_t)(y + height / 2U);
  target->update_ms = HAL_GetTick();
  target->sequence = s_target.sequence + 1U;
  target->valid = true;
  return true;
}

HAL_StatusTypeDef K230Link_Init(UART_HandleTypeDef *uart)
{
  if (uart == NULL)
  {
    return HAL_ERROR;
  }

  s_uart = uart;
  s_rx_byte = 0U;
  s_work_length = 0U;
  s_pending_length = 0U;
  s_collecting = false;
  s_frame_ready = false;
  memset(&s_target, 0, sizeof(s_target));
  memset((void *)&s_stats, 0, sizeof(s_stats));

  if (uart->Instance == USART2)
  {
    HAL_NVIC_SetPriority(USART2_IRQn, K230_UART_IRQ_PREEMPT_PRIORITY,
                         K230_UART_IRQ_SUBPRIORITY);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }

  return HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
}

void K230Link_Task(void)
{
  K230Target_t parsed_target;

  if (!s_frame_ready)
  {
    return;
  }

  if (K230Link_ParseFrame(s_pending_frame, s_pending_length, &parsed_target))
  {
    s_target = parsed_target;
    s_stats.valid_frames++;
  }
  else
  {
    s_stats.invalid_frames++;
  }

  s_frame_ready = false;
}

bool K230Link_GetTarget(K230Target_t *target, uint32_t now_ms,
                        uint32_t timeout_ms)
{
  if ((target == NULL) || !s_target.valid ||
      ((now_ms - s_target.update_ms) > timeout_ms))
  {
    return false;
  }

  *target = s_target;
  return true;
}

void K230Link_GetStats(K230LinkStats_t *stats)
{
  if (stats == NULL)
  {
    return;
  }

  stats->valid_frames = s_stats.valid_frames;
  stats->invalid_frames = s_stats.invalid_frames;
  stats->dropped_frames = s_stats.dropped_frames;
  stats->uart_errors = s_stats.uart_errors;
}

void K230Link_UartRxCpltCallback(UART_HandleTypeDef *uart)
{
  uint16_t i;

  if ((s_uart == NULL) || (uart != s_uart))
  {
    return;
  }

  if (s_rx_byte == (uint8_t)'$')
  {
    s_work_length = 0U;
    s_collecting = true;
  }

  if (s_collecting)
  {
    if (s_work_length < K230_FRAME_MAX_LENGTH)
    {
      s_work_frame[s_work_length++] = s_rx_byte;

      if (s_rx_byte == (uint8_t)'#')
      {
        if (!s_frame_ready)
        {
          for (i = 0U; i < s_work_length; ++i)
          {
            s_pending_frame[i] = s_work_frame[i];
          }
          s_pending_length = s_work_length;
          s_frame_ready = true;
        }
        else
        {
          s_stats.dropped_frames++;
        }
        s_collecting = false;
        s_work_length = 0U;
      }
    }
    else
    {
      s_stats.invalid_frames++;
      s_collecting = false;
      s_work_length = 0U;
    }
  }

  (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
}

void K230Link_UartErrorCallback(UART_HandleTypeDef *uart)
{
  if ((s_uart == NULL) || (uart != s_uart))
  {
    return;
  }

  s_stats.uart_errors++;
  s_collecting = false;
  s_work_length = 0U;
  (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
}
