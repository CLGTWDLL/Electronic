#include "motor.h"

#include <stddef.h>

#define MOTOR_FRAME_HEAD       0x7AU
#define MOTOR_FRAME_TAIL       0x7BU

static UART_HandleTypeDef *s_bus_uart;
static Motor_t *s_motors[MOTOR_MAX_DEVICES];
static uint8_t s_motor_count;
static uint8_t s_rx_byte;
static uint8_t s_rx_frame[9];
static uint8_t s_rx_count;

static uint8_t Motor_Bcc(const uint8_t *data, uint8_t length)
{
  uint8_t bcc = 0U;
  uint8_t i;

  for (i = 0U; i < length; ++i)
  {
    bcc ^= data[i];
  }
  return bcc;
}

static HAL_StatusTypeDef Motor_Send(Motor_t *motor, uint8_t function,
                                    const uint8_t *data, uint8_t data_length)
{
  uint8_t frame[9];
  uint8_t length;
  uint8_t i;

  if ((motor == NULL) || (motor->uart == NULL) || (data_length > 4U))
  {
    return HAL_ERROR;
  }

  frame[0] = MOTOR_FRAME_HEAD;
  frame[1] = motor->id;
  frame[2] = function;
  for (i = 0U; i < data_length; ++i)
  {
    frame[3U + i] = data[i];
  }
  frame[3U + data_length] = Motor_Bcc(frame, (uint8_t)(3U + data_length));
  frame[4U + data_length] = MOTOR_FRAME_TAIL;
  length = (uint8_t)(5U + data_length);

  return HAL_UART_Transmit(motor->uart, frame, length, MOTOR_TX_TIMEOUT_MS);
}

static Motor_t *Motor_FindById(uint8_t id)
{
  uint8_t i;

  for (i = 0U; i < s_motor_count; ++i)
  {
    if ((s_motors[i] != NULL) && (s_motors[i]->id == id))
    {
      return s_motors[i];
    }
  }
  return NULL;
}

static void Motor_ParseFeedback(void)
{
  Motor_t *motor;
  uint8_t type;
  uint32_t raw;
  uint32_t now;

  if ((s_rx_frame[0] != MOTOR_FRAME_HEAD) ||
      (s_rx_frame[8] != MOTOR_FRAME_TAIL) ||
      (Motor_Bcc(s_rx_frame, 7U) != s_rx_frame[7]))
  {
    return;
  }

  motor = Motor_FindById(s_rx_frame[1]);
  if (motor == NULL)
  {
    return;
  }

  type = s_rx_frame[2];
  raw = ((uint32_t)s_rx_frame[3] << 24) |
        ((uint32_t)s_rx_frame[4] << 16) |
        ((uint32_t)s_rx_frame[5] << 8) |
        (uint32_t)s_rx_frame[6];
  now = HAL_GetTick();

  if (type == (uint8_t)MOTOR_FEEDBACK_SPEED)
  {
    motor->speed_rpm = (int32_t)raw;
    motor->speed_update_ms = now;
    motor->speed_valid = true;
  }
  else if (type == (uint8_t)MOTOR_FEEDBACK_MULTI_POSITION)
  {
    motor->multi_position_x10 = (int32_t)raw;
    motor->position_update_ms = now;
    motor->position_valid = true;
  }
  else
  {
    return;
  }
  motor->communication_ok = true;
}

static void Motor_RxByte(uint8_t byte)
{
  if ((s_rx_count == 0U) && (byte != MOTOR_FRAME_HEAD))
  {
    return;
  }

  s_rx_frame[s_rx_count++] = byte;
  if (s_rx_count == sizeof(s_rx_frame))
  {
    Motor_ParseFeedback();
    s_rx_count = 0U;
  }
}

HAL_StatusTypeDef Motor_Init(Motor_t *motor, UART_HandleTypeDef *uart, uint8_t id)
{
  HAL_StatusTypeDef status;
  uint8_t i;

  if ((motor == NULL) || (uart == NULL))
  {
    return HAL_ERROR;
  }

  if ((s_motor_count >= MOTOR_MAX_DEVICES) ||
      ((s_motor_count > 0U) && (uart != s_bus_uart)))
  {
    return HAL_ERROR;
  }
  for (i = 0U; i < s_motor_count; ++i)
  {
    if (s_motors[i]->id == id)
    {
      return HAL_ERROR;
    }
  }

  motor->uart = uart;
  motor->id = id;
  motor->speed_rpm = 0;
  motor->multi_position_x10 = 0;
  motor->speed_update_ms = 0U;
  motor->position_update_ms = 0U;
  motor->speed_valid = false;
  motor->position_valid = false;
  motor->communication_ok = false;

  if (s_motor_count == 0U)
  {
    s_bus_uart = uart;
    s_rx_count = 0U;
    s_motors[s_motor_count++] = motor;
    HAL_NVIC_SetPriority(USART1_IRQn, MOTOR_UART_IRQ_PREEMPT_PRIORITY,
                         MOTOR_UART_IRQ_SUBPRIORITY);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    status = HAL_UART_Receive_IT(uart, &s_rx_byte, 1U);
    if (status != HAL_OK)
    {
      s_motor_count = 0U;
      s_bus_uart = NULL;
    }
    return status;
  }

  s_motors[s_motor_count++] = motor;
  return HAL_OK;
}

HAL_StatusTypeDef Motor_Enable(Motor_t *motor)
{
  return Motor_Send(motor, 0x06U, NULL, 0U);
}

HAL_StatusTypeDef Motor_Disable(Motor_t *motor)
{
  return Motor_Send(motor, 0x05U, NULL, 0U);
}

HAL_StatusTypeDef Motor_SetMode(Motor_t *motor, MotorMode_t mode)
{
  uint8_t data[2] = {0U, (uint8_t)mode};
  return Motor_Send(motor, 0x00U, data, 2U);
}

HAL_StatusTypeDef Motor_SetSpeed(Motor_t *motor, float speed_rpm)
{
  int16_t command;
  uint8_t data[2];

  if (speed_rpm > MOTOR_SPEED_LIMIT_RPM)
  {
    speed_rpm = MOTOR_SPEED_LIMIT_RPM;
  }
  else if (speed_rpm < -MOTOR_SPEED_LIMIT_RPM)
  {
    speed_rpm = -MOTOR_SPEED_LIMIT_RPM;
  }

  command = (int16_t)((speed_rpm >= 0.0f) ? (speed_rpm + 0.5f) : (speed_rpm - 0.5f));
  data[0] = (uint8_t)((uint16_t)command >> 8);
  data[1] = (uint8_t)command;
  return Motor_Send(motor, 0x01U, data, 2U);
}

HAL_StatusTypeDef Motor_SetSingleTurnZero(Motor_t *motor)
{
  /*
   * Protocol frame:
   *   7A <motor ID> 0A <BCC> 7B
   * The motor stores the current shaft position as the persistent
   * single-turn absolute-angle zero point.
   */
  return Motor_Send(motor, 0x0AU, NULL, 0U);
}

HAL_StatusTypeDef Motor_RequestFeedback(Motor_t *motor, MotorFeedbackType_t type)
{
  uint8_t data = (uint8_t)type;
  return Motor_Send(motor, 0x0EU, &data, 1U);
}

float Motor_GetSpeed(const Motor_t *motor)
{
  return (motor == NULL) ? 0.0f : (float)motor->speed_rpm;
}

float Motor_GetMultiPosition(const Motor_t *motor)
{
  return (motor == NULL) ? 0.0f : ((float)motor->multi_position_x10 * 0.1f);
}

bool Motor_FeedbackIsFresh(const Motor_t *motor, uint32_t now_ms)
{
  if ((motor == NULL) || !motor->communication_ok || !motor->position_valid)
  {
    return false;
  }
  return ((now_ms - motor->position_update_ms) <= MOTOR_FEEDBACK_TIMEOUT_MS);
}

void Motor_UartRxCpltCallback(UART_HandleTypeDef *uart)
{
  if ((s_bus_uart != NULL) && (uart == s_bus_uart))
  {
    Motor_RxByte(s_rx_byte);
    (void)HAL_UART_Receive_IT(uart, &s_rx_byte, 1U);
  }
}

void Motor_UartErrorCallback(UART_HandleTypeDef *uart)
{
  if ((s_bus_uart != NULL) && (uart == s_bus_uart))
  {
    s_rx_count = 0U;
    (void)HAL_UART_Receive_IT(uart, &s_rx_byte, 1U);
  }
}
