#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "app_config.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  MOTOR_MODE_SPEED = 0,
  MOTOR_MODE_MULTI_POSITION_T = 1,
  MOTOR_MODE_SINGLE_POSITION_T = 2,
  MOTOR_MODE_MULTI_POSITION_DIRECT = 3,
  MOTOR_MODE_SINGLE_POSITION_DIRECT = 4
} MotorMode_t;

typedef enum
{
  MOTOR_FEEDBACK_SPEED = 0,
  MOTOR_FEEDBACK_MULTI_POSITION = 1,
  MOTOR_FEEDBACK_SINGLE_POSITION = 2,
  MOTOR_FEEDBACK_ACCELERATION = 3,
  MOTOR_FEEDBACK_BUS_VOLTAGE = 4
} MotorFeedbackType_t;

typedef struct
{
  UART_HandleTypeDef *uart;
  uint8_t id;
  volatile int32_t speed_rpm;
  volatile int32_t multi_position_x10;
  volatile uint32_t speed_update_ms;
  volatile uint32_t position_update_ms;
  volatile bool speed_valid;
  volatile bool position_valid;
  volatile bool communication_ok;
} Motor_t;

HAL_StatusTypeDef Motor_Init(Motor_t *motor, UART_HandleTypeDef *uart, uint8_t id);
HAL_StatusTypeDef Motor_Enable(Motor_t *motor);
HAL_StatusTypeDef Motor_Disable(Motor_t *motor);
HAL_StatusTypeDef Motor_SetMode(Motor_t *motor, MotorMode_t mode);
HAL_StatusTypeDef Motor_SetSpeed(Motor_t *motor, float speed_rpm);
HAL_StatusTypeDef Motor_SetSingleTurnZero(Motor_t *motor);
HAL_StatusTypeDef Motor_RequestFeedback(Motor_t *motor, MotorFeedbackType_t type);

float Motor_GetSpeed(const Motor_t *motor);
float Motor_GetMultiPosition(const Motor_t *motor);
bool Motor_FeedbackIsFresh(const Motor_t *motor, uint32_t now_ms);

void Motor_UartRxCpltCallback(UART_HandleTypeDef *uart);
void Motor_UartErrorCallback(UART_HandleTypeDef *uart);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
