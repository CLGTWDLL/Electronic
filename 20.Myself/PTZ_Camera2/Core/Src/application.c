#include "application.h"

#include "app_config.h"
#include "control.h"
#include "k230_link.h"
#include "motor.h"
#include "vision_control.h"

#include <stdbool.h>

static Motor_t s_motor_1;
static Motor_t s_motor_2;
static MotorControl_t s_motor_control_1;
static MotorControl_t s_motor_control_2;
static DualMotorControl_t s_dual_motor_control;
static VisionControl_t s_vision_control;
static bool s_initialized;

static HAL_StatusTypeDef Application_RunSingleTurnZeroCalibration(void)
{
  HAL_StatusTypeDef status;

  /*
   * Keep both axes unpowered while saving the current mechanical pose.
   * No normal control task is started in calibration mode.
   */
  status = Motor_Disable(&s_motor_1);
  if (status != HAL_OK)
  {
    return status;
  }
  HAL_Delay(APP_ZERO_COMMAND_GAP_MS);

  status = Motor_Disable(&s_motor_2);
  if (status != HAL_OK)
  {
    return status;
  }
  HAL_Delay(APP_ZERO_COMMAND_GAP_MS);

  if (APP_ZERO_MOTOR_1_ENABLE != 0U)
  {
    status = Motor_SetSingleTurnZero(&s_motor_1);
    if (status != HAL_OK)
    {
      return status;
    }
    HAL_Delay(APP_ZERO_COMMAND_GAP_MS);
  }

  if (APP_ZERO_MOTOR_2_ENABLE != 0U)
  {
    status = Motor_SetSingleTurnZero(&s_motor_2);
    if (status != HAL_OK)
    {
      return status;
    }
    HAL_Delay(APP_ZERO_COMMAND_GAP_MS);
  }

  return HAL_OK;
}

HAL_StatusTypeDef Application_Init(UART_HandleTypeDef *motor_uart,
                                   UART_HandleTypeDef *camera_uart)
{
  if ((motor_uart == NULL) || (camera_uart == NULL) ||
      (motor_uart == camera_uart))
  {
    return HAL_ERROR;
  }

  if (Motor_Init(&s_motor_1, motor_uart, APP_MOTOR_1_ID) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (Motor_Init(&s_motor_2, motor_uart, APP_MOTOR_2_ID) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (APP_SINGLE_TURN_ZERO_CALIBRATION_ENABLE != 0U)
  {
    /*
     * Deliberately leave s_initialized false. Application_Task() will remain
     * idle after both persistent zero commands have been transmitted.
     */
    return Application_RunSingleTurnZeroCalibration();
  }

  Control_Init(&s_motor_control_1, &s_motor_1);
  Control_Init(&s_motor_control_2, &s_motor_2);
  DualControl_Init(&s_dual_motor_control, &s_motor_control_1,
                   &s_motor_control_2);
  VisionControl_Init(&s_vision_control, &s_motor_control_1,
                     &s_motor_control_2);

  if (K230Link_Init(camera_uart) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (DualControl_Start(&s_dual_motor_control) != HAL_OK)
  {
    return HAL_ERROR;
  }

  s_initialized = true;
  return HAL_OK;
}

void Application_Task(void)
{
  if (!s_initialized)
  {
    return;
  }

  K230Link_Task();
  VisionControl_Task(&s_vision_control);
  DualControl_Task(&s_dual_motor_control);
}
