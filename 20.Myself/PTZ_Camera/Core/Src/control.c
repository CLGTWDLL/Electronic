#include "control.h"

#include <stddef.h>

static float Control_Clamp(float value, float minimum, float maximum)
{
  if (value > maximum)
  {
    return maximum;
  }
  if (value < minimum)
  {
    return minimum;
  }
  return value;
}

static float Control_ShortestPositionError(float target_deg, float current_deg)
{
  float error = target_deg - current_deg;
  int32_t complete_turns = (int32_t)(error / POSITION_FULL_TURN_DEG);

  /* 去掉完整圈数，再把位置误差限制到 -180 至 +180 度。 */
  error -= (float)complete_turns * POSITION_FULL_TURN_DEG;
  if (error > POSITION_HALF_TURN_DEG)
  {
    error -= POSITION_FULL_TURN_DEG;
  }
  else if (error < -POSITION_HALF_TURN_DEG)
  {
    error += POSITION_FULL_TURN_DEG;
  }
  return error;
}

static float Control_UpdateTrajectory(float target_deg,
                                      float trajectory_deg,
                                      float dt)
{
  float remaining_error =
      Control_ShortestPositionError(target_deg, trajectory_deg);
  float maximum_step = POSITION_TARGET_SLEW_DEG_PER_S * dt;

  return trajectory_deg +
         Control_Clamp(remaining_error, -maximum_step, maximum_step);
}

static void PID_Reset(PID_t *pid)
{
  pid->integral = 0.0f;
  pid->previous_error = 0.0f;
}

static float PID_Update(PID_t *pid, float error, float dt)
{
  float derivative = (error - pid->previous_error) / dt;
  float candidate_integral = pid->integral + error * dt;
  float output = pid->kp * error + pid->ki * candidate_integral + pid->kd * derivative;
  float limited = Control_Clamp(output, pid->output_min, pid->output_max);

  if ((output == limited) ||
      ((output > pid->output_max) && (error < 0.0f)) ||
      ((output < pid->output_min) && (error > 0.0f)))
  {
    pid->integral = candidate_integral;
  }
  pid->previous_error = error;
  return limited;
}

static void Control_UpdateAndSend(MotorControl_t *control, uint32_t now, float dt)
{
  float current_position;
  float position_error;

  if (!Motor_FeedbackIsFresh(control->motor, now))
  {
    control->output_speed_rpm = 0.0f;
    control->trajectory_initialized = false;
    PID_Reset(&control->position_pid);
    (void)Motor_SetSpeed(control->motor, 0.0f);
    return;
  }

  current_position = Motor_GetMultiPosition(control->motor);
  if (!control->trajectory_initialized)
  {
    /*
     * Start at the measured shaft position. The next cycles move this internal
     * setpoint gradually, so enabling the motors cannot create a position step.
     */
    control->trajectory_position_deg = current_position;
    control->trajectory_initialized = true;
    control->target_speed_rpm = 0.0f;
    control->output_speed_rpm = 0.0f;
    PID_Reset(&control->position_pid);
    (void)Motor_SetSpeed(control->motor, 0.0f);
    return;
  }

  control->trajectory_position_deg =
      Control_UpdateTrajectory(control->target_position_deg,
                               control->trajectory_position_deg, dt);
  position_error = Control_ShortestPositionError(
      control->trajectory_position_deg, current_position);
  control->target_speed_rpm = PID_Update(&control->position_pid,
                                         position_error, dt);
  control->output_speed_rpm = control->target_speed_rpm;
  (void)Motor_SetSpeed(control->motor, control->output_speed_rpm);
}

void Control_Init(MotorControl_t *control, Motor_t *motor)
{
  if (control == NULL)
  {
    return;
  }

  control->motor = motor;
  control->target_position_deg = 0.0f;
  control->trajectory_position_deg = 0.0f;
  control->target_speed_rpm = 0.0f;
  control->output_speed_rpm = 0.0f;
  control->next_action_ms = HAL_GetTick();
  control->action_phase = 0U;
  control->trajectory_initialized = false;
  control->running = false;

  control->position_pid = (PID_t){POSITION_PID_KP, POSITION_PID_KI,
                                  POSITION_PID_KD, 0.0f, 0.0f,
                                  -POSITION_SPEED_LIMIT_RPM, POSITION_SPEED_LIMIT_RPM};
}

HAL_StatusTypeDef Control_Start(MotorControl_t *control)
{
  HAL_StatusTypeDef status;

  if ((control == NULL) || (control->motor == NULL))
  {
    return HAL_ERROR;
  }

  status = Motor_Enable(control->motor);
  if (status != HAL_OK)
  {
    return status;
  }
  HAL_Delay(MOTOR_COMMAND_GAP_MS);
  status = Motor_SetMode(control->motor, MOTOR_MODE_SPEED);
  if (status == HAL_OK)
  {
    PID_Reset(&control->position_pid);
    control->trajectory_initialized = false;
    control->next_action_ms = HAL_GetTick() + MOTOR_COMMAND_GAP_MS;
    control->action_phase = 0U;
    control->running = true;
  }
  else
  {
    HAL_Delay(MOTOR_COMMAND_GAP_MS);
    (void)Motor_Disable(control->motor);
  }
  return status;
}

void Control_Stop(MotorControl_t *control)
{
  if ((control == NULL) || (control->motor == NULL))
  {
    return;
  }
  control->running = false;
  control->trajectory_initialized = false;
  (void)Motor_SetSpeed(control->motor, 0.0f);
  HAL_Delay(MOTOR_COMMAND_GAP_MS);
  (void)Motor_Disable(control->motor);
  PID_Reset(&control->position_pid);
}

void Control_SetTargetPosition(MotorControl_t *control, float position_deg)
{
  if (control != NULL)
  {
    control->target_position_deg = position_deg;
  }
}

void Control_SetPositionPID(MotorControl_t *control, float kp, float ki, float kd)
{
  if (control != NULL)
  {
    control->position_pid.kp = kp;
    control->position_pid.ki = ki;
    control->position_pid.kd = kd;
    PID_Reset(&control->position_pid);
  }
}

void Control_Task(MotorControl_t *control)
{
  uint32_t now;

  if ((control == NULL) || (control->motor == NULL) || !control->running)
  {
    return;
  }

  now = HAL_GetTick();
  if ((int32_t)(now - control->next_action_ms) < 0)
  {
    return;
  }

  if (control->action_phase == 0U)
  {
    (void)Motor_RequestFeedback(control->motor, MOTOR_FEEDBACK_MULTI_POSITION);
    control->action_phase = 1U;
    control->next_action_ms += POSITION_TO_CONTROL_INTERVAL_MS;
    return;
  }

  control->action_phase = 0U;
  control->next_action_ms += CONTROL_TO_POSITION_INTERVAL_MS;

  Control_UpdateAndSend(control, now, CONTROL_PERIOD_S);
}

void DualControl_Init(DualMotorControl_t *dual_control,
                      MotorControl_t *motor_1, MotorControl_t *motor_2)
{
  if (dual_control == NULL)
  {
    return;
  }

  dual_control->motor_1 = motor_1;
  dual_control->motor_2 = motor_2;
  dual_control->next_action_ms = HAL_GetTick();
  dual_control->action_phase = 0U;
  dual_control->running = false;
}

HAL_StatusTypeDef DualControl_Start(DualMotorControl_t *dual_control)
{
  HAL_StatusTypeDef status;

  if ((dual_control == NULL) || (dual_control->motor_1 == NULL) ||
      (dual_control->motor_2 == NULL))
  {
    return HAL_ERROR;
  }

  status = Control_Start(dual_control->motor_1);
  if (status != HAL_OK)
  {
    return status;
  }
  HAL_Delay(MOTOR_COMMAND_GAP_MS);

  status = Control_Start(dual_control->motor_2);
  if (status != HAL_OK)
  {
    Control_Stop(dual_control->motor_1);
    return status;
  }
  HAL_Delay(MOTOR_COMMAND_GAP_MS);

  dual_control->next_action_ms = HAL_GetTick() + MOTOR_BUS_SLOT_INTERVAL_MS;
  dual_control->action_phase = 0U;
  dual_control->running = true;
  return HAL_OK;
}

void DualControl_Stop(DualMotorControl_t *dual_control)
{
  if ((dual_control == NULL) || (dual_control->motor_1 == NULL) ||
      (dual_control->motor_2 == NULL))
  {
    return;
  }

  dual_control->running = false;
  Control_Stop(dual_control->motor_1);
  HAL_Delay(MOTOR_COMMAND_GAP_MS);
  Control_Stop(dual_control->motor_2);
}

void DualControl_Task(DualMotorControl_t *dual_control)
{
  uint32_t now;

  if ((dual_control == NULL) || !dual_control->running)
  {
    return;
  }

  now = HAL_GetTick();
  if ((int32_t)(now - dual_control->next_action_ms) < 0)
  {
    return;
  }

  switch (dual_control->action_phase)
  {
    case 0U:
      (void)Motor_RequestFeedback(dual_control->motor_1->motor,
                                  MOTOR_FEEDBACK_MULTI_POSITION);
      break;

    case 1U:
      Control_UpdateAndSend(dual_control->motor_1, now, DUAL_CONTROL_PERIOD_S);
      break;

    case 2U:
      (void)Motor_RequestFeedback(dual_control->motor_2->motor,
                                  MOTOR_FEEDBACK_MULTI_POSITION);
      break;

    default:
      Control_UpdateAndSend(dual_control->motor_2, now, DUAL_CONTROL_PERIOD_S);
      break;
  }

  dual_control->action_phase++;
  if (dual_control->action_phase >= 4U)
  {
    dual_control->action_phase = 0U;
  }
  /* Base the next slot on real time so a delayed loop cannot burst commands. */
  dual_control->next_action_ms = now + MOTOR_BUS_SLOT_INTERVAL_MS;
}
