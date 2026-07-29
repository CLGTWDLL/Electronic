#include "vision_control.h"

#include "app_config.h"
#include "k230_link.h"

#include <stddef.h>

static float VisionControl_Clamp(float value, float minimum, float maximum)
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

static float VisionControl_ApplyDeadZone(float error, float dead_zone)
{
  if ((error > -dead_zone) && (error < dead_zone))
  {
    return 0.0f;
  }
  return error;
}

static void VisionControl_SetNeutral(VisionControl_t *vision)
{
  vision->motor_1_target_deg = APP_MOTOR_1_TARGET_POSITION_DEG;
  vision->motor_2_target_deg = APP_MOTOR_2_TARGET_POSITION_DEG;
  Control_SetTargetPosition(vision->motor_1, vision->motor_1_target_deg);
  Control_SetTargetPosition(vision->motor_2, vision->motor_2_target_deg);
}

void VisionControl_Init(VisionControl_t *vision,
                        MotorControl_t *motor_1,
                        MotorControl_t *motor_2)
{
  if (vision == NULL)
  {
    return;
  }

  vision->motor_1 = motor_1;
  vision->motor_2 = motor_2;
  vision->filtered_x = APP_BALL_TARGET_X_PX;
  vision->filtered_y = APP_BALL_TARGET_Y_PX;
  vision->error_x = 0.0f;
  vision->error_y = 0.0f;
  vision->processed_sequence = 0U;
  vision->next_update_ms = HAL_GetTick();
  vision->filter_initialized = false;
  vision->target_fresh = false;
  VisionControl_SetNeutral(vision);
}

void VisionControl_Task(VisionControl_t *vision)
{
  K230Target_t target;
  uint32_t now;
  float motor_1_offset;
  float motor_2_offset;

  if ((vision == NULL) || (vision->motor_1 == NULL) ||
      (vision->motor_2 == NULL))
  {
    return;
  }

  now = HAL_GetTick();
  if ((int32_t)(now - vision->next_update_ms) < 0)
  {
    return;
  }
  vision->next_update_ms = now + APP_VISION_TASK_INTERVAL_MS;

  if (!K230Link_GetTarget(&target, now, K230_TARGET_TIMEOUT_MS))
  {
    vision->target_fresh = false;
    vision->filter_initialized = false;
    vision->error_x = 0.0f;
    vision->error_y = 0.0f;
    VisionControl_SetNeutral(vision);
    return;
  }

  vision->target_fresh = true;
  if (target.sequence != vision->processed_sequence)
  {
    vision->processed_sequence = target.sequence;
    if (!vision->filter_initialized)
    {
      vision->filtered_x = (float)target.center_x;
      vision->filtered_y = (float)target.center_y;
      vision->filter_initialized = true;
    }
    else
    {
      vision->filtered_x += APP_VISION_FILTER_ALPHA *
                            ((float)target.center_x - vision->filtered_x);
      vision->filtered_y += APP_VISION_FILTER_ALPHA *
                            ((float)target.center_y - vision->filtered_y);
    }
  }

  vision->error_x = VisionControl_ApplyDeadZone(
      vision->filtered_x - APP_BALL_TARGET_X_PX,
      APP_VISION_DEAD_ZONE_X_PX);
  vision->error_y = VisionControl_ApplyDeadZone(
      vision->filtered_y - APP_BALL_TARGET_Y_PX,
      APP_VISION_DEAD_ZONE_Y_PX);

  if (APP_VISION_CONTROL_ENABLE == 0U)
  {
    VisionControl_SetNeutral(vision);
    return;
  }

  motor_1_offset = APP_VISION_M1_X_DEG_PER_PX * vision->error_x +
                   APP_VISION_M1_Y_DEG_PER_PX * vision->error_y;
  motor_2_offset = APP_VISION_M2_X_DEG_PER_PX * vision->error_x +
                   APP_VISION_M2_Y_DEG_PER_PX * vision->error_y;

  motor_1_offset = VisionControl_Clamp(motor_1_offset,
                                       -APP_VISION_MAX_OFFSET_DEG,
                                       APP_VISION_MAX_OFFSET_DEG);
  motor_2_offset = VisionControl_Clamp(motor_2_offset,
                                       -APP_VISION_MAX_OFFSET_DEG,
                                       APP_VISION_MAX_OFFSET_DEG);

  vision->motor_1_target_deg =
      APP_MOTOR_1_TARGET_POSITION_DEG + motor_1_offset;
  vision->motor_2_target_deg =
      APP_MOTOR_2_TARGET_POSITION_DEG + motor_2_offset;
  Control_SetTargetPosition(vision->motor_1, vision->motor_1_target_deg);
  Control_SetTargetPosition(vision->motor_2, vision->motor_2_target_deg);
}
