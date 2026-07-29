#ifndef VISION_CONTROL_H
#define VISION_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "control.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  MotorControl_t *motor_1;
  MotorControl_t *motor_2;
  float filtered_x;
  float filtered_y;
  float error_x;
  float error_y;
  float motor_1_target_deg;
  float motor_2_target_deg;
  uint32_t processed_sequence;
  uint32_t next_update_ms;
  bool filter_initialized;
  bool target_fresh;
} VisionControl_t;

void VisionControl_Init(VisionControl_t *vision,
                        MotorControl_t *motor_1,
                        MotorControl_t *motor_2);
void VisionControl_Task(VisionControl_t *vision);

#ifdef __cplusplus
}
#endif

#endif /* VISION_CONTROL_H */
