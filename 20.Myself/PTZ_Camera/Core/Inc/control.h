#ifndef CONTROL_H
#define CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor.h"

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral;
  float previous_error;
  float output_min;
  float output_max;
} PID_t;

typedef struct
{
  Motor_t *motor;
  PID_t position_pid;
  float target_position_deg;
  float trajectory_position_deg;
  float target_speed_rpm;
  float output_speed_rpm;
  uint32_t next_action_ms;
  uint8_t action_phase;
  bool trajectory_initialized;
  bool running;
} MotorControl_t;

typedef struct
{
  MotorControl_t *motor_1;
  MotorControl_t *motor_2;
  uint32_t next_action_ms;
  uint8_t action_phase;
  bool running;
} DualMotorControl_t;

void Control_Init(MotorControl_t *control, Motor_t *motor);
HAL_StatusTypeDef Control_Start(MotorControl_t *control);
void Control_Stop(MotorControl_t *control);
void Control_SetTargetPosition(MotorControl_t *control, float position_deg);
void Control_SetPositionPID(MotorControl_t *control, float kp, float ki, float kd);
void Control_Task(MotorControl_t *control);

void DualControl_Init(DualMotorControl_t *dual_control,
                      MotorControl_t *motor_1, MotorControl_t *motor_2);
HAL_StatusTypeDef DualControl_Start(DualMotorControl_t *dual_control);
void DualControl_Stop(DualMotorControl_t *dual_control);
void DualControl_Task(DualMotorControl_t *dual_control);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */
