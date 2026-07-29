#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>


#define ANGLE_PWM_K 2.7777777777f

extern float Voltage;
//extern float PTZ_NowAngle,PTZ_TargetAngle;
//extern uint8_t PTZ_ControlStep,select_mode,target_reach_flag;
extern volatile float Target_Angle_X ;   // X 轴目标角度 (0.1°)
extern volatile float Target_Angle_Y ;   // Y 轴目标角度 (0.1°)
extern float real_arm_angle;
extern float real_base_angle;

extern float step_base;
extern float step_arm;
void oled_show(void);
#endif
