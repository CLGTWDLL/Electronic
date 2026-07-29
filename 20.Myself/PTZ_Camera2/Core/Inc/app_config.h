#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/*
 * One-shot single-turn absolute-angle zero calibration.
 *
 * Set APP_SINGLE_TURN_ZERO_CALIBRATION_ENABLE to 1, place both axes at their
 * desired mechanical zero positions, then program/reset the STM32 once. In
 * this mode the application disables both motors, writes zero to motor 1 and
 * motor 2, and deliberately does not start normal closed-loop control.
 *
 * The command is stored by the motor. Set this option back to 0 immediately
 * after calibration so later resets cannot overwrite the saved zero points.
 */
#define APP_SINGLE_TURN_ZERO_CALIBRATION_ENABLE  0U
#define APP_ZERO_MOTOR_1_ENABLE                  1U
#define APP_ZERO_MOTOR_2_ENABLE                  1U
#define APP_ZERO_COMMAND_GAP_MS                  20U

/* ========================================================================== */
/*                              用户可调参数区                                */
/* ========================================================================== */

/* F32C 电机基本参数。 */
#define APP_MOTOR_1_ID                       1U       /* 一号电机通信地址，必须与电机实际地址一致 */
#define APP_MOTOR_2_ID                       2U       /* 二号电机通信地址，必须与电机实际地址一致 */
#define APP_MOTOR_1_TARGET_POSITION_DEG      180.0f    /* 一号电机上电后的目标位置，单位：度 */
#define APP_MOTOR_2_TARGET_POSITION_DEG      0.0f    /* 二号电机上电后的目标位置，单位：度 */
#define MOTOR_SPEED_LIMIT_RPM                1000.0f  /* 最终速度指令限幅，单位：RPM */
#define MOTOR_MAX_DEVICES                    2U       /* 同一串口总线上最多注册的电机数量 */

/* 位置环 PID：位置误差（度）转换为目标速度（RPM）。 */
#define POSITION_PID_KP                      2.0f     /* 位置环比例系数 */
#define POSITION_PID_KI                      0.0f     /* 位置环积分系数 */
#define POSITION_PID_KD                      0.02f    /* 位置环微分系数 */
#define POSITION_SPEED_LIMIT_RPM             100.0f   /* 位置环输出速度限幅，单位：RPM */

/*
 * Slew the internal position setpoint from the measured startup position
 * toward the requested target. This prevents a full 20/180-degree step from
 * being applied to the PID immediately after power-up.
 */
#define POSITION_TARGET_SLEW_DEG_PER_S       60.0f

/* 速度闭环由 F32C 内部完成，STM32 位置环输出直接作为目标速度。 */

/* 最短路径角度参数。 */
#define POSITION_FULL_TURN_DEG               360.0f   /* 机械结构一整圈的角度，单位：度 */
#define POSITION_HALF_TURN_DEG               180.0f   /* 最短路径的正反转切换边界，单位：度 */

/* 单电机控制和串口通信时序。两个阶段之和为 10 ms，即控制频率为 100 Hz。 */
#define POSITION_TO_CONTROL_INTERVAL_MS      5U       /* 位置反馈请求到控制输出的间隔，单位：ms */
#define CONTROL_TO_POSITION_INTERVAL_MS      5U       /* 控制输出到下次位置反馈请求的间隔，单位：ms */
#define CONTROL_PERIOD_S                     0.01f    /* PID 计算周期，单位：s */
#define MOTOR_COMMAND_GAP_MS                 2U       /* 启动和停止指令之间的间隔，单位：ms */
#define MOTOR_BUS_SLOT_INTERVAL_MS           2U       /* 双电机总线上相邻两条指令的间隔，单位：ms */
#define DUAL_CONTROL_PERIOD_S                (MOTOR_BUS_SLOT_INTERVAL_MS * 4.0f * 0.001f) /* 双电机各自的 PID 周期，默认 8 ms */

/* 通信保护参数。 */
#define MOTOR_FEEDBACK_TIMEOUT_MS            100U     /* 反馈超时时间，超时后发送零速度，单位：ms */
#define MOTOR_TX_TIMEOUT_MS                  5U       /* 串口发送最长等待时间，单位：ms */
#define MOTOR_UART_IRQ_PREEMPT_PRIORITY      5U       /* USART1 中断抢占优先级 */
#define MOTOR_UART_IRQ_SUBPRIORITY           0U       /* USART1 中断响应优先级 */

/* K230 object-detection link: USART2, 115200 8N1. */
#define K230_FUNCTION_ID                     14U
#define K230_EXPECTED_CLASS_NAME             "Steel_ball"
#define K230_IMAGE_WIDTH_PX                  1280U
#define K230_IMAGE_HEIGHT_PX                 720U
#define K230_FRAME_MAX_LENGTH                96U
#define K230_TARGET_TIMEOUT_MS               300U
#define K230_BOX_MIN_WIDTH_PX                20U
#define K230_BOX_MAX_WIDTH_PX                300U
#define K230_BOX_MIN_HEIGHT_PX               20U
#define K230_BOX_MAX_HEIGHT_PX               300U
#define K230_UART_IRQ_PREEMPT_PRIORITY        6U
#define K230_UART_IRQ_SUBPRIORITY             0U

/* Measured center of the physical platform in the 1280 x 720 camera image. */
#define APP_BALL_TARGET_X_PX                 642.0f
#define APP_BALL_TARGET_Y_PX                 369.0f
#define APP_VISION_TASK_INTERVAL_MS          20U
#define APP_VISION_FILTER_ALPHA              0.35f
#define APP_VISION_DEAD_ZONE_X_PX            10.0f
#define APP_VISION_DEAD_ZONE_Y_PX            6.0f

/*
 * Motor directions verified on the real mechanism:
 *   motor 1 positive -> ball moves left in the image
 *   motor 2 positive -> ball moves down in the image
 */
#define APP_VISION_CONTROL_ENABLE            1U
#define APP_VISION_MAX_OFFSET_DEG            10.0f

/*
 * Image-error to motor-angle matrix:
 *   motor_1_offset = M1_X * error_x + M1_Y * error_y
 *   motor_2_offset = M2_X * error_x + M2_Y * error_y
 * Negate or swap coefficients after checking the real mechanical directions.
 */
#define APP_VISION_M1_X_DEG_PER_PX           0.02f
#define APP_VISION_M1_Y_DEG_PER_PX           0.0f
#define APP_VISION_M2_X_DEG_PER_PX           0.0f
#define APP_VISION_M2_Y_DEG_PER_PX          -0.02f

#endif /* APP_CONFIG_H */
