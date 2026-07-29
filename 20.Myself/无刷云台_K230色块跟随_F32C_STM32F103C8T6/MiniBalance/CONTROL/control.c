#include <math.h>
#include "stdio.h"
#include "control.h"
#include "delay.h"
#include "oled.h"
#include "key.h"
#include "usart.h"
#include "LED.h"
#include "motor.h"
#include "adc.h"
#include "stm32f10x.h"

/********************************************
*                                           *
*    可调参数                               *
*    调整参数时，请同时关注对应函数内的         *
*    公式和限幅逻辑                         *
*                                           *
********************************************/

/* ============================================================
 * 1. 视觉伺服控制参数（Visual_Servo_Process 函数）
 *    调整逻辑：先调KP让跟踪有力度，再调KD抑制过冲和振荡
 * ============================================================ */

/* -- 图像参数 -- */
#define VISION_CENTER_X     166       // 图像中心X坐标（像素），由摄像头分辨率决定
#define VISION_CENTER_Y     128       // 图像中心Y坐标（像素），由摄像头分辨率决定
#define VISION_DEADZONE     15        // 死区像素值，误差小于此值时不输出（抑制中心点附近微小抖动）

/* -- 方向参数（±1，反转可改变电机旋转方向） -- */
#define VISION_DIR_X        -1        // X轴电机方向（1=正向 / -1=反向）
#define VISION_DIR_Y        1         // Y轴电机方向（1=正向 / -1=反向）

/* -- PD控制系数 -- */
#define KP_VISION_X         0.25f     // X轴P系数（比例项）：误差→速度的增益，越大跟踪越快但容易过冲
#define KP_VISION_Y         0.4f      // Y轴P系数（比例项）：同理，Y轴惯性较小可用稍大值
#define KD_VISION_X         0.20f     // X轴D系数（微分/阻尼项）：反馈速度变化率，核心抑制过冲和振荡
#define KD_VISION_Y         0.25f     // Y轴D系数（微分/阻尼项）：同理

/* -- 限幅与滤波 -- */
#define MAX_SPEED_RPM       30        // 最大输出转速(RPM)，防止电机过速
#define FILTER_ALPHA        0.7f      // 一阶低通滤波系数（0~1），越大响应越快但滤波效果越弱


/* ============================================================
 * 2. 基座角度解算参数（base_x / base_y 函数）
 *    通过三角函数将目标位置反解为基座/云台角度
 *    公式：angle = 中心角度 - atan(位置/距离) * 弧度转角度
 * ============================================================ */

#define BASE_X_CENTER       135.0f    // X轴中心基准角度（度），即目标在正前方时的基座角度
#define BASE_Y_CENTER       90.0f     // Y轴中心基准角度（度）
#define RAD_TO_DEG          57.3f     // 弧度转角度系数（≈180/π）


/* ============================================================
 * 3. 系统与采样参数
 * ============================================================ */

#define VOLTAGE_SAMPLE_CNT  100       // 电压采样次数（滑动平均窗口大小）
#define VOLTAGE_DIVIDER      10000.0f // 电压分压换算系数
#define MOTOR_CMD_DELAY_US  200       // 两路电机指令间隔延时(us)，避免总线冲突
#define MOVE_STEPS          100       // 步进移动总步数（供外部步进逻辑使用）


/********************************************
*                                           *
*               内部变量                     *
*                                           *
********************************************/
// 电池电压采集辅助变量（用于滑动平均）
static int Voltage_All = 0;
static uint8_t Voltage_Count = 0;


/********************************************
*                                           *
*               全局变量                     *
*                                           *
********************************************/
float Voltage;// 电池电压（滑动平均后的结果）


/*
 * 函数：base_x
 * 功能：根据X轴目标位置和传感器距离，反解基座X轴期望角度
 * 参数：ts     - 传感器到目标的直线距离
 *       x_point - 目标在X轴上的偏移量
 * 公式：angle = BASE_X_CENTER - atan(x_point / ts) * RAD_TO_DEG
 */
float base_x(float ts, float x_point)
{
    float angle = 0;
    angle = BASE_X_CENTER - (atan(x_point / ts)) * RAD_TO_DEG;
    return angle;
}

/*
 * 函数：base_y
 * 功能：根据Y轴目标位置和传感器距离，反解基座Y轴期望角度
 * 参数：ts     - 传感器到目标的直线距离
 *       y_point - 目标在Y轴上的偏移量
 * 公式：angle = BASE_Y_CENTER - atan(y_point / ts) * RAD_TO_DEG
 */
float base_y(float ts, float y_point)
{
    float angle = 0;
    angle = BASE_Y_CENTER - (atan(y_point / ts)) * RAD_TO_DEG;
    return angle;
}

float real_arm_angle;
float real_base_angle;

float step_base;
float step_arm;

const float ts_dis = 0.35f;           // 传感器到基座转轴的距离(m)，供外部引用
extern int tar_angle1, tar_angle2;    // 外部目标角度变量


/*
 * 视觉伺服主处理函数（在TIM2中断中调用）
 *
 * 控制策略：PD控制（比例 + 微分）
 *   - P项：将像素误差转换为跟踪速度（误差越大速度越快）
 *   - D项：感知误差变化趋势，目标靠近时自动刹车（负微分刹车）
 *
 * 处理流程：
 *   1. 计算像素误差（中心坐标 - 目标坐标）
 *   2. 计算误差变化率（微分项）
 *   3. PD控制律计算原始速度
 *   4. 死区过滤（小误差不输出，防止微震）
 *   5. 限幅保护（防止电机过速）
 *   6. 一阶低通滤波（平滑输出，抑制跳帧噪声）
 *   7. 通过BLDC总线下发速度指令（两路间隔延时避免冲突）
 */
void Visual_Servo_Process(void)
{
    if (g_data_ready == 0) return;   // 无新数据则跳过本次处理
    g_data_ready = 0;

    // 1. 计算当前像素误差（中心 - 目标，正值表示目标偏左/偏上）
    float err_x = (float)VISION_CENTER_X - (float)g_follow_x;
    float err_y = (float)VISION_CENTER_Y - (float)g_follow_y;

    // 2. 计算误差变化率（微分项），用于感知目标移动方向与速度
    static float prev_err_x = 0.0f, prev_err_y = 0.0f;
    float diff_err_x = err_x - prev_err_x;
    float diff_err_y = err_y - prev_err_y;
    prev_err_x = err_x;
    prev_err_y = err_y;

    float raw_speed_x = 0.0f, raw_speed_y = 0.0f;

    // 3. PD控制律：speed = P*err + D*diff
    // 关键特性：目标靠近中心时，err减小且diff为负值，
    // D项输出负值产生反向刹车力矩，自然减速防过冲
    raw_speed_x = KP_VISION_X * err_x + KD_VISION_X * diff_err_x;
    raw_speed_y = KP_VISION_Y * err_y + KD_VISION_Y * diff_err_y;

    // 4. 死区处理：像素误差在死区内时清零速度输出
    // 防止目标已接近中心时因传感器噪声产生微小抖动
    if (fabsf(err_x) <= VISION_DEADZONE) raw_speed_x = 0.0f;
    if (fabsf(err_y) <= VISION_DEADZONE) raw_speed_y = 0.0f;

    // 5. 限幅保护：将速度钳制在 MAX_SPEED_RPM 以内
    if (raw_speed_x >  MAX_SPEED_RPM) raw_speed_x =  MAX_SPEED_RPM;
    if (raw_speed_x < -MAX_SPEED_RPM) raw_speed_x = -MAX_SPEED_RPM;
    if (raw_speed_y >  MAX_SPEED_RPM) raw_speed_y =  MAX_SPEED_RPM;
    if (raw_speed_y < -MAX_SPEED_RPM) raw_speed_y = -MAX_SPEED_RPM;

    // 6. 一阶低通滤波：filter = old*(1-alpha) + new*alpha
    // 平滑速度输出曲线，抑制视觉跳帧和D项高频噪声
    static float filter_speed_x = 0.0f;
    static float filter_speed_y = 0.0f;
    filter_speed_x = filter_speed_x * (1.0f - FILTER_ALPHA) + raw_speed_x * FILTER_ALPHA;
    filter_speed_y = filter_speed_y * (1.0f - FILTER_ALPHA) + raw_speed_y * FILTER_ALPHA;

    // 7. 下发速度指令：方向修正 + 两路分时发送避免BLDC总线冲突
    BLDC_SetSpeed(BLDC_ADDR_1, (int16_t)(filter_speed_x * VISION_DIR_X));
    delay_us(MOTOR_CMD_DELAY_US);
    BLDC_SetSpeed(BLDC_ADDR_2, (int16_t)(filter_speed_y * VISION_DIR_Y));
}


/*
 * 定时器2更新中断服务函数
 * 调用频率：由TIM2定时周期决定
 * 每进入一次：执行视觉伺服 + 采集电池电压
 * 每 VOLTAGE_SAMPLE_CNT 次更新一次电压值并翻转LED
 */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        Visual_Servo_Process();                          // 视觉伺服主循环
        Voltage_All += Get_battery_volt();               // 累加ADC电压采样值
        if (++Voltage_Count == VOLTAGE_SAMPLE_CNT) {     // 滑动平均：每N次求一次均值
            Voltage = (float)Voltage_All / VOLTAGE_DIVIDER;
            Voltage_All = 0;
            Voltage_Count = 0;
            LED = !LED;                                  // 翻转LED指示系统运行
        }
    }
}






/*========================================================================
 * 函数名：oled_show
 * 功能：OLED显示云台运行状态
 * 布局：
 *   左列 — 电机1转速(M1) / 电机2转速(M2) 带正负方向
 *   右列 — 目标坐标(X/Y)、数据就绪标志(DAT)、固件版本
 *========================================================================*/
void oled_show(void)
{
    OLED_Refresh_Gram();

    /* ================= 左列：电机转速 ================= */
    // 电机1转速（带方向符号）
    OLED_ShowString(0,  0, "1:");
    if (BLDC_Motor1.speed < 0) OLED_ShowString(16,  0, "-");
    else OLED_ShowString(16,  0, "+");
    OLED_ShowNumber(24,  0, (int)abs(BLDC_Motor1.speed), 4, 12);

    // 电机2转速（带方向符号）
    OLED_ShowString(0, 10, "2:");
    if (BLDC_Motor2.speed < 0) OLED_ShowString(16, 10, "-");
    else OLED_ShowString(16, 10, "+");
    OLED_ShowNumber(24, 10, (int)abs(BLDC_Motor2.speed), 4, 12);

    /* ================= 右列：视觉跟踪坐标 ================= */
    // 目标X坐标（像素，0~320）
    OLED_ShowString(80,  0, "X:");
    OLED_ShowNumber(96,  0, (int)g_follow_x, 4, 12);

    // 目标Y坐标（像素，0~240）
    OLED_ShowString(80, 10, "Y:");
    OLED_ShowNumber(96, 10, (int)g_follow_y, 4, 12);

    /* ================= 右下：通信状态 ================= */
    // DAT:OK = 收到K210数据，DAT:-- = 等待数据
    OLED_ShowString(80, 20, "DAT:");
    if (g_data_ready) OLED_ShowString(104, 20, "OK");
    else OLED_ShowString(104, 20, "--");
    OLED_ShowString(80, 30, "75acc");    
}



