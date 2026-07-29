#ifndef __MOTOR_H
#define __MOTOR_H

#include "usart.h"
#include "stm32f10x.h"
#include <stdint.h>
#include <stdbool.h>

// --- 协议定义 ---
#define BLDC_HEADER     0x7A
#define BLDC_TAIL       0x7B

// 默认地址
#define BLDC_ADDR_1     0x01
#define BLDC_ADDR_2     0x02

// 功能码
#define CMD_ENABLE      0x06
#define CMD_DISABLE     0x05
#define CMD_MODE        0x00
#define CMD_SPEED       0x01
#define CMD_MULTI_POS   0x02
#define CMD_SINGLE_POS  0x03
#define CMD_FEEDBACK    0x0E
#define CMD_ACC         0x07        // 设置加速度 (表 8)
#define CMD_SAVE        0x08        // 保存参数到闪存 (表 9)
#define CMD_CLEAR_MULTI 0x09        // 多圈角度清零 (表 10)
#define CMD_SET_ZERO    0x0A        // 单圈绝对角度置零 (表 11)
#define CMD_FACTORY_RST 0x0B        // 恢复出厂设置 (表 12)
#define CMD_SET_ADDR    0x0D        // 设置电机地址 (表 13)
// 模式
#define MODE_SPEED          0x0000
#define MODE_MULTI_POS      0x0001
#define MODE_SINGLE_POS     0x0002

// 反馈类型
#define FB_SPEED        0x00
#define FB_MULTI_ANGLE  0x01
#define FB_SINGLE_ANGLE 0x02
#define FB_ACC          0x03
#define FB_VOLTAGE      0x04

// --- 电机数据结构体 ---
typedef struct {
    int16_t  speed;         // 转速 (RPM)
    int32_t  multi_angle;   // 多圈角度 (度*10)
    uint16_t single_angle;  // 单圈角度 (度*10)
    int16_t  acc;           // 加速度 (转/s2)
    uint16_t voltage;       // 母线电压 (0.01V)
    uint8_t  data_ready;    // 数据更新标志位
} BLDC_MotorData_t;

// --- 全局变量 ---
extern volatile BLDC_MotorData_t BLDC_Motor1;  // 地址0x01
extern volatile BLDC_MotorData_t BLDC_Motor2;  // 地址0x02
uint8_t Calc_BCC(uint8_t *data, uint8_t len);
// --- API ---
void BLDC_SendCmd(uint8_t addr, uint8_t cmd, uint8_t *data, uint8_t len);
void BLDC_Enable(uint8_t addr);
void BLDC_Disable(uint8_t addr);
void BLDC_SetSpeed(uint8_t addr, int16_t rpm);
void BLDC_SetMode(uint8_t addr, uint16_t mode);
void BLDC_ReqFeedback(uint8_t addr, uint8_t type);
void BLDC_SetMultiAngle(uint8_t addr, int32_t angle_x10);
void BLDC_SetSingleAngle(uint8_t addr, uint16_t angle_x10);


// --- API (新增) ---
void BLDC_SetAcc(uint8_t addr, uint16_t acc);             // 设置加速度 (单位：转/s2)
void BLDC_SaveParams(uint8_t addr);                       // 保存参数到闪存
void BLDC_ClearMultiAngle(uint8_t addr);                  // 多圈角度清零
void BLDC_SetSingleAngleZero(uint8_t addr);               // 当前位置设为单圈零点
void BLDC_FactoryReset(uint8_t addr);                     // 恢复出厂设置
void BLDC_SetAddress(uint8_t addr, uint8_t new_addr);     // 修改电机地址

// 串口中断解析函数（在USART3_IRQHandler中调用）
void BLDC_ParseRxData(uint8_t rx_byte);

// 便捷宏定义
//#define BLDC1_Enable()          BLDC_Enable(BLDC_ADDR_1)
//#define BLDC1_Disable()         BLDC_Disable(BLDC_ADDR_1)
//#define BLDC1_SetSpeed(rpm)     BLDC_SetSpeed(BLDC_ADDR_1, rpm)
//#define BLDC1_SetMode(mode)     BLDC_SetMode(BLDC_ADDR_1, mode)
//#define BLDC1_SetMultiAngle(a)  BLDC_SetMultiAngle(BLDC_ADDR_1, a)
//#define BLDC1_SetSingleAngle(a) BLDC_SetSingleAngle(BLDC_ADDR_1, a)
//#define BLDC1_ReqFeedback(t)    BLDC_ReqFeedback(BLDC_ADDR_1, t)

//#define BLDC2_Enable()          BLDC_Enable(BLDC_ADDR_2)
//#define BLDC2_Disable()         BLDC_Disable(BLDC_ADDR_2)
//#define BLDC2_SetSpeed(rpm)     BLDC_SetSpeed(BLDC_ADDR_2, rpm)
//#define BLDC2_SetMode(mode)     BLDC_SetMode(BLDC_ADDR_2, mode)
//#define BLDC2_SetMultiAngle(a)  BLDC_SetMultiAngle(BLDC_ADDR_2, a)
//#define BLDC2_SetSingleAngle(a) BLDC_SetSingleAngle(BLDC_ADDR_2, a)
//#define BLDC2_ReqFeedback(t)    BLDC_ReqFeedback(BLDC_ADDR_2, t)

#endif

