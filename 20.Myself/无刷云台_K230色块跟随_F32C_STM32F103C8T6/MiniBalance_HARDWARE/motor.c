#include "motor.h"
#include <stdio.h>
#include <string.h>
#include "usart.h"

// 全局变量定义
volatile BLDC_MotorData_t BLDC_Motor1 = {0};
volatile BLDC_MotorData_t BLDC_Motor2 = {0};

// 接收状态机
static uint8_t rx_buf[10];
static uint8_t rx_index = 0;
static uint8_t rx_state = 0;    // 0:等待帧头, 1:接收地址, 2:接收功能码/类型, 3:接收数据, 4:接收校验, 5:接收帧尾

// 计算 BCC 校验 (异或)
uint8_t Calc_BCC(uint8_t *data, uint8_t len)
{
    uint8_t bcc = 0;
    for (int i = 0; i < len; i++) bcc ^= data[i];
    return bcc;
}

// 发送指令
void BLDC_SendCmd(uint8_t addr, uint8_t cmd, uint8_t *data, uint8_t len)
{
    uint8_t tx_buf[20];
    uint8_t idx = 0;
    
    tx_buf[idx++] = BLDC_HEADER;
    tx_buf[idx++] = addr;
    tx_buf[idx++] = cmd;
    
    if (len > 0 && data != NULL) {
        memcpy(&tx_buf[idx], data, len);
        idx += len;
    }
    
    tx_buf[idx++] = Calc_BCC(tx_buf, idx);
    tx_buf[idx++] = BLDC_TAIL;
    
    USART3_SendArray(tx_buf, idx);
}

void BLDC_Enable(uint8_t addr)  { BLDC_SendCmd(addr, CMD_ENABLE, NULL, 0); }
void BLDC_Disable(uint8_t addr) { BLDC_SendCmd(addr, CMD_DISABLE, NULL, 0); }

void BLDC_SetSpeed(uint8_t addr, int16_t rpm)
{
    uint8_t data[2] = {(rpm >> 8) & 0xFF, rpm & 0xFF};
    BLDC_SendCmd(addr, CMD_SPEED, data, 2);
}

void BLDC_SetMode(uint8_t addr, uint16_t mode)
{
    uint8_t data[2] = {(mode >> 8) & 0xFF, mode & 0xFF};
    BLDC_SendCmd(addr, CMD_MODE, data, 2);
}

void BLDC_SetMultiAngle(uint8_t addr, int32_t angle_x10)
{
    uint8_t data[4];
    data[0] = (angle_x10 >> 24) & 0xFF;
    data[1] = (angle_x10 >> 16) & 0xFF;
    data[2] = (angle_x10 >> 8) & 0xFF;
    data[3] = angle_x10 & 0xFF;
    BLDC_SendCmd(addr, CMD_MULTI_POS, data, 4);
}

void BLDC_SetSingleAngle(uint8_t addr, uint16_t angle_x10)
{
    uint8_t data[2];
    if (angle_x10 > 3599) angle_x10 = 3599;
    data[0] = (angle_x10 >> 8) & 0xFF;
    data[1] = angle_x10 & 0xFF;
    BLDC_SendCmd(addr, CMD_SINGLE_POS, data, 2);
}

void BLDC_ReqFeedback(uint8_t addr, uint8_t type)
{
    uint8_t data[1] = {type};
    BLDC_SendCmd(addr, CMD_FEEDBACK, data, 1);
}

/**
 * @brief 设置加速度
 * @param addr 电机地址
 * @param acc 加速度值 (单位：转/s2)，例如 100 表示 100 转/s2
 */
void BLDC_SetAcc(uint8_t addr, uint16_t acc)
{
    uint8_t data[2] = {(acc >> 8) & 0xFF, acc & 0xFF};
    BLDC_SendCmd(addr, CMD_ACC, data, 2);
}

/**
 * @brief 保存参数到闪存 (掉电保存)
 * @param addr 电机地址
 * @note 修改加速度、地址等参数后调用此函数
 */
void BLDC_SaveParams(uint8_t addr)
{
    BLDC_SendCmd(addr, CMD_SAVE, NULL, 0);
}

/**
 * @brief 多圈角度清零
 * @param addr 电机地址
 * @note 电机上电后转过的角度清零
 */
void BLDC_ClearMultiAngle(uint8_t addr)
{
    BLDC_SendCmd(addr, CMD_CLEAR_MULTI, NULL, 0);
}

/**
 * @brief 单圈绝对角度置零
 * @param addr 电机地址
 * @note 将当前位置设置为单圈绝对角度 0 度，可掉电保存
 */
void BLDC_SetSingleAngleZero(uint8_t addr)
{
    BLDC_SendCmd(addr, CMD_SET_ZERO, NULL, 0);
}

/**
 * @brief 恢复出厂设置
 * @param addr 电机地址
 */
void BLDC_FactoryReset(uint8_t addr)
{
    BLDC_SendCmd(addr, CMD_FACTORY_RST, NULL, 0);
}

/**
 * @brief 设置电机地址
 * @param addr 当前电机地址
 * @param new_addr 要设置的新地址
 * @note 发送成功后，后续通信需使用 new_addr
 */
void BLDC_SetAddress(uint8_t addr, uint8_t new_addr)
{
    uint8_t data[1] = {new_addr};
    BLDC_SendCmd(addr, CMD_SET_ADDR, data, 1);
}


/**
 * @brief 串口接收解析函数 - 在USART3_IRQHandler中调用
 * @param rx_byte 接收到的字节
 * 
 * 反馈帧格式: 7A 01 XX HH1 LL1 HH2 LL2 BCC 7B (共9字节)
 * XX: 反馈类型(00-04)
 */
void BLDC_ParseRxData(uint8_t rx_byte)
{
    switch(rx_state) {
        case 0: // 等待帧头
            if (rx_byte == BLDC_HEADER) {
                rx_buf[0] = rx_byte;
                rx_index = 1;
                rx_state = 1;
            }
            break;
            
        case 1: // 接收地址
            if (rx_byte == BLDC_ADDR_1 || rx_byte == BLDC_ADDR_2) {
                rx_buf[rx_index++] = rx_byte;
                rx_state = 2;
            } else {
                rx_state = 0; // 地址错误，重新同步
            }
            break;
            
        case 2: // 接收反馈类型
            if (rx_byte <= 0x04) {  // 有效反馈类型 00-04
                rx_buf[rx_index++] = rx_byte;
                rx_state = 3;
            } else {
                rx_state = 0;
            }
            break;
            
        case 3: // 接收数据(4字节: 高16位+低16位)
            rx_buf[rx_index++] = rx_byte;
            if (rx_index >= 7) {  // 已收到 帧头+地址+类型+4字节数据
                rx_state = 4;
            }
            break;
            
        case 4: // 接收校验
            rx_buf[rx_index++] = rx_byte;
            rx_state = 5;
            break;
            
        case 5: // 接收帧尾并校验
            if (rx_byte == BLDC_TAIL) {
                rx_buf[rx_index++] = rx_byte;
                
                // 校验BCC (帧头到数据末尾，共7字节)
                uint8_t calc_bcc = Calc_BCC(rx_buf, 7);
                if (calc_bcc == rx_buf[7]) {
					
                    // 校验通过，解析数据
                    uint8_t addr = rx_buf[1];
                    uint8_t type = rx_buf[2];
                    int32_t value = ((int32_t)rx_buf[3] << 24) | 
                                   ((int32_t)rx_buf[4] << 16) | 
                                   ((int32_t)rx_buf[5] << 8) | 
                                   rx_buf[6];
                    volatile BLDC_MotorData_t *motor = (addr == BLDC_ADDR_1) ? &BLDC_Motor1 : &BLDC_Motor2;
                    
                    switch(type) {
                        case FB_SPEED:        // 00 - 速度
                            motor->speed = (int16_t)(value & 0xFFFF);
                            break;
                        case FB_MULTI_ANGLE:  // 01 - 多圈角度
                            motor->multi_angle = value;
                            break;
                        case FB_SINGLE_ANGLE: // 02 - 单圈角度
                            motor->single_angle = (uint16_t)(value & 0xFFFF);
                            break;
                        case FB_ACC:          // 03 - 加速度
                            motor->acc = (int16_t)(value & 0xFFFF);
                            break;
                        case FB_VOLTAGE:      // 04 - 母线电压
                            motor->voltage = (uint16_t)(value & 0xFFFF);
                            break;
                    }
                    motor->data_ready = 1;  // 标记数据已更新
					
					
                }
            }
            rx_state = 0;  // 重置状态机
            rx_index = 0;
            break;
            
        default:
            rx_state = 0;
            rx_index = 0;
            break;
    }
}
