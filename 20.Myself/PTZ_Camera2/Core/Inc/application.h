#ifndef APPLICATION_H
#define APPLICATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

HAL_StatusTypeDef Application_Init(UART_HandleTypeDef *motor_uart,
                                   UART_HandleTypeDef *camera_uart);
void Application_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_H */
