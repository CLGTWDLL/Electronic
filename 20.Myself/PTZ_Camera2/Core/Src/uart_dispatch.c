#include "k230_link.h"
#include "motor.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
  Motor_UartRxCpltCallback(uart);
  K230Link_UartRxCpltCallback(uart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  Motor_UartErrorCallback(uart);
  K230Link_UartErrorCallback(uart);
}
