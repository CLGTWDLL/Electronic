#include <stdio.h>
#include <string.h>

#include "usart.h"
#include "stm32f10x.h"

#include "control.h"

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif


//软复位进BootLoader区域
//static void _System_Reset_(u8 uart_recv)
//{
//	static u8 res_buf[5];
//	static u8 res_count=0;
//	
//	res_buf[res_count]=uart_recv;
//	
//	if( uart_recv=='r'||res_count>0 )
//		res_count++;
//	else
//		res_count = 0;
//	
//	if(res_count==5)
//	{
//		res_count = 0;
//		//接受到上位机请求的复位字符“reset”，执行软件复位
//		if( res_buf[0]=='r'&&res_buf[1]=='e'&&res_buf[2]=='s'&&res_buf[3]=='e'&&res_buf[4]=='t' )
//		{
//			NVIC_SystemReset();//进行软件复位，复位后执行 BootLoader 程序
//		}
//	}
//}

/* ================= 局部静态变量 (中断内部使用) ================= */
// 协议总长度：1(Head) + 8(4*uint16) + 1(BCC) + 1(End) = 11 字节
#define UART_FRAME_LEN      11
#define UART_HEAD           0xCC
#define UART_END            0xDD

static uint8_t  rx_buffer[UART_FRAME_LEN];
static uint8_t  rx_cnt = 0;
static uint8_t  rx_state = 0; // 0: 等待帧头，1: 接收数据中
volatile uint16_t g_follow_x = 0;      // 接收到的目标 X 坐标
volatile uint16_t g_follow_y = 0;      // 接收到的目标 Y 坐标
volatile uint8_t  g_data_ready = 0;    // 数据接收完成标志 (1 表示有新数据)

void USART1_Init(uint32_t baud)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 1. 使能时钟（默认引脚不需要 AFIO）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    // 2. 配置 GPIO（无需重映射）
    // PA9 - USART1_TX (复用推挽输出)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA10 - USART1_RX (浮空输入)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 配置 USART1
    USART_InitStructure.USART_BaudRate = baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    // 4. 配置中断
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  // 使能接收中断
    USART_Cmd(USART1, ENABLE);                      // 使能串口
}


/**************************************************************************
函数功能：串口1中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
//extern void update_PTZTarget(ReportDataRecv_t* p);

void USART1_IRQHandler(void)
{

	
//	uint8_t recv = 0;
	
    if(USART_GetITStatus(USART1, USART_IT_RXNE)) //接收到数据
    {
//		recv = USART_ReceiveData(USART1);
    }
}


void USART3_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure; // 用于配置中断优先级

    // 使能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    // PB10 = TX (复用推挽输出)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // PB11 = RX (浮空输入或上拉输入)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
    // 建议：如果外部没有上拉电阻，改为 GPIO_Mode_IPU 更稳定
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 配置串口参数
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    // 2. 修改这里：开启发送和接收模式
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; 
    USART_Init(USART3, &USART_InitStructure);

    // 3. 配置 NVIC 中断 (关键步骤)
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;             //  USART3 中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     //  抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;            //  子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               //  使能中断通道
    NVIC_Init(&NVIC_InitStructure);

    // 4. 使能串口接收中断
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART3, ENABLE);
    USART_ClearFlag(USART3, USART_FLAG_TC);
}
// 发送一个字节
void USART3_SendByte(uint8_t data)
{
    USART_SendData(USART3, data);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

// 发送数组
void USART3_SendArray(uint8_t *data, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        USART3_SendByte(data[i]);
    }
}
extern int temp;
uint8_t data=0;
#include "motor.h"
void USART3_IRQHandler(void)
{
    // 接收中断
     if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
		data=USART_ReceiveData(USART3);
		BLDC_ParseRxData(data);
	}
}



volatile uint8_t USART2_RxData;      // 接收数据缓存
volatile uint8_t USART2_RxFlag = 0;  // 接收完成标志

void USART2_Init(uint32_t baud)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 1. 使能时钟（USART2 在 APB1，GPIOA 在 APB2）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // 2. 配置 GPIO（仅配置 RX 引脚 PA3）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  // 浮空输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 配置 USART2（仅开启接收模式）
    USART_InitStructure.USART_BaudRate = baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx;  // 只开启接收
    USART_Init(USART2, &USART_InitStructure);

    // 4. 配置中断
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);  // 使能接收中断
    USART_Cmd(USART2, ENABLE);                      // 使能串口
}

/* =================  ================= */
void USART2_IRQHandler(void)
{
    uint8_t res;

    // ==============================================
    // 【1】硬件溢出错误清除
    // 不清这个，串口直接死！
    // ==============================================
    if(USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)
    {
        // 清除溢出错误的唯一正确方法
        res = USART_ReceiveData(USART2);
        (void)res;
        USART_ClearFlag(USART2, USART_FLAG_ORE);
        return;
    }

    // ==============================================
    // 【2】正常接收
    // ==============================================
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        res = USART_ReceiveData(USART2);

        
        switch (rx_state)
        {
            case 0:
                if (res == UART_HEAD)
                {
                    rx_buffer[0] = res;
                    rx_cnt = 1;
                    rx_state = 1;
                }
                break;

            case 1:
                rx_buffer[rx_cnt++] = res;

                if (rx_cnt >= UART_FRAME_LEN)
                {
                    if (rx_buffer[10] == UART_END)
                    {
                        uint8_t bcc_recv = rx_buffer[9];
                        uint8_t bcc_calc = Calc_BCC(rx_buffer, 9);

                        if (bcc_calc == bcc_recv)
                        {
                            g_follow_x = (uint16_t)rx_buffer[5] | ((uint16_t)rx_buffer[6] << 8);
                            g_follow_y = (uint16_t)rx_buffer[7] | ((uint16_t)rx_buffer[8] << 8);
                            g_data_ready = 1;
                        }
                    }
                    // 强制重置
                    rx_state = 0;
                    rx_cnt = 0;
                }
                break;

            default:
                rx_state = 0;
                rx_cnt = 0;
                break;
        }
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}
