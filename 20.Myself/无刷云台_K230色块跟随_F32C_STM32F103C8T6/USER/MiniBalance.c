/***********************************************
公司：轮趣科技(东莞)有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-10-13

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2022-10-13

All rights reserved
***********************************************/

#include "stm32f10x.h"

#include "sys.h"
#include "delay.h"
#include "usart.h"

#include "led.h"
#include "oled.h"
#include "timer.h"
#include "adc.h"
#include "key.h"
#include "control.h"
#include "motor.h"
#include <stdio.h>
#include <math.h>
int temp=0;
int tar_angle1=0,tar_angle2=900;

int main(void)
{
	//中断优先级分组
	NVIC_SetPriorityGrouping(NVIC_PriorityGroup_4);
	
	//禁用JATG,使用SWD
	JTAG_Set(JTAG_SWD_DISABLE);
	JTAG_Set(SWD_ENABLE);
	
	//滴答定时器初始化
	SysTick_Init(1000);
	
	//串口1初始化
	USART1_Init(115200);
	
	//LED
	LED_Init();
	
	//按键
	KEY_Init();
	
	//OLED
	OLED_Init();
	
	//ADC初始化
	Adc_Init();
	delay_ms(500);
	// 初始化串口3，波特率 115200
	USART3_Init(115200);
	delay_ms(200);

	BLDC_Disable(BLDC_ADDR_1);
	delay_ms(200);
	BLDC_SetAcc(BLDC_ADDR_1,75);
	delay_ms(200);
	BLDC_Enable(BLDC_ADDR_1);
	delay_ms(200);
	BLDC_SetMode(BLDC_ADDR_1,MODE_SPEED);
	delay_ms(200);
	BLDC_SetSpeed(BLDC_ADDR_1,0);
	delay_ms(200);
	BLDC_Disable(BLDC_ADDR_2);
	delay_ms(200);
	BLDC_SetAcc(BLDC_ADDR_2,75);
	delay_ms(200);
	BLDC_Enable(BLDC_ADDR_2);
	delay_ms(200);
	BLDC_SetMode(BLDC_ADDR_2,MODE_SPEED);
	delay_ms(200);
	BLDC_SetSpeed(BLDC_ADDR_2,0);
	delay_ms(200);
	//定时器初始化（100hz,10ms）
	TIM2_Int_Init(99,7199);
	
	USART2_Init(9600);
	while(1)
	{
//		// 查询电机1速度
		BLDC_ReqFeedback(BLDC_ADDR_1, FB_SPEED);
		delay_ms(2);
		BLDC_ReqFeedback(BLDC_ADDR_2, FB_SPEED);
		delay_ms(2);
//		printf("T1:%d\tC1:%d\tT2:%d\tC2:%d\r\n",tar_angle1,tar_angle2,);
		oled_show();
		//delay_ms(20);
	}
}


