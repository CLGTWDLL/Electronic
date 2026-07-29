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
#ifndef __DELAY_H
#define __DELAY_H 			   
#include "stm32f10x.h"

//SysTick时钟
#define SysTickFreq (SystemCoreClock/8)

//对外函数
void SysTick_Init(uint32_t Fre);
uint32_t User_GetSysTickTime(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);


#endif





























