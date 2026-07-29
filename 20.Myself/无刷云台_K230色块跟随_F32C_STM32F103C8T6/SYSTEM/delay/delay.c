#include "delay.h"

static volatile uint32_t systick_time = 0;      //系统时基
static volatile uint32_t systick_reloadval = 0; //保存的重装载值,与用户配置有关
static volatile uint32_t max_delayus_val = 0;   //最大us延迟时间
static volatile uint32_t systick_intFreq = 0;   //systick中断频率

//返回系统当前时刻
uint32_t User_GetSysTickTime(void)
{
	return systick_time;
}

/*
SysTick时钟源只有两个：AHB/8 或 AHB (AHB为系统时钟)
*/
void SysTick_Init(uint32_t Fre)
{
	//设置systick中断优先级
	NVIC_SetPriority(SysTick_IRQn,0); //参数2-priority：只使用最后4bit,与中断优先级分组相关,例如中断优先级分组1，则高1bit表示抢占,低3bit表示响应
	
	//记录配置的中断频率
	systick_intFreq = Fre;
	
	//选择AHB/8作为Systick时钟源
	SysTick->CTRL &= ~(1<<2); 
	
	//配置重装载值
	SysTick->LOAD = (SystemCoreClock/8)/Fre - 1; //Systick溢出频率 = SysTick时钟/(重装载值+1)
	
	//记录重装载值,便于用于延迟函数使用
	systick_reloadval = (SystemCoreClock/8)/Fre;
	
	//计算最大us延时的时间
	max_delayus_val = 1000000 / Fre;
	
	//清空当前计数值
	SysTick->VAL = 0;
	
	//开启中断
	SysTick->CTRL |= 1<<1;
	
	//启动SysTick计数器
	SysTick->CTRL |= 1<<0;
}

//SysTick中断回调函数
void SysTick_Handler(void)
{
	systick_time++;
}

//us延迟
void delay_us(uint32_t us)
{
	//最大的us延迟与中断频率有关
	if( us > max_delayus_val ) us = max_delayus_val;
	
	//单位转换
	us = us*(SysTickFreq/1000000);
	
	//用于保存已走过的时间
	uint32_t runningtime = 0;
	
	//获得当前时刻的计数值
	uint32_t InserTick = SysTick->VAL;
	
	//用于刷新实时时间
	uint32_t tick = 0;
	
	uint8_t countflag = 0;
	//等待延迟
	while(1)
	{
		tick = SysTick->VAL;//刷新当前时刻计数值
		
		if( tick > InserTick ) countflag = 1;//出现溢出轮询,则切换走时的计算方式
		
		if( countflag ) runningtime = InserTick + systick_reloadval - tick;
		else runningtime = InserTick - tick;
		
		if( runningtime>=us ) break;
	}
}

//ms延迟
void delay_ms(uint32_t ms)
{
	uint32_t tickstart = User_GetSysTickTime();
	uint32_t wait = ms * (systick_intFreq / 1000);
	
	//等待延迟完成
    while ((User_GetSysTickTime() - tickstart) < wait)
    {
       
    }
}


