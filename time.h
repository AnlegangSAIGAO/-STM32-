#ifndef _time_H
#define _time_H
#include "pbdata.h"
//******************定义变量***************************************
extern u16 time2;
extern u16 time2_bat;
extern int time_out;
extern u8 out_control_flag;
extern u16 time_stunt;
extern u16 Oil_Auto_tome;
extern u16 set_yaw_time;   
extern u16 set_yaw_time_n; 
extern u8 yaw_led_flag;  
extern u8 led_n;
//**********************申明函数************************************
void ConfigureTimeForRunTimeStats(void);
void TIM2_RCC_Configuration(void);                //定时器时钟初始化
void TIM2_Configuration(void);                    //定时器中断函数配置
void TIM2_NVIC_Configuration(void);               //定时器优先级配置
void TIM2_IRQHandler(void);                       //定时器中断代码
void time2_init(void);                            //定时器初始化
void TIM4_RCC_Configuration(void);                //定时器时钟初始化
void TIM4_Configuration(void);                    //定时器中断函数配置
void TIM4_NVIC_Configuration(void);               //定时器优先级配置
void TIM4_IRQHandler(void);                       //定时器中断代码
void time4_init(void); 
#endif



