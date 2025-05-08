#ifndef _ADC_H
#define _ADC_H
#include "pbdata.h"
extern float tmp_num;
extern float Bat_adc;
extern u8 Bat_n;      //电量百分比
extern u8 tmp_value;  //温度值

void ADC_RCC(void);                     //ADC时钟配置
void ADC_GPIO(void);                    //ADC管脚配置
void ADC_Configuration(void);           //ADC系统配置
void ADC_Check_Init(void);              //ADC初始化函数
u16 T_Get_Adc(u8 ch);
u16 T_Get_Adc_Average(u8 ch,u8 times);
void BAT_ADC(void);
void Temperature_check(void);
#endif
































