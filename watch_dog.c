#include "pbdata.h" 
int dog_time=0;
//**************************看门狗配置*******************************
void WATCH_DOG_WWDG_Configuration(void)
{
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);                //使能寄存器写
	IWDG_SetPrescaler(IWDG_Prescaler_64);                        //设置预分频，64分频  40KHz分64，一个周期为1.6ms
  IWDG_SetReload(1200);	                                       //设置计数器初值
	IWDG_ReloadCounter();                                        //喂狗
	IWDG_Enable();                                               //使能独立看门狗
}
//*****************看门狗初始化*****************************************************
void WATCH_DOG_Init(void)
{ 
	 WATCH_DOG_WWDG_Configuration();
}
//*******************喂狗函数************************************************
void Feed_dog(void)
{
   IWDG_ReloadCounter();       //喂狗
}










