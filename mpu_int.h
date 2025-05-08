#ifndef __mpu_int_H
#define __mpu_int_H 
#include "pbdata.h" 
extern int mpu_button;
void MPU_INT_RCC_Init(void);
void MPU_INT_GPIO_Init(void);
void MPU_INT_EINT_Init(void);
void MPU_INT_NVIC_Init(void);
void MPU_INT_Init(void);
void EXTI9_5_IRQHandler(void);
#endif

