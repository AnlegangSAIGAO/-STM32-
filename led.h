#ifndef __LED_H
#define __LED_H	 
#include "pbdata.h" 
#define LED_R PAout(5)	// PA5
#define LED_G PAout(8)	// PA8
#define LED_B PAout(15)	// PA15
void LED_RCC_Init(void);
void LED_GPIO_Init(void);
void LED_Init(void);//≥ı ºªØ		 				    
#endif

