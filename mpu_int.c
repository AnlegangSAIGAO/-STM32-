#include "pbdata.h"
int mpu_button=0;
//*****************MPU_INT时钟初始化********************
void MPU_INT_RCC_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //使能PC端口时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	
}
//*****************MPU_INT引脚初始化********************
void MPU_INT_GPIO_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;	
  //陀螺仪中断
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;            //PC6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz  
	GPIO_Init(GPIOC, &GPIO_InitStructure);		
}
//***************配置按钮中断************************
void MPU_INT_EINT_Init(void)
{
   EXTI_InitTypeDef  EXTI_InitStructure;	

	 EXTI_ClearITPendingBit(EXTI_Line6);                      
	 GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource6);
	 EXTI_InitStructure.EXTI_Line=EXTI_Line6;                
	 EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;        
	 EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;    
	 EXTI_InitStructure.EXTI_LineCmd = ENABLE;                
	 EXTI_Init(&EXTI_InitStructure); 
}
//***************配置按钮优先级************************
void MPU_INT_NVIC_Init(void)
{
		NVIC_InitTypeDef NVIC_InitStructure;  

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);             
		NVIC_InitStructure.NVIC_IRQChannel =EXTI9_5_IRQn;          
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;   
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                 
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
		NVIC_Init(&NVIC_InitStructure); 
}
//*****************MPU_INT初始化************************
void MPU_INT_Init(void)
{
  MPU_INT_RCC_Init();
  MPU_INT_GPIO_Init();
	MPU_INT_EINT_Init();
	MPU_INT_NVIC_Init();
}
//****************中断服务函数*********************************
void EXTI9_5_IRQHandler(void)             
{	
	if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)==0)
  { 

  }

	EXTI_ClearITPendingBit(EXTI_Line6);	
}





