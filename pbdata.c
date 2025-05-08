#include "pbdata.h"
u8 mpu_dmp_flag=0;    //陀螺仪初始化判断
int user_number=0;    //用户开机次数判断
//**********************不精确微妙延时*****************************
void us(u16 time) 
{ u16 i=0;    
  while(time--)   
 {       
  i=10;     
  while(i--);       
 } 
} 
//***********************不精确毫秒延时****************************
void ms(u16 time) 
{ 
  u16 i=0;    
  while(time--)   
 {      
  i=12000;       
 while(i--);      
 } 
} 
//***********************微秒延时**精确******************************
void delay_us(u32 nus)
{
	 u32 temp;
	 SysTick->LOAD = 9*nus;
	 SysTick->VAL=0X00;                                     //清空计数器
	 SysTick->CTRL=0X01;                                    //使能，减到0是无动作，采用外部时钟源
	 do
	 {
	  temp=SysTick->CTRL;                                  //读取当前倒计数值
	 }while((temp&0x01)&&(!(temp&(1<<16))));               //等待时间到达
	 
	 SysTick->CTRL=0x00;                                   //关闭计数器
	 SysTick->VAL =0X00;                                   //清空计数器
}

//***********************毫秒延时**精确******************************
void delay_ms(u16 nms)
{
	 u32 temp;
	 SysTick->LOAD = 9000*nms;
	 SysTick->VAL=0X00;                                   //清空计数器
	 SysTick->CTRL=0X01;                                  //使能，减到0是无动作，采用外部时钟源
	 do
	 {
	  temp=SysTick->CTRL;                                 //读取当前倒计数值
	 }while((temp&0x01)&&(!(temp&(1<<16))));              //等待时间到达
	 SysTick->CTRL=0x00;                                  //关闭计数器
	 SysTick->VAL =0X00;                                  //清空计数器
}
//******************JTAG引脚复用取消******************************
//说明：
//JTAG复用取消只能调用一次语句，否则死机。并且初始化的GPIO要放在所有GPIO初始化的后面
//建议单独处理
//****************************************************************
void JTAG_IO_Init(void)
{
	 GPIO_InitTypeDef GPIO_InitStructure;	
	 GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
				
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;          
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	 GPIO_Init(GPIOA, &GPIO_InitStructure);	
	 GPIO_SetBits(GPIOA,GPIO_Pin_15);	
	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;            
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	 GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;           
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	 GPIO_Init(GPIOB, &GPIO_InitStructure);
}
//**************系统启动函数***************************
void uboot(void)
{
  SystemInit();                                  //系统初始化
}
//********************************************************************
//                 系统初始化函数
//********************************************************************
void OS_Init(void)
{ 	
	  LED_Init();		  					                     //LED初始化	  
		PWM_Init();                                    //PWM初始化
	  ADC_Check_Init();                              //ADC初始化	
    USART1_MSG_Init();                             //初始化串口1		
	  MPU_Init();					                           //初始化MPU6050
	  mpu_dmp_flag=mpu_dmp_init();                   //初始化MPU6050的DMP  
    while(mpu_dmp_flag)                            //初始化成功后执行后面程序
		{
		  //初始化失败进入死循环，禁止继续执行代码
		}			
    time2_init();                                 //定时器2初始化
		time4_init();                                  //定时器4初始化
		WATCH_DOG_Init();                               //看门狗初始化
		JTAG_IO_Init();                                 //解除仿真引脚复用
}



































