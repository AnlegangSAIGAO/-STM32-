#include "pbdata.h" 
u16 time2=0;          //基准定时单位
u16 time2_bat=0;      //电量定时发送
int time_out=0;       //是空定时器计数单位
u8 out_control_flag=0;//飞机失控标志位
u16 time_stunt=0;     //特技飞行模式控制时间计数单位
u16 Oil_Auto_tome=0;  //油门自动降落定时单位
u16 set_yaw_time=0;   //航向角稳定计数计时
u16 set_yaw_time_n=0; //航向角稳定总计时
u8 yaw_led_flag=0;    //航向角稳定灯光提示
u8 led_n=0;           //灯光亮灭控制
//**********************配置系统时钟*********************************
void TIM2_RCC_Configuration(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  //打开time2的中断时钟
}
//**********************配置系统时钟*********************************
void TIM4_RCC_Configuration(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  //打开time4的中断时钟
}
//**********************时钟中断配置函数*********************************
void TIM2_Configuration(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ClearITPendingBit(TIM2,TIM_IT_Update); 	                //清除定时器中断
  TIM_TimeBaseStructure.TIM_Period=3599;                      //计数3600,1毫秒  
  TIM_TimeBaseStructure.TIM_Prescaler=19;                     //20分频
  TIM_TimeBaseStructure.TIM_ClockDivision=0;                  //不滤波
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;   //向上计数模式
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);              //初始化
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);                    //打开定时器中断
	TIM_Cmd(TIM2,ENABLE);                                       //打开定时器外设
}
//**********************时钟中断配置函数*********************************
void TIM4_Configuration(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ClearITPendingBit(TIM4,TIM_IT_Update); 	                //清除定时器中断
  TIM_TimeBaseStructure.TIM_Period=3599;                        //计数36,10uS
  TIM_TimeBaseStructure.TIM_Prescaler=19;                     //20分频
  TIM_TimeBaseStructure.TIM_ClockDivision=0;                  //不滤波
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;   //向上计数模式
  TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);              //初始化
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);                    //打开定时器中断
	TIM_Cmd(TIM4,ENABLE);                                       //打开定时器外设
}
//**************************配置优先级***********************************
void TIM2_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;                        //为结构体定义结构体变量
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);             //对优先级进行分组
  NVIC_InitStructure.NVIC_IRQChannel =TIM2_IRQn;              
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;   //抢占优先级为3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          //响应优先级为0         
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //使能
  NVIC_Init(&NVIC_InitStructure);                             //初始化
}
//**************************配置优先级***********************************
void TIM4_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;                        //为结构体定义结构体变量
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);             //对优先级进行分组
  NVIC_InitStructure.NVIC_IRQChannel =TIM4_IRQn;             
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   //抢占优先级为2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          //响应优先级为0         
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //使能
  NVIC_Init(&NVIC_InitStructure);                             //初始化
}
//*********************定时器初始化********************************************
void time2_init(void)
{
  TIM2_RCC_Configuration();
	TIM2_Configuration();
  TIM2_NVIC_Configuration();
}
//*********************定时器初始化********************************************
void time4_init(void)
{
  TIM4_RCC_Configuration();
	TIM4_Configuration();
  TIM4_NVIC_Configuration();
}
//*************************************中断函数配置 ***************************
void TIM2_IRQHandler(void)
{
	time2++;
	time2_bat++;
	//定时向手柄发送飞机电量
	if(time2_bat>=100)
	{
	 time2_bat=0;	
	 msg_send[0]=0x01;
	 msg_send[1]=Bat_n;	
	 MSG_Send(msg_send);
	}
	//航向角稳定灯光提示
	if(yaw_led_flag==0)
	{
	  set_yaw_time++;
		if(set_yaw_time>=100)  
		{
		  set_yaw_time=0;  
			set_yaw_time_n++; //航向角没有稳定，黄色等闪烁
			LED_R=led_n;
		  LED_G=led_n;
			led_n=~led_n;
			LED_B=1;
		}
		if(set_yaw_time_n>=165)  //实测航向角稳定需要16秒
		{
		  set_yaw_time_n=0;
			set_ywa_flag=1;    //自动保存一次yaw的机械0点
			yaw_led_flag=1;    //不再等待
	
//  //调试专用
//	Oil_n=80;
//	Plane_status=2;
//	Pitch_n=50;
//	Roll_n=50;
//  Yaw_n=0;
		}
	}
	//心跳灯控制，喂狗
  if(time2>=300)
	{
		 time2=0;	
		 Feed_dog();    //看门狗喂狗
		 if(out_control_flag==0)  //正常状态
		 { 
			 if(Plane_status==0x02) //解锁成功蓝色灯闪烁
			 {
				LED_R=1;
				LED_G=1;
				LED_B=~LED_B;
			 }
			else                   //没有解锁绿色灯闪烁
			{
			 if(yaw_led_flag==1)  //yaw稳定后才绿色闪烁
				{
				 LED_R=1;
				 LED_B=1;
				 LED_G=~LED_G;
				}		  
			}
		 }			
		 //飞机失控状态灯光显示
		 if(out_control_flag==1)  //失控状态红色灯闪烁
		 {
			 LED_R=~LED_R;
			 LED_G=1;
			 LED_B=1;  		 
		 }		
	 }
	if(Plane_status==0x02) //解锁状态下
  {
	  //判断飞机是否失控
		if(USART1_Cmd_Flag==0)  //没有遥控器数据
		{
			time_out++;
		}
		if(USART1_Cmd_Flag!=0)  //收到遥控器数据
		{
			time_out=0;   
			out_control_flag=0;  //关闭失控提示
		}
		if(time_out>=500)
		{
			time_out=0;
			out_control_flag=1;  //失控了
		}	
		//失控状态下，自动控制接管飞机姿态
    if(out_control_flag==3)  
		{
			Oil_Auto_tome++;
			if(Oil_Auto_tome>=OIL_AUTO_SPEED)   //失控状态下，每固定的时间减少油门数值，直到可以平稳降落的油门数值
			{
				Oil_Auto_tome=0;
			  if(Oil_n>OIL_AUTO)
				{
					Oil_n--;
				}
			}
		 }	
	}		
  TIM_ClearITPendingBit(TIM2,TIM_IT_Update); 	         //清除定时器中断	
}
//*************************************中断函数配置 ***************************
void TIM4_IRQHandler(void)
{
	MPU6050_Data_read();	  //获取陀螺仪数据
  if(set_ywa_flag==1)
	{
		set_ywa_flag=0;           //标志位置0，如果再次按下解锁则再次充值yaw的机械0点
	  single_Fly_Yaw_Zero=yaw;  //用于单级PID控制中的yaw机械0点重置
		double_Fly_Yaw_Zero=yaw;  //用于串级PID控制中的yaw机械0点重置
	}		
	if(Plane_status==0x02)  //飞机解锁状态
	{
		if(stunt_flag==0)  //正常飞行模式
		{
			if(Oil_n>=30)  //油门大于30进入姿态调整
			{
		    //single_Fly_Task();           //单环PID飞机运动任务
        double_Fly_Task();		       //双环PID飞机运动任务			  
			}
			if(Oil_n<30)   //直接作用于电机
			{
			 Moto_Ctrl(Oil_n*PWM_OIL,1); 
	     Moto_Ctrl(Oil_n*PWM_OIL,2); 
	     Moto_Ctrl(Oil_n*PWM_OIL,3); 
	     Moto_Ctrl(Oil_n*PWM_OIL,4); 
			}
		}
    if(stunt_flag==1)  //特技飞行，横滚翻跟头
		{
			//左侧电机转速减半，右侧满转速，飞机从右侧往左侧翻滚
//		  Moto_Ctrl(3600,1); 
//	    Moto_Ctrl(7200,2); 
//	    Moto_Ctrl(3600,3); 
//	    Moto_Ctrl(7200,4); 			
			time_stunt++;
			if(time_stunt>=100)  //固定延时后关闭特技重新进行PID调参
			{
			  time_stunt=0;
				stunt_flag=0;
			}
		}
	} 
  TIM_ClearITPendingBit(TIM4,TIM_IT_Update); 	 //清除定时器中断	
}




	










