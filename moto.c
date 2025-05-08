#include "pbdata.h"
u8 mpu6050_data_flag;    								  //是否正常读取数据
float pitch,roll,yaw;     								//欧拉角
short aacx,aacy,aacz;	   								  //加速度传感器原始数据
short gyrox,gyroy,gyroz; 								  //陀螺仪原始数据
short mpu6050_temp;		   								  //温度	
u8 Oil_n=0;   														//油门数值
u8 Pitch_n=50; 														//俯仰数值
u8 Roll_n=50; 														 //翻滚数值
u8 Yaw_n=0;   														//航向数值
int PWM_Out1=0; 													//最终作用到电机的PWM
int PWM_Out2=0;													  //最终作用到电机的PWM
int PWM_Out3=0;													  //最终作用到电机的PWM
int PWM_Out4=0; 												  //最终作用到电机的PWM
int single_pitch_balance_out=0;  					//俯仰平衡环的最终输出
int single_roll_balance_out=0;   					//翻滚平衡环的最终输出
int single_yaw_balance_out=0;             //偏航平衡坏的最终输出
int double_pitch_balance_angle_out=0;     //俯仰平衡外环的输出
int double_roll_balance_angle_out=0;      //翻滚平衡外环的输出
int double_yaw_balance_angle_out=0;       //偏航平衡外环的输出  
int double_pitch_balance_out=0;           //俯仰平衡环的最终输出
int double_roll_balance_out=0;            //翻滚平衡环的最终输出
int double_yaw_balance_out=0;             //偏航平衡环的最终输出
int yaw_out=0;           									//航向环的最终输出
float pwm_adc=0.0f;                       //用于补偿电压降低是PWM的输出，因为电机的电压直连电池，会一直降低
float double_PID_Pitch_err;               //俯仰内环误差值  
float double_PID_Pitch_last_err;          //俯仰内环上次误差值
float double_PID_Pitch_Angle_Integral;    //俯仰外环积分
float double_PID_Pitch_Gyro_Integral;     //俯仰内环积分
float double_PID_Roll_err;                //翻滚内环误差值
float double_PID_Roll_last_err;           //翻滚内环上次误差值
float double_PID_Roll_Angle_Integral;     //翻滚外环积分
float double_PID_Roll_Gyro_Integral;      //翻滚内环积分
float double_PID_Yaw_err;                 //翻滚内环误差值
float double_PID_Yaw_last_err;            //翻滚内环上次误差值
float double_PID_Yaw_Angle_Integral;      //航向外环积分
float double_PID_Yaw_Gyro_Integral;       //航向内环积分
//****************飞机的机械零点***************
//机械零点除了可以矫正安装误差，还可以校准由于电机，桨叶，安装角度等各种原因产生的飞机无法平衡的问题
float single_Fly_Pitch_Zero=10.0f;   
float single_Fly_Roll_Zero=3.0f;  
float single_Fly_Yaw_Zero=0.0f;
//****************飞机的机械零点***************
//机械零点除了可以矫正安装误差，还可以校准由于电机，桨叶，安装角度等各种原因产生的飞机无法平衡的问题
//float double_Fly_Pitch_Zero=5.5f;   //平衡的时候 飞机后退加大，飞机前进减小
//float double_Fly_Roll_Zero=3.2f;    //平衡的时候  飞机往右加大，飞机往左减小
//float double_Fly_Yaw_Zero=0.0f; 

float double_Fly_Pitch_Zero=5.0f;   //平衡的时候 飞机后退加大，飞机前进减小
float double_Fly_Roll_Zero=3.0f;    //平衡的时候  飞机往右加大，飞机往左减小
float double_Fly_Yaw_Zero=0.0f; 

//*************单级PID参数定义*****************
//俯仰平衡环
float single_pitch_balance_Kp=-40.0f; 
float single_pitch_balance_Kd=0.8f; 
//翻滚平衡环
float single_roll_balance_Kp=-40.0f; 
float single_roll_balance_Kd=0.8f; 
//航向平衡环
float single_yaw_balance_Kp=10.0f; 
float single_yaw_balance_Kd=-0.2f; 
//*************串级PID参数定义*****************
//俯仰平衡环外环
float double_pitch_balance_Angle_Kp=105.0f; 
float double_pitch_balance_Angle_Ki=0.0;
float double_pitch_balance_Angle_Kd=-0.8f;  
//翻滚平衡环外环
float double_roll_balance_Angle_Kp=50.0f;
float double_roll_balance_Angle_Ki=0.0f;
float double_roll_balance_Angle_Kd=-0.8f; 
//航向平衡环外环
float double_yaw_balance_Angle_Kp=-15.0f;
float double_yaw_balance_Angle_Ki=-0.0f;
float double_yaw_balance_Angle_Kd=0.12f; 
//***********************************************
//俯仰平衡环内环
float double_pitch_balance_Gyro_Kp=-0.45f;  
float double_pitch_balance_Gyro_Ki=-0.0f; 
float double_pitch_balance_Gyro_Kd=-0.008f; 
//翻滚平衡环内环
float double_roll_balance_Gyro_Kp=-0.45f; 
float double_roll_balance_Gyro_Ki=-0.0f; 
float double_roll_balance_Gyro_Kd=-0.008f; 
//航向平衡环内环
float double_yaw_balance_Gyro_Kp=0.45f; 
float double_yaw_balance_Gyro_Ki=0.0f;
float double_yaw_balance_Gyro_Kd=0.008f; 
//***********************************************
//航向控制
float yaw_Kp=1.0f; 
//************MPU6050相关数据读取********************
void MPU6050_Data_read(void)
{
	//收到安装位置影响，交换了pitch和roll
	//前倾pitch正数增大，后仰pitch负数增大    对应gyroy符号相同
	//左翻滚roll正数增大，右翻滚roll负数增大  对应gyrox符号相同
	//航向yaw左转增大，右转减小               对应gyroz符号相同
   mpu6050_data_flag=mpu_dmp_get_data(&roll,&pitch,&yaw); //得到角度数据
	//MPU_Get_Accelerometer(&aacy,&aacx,&aacz); //得到加速度传感器数据
	 MPU_Get_Gyroscope(&gyroy,&gyrox,&gyroz);	  //得到陀螺仪数据	  
}
//***********绝对值函数*****************************
int my_abs(int n)
{
  if(n<0)
	{
	  n=(-n);
	}
	if(n>=0)
	{
	 n=n;
	}
	return n;
}
//***********电机控制*******************************
//飞机电机对应图
//        1号    2号
//  机头↑     \/
//            /\
//        3号    4号
//pwm: 电机pwm
//n:   电机编号
void Moto_Ctrl(int pwm,u8 n)
{
	if(pwm<=0)  //限制输入幅度
	{
	 pwm=0;
	}
	if(pwm>=7200)  //限制输入幅度
	{
	 pwm=7200;
	}
	if(n==1)
	{
	  TIM_SetCompare3(TIM3,pwm);  //1号电机
	}
	if(n==2)
	{
	  TIM_SetCompare2(TIM3,pwm);  //2号电机
	}
	if(n==3)
	{
	  TIM_SetCompare1(TIM3,pwm);  //3号电机
	}
	if(n==4)
	{
	  TIM_SetCompare4(TIM3,pwm);  //4号电机 
	}			   
}
//********遥控数据转换函数************************************
//n:遥控器接收的数据
float pitch_remote_num(u8 n)
{
	float angle=0.0f;
	//飞机后仰，向后运动
  if(n<50)
	{
	  angle=-MAX_PITCH_ANGLE*(50-n);
	}
	//飞机保持平衡
  if(n==50)
	{
	  angle=0.0f;
	}	
  //飞机前倾，向前运动
  if(n>50)
	{
	  angle=MAX_PITCH_ANGLE*(n-50);
	}		
	return angle;
}
//********遥控数据转换函数************************************
//n:遥控器接收的数据
float roll_remote_num(u8 n)
{
	float angle=0.0f;
	//飞机左倾，向左运动
  if(n<50)
	{
	  angle=MAX_ROLL_ANGLE*(50-n);
	}
	//飞机保持平衡
  if(n==50)
	{
	  angle=0.0f;
	}	
  //飞机右倾，向右运动
  if(n>50)
	{
	  angle=-MAX_ROLL_ANGLE*(n-50);
	}		
	return angle;
}
//*************************************************************
//这里航向为非精确控制，不区分单环和双环,
//机头旋转后将最新的机头角度赋值给航向的机械零点
//当对角线的两个电机转速加大后产生的反力矩会使机身向电机旋转的反方向偏航
//*************************************************************
//函数功能：控制飞机航向
//Set_turn：目标旋转角速度
int yaw_control(float Set_turn)
{
  int pwm_yaw=0; 
	pwm_yaw=yaw_Kp*YAW_SPEED*Set_turn; //有转向需求，Kp为期望飞机转向 
	return pwm_yaw;
}


//**************************************************************
//              无人机姿态控制算法单级PID控制
//说明：单级PID控制 采用角度环PD控制器
//      单级PID控制，小角度姿态调整很稳定，但是抗干扰能力差
//      大角度控制响应之后，优点是代码简单！
//***********无人机控制*****************************************
//函数功能：控制飞机俯仰方向保持水平
//Angle：采集到的实际角度值
//Gyro： 采集到的实际角速度值
int single_pitch_balance(float Angle,float Gyro)
{  
   float err;
	 int pwm_balance;
	 float angle_num=0.0f;
	 angle_num=pitch_remote_num(Pitch_n); //遥控数据
	 err=(single_Fly_Pitch_Zero+angle_num)-Angle;     //期望值-实际值，这里期望飞机平衡，因此期望值就是机械零点否则期望值为遥控数据，遥控数据是相对于机械零点的    
	 pwm_balance=single_pitch_balance_Kp*err+Gyro*single_pitch_balance_Kd;//计算平衡控制的电机PWM
	 return pwm_balance;
}
//***************************************************
//函数功能：控制飞机翻滚方向保持水平
//Angle：采集到的实际角度值
//Gyro： 采集到的实际角速度值
int single_roll_balance(float Angle,float Gyro)
{  
   float err;
	 int pwm_balance;
	 float angle_num=0.0f;
	 angle_num=roll_remote_num(Roll_n); //遥控数据
	 err=(single_Fly_Roll_Zero+angle_num)-Angle;      //期望值-实际值，这里期望飞机平衡，因此期望值就是机械零点否则期望值为遥控数据，遥控数据是相对于机械零点的     
	 pwm_balance=single_roll_balance_Kp*err+Gyro*single_roll_balance_Kd;//计算平衡控制的电机PWM
	 return pwm_balance;
}
//***************************************************
//函数功能：控制飞机偏航方向保持水平
//Angle：采集到的实际角度值
//Gyro： 采集到的实际角速度值
int single_yaw_balance(float Angle,float Gyro)
{  
   float err;
	 int pwm_balance;
	 err=single_Fly_Yaw_Zero-Angle;    //期望值-实际值，这里期望飞机平衡，因此期望值就是机械零点否则期望值为遥控数据，遥控数据是相对于机械零点的     
	 pwm_balance=single_yaw_balance_Kp*err+Gyro*single_yaw_balance_Kd;//计算平衡控制的电机PWM
	 return pwm_balance;
}
//*********无人机运动任务**********************************
void single_Fly_Task(void)
{
	single_pitch_balance_out=single_pitch_balance(pitch,gyroy);	//俯仰平衡环的输出
	single_roll_balance_out=single_roll_balance(roll,gyrox);    //翻滚平衡环的输出	
	if(Yaw_n==0)      
	{
	  yaw_out=0;
		single_yaw_balance_out=single_yaw_balance(yaw,gyroz);       //航向平衡环的输出
	}	
	if(Yaw_n==0x01)
	{
		single_yaw_balance_out=0;
	  yaw_out=yaw_control(350);                                   //航向转向环的输出
	  single_Fly_Yaw_Zero=yaw;                                    //更新航向角0点，屏蔽此代码飞机转向结束后会自动回位机头
	}
	if(Yaw_n==0x02)
	{
		single_yaw_balance_out=0;
	  yaw_out=yaw_control(-200);                                   //航向转向环的输出
	  single_Fly_Yaw_Zero=yaw;                                     //更新航向角0点，屏蔽此代码飞机转向结束后会自动回位机头
	}

	//控制部分组合算法
	PWM_Out1=Oil_n*PWM_OIL+single_pitch_balance_out+single_roll_balance_out+single_yaw_balance_out+yaw_out;                 
	PWM_Out2=Oil_n*PWM_OIL+single_pitch_balance_out-single_roll_balance_out-single_yaw_balance_out-yaw_out;               
  PWM_Out3=Oil_n*PWM_OIL-single_pitch_balance_out+single_roll_balance_out-single_yaw_balance_out-yaw_out;                
	PWM_Out4=Oil_n*PWM_OIL-single_pitch_balance_out-single_roll_balance_out+single_yaw_balance_out+yaw_out; 
  //补偿电池压降
	PWM_Out1*=pwm_adc;
	PWM_Out2*=pwm_adc;
	PWM_Out3*=pwm_adc;
	PWM_Out4*=pwm_adc;
	//作用到电机
	Moto_Ctrl(PWM_Out1,1); //作用到1号电机
	Moto_Ctrl(PWM_Out2,2); //作用到2号电机
	Moto_Ctrl(PWM_Out3,3); //作用到3号电机
	Moto_Ctrl(PWM_Out4,4); //作用到4号电机
}
//**************************************************************
//              无人机姿态控制算法串级PID控制
//说明：串级PID控制，外环采用角度环PD控制，内环采用角速度环PD控制
//      串级PID控制，抗干扰能力强，各种角度的姿态控制响应速度快。
//      程序比单环复杂！
//***********无人机控制*****************************************
//函数功能：俯仰控制，外环角度环
//Angle：采集到的实际角度值
//Gyro： 采集到的实际角速度值
int double_pitch_balance_Angle(float Angle,float Gyro)
{
   float err;
	 int PID_Angle_Out;
	 float angle_num=0.0f;
	 angle_num=pitch_remote_num(Pitch_n); //遥控数据
	 err=(double_Fly_Pitch_Zero+angle_num)-Angle;    //期望值-实际值，这里期望飞机平衡，因此期望值就是机械零点否则期望值为遥控数据，遥控数据是相对于机械零点的    
   if(Oil_n<30)
	 {
	   double_PID_Pitch_Angle_Integral=0; //积分清零
	 }		 
	 if(Oil_n>=30)
	 {
		double_PID_Pitch_Angle_Integral+=err; //积分分离
	 }
	 //积分现幅
	 if(double_PID_Pitch_Angle_Integral>=Angle_IMAX)
	 {
	   double_PID_Pitch_Angle_Integral=Angle_IMAX;
	 }
	 if(double_PID_Pitch_Angle_Integral<=-Angle_IMAX)
	 {
	   double_PID_Pitch_Angle_Integral=-Angle_IMAX;
	 }
	 PID_Angle_Out=double_pitch_balance_Angle_Kp*err+double_pitch_balance_Angle_Ki*double_PID_Pitch_Angle_Integral+Gyro*double_pitch_balance_Angle_Kd;//计算外环的输出值，即角速度期望值
	 return PID_Angle_Out;
}
//***************************************************************
//函数功能：俯仰控制，内环角速度环
//PID_Angle_Out： 外环的输出
//Gyro：          采集到的角速度
int double_pitch_balance_Gyro(float PID_Angle_Out,float Gyro)
{
	 int pwm_balance;
	 double_PID_Pitch_err=PID_Angle_Out-Gyro;    //期望值-实际值  
   if(Oil_n<30)
	 {
	   double_PID_Pitch_Gyro_Integral=0; //积分清零
	 }		 
	 if(Oil_n>=30)
	 {
		double_PID_Pitch_Gyro_Integral+=double_PID_Pitch_err; //积分分离
	 }
	 //积分现幅
	 if(double_PID_Pitch_Gyro_Integral>=Gyro_IMIN)
	 {
	   double_PID_Pitch_Gyro_Integral=Gyro_IMIN;
	 }
	 if(double_PID_Pitch_Gyro_Integral<=-Gyro_IMIN)
	 {
	   double_PID_Pitch_Gyro_Integral=-Gyro_IMIN;
	 }  
	 pwm_balance=double_pitch_balance_Gyro_Kp*double_PID_Pitch_err+double_pitch_balance_Gyro_Ki*double_PID_Pitch_Gyro_Integral+double_pitch_balance_Gyro_Kd*(double_PID_Pitch_err-double_PID_Pitch_last_err);//计算平衡控制的电机PWM
	 double_PID_Pitch_last_err=double_PID_Pitch_err;    //保存本次误差值，做为下次计算时的上次误差值使用
	 return pwm_balance;
}
//****************************************************************
//函数功能：翻滚控制，外环角度环
//Angle：采集到的实际角度值
//Gyro： 采集到的实际角速度值
int double_roll_balance_Angle(float Angle,float Gyro)
{
   float err;
	 int PID_Angle_Out;
	 float angle_num=0.0f;
	 angle_num=roll_remote_num(Roll_n); //遥控数据
	 err=(double_Fly_Roll_Zero+angle_num)-Angle;    //期望值-实际值，这里期望飞机平衡，因此期望值就是机械零点否则期望值为遥控数据，遥控数据是相对于机械零点的    
	 if(Oil_n<30)
	 {
	   double_PID_Roll_Angle_Integral=0; //积分清零
	 }		 
	 if(Oil_n>=30)
	 {
		double_PID_Roll_Angle_Integral+=err; //积分分离
	 }
	 //积分现幅
	 if(double_PID_Roll_Angle_Integral>=Angle_IMAX)
	 {
	   double_PID_Roll_Angle_Integral=Angle_IMAX;
	 }
	 if(double_PID_Roll_Angle_Integral<=-Angle_IMAX)
	 {
	   double_PID_Roll_Angle_Integral=-Angle_IMAX;
	 }
	 PID_Angle_Out=double_roll_balance_Angle_Kp*err+double_roll_balance_Angle_Ki*double_PID_Roll_Angle_Integral+Gyro*double_roll_balance_Angle_Kd;//计算外环的输出值，即角速度期望值
	 return PID_Angle_Out;
}
//***************************************************************
//函数功能：翻滚控制，内环角速度环
//PID_Angle_Out： 外环的输出
//Gyro：          采集到的角速度
int double_roll_balance_Gyro(float PID_Angle_Out,float Gyro)
{
	 int pwm_balance;
	 double_PID_Roll_err=PID_Angle_Out-Gyro;    //期望值-实际值    
   if(Oil_n<30)
	 {
	   double_PID_Roll_Gyro_Integral=0; //积分清零
	 }		 
	 if(Oil_n>=30)
	 {
		double_PID_Roll_Gyro_Integral+=double_PID_Roll_err; //积分分离
	 }
	 //积分现幅
	 if(double_PID_Roll_Gyro_Integral>=Gyro_IMIN)
	 {
	   double_PID_Roll_Gyro_Integral=Gyro_IMIN;
	 }
	 if(double_PID_Roll_Gyro_Integral<=-Gyro_IMIN)
	 {
	   double_PID_Roll_Gyro_Integral=-Gyro_IMIN;
	 }
	 pwm_balance=double_roll_balance_Gyro_Kp*double_PID_Roll_err+double_roll_balance_Gyro_Ki*double_PID_Roll_Gyro_Integral+double_roll_balance_Gyro_Kd*(double_PID_Roll_err-double_PID_Roll_last_err);//计算平衡控制的电机PWM
	 double_PID_Roll_last_err=double_PID_Roll_err;    //保存本次误差值，做为下次计算时的上次误差值使用
	 return pwm_balance;
}
//*****************************************************************
//函数功能：航向控制，外环角度环
//Angle：采集到的实际角度值
//Gyro： 采集到的实际角速度值
int double_yaw_balance_Angle(float Angle,float Gyro)
{
   float err;
	 int PID_Angle_Out;
	 err=double_Fly_Yaw_Zero-Angle;    //期望值-实际值，这里期望飞机平衡，因此期望值就是机械零点否则期望值为遥控数据，遥控数据是相对于机械零点的    
	 if(Oil_n<30)
	 {
	   double_PID_Yaw_Angle_Integral=0; //积分清零
	 }		 
	 if(Oil_n>=30)
	 {
		double_PID_Yaw_Angle_Integral+=err; //积分分离
	 }
	 //积分现幅
	 if(double_PID_Yaw_Angle_Integral>=Angle_IMAX)
	 {
	   double_PID_Yaw_Angle_Integral=Angle_IMAX;
	 }
	 if(double_PID_Yaw_Angle_Integral<=-Angle_IMAX)
	 {
	   double_PID_Yaw_Angle_Integral=-Angle_IMAX;
	 }	 
	 PID_Angle_Out=double_yaw_balance_Angle_Kp*err+double_yaw_balance_Angle_Ki*double_PID_Yaw_Angle_Integral+Gyro*double_yaw_balance_Angle_Kd;//计算外环的输出值，即角速度期望值
	 return PID_Angle_Out;
}
//***************************************************************
//函数功能：航向控制，内环角速度环
//PID_Angle_Out： 外环的输出
//Gyro：          采集到的角速度
int double_yaw_balance_Gyro(float PID_Angle_Out,float Gyro)
{
	 int pwm_balance;
	 double_PID_Yaw_err=PID_Angle_Out-Gyro;    //期望值-实际值    
   if(Oil_n<30)
	 {
	   double_PID_Yaw_Gyro_Integral=0; //积分清零
	 }		 
	 if(Oil_n>=30)
	 {
		double_PID_Yaw_Gyro_Integral+=double_PID_Roll_err; //积分分离
	 }
	 //积分现幅
	 if(double_PID_Yaw_Gyro_Integral>=Gyro_IMIN)
	 {
	   double_PID_Yaw_Gyro_Integral=Gyro_IMIN;
	 }
	 if(double_PID_Yaw_Gyro_Integral<=-Gyro_IMIN)
	 {
	   double_PID_Yaw_Gyro_Integral=-Gyro_IMIN;
	 }
 	 pwm_balance=double_yaw_balance_Gyro_Kp*double_PID_Yaw_err+double_yaw_balance_Gyro_Ki*double_PID_Yaw_Gyro_Integral+double_yaw_balance_Gyro_Kd*(double_PID_Yaw_err-double_PID_Yaw_last_err);//计算平衡控制的电机PWM
	 double_PID_Yaw_last_err=double_PID_Yaw_err;    //保存本次误差值，做为下次计算时的上次误差值使用
	 return pwm_balance;
}
//*********无人机运动任务**********************************
void double_Fly_Task(void)
{
	double_pitch_balance_angle_out=double_pitch_balance_Angle(pitch,gyroy);									  //俯仰平衡环外环的输出
	double_roll_balance_angle_out=double_roll_balance_Angle(roll,gyrox);   										//翻滚平衡环外环的输出
	double_pitch_balance_out=double_pitch_balance_Gyro(double_pitch_balance_angle_out,gyroy); //俯仰平衡环内环的输出
	double_roll_balance_out=double_roll_balance_Gyro(double_roll_balance_angle_out,gyrox);    //翻滚平衡环内环的输出
	if(Yaw_n==0)      
	{
	  yaw_out=0;
		double_yaw_balance_angle_out=double_yaw_balance_Angle(yaw,gyroz);									  //俯仰平衡环外环的输出
	  double_yaw_balance_out=double_yaw_balance_Gyro(double_yaw_balance_angle_out,gyroz); //俯仰平衡环内环的输出
	}	
	if(Yaw_n==0x01)
	{
		double_yaw_balance_out=0;
	  yaw_out=yaw_control(350);                                   //航向转向环的输出
	  double_Fly_Yaw_Zero=yaw;                                    //更新航向角0点，屏蔽此代码飞机转向结束后会自动回位机头
	}
	if(Yaw_n==0x02)
	{
		double_yaw_balance_out=0;
	  yaw_out=yaw_control(-200);                                   //航向转向环的输出
	  double_Fly_Yaw_Zero=yaw;                                     //更新航向角0点，屏蔽此代码飞机转向结束后会自动回位机头
	}
	//控制部分组合算法
	PWM_Out1=Oil_n*PWM_OIL+double_pitch_balance_out+double_roll_balance_out+double_yaw_balance_out+yaw_out;                 
	PWM_Out2=Oil_n*PWM_OIL+double_pitch_balance_out-double_roll_balance_out-double_yaw_balance_out-yaw_out;               
  PWM_Out3=Oil_n*PWM_OIL-double_pitch_balance_out+double_roll_balance_out-double_yaw_balance_out-yaw_out;                
	PWM_Out4=Oil_n*PWM_OIL-double_pitch_balance_out-double_roll_balance_out+double_yaw_balance_out+yaw_out; 
  //补偿电池压降
	PWM_Out1*=pwm_adc;
	PWM_Out2*=pwm_adc;
	PWM_Out3*=pwm_adc;
	PWM_Out4*=pwm_adc;
	//作用到电机
	Moto_Ctrl(PWM_Out1,1); //作用到1号电机
	Moto_Ctrl(PWM_Out2,2); //作用到2号电机
	Moto_Ctrl(PWM_Out3,3); //作用到3号电机
	Moto_Ctrl(PWM_Out4,4); //作用到4号电机
}
















