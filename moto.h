#ifndef _moto_H
#define _moto_H
#include "pbdata.h"
//*************相关变量***********************
extern u8 mpu6050_data_flag;   								//是否正常读取数据
extern float pitch,roll,yaw;  								//欧拉角
extern short aacx,aacy,aacz;									//加速度传感器原始数据
extern short gyrox,gyroy,gyroz;								//陀螺仪原始数据
extern short mpu6050_temp;		 								//温度	
extern u8 Oil_n;   														//油门数值
extern u8 Pitch_n; 														//俯仰数值
extern u8 Roll_n;  														//翻滚数值
extern u8 Yaw_n;   														//航向数值
extern int PWM_Out1; 													//最终作用到电机的PWM
extern int PWM_Out2; 													//最终作用到电机的PWM
extern int PWM_Out3; 													//最终作用到电机的PWM
extern int PWM_Out4; 													//最终作用到电机的PWM
extern int single_pitch_balance_out;  				//俯仰平衡环的最终输出
extern int single_roll_balance_out;   				//翻滚平衡环的最终输出
extern int single_yaw_balance_out;            //偏航平衡环的最终输出
extern int double_pitch_balance_angle_out;    //俯仰平衡外环的输出
extern int double_roll_balance_angle_out;     //翻滚平衡外环的输出
extern int double_yaw_balance_angle_out;      //偏航平衡外环的输出 
extern int double_pitch_balance_out;          //俯仰平衡环的最终输出
extern int double_roll_balance_out;           //翻滚平衡环的最终输出
extern int double_yaw_balance_out;            //偏航平衡环的最终输出
extern int yaw_out;                           //航向环的最终输出
extern float pwm_adc;                         //用于补偿电压降低是PWM的输出
extern float double_PID_Pitch_err;            //俯仰内环误差值  
extern float double_PID_Pitch_last_err;       //俯仰内环上次误差值
extern float double_PID_Pitch_Angle_Integral; //俯仰外环积分
extern float double_PID_Pitch_Gyro_Integral;  //俯仰内环积分
extern float double_PID_Roll_err;             //翻滚内环误差值
extern float double_PID_Roll_last_err;        //翻滚内环上次误差值
extern float double_PID_Roll_Angle_Integral;  //翻滚外环积分
extern float double_PID_Roll_Gyro_Integral;   //翻滚内环积分
extern float double_PID_Yaw_err;              //翻滚内环误差值
extern float double_PID_Yaw_last_err;         //翻滚内环上次误差值
extern float double_PID_Yaw_Angle_Integral;   //航向外环积分
extern float double_PID_Yaw_Gyro_Integral;    //航向内环积分
//****************飞机的机械零点***************
extern float single_Fly_Pitch_Zero;   
extern float single_Fly_Roll_Zero; 
extern float single_Fly_Yaw_Zero; 
//****************飞机的机械零点***************
extern float double_Fly_Pitch_Zero;   
extern float double_Fly_Roll_Zero; 
extern float double_Fly_Yaw_Zero; 
//*************单级PID参数定义*****************
//俯仰平衡环
extern float single_pitch_balance_Kp; 
extern float single_pitch_balance_Kd; 
//翻滚平衡环
extern float single_roll_balance_Kp; 
extern float single_roll_balance_Kd; 
//航向平衡环
extern float single_yaw_balance_Kp; 
extern float single_yaw_balance_Kd;
//*************串级PID参数定义*****************
//俯仰平衡环外环
extern float double_pitch_balance_Angle_Kp; 
extern float double_pitch_balance_Angle_Ki; 
extern float double_pitch_balance_Angle_Kd; 
//翻滚平衡环外环
extern float double_roll_balance_Angle_Kp; 
extern float double_roll_balance_Angle_Ki; 
extern float double_roll_balance_Angle_Kd; 
//航向平衡环外环
extern float double_yaw_balance_Angle_Kp;
extern float double_yaw_balance_Angle_Ki;
extern float double_yaw_balance_Angle_Kd; 
//***********************************************
//俯仰平衡环内环
extern float double_pitch_balance_Gyro_Kp; 
extern float double_pitch_balance_Gyro_Ki; 
extern float double_pitch_balance_Gyro_Kd; 
//翻滚平衡环内环
extern float double_roll_balance_Gyro_Kp;
extern float double_roll_balance_Gyro_Ki;
extern float double_roll_balance_Gyro_Kd; 
//航向平衡环内环
extern float double_yaw_balance_Gyro_Kp; 
extern float double_yaw_balance_Gyro_Ki; 
extern float double_yaw_balance_Gyro_Kd;
//航向控制
extern float yaw_Kp; 
//*****************函数申明************************
void MPU6050_Data_read(void);
int my_abs(int n);
void Moto_Ctrl(int pwm,u8 n);
float pitch_remote_num(u8 n);
float roll_remote_num(u8 n);
int yaw_control(float Set_turn);
int single_pitch_balance(float Angle,float Gyro);    
int single_roll_balance(float Angle,float Gyro);
int single_yaw_balance(float Angle,float Gyro);
void single_Fly_Task(void);
int double_pitch_balance_Angle(float Angle,float Gyro);
int double_pitch_balance_Gyro(float PID_Angle_Out,float Gyro);
int double_roll_balance_Angle(float Angle,float Gyro);
int double_roll_balance_Gyro(float PID_Angle_Out,float Gyro);
int double_yaw_balance_Angle(float Angle,float Gyro);
int double_yaw_balance_Gyro(float PID_Angle_Out,float Gyro);
void double_Fly_Task(void);
#endif

