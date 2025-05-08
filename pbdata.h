#ifndef _pbdata_H
#define _pbdata_H
#include "pbdata.h"
//****************************硬件相关*****************************
#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_iwdg.h"
#include "misc.h"	 
#include "stm32f10x_tim.h"
#include "math.h"
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "sys.h"
//*********************用户自定义相关******************************
#include "led.h"
#include "time.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "mpu6050.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "moto.h"
#include "pwm.h"
#include "adc.h"
#include "remote.h"
#include "mpu_int.h"
#include "watch_dog.h"
#include "ui.h"
//****************************************************************
//0：默认开启校准，初始化需要把芯片放平，且以此时的状态为0
//1：关闭校准，以地球重力为基准，不需要放平芯片
//DMP稳定时间为8秒左右
#define CHECK_FLAG 1  
//陀螺仪采样频率，单位Hz
#define MPU_HZ 200
//陀螺仪IIC延时，计数值，不是毫秒
#define MPU_DELAY 1
//************积分饱和宏定义**************************
#define Angle_IMAX 5000
#define Gyro_IMIN  3000
//*************电池电量对应百分比**********************************
#define BAT_MAX  4.15
#define BAT_MIN  2.8
//*************占空比限幅****************************************
//需要给飞机姿态调整预留PWM，所以油门为100时不能达到7200占空比
//油门达到100时，PWM为 100*PWM_OIL
#define PWM_OIL 45
//************飞机平稳降落时的油门数值*************************
#define OIL_AUTO 30
//*************飞机自动降落时的降落速度***********************
#define OIL_AUTO_SPEED 150
//**********飞机俯仰最大倾角********************************
//说明：遥控器数据为0-50  50-100 最大倾角为±Angle，因此对应每个分度值为Angle/50
//举例：最大倾角为±30°  这里的宏定义为30/50=0.6
#define MAX_PITCH_ANGLE 0.2
//**********飞机翻滚最大倾角********************************
//说明：遥控器数据为0-50  50-100 最大倾角为±Angle，因此对应每个分度值为Angle/50
//举例：最大倾角为±30°  这里的宏定义为30/50=0.6
#define MAX_ROLL_ANGLE 0.2
//************飞机航向旋转最大速率比**************************
//飞控代码中为旋转的最大速度，1表示按100%转，如果太快可以减小
#define YAW_SPEED 0.5
//****************************************************************
extern u8 mpu_dmp_flag;
extern int user_numbe;
//****************************************************************
void us(u16 time);   
void ms(u16 time);
void delay_us(u32 nus);
void delay_ms(u16 nms);
void JTAG_IO_Init(void);
void uboot(void);
void OS_Init(void);       //系统初始化函数
#endif



