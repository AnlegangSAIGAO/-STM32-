#include "pbdata.h"
float tmp_num=0;
float Bat_adc=0.0f;
u8 Bat_n=0;      //电量百分比
u8 tmp_value=0;  //温度值
//**********************配置系统时钟*********************************
void ADC_RCC(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ADC PA1端口时钟配置
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);//打开ADC功能的时钟
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);                   //对ADC专用时钟进行6分频12MHz
}
//**********************配置GPIO管脚*********************************
void ADC_GPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  //ADC管脚配置PA4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;   //模拟量输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}	
//**************************ADC配置函数**********************************
void ADC_Configuration(void)
{
  ADC_InitTypeDef ADC_InitStructure;                            //定义结构体变量
	ADC_DeInit(ADC1);                                             //将外设ADC1的全部寄存器重设为缺省值
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;            //独立模式
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;                 //单通道扫描模式
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;           //单次循环，只采集一次
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//采用软件触发模式
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;        //采用右对齐模式
  ADC_InitStructure.ADC_NbrOfChannel = 2;                       //只用了4个通道所以赋值为2
  ADC_Init(ADC1, &ADC_InitStructure);                           //ADC初始化
	ADC_TempSensorVrefintCmd(DISABLE);                            //关闭内部温度传感器
	ADC_Cmd(ADC1, ENABLE);	                                      //使能指定的ADC1
	ADC_ResetCalibration(ADC1);	                                  //重置指定的ADC1的复位寄存器
  while(ADC_GetResetCalibrationStatus(ADC1));	                  //获取ADC1重置校准寄存器的状态，设置状态为等待
	ADC_StartCalibration(ADC1);	 
	while(ADC_GetCalibrationStatus(ADC1));		                    //获取指定ADC1的校准程序，设置状态为等待
}
//*********************ADC初始化函数**************************************
void ADC_Check_Init(void)
{
  ADC_RCC(); 
  ADC_GPIO();	
	ADC_Configuration();
}
//*****************获得某个通道的值**************************************
u16 T_Get_Adc(u8 ch)   
{ 
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	  			     
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));
	return ADC_GetConversionValue(ADC1);	
}
//*****************获取times次的平均值**********************************
u16 T_Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=T_Get_Adc(ch);
		delay_ms(5);
	}
	return temp_val/times;
} 
//************电池电量ADC检测****************
void BAT_ADC(void)
{
	u32 ad=0;
	ad=T_Get_Adc_Average(ADC_Channel_4,10);
  Bat_adc=(float)ad*(3.3/4096);  //此值为电池电压分压值，电池电压/268*68	
	Bat_adc=Bat_adc*3.9412;        //倒推出电池电压为 检测值/68*268  相当于检测值*268/68
	pwm_adc=BAT_MAX/Bat_adc;       //用于补偿电池降压后PWM的损耗
	Bat_n=(u8)(100/(BAT_MAX-BAT_MIN)*(Bat_adc-BAT_MIN));//转换为百分比
	//限制幅度
	if(Bat_n<=0)
	{
	 Bat_n=0;
	}
	if(Bat_n>=100)
	{
		Bat_n=100;
	}
}
//*********************温度检测函数**************************************
void Temperature_check(void)
{
  u32 ad=0;                    //初始化ad变量
	ad=T_Get_Adc_Average(ADC_Channel_16,10);  //通道16为温度采样通道，取10次平均值	 	
  tmp_num=(float)ad*(3.3/4096);
	//25℃对应 1430mV 
	//变化1℃ 对应电压变化4.3mV 
	tmp_value=(u8)(((1.43-tmp_num)/0.0043+25)/1.32);     //实际温度比测量温度低，所以添加修正系数1.32
	
}



































