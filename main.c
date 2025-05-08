#include "pbdata.h"
int main(void)
{	
 uboot();
 OS_Init(); 
	Moto_Ctrl(0,1); //作用到1号电机
	Moto_Ctrl(0,2); //作用到2号电机
	Moto_Ctrl(0,3); //作用到3号电机
	Moto_Ctrl(0,4); //作用到4号电机
	Plane_status=0;
  Oil_n=0;	
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);	
 while(1)
 {
	 BAT_ADC();  //电量实时检测
	 MSG_Recieve();    
//	 UI(); 		
	}
}


