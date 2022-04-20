#include "timer.h"

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void delay(int ms)

{

	int i;

	for(; ms>0 ;ms--){

		for(i =0; i<3195;i++);

	}

}

long map1(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void servo_write(uint8_t angle)
	{
	
	if(angle<0){angle=0;}
	if(angle>180){angle=180;}
	
	
	TIM2->CCR1 = map1(angle,0,180,2500,7000);
	}
/*
int map(int st1, int fn1, int st2, int fn2, int value)
{
    return (1.0*(value-st1))/((fn1-st1)*1.0) * (fn2-st2)+st2;
}
*/

	/*
void servo_write(float angle){
		htim2.Instance->CCR1 = map(0,180,50,250,angle);
}
*/
void servo_sweep(void){
		for(int i = 0; i <= 180; i++){
			servo_write(i);
			HAL_Delay(10);
		}
		
		for(int i = 180; i >= 0; i--){
			servo_write(i);
			HAL_Delay(10);
		}
}

