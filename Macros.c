#include "Macros.h"
#include "tm4c123gh6pm.h"

void Motor_Up(void)
{
	GPIO_PORTA_DATA_R  |= (1<<3) ;
}

void Motor_Down(void){
	GPIO_PORTA_DATA_R  |= (1<<2) ;
}

void Motor_Off(void){
	GPIO_PORTA_DATA_R  = 0X0 ;
}

void Leds_Off(void){
		GPIO_PORTF_DATA_R = 0X0 ;						
}

