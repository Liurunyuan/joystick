#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "Timer_ISR.h"
#include <stdio.h>


void Timer0_ISR_Thread(void){

	int i = 0;
	for(i = 0; i < 100; ++i){
		++i;
	}
}


void Timer1_ISR_Thread(void){
	//printf("tiemr0 interrupt function\r\n");
	int i = 0;
	for(i = 0; i < 100; ++i){
		++i;
	}
}
