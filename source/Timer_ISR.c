#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "Timer_ISR.h"
#include "SCI_TX.h"
#include <stdio.h>

#define N (10)



/***************************************************************
 *Name:						Timer0_ISR_Thread
 *Function:					period = 0.2ms, pack the data
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
void Timer0_ISR_Thread(void){

	static unsigned char count = 0;
	//GpioDataRegs.GPCTOGGLE.bit.GPIO82 = 1;
	//GpioDataRegs.GPCSET.bit.GPIO82 = 1;

	++count;

	if(count > N){
		testrs422tx();
		count = 0;
	}
	//GpioDataRegs.GPCCLEAR.bit.GPIO82 = 1;
}
/***************************************************************
 *Name:						Timer1_ISR_Thread
 *Function:					priod = 10ms, transmit datat on rs422
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
void Timer1_ISR_Thread(void){

	if(gRS422TxQue.front != gRS422TxQue.rear
			&& ScicRegs.SCIFFTX.bit.TXFFST == 0){
		 ScicRegs.SCIFFTX.bit.TXFFINTCLR = 1;
		 ScicRegs.SCIFFTX.bit.TXFFIENA = 1;
	}
}
