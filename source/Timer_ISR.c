#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "Timer_ISR.h"
#include "SCI_ISR.h"
#include "SCI_TX.h"
#include <stdio.h>

#define N (10)
#define RS422STATUSCHECK (1000)



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

	++count;

	if(count > N){
		//testrs422tx();
		count = 0;
	}
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
	static Uint16 count = 0;

	if(count >= RS422STATUSCHECK){
		printf(">>>>>>>>>Check RS422 channel\r\n");
		count = 0;

		if(gRS422Status.rs422A == 0 && gRS422Status.rs422B == 0){
			printf(">>>>>>>>>>RS422A and RS422B both failed to connect\r\n");
			return;
		}

		if(gRS422Status.rs422CurrentChannel == RS422_CHANNEL_A){
			if(gRS422Status.rs422A){
				gRS422Status.rs422A = 0;
			}
			else{
				//TODO need to switch rs422A to rs422b.
				printf(">>>>>>>>>>Switch to RS422B channel\r\n");
				gRS422Status.rs422CurrentChannel = RS422_CHANNEL_B;
			}
		}
		else if(gRS422Status.rs422CurrentChannel == RS422_CHANNEL_B){
			if(gRS422Status.rs422B){
				gRS422Status.rs422B = 0;
			}
			else{
				//TODO need to switch rs422B to rs422A.
				gRS422Status.rs422CurrentChannel = RS422_CHANNEL_A;
			}
		}
		else{
			printf(">>>>>>>>>>>>>>>>>>>>Unknow RS422 channel\r\n");
		}
	}

	++count;

	if(gRS422TxQue.front != gRS422TxQue.rear
			&& ScicRegs.SCIFFTX.bit.TXFFST == 0){
		 ScicRegs.SCIFFTX.bit.TXFFINTCLR = 1;
		 ScicRegs.SCIFFTX.bit.TXFFIENA = 1;
	}
}
