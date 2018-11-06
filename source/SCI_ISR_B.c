#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "SCI_ISR_B.h"
#include "SCI_TX.h"
#include <stdio.h>


/***********globle variable define here***************/
RS422RXQUEB gRS422RxQueB = {0};

/***************************************************************
 *Name:						EnQueue
 *Function:					insert the element in the queue
 *Input:				    received data from SCIC
 *Output:					1 or 0, 1 means insert success, 0 means the queue is full
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
inline int EnQueue_B(int e){
	if((gRS422RxQueB.rear + 1) % MAXQSIZE_B == gRS422RxQueB.front){
		//printf("EnQueue FULL \r\n");
		return 0;
	}

	gRS422RxQueB.rxBuff[gRS422RxQueB.rear] = e;
	gRS422RxQueB.rear = (gRS422RxQueB.rear + 1) % MAXQSIZE_B;
	return 1;
}
/***************************************************************
 *Name:						DeQueue
 *Function:					remove the element in the queue
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
int DeQueue_B(void){
	if(gRS422RxQueB.front == gRS422RxQueB.rear){
		return 0;
	}

	gRS422RxQueB.front = (gRS422RxQueB.front + 1) % MAXQSIZE_B;
	return 1;
}
/***************************************************************
 *Name:						RS422RxQueLength
 *Function:
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
int RS422RxQueLengthB(){
	int length;
	length = (gRS422RxQueB.rear - gRS422RxQueB.front + MAXQSIZE_B) % MAXQSIZE_B;
	return length;
}
/***************************************************************
 *Name:						RS422A_receive
 *Function:
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
void RS422B_receive(void){
	while(ScibRegs.SCIFFRX.bit.RXFFST != 0){// rs422b rx fifo is not empty
		if(EnQueue_B(ScibRegs.SCIRXBUF.all) == 0){
			//printf("RS422B rx queue full\r\n");
			//TODO update error msg
		}
	}
}
/***************************************************************
 *Name:						updatehead
 *Function:					move the front head to another position
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.27
 ****************************************************************/
void updateheadB(int len){
	gRS422RxQueB.front = (gRS422RxQueB.front + len) % MAXQSIZE_B;
}
