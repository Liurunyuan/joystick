#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "SCI_TX.h"
#include <stdio.h>

GRX422TX gRx422TxVar[TOTAL_TX_VAR] = {0};
Uint16 gRx422TxEnableFlag[TOTAL_TX_VAR] = {0};
RS422TXQUE gRS422TxQue = {0};
#define S (1)


/***************************************************************
 *Name:						RX422TXEnQueue
 *Function:
 *Input:				    char e, come from tx queue
 *Output:					1 or 0, 1 means success, 0 means the queue is full already
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
int RX422TXEnQueue(char e){
	if((gRS422TxQue.rear + 1) % TXMAXQSIZE == gRS422TxQue.front){
		asm ("      ESTOP0");
		return 0;
	}

	gRS422TxQue.txBuf[gRS422TxQue.rear] = e;
	gRS422TxQue.rear = (gRS422TxQue.rear + 1) % TXMAXQSIZE;
	return 1;
}
/***************************************************************
 *Name:						RX422TXDeQueue
 *Function:
 *Input:				    none
 *Output:					1 or 0, 1 means success, 0 means the queue is empty already
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
int RX422TXDeQueue(void)
{
	if(gRS422TxQue.front == gRS422TxQue.rear){
		return 0;
	}

	gRS422TxQue.front = (gRS422TxQue.front + 1) % TXMAXQSIZE;
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
int RS422TxQueLength(){
	int length;
	length = (gRS422TxQue.rear - gRS422TxQue.front + TXMAXQSIZE) % TXMAXQSIZE;
	return length;
}
/***************************************************************
 *Name:						CalCrc
 *Function:
 *Input:				    rs422 rx data
 *Output:					int, should be zero
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
int calCrc(int crc, const char *buf, int len)
{
	int x;
	int i;

	for(i = 0; i < len; ++i){
		x = ((crc >> 8) ^ buf[i]) & 0xff;
		x ^= x >> 4;
		crc = (crc << 8) ^ (x  << 12) ^ (x << 5) ^ x;
		crc &= 0xffff;
	}
	return crc;
}
/**************************************************************
 *Name:		   updateTxEnableFlag
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018年11月6日下午7:43:55
 **************************************************************/
void updateTxEnableFlag(void) {

	int i;
	for (i = 0; i < TOTAL_TX_VAR; ++i) {
		gRx422TxVar[i].isTx = gRx422TxEnableFlag[i];
	}
}

/***************************************************************
 *Name:						testrs422tx
 *Function:					pack the data that need to be sent
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
void testrs422tx(void){

	//TODO need do some test, because we sync the tx enable flag here
	int i;
	char crcl;
	char crch;
	static unsigned char count = 0;
	static int crc = 0;
	char tmp[3] = {0};
	int lenPosition = 0;
	Uint16 total =0;

	if(count == 0){
		if(RX422TXEnQueue(0x5a) == 0){
			asm ("      ESTOP0");
			return;
		}
		if(RX422TXEnQueue(0x5a) == 0){
			asm ("      ESTOP0");
			return;
		}
		lenPosition = gRS422TxQue.rear;
		if(RX422TXEnQueue(0x05) == 0){
			asm ("      ESTOP0");
			return;
		}

		updateTxEnableFlag();
	}

	for(i = 0; i < TOTAL_TX_VAR; ++i){
		if(gRx422TxVar[i].isTx){
			++total;
			gRx422TxVar[i].value = ((AdcRegs.ADCRESULT0) >> 4);

			tmp[0] = gRx422TxVar[i].index;
			tmp[1] = gRx422TxVar[i].value >> 8;
			tmp[2] = gRx422TxVar[i].value;
			if(RX422TXEnQueue(gRx422TxVar[i].index) == 0){
				asm ("      ESTOP0");
				return;
			}
			if(RX422TXEnQueue(gRx422TxVar[i].value >> 8) == 0){
				asm ("      ESTOP0");
				return;
			}
			if(RX422TXEnQueue(gRx422TxVar[i].value) == 0){
				asm ("      ESTOP0");
				return;
			}
			crc = calCrc(crc, tmp, 3);
		}
	}

	if(count == 0){
		gRS422TxQue.txBuf[lenPosition] = total * (S + 1);//timer0 interrupt isr can not be interrupted by TX, so we can set length value here
	}

	++count;

	if(count > S){

		crcl = (char)crc;
		crch = (char)(crc >> 8);
		crc = 0;
		count = 0;
		if(RX422TXEnQueue(crch) == 0){
			asm ("      ESTOP0");
			return;
		}
		if(RX422TXEnQueue(crcl) == 0){
			asm ("      ESTOP0");
			return;
		}
		if(RX422TXEnQueue(0xa5) == 0){
			asm ("      ESTOP0");
			return;
		}
		if(RX422TXEnQueue(0xa5) == 0){
			asm ("      ESTOP0");
			return;
		}
	}
}
/**************************************************************
 *Name:		   ScibTxByte
 *Comment:
 *Input:	   one byte to send by sci B
 *Output:	   none
 *Author:	   Simon
 *Date:		   2018年11月12日下午9:47:37
 **************************************************************/
void ScibTxByte(Uint16 t){

	ScibRegs.SCITXBUF = t;
}
/**************************************************************
 *Name:		   ScicTxByte
 *Comment:
 *Input:	   one byte to send by sci C
 *Output:	   none
 *Author:	   Simon
 *Date:		   2018年11月12日下午9:47:37
 **************************************************************/
void ScicTxByte(Uint16 t){

	ScicRegs.SCITXBUF = t;
}

/***************************************************************
 *Name:						rs422 tx interrupt isr
 *Function:			  none
 *Input:				  none
 *Output:					none
 *Author:					Simon
 *Date:						2018.11.3
 ****************************************************************/
void RS422A_Transmit(void){

	if(gRS422TxQue.front == gRS422TxQue.rear){

		ScicRegs.SCIFFTX.bit.TXFFIENA = 0;//disable the tx interrupt when tx fifo empty
		return;
	}

	while((ScicRegs.SCIFFTX.bit.TXFFST != 16)
				&& (ScibRegs.SCIFFTX.bit.TXFFST != 16)){
		ScibTxByte(gRS422TxQue.txBuf[gRS422TxQue.front]);
		//ScicTxByte(gRS422TxQue.txBuf[gRS422TxQue.front]);//printf by Scic

		if(RX422TXDeQueue() == 0){
			ScicRegs.SCIFFTX.bit.TXFFIENA = 0;
			return;
		}
	}
}
