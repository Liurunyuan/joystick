#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "SCI_TX.h"
#include <stdio.h>

GRX422TX gRx422TxVar[20] = {0};
char Rx4225TxBuf[900] = {0};
RS422TXQUE gRS422TxQue = {0};
#define S (0)


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
		//printf("EnQueue FULL \r\n");
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
/***************************************************************
 *Name:						testrs422tx
 *Function:					pack the data that need to be sent
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
void testrs422tx(void){
	int i;
	char crcl;
	char crch;
	static unsigned char count = 0;
	static int crc = 0;
	char tmp[3] = {0};
	int lenPosition = 0;
	int total =0;

	if(count == 0){
		if(RX422TXEnQueue(0x5a) == 0){
			//printf("���ͻ�����FULL\r\n");
			return;
		}
		if(RX422TXEnQueue(0x5a) == 0){
			//printf("���ͻ�����FULL\r\n");
			return;
		}
		lenPosition = gRS422TxQue.rear;
		if(RX422TXEnQueue(0x05) == 0){
			//printf("���ͻ�����FULL\r\n");
			return;
		}
	}

	for(i = 0; i < 3; ++i){
		if(gRx422TxVar[i].isTx){
			++total;
			gRx422TxVar[i].value = ((AdcRegs.ADCRESULT0) >> 4);

			tmp[0] = gRx422TxVar[i].index;
			tmp[1] = gRx422TxVar[i].value >> 8;
			tmp[2] = gRx422TxVar[i].value;
			if(RX422TXEnQueue(gRx422TxVar[i].index) == 0){
				//printf("���ͻ�����FULL\r\n");
				return;
			}
			if(RX422TXEnQueue(gRx422TxVar[i].value >> 8) == 0){
				//printf("���ͻ�����FULL\r\n");
				return;
			}
			if(RX422TXEnQueue(gRx422TxVar[i].value) == 0){
				//printf("���ͻ�����FULL\r\n");
				return;
			}
			crc = calCrc(crc, tmp, 3);
		}
	}

	gRS422TxQue.txBuf[lenPosition] = total;
	++count;

	if(count > S){

		crcl = (char)crc;
		crch = (char)(crc >> 8);
		crc = 0;
		count = 0;
		if(RX422TXEnQueue(crch) == 0){
			//printf("���ͻ�����FULL\r\n");
			return;
		}
		if(RX422TXEnQueue(crcl) == 0){
			//printf("���ͻ�����FULL\r\n");
			return;
		}
		if(RX422TXEnQueue(0xa5) == 0){
			//printf("���ͻ�����FULL\r\n");
			return;
		}
		if(RX422TXEnQueue(0xa5) == 0){
			//printf("���ͻ�����FULL\r\n");
			return;
		}
	}
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
	int count;
	//when tx queue is empty, that means nothing need to be sent.
	//so disable the tx interrupt.
	/*
	if(RS422TxQueLength() == 0){

		ScicRegs.SCIFFTX.bit.TXFFIENA = 0;//disable the tx interrupt when tx fifo empty
		return;
	}

	for(count = 0; count < 16; ++count){
		if(ScicRegs.SCIFFTX.bit.TXFFST == 16){
			return;
		}
		ScicRegs.SCITXBUF = gRS422TxQue.txBuf[gRS422TxQue.front];

		if(RX422TXDeQueue() == 0){
			//printf("rs422a tx queue is empty\r\n");
			ScicRegs.SCIFFTX.bit.TXFFIENA = 0;
			return;
		}
	}
	*/
	//if(RS422TxQueLength() == 0){
	if(gRS422TxQue.front == gRS422TxQue.rear){

		ScicRegs.SCIFFTX.bit.TXFFIENA = 0;//disable the tx interrupt when tx fifo empty
		return;
	}
	while(ScicRegs.SCIFFTX.bit.TXFFST != 15){
		ScicRegs.SCITXBUF = gRS422TxQue.txBuf[gRS422TxQue.front];
		if(RX422TXDeQueue() == 0){
		//printf("rs422a tx queue is empty\r\n");
			ScicRegs.SCIFFTX.bit.TXFFIENA = 0;
			return;
		}
	}


}
