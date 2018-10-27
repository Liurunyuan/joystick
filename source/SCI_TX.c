#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "SCI_TX.h"
#include <stdio.h>

GRX422TX gRx422TxVar[5] = {0};
char Rx4225TxBuf[128] = {0};
RS422TXQUE gRS422TxQue = {0};
#define S (2)



int RX422TXEnQueue(int e){
	if((gRS422TxQue.rear + 1) % MAXQSIZE == gRS422TxQue.front){
		printf("EnQueue FULL \r\n");
		return 0;
	}

	gRS422TxQue.txBuf[gRS422TxQue.rear] = e;
	gRS422TxQue.rear = (gRS422TxQue.rear + 1) % MAXQSIZE;
	return 1;
}
int RX422TXDeQueue(void)
{
	if(gRS422TxQue.front == gRS422TxQue.rear){
		return 0;
	}

	gRS422TxQue.front = (gRS422TxQue.front + 1) % MAXQSIZE;
	return 1;
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
/***************************************************************/
void testrs422tx(void){
	int i;
	static unsigned char count = 0;
	static int crc = 0;
	static int txindex = 0;

	if(count == 0){
		Rx4225TxBuf[txindex] = 0x5a;
		Rx4225TxBuf[txindex + 1] = 0x5a;
		Rx4225TxBuf[txindex + 2] = 0x05;
		if(RX422TXEnQueue(0x5a) == 0){
			printf("·¢ËÍ»º³åÇøFULL\r\n");
		}
		if(RX422TXEnQueue(0x5a) == 0){
			printf("·¢ËÍ»º³åÇøFULL\r\n");
		}
		if(RX422TXEnQueue(0x05) == 0){
			printf("·¢ËÍ»º³åÇøFULL\r\n");
		}
	}

	for(i = 0; i < 3; ++i){
		if(gRx422TxVar[i].isTx){

			Rx4225TxBuf[txindex*3 + 3 + i] = gRx422TxVar[i].index;
			Rx4225TxBuf[txindex*3 + 3 + i + 1] = gRx422TxVar[i].var.datahl.l;
			Rx4225TxBuf[txindex*3 + 3 + i + 2] = gRx422TxVar[i].var.datahl.h;
			if(RX422TXEnQueue(gRx422TxVar[i].index) == 0){
				printf("·¢ËÍ»º³åÇøFULL\r\n");
			}
			if(RX422TXEnQueue(gRx422TxVar[i].var.datahl.h) == 0){
				printf("·¢ËÍ»º³åÇøFULL\r\n");
			}
			if(RX422TXEnQueue(gRx422TxVar[i].var.datahl.l) == 0){
				printf("·¢ËÍ»º³åÇøFULL\r\n");
			}
		}
	}

	crc = calCrc(crc, Rx4225TxBuf + txindex*3 + 3, 3);
	++count;
	++txindex;
	if(count > S){

		Rx4225TxBuf[3+3*count + 1] = (char)crc;
		Rx4225TxBuf[3+3*count] = (char)(crc >> 8);
		Rx4225TxBuf[3+3*count + 2] = 0xa5;
		Rx4225TxBuf[3+3*count + 3] = 0xa5;
		if(RX422TXEnQueue((char)crc) == 0){
			printf("·¢ËÍ»º³åÇøFULL\r\n");
		}
		if(RX422TXEnQueue((char)(crc >> 8)) == 0){
			printf("·¢ËÍ»º³åÇøFULL\r\n");
		}
		if(RX422TXEnQueue(0xa5) == 0){
			printf("·¢ËÍ»º³åÇøFULL\r\n");
		}
		if(RX422TXEnQueue(0xa5) == 0){
			printf("·¢ËÍ»º³åÇøFULL\r\n");
		}
		crc = 0;
		count = 0;
		txindex = 0;
	}
}
