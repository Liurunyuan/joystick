#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "SCI_TX.h"
#include <stdio.h>

GRX422TX gRx422TxVar[20] = {0};
char Rx4225TxBuf[900] = {0};
RS422TXQUE gRS422TxQue = {0};
#define S (3)



int RX422TXEnQueue(char e){
	if((gRS422TxQue.rear + 1) % TXMAXQSIZE == gRS422TxQue.front){
		printf("EnQueue FULL \r\n");
		return 0;
	}

	gRS422TxQue.txBuf[gRS422TxQue.rear] = e;
	gRS422TxQue.rear = (gRS422TxQue.rear + 1) % TXMAXQSIZE;
	return 1;
}
int RX422TXDeQueue(void)
{
	if(gRS422TxQue.front == gRS422TxQue.rear){
		return 0;
	}

	gRS422TxQue.front = (gRS422TxQue.front + 1) % TXMAXQSIZE;
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
	char crcl;
	char crch;
	static unsigned char count = 0;
	static int crc = 0;
	char tmp[3] = {0};
	int lenPosition = 0;
	int total =0;

	if(count == 0){
		if(RX422TXEnQueue(0x5a) == 0){
			printf("·¢ËÍ»º³åÇøFULL\r\n");
			return;
		}
		if(RX422TXEnQueue(0x5a) == 0){
			printf("·¢ËÍ»º³åÇøFULL\r\n");
			return;
		}
		lenPosition = gRS422TxQue.rear;
		if(RX422TXEnQueue(0x05) == 0){
			printf("·¢ËÍ»º³åÇøFULL\r\n");
			return;
		}
	}

	for(i = 0; i < 20; ++i){
		if(gRx422TxVar[i].isTx){
			++total;
			tmp[0] = gRx422TxVar[i].index;
			tmp[1] = gRx422TxVar[i].var.datahl.h;
			tmp[2] = gRx422TxVar[i].var.datahl.l;
			if(RX422TXEnQueue(gRx422TxVar[i].index) == 0){
				printf("·¢ËÍ»º³åÇøFULL\r\n");
				return;
			}
			if(RX422TXEnQueue(gRx422TxVar[i].var.datahl.h) == 0){
				printf("·¢ËÍ»º³åÇøFULL\r\n");
				return;
			}
			if(RX422TXEnQueue(gRx422TxVar[i].var.datahl.l) == 0){
				printf("·¢ËÍ»º³åÇøFULL\r\n");
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
			printf("·¢ËÍ»º³åÇøFULL\r\n");
			return;
		}
		if(RX422TXEnQueue(crcl) == 0){
			printf("·¢ËÍ»º³åÇøFULL\r\n");
			return;
		}
		if(RX422TXEnQueue(0xa5) == 0){
			printf("·¢ËÍ»º³åÇøFULL\r\n");
			return;
		}
		if(RX422TXEnQueue(0xa5) == 0){
			printf("·¢ËÍ»º³åÇøFULL\r\n");
			return;
		}
	}
}
