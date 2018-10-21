#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "SCI_ISR.h"
#include <stdio.h>


/***********globle variable define here***************/
int recievechar[RXBUGLEN]={0};
RS422RXQUE gRS422RxQue = {0};
char rs422rxPack[16];

static void MsgStatusUnpack(int a, int b, int c)
{
	//TODO just an example

}

const functionMsgCodeUnpack msgInterface[] =
{
		0,
		MsgStatusUnpack,
		0,
		0,
		0,
		0
};
/***************************************************************
 *Name:						EnQueue
 *Function:					insert the element in the queue
 *Input:				    received data from SCIC
 *Output:					1 or 0, 1 means insert success, 0 means the queue is full
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
int EnQueue(int e){
	if((gRS422RxQue.rear + 1) % MAXQSIZE == gRS422RxQue.front){
		printf("EnQueue FULL \r\n");
		return 0;
	}

	gRS422RxQue.rxBuff[gRS422RxQue.rear] = e;
	gRS422RxQue.rear = (gRS422RxQue.rear + 1) % MAXQSIZE;
	return 1;
}
//head:0x55
//length:0xXX
//data:.....
//CRC:0xXX
//tail:0xAA
/***************************************************************
 *Name:						DeQueue
 *Function:					remove the element in the queue
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
int DeQueue(void)
{
	if(gRS422RxQue.front == gRS422RxQue.rear){
		return 0;
	}

	gRS422RxQue.front = (gRS422RxQue.front + 1) % MAXQSIZE;
	return 1;
}
/***************************************************************
 *Name:						main
 *Function:
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
int RS422RxQueLength(){
	int length;
	length = (gRS422RxQue.rear - gRS422RxQue.front + MAXQSIZE) % MAXQSIZE;
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
void RS422A_receive(void)
{
	//当接收fifo不为空时
	while(ScicRegs.SCIFFRX.bit.RXFFST != 0){// rs422 rx fifo is not empty
		if(EnQueue(ScicRegs.SCIRXBUF.all) == 0){
			printf("接收缓冲区FULL\r\n");
			//TODO update error msg
		}
	}
}
/***************************************************************
 *Name:						CalCrc
 *Function:
 *Input:				    rs422 rx data
 *Output:					int, should be zero
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
int CalCrc(int crc, const char *buf, int len)
{
	unsigned int byte;
	unsigned char k;
	unsigned short ACC, TOPBIT;
	unsigned short remainder = crc;
	TOPBIT = 0x8000;
	for (byte = 0; byte < len; ++byte)
	{
		ACC = buf[byte];
		remainder ^= (ACC <<8);
		for(k = 8; k > 0; --k)
		{
			if (remainder & TOPBIT)
			{
				remainder = (remainder << 1) ^0x8005;
			}
			else
			{
				remainder = (remainder << 1);
			}
		}
	}
	remainder = remainder^0x0000;
	return remainder;
}
/***************************************************************
 *Name:						UnpackRS422A
 *Function:
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
void UnpackRS422A(void)
{
	static int status = FindHead;
	int i;
	int length;
	int tail;

	printf("front = %d\r\n",gRS422RxQue.front);
	printf("rear = %d\r\n",gRS422RxQue.rear);
	switch (status)
	{
	case FindHead:
		while(gRS422RxQue.rxBuff[gRS422RxQue.front] != HEAD || gRS422RxQue.front == gRS422RxQue.rear){
			if(DeQueue() == 0){
				printf("接收缓冲区为空\r\n");
				//没有包可以解，接收缓冲区为空
				return;
			}
		}

		status = CheckLength;
	case CheckLength:
		if(gRS422RxQue.rxBuff[(gRS422RxQue.front + 1) % MAXQSIZE] < RS422RxQueLength())
		{
			printf("check length ------------success \r\n");
			length = gRS422RxQue.rxBuff[(gRS422RxQue.front + 1) % MAXQSIZE];
			for(i = 0; i < length; ++i){
				rs422rxPack[i] = gRS422RxQue.rxBuff[(gRS422RxQue.front + i) % MAXQSIZE];
			}
			tail = rs422rxPack[length - 1];
			status = CheckTail;
		}
		else
		{
			status = FindHead;
			printf("check length ------------failed!!!!!!, waiting for the data \r\n");
			break;
		}
	case CheckTail:
		if(tail == TAIL)
		{
			printf("check tail --------------success \r\n");
			status = CheckCRC;
		}
		else
		{
			gRS422RxQue.front = (gRS422RxQue.front + 1) % MAXQSIZE;
			status = FindHead;
			printf("check tail ------------failed!!!!!!, move front to next one, continue find head...... \r\n");
			break;
		}
	case CheckCRC:
		// TODO CRC校验
		if(CalCrc(0, rs422rxPack + 2, length - 3 ) == 0){
			printf("crc check ===============success \r\n");
			status = Unpack;
		}
		else
		{
			gRS422RxQue.front = (gRS422RxQue.front + 1) % MAXQSIZE;
			status = FindHead;
			printf("crc check ------------failed!!!!!!, move front to next one, continue find head...... \r\n");
			break;
		}

		status = Unpack;
	case Unpack:
		//TODO
		status = UpdateHead;
	case UpdateHead:
		gRS422RxQue.front = (gRS422RxQue.front  + gRS422RxQue.rxBuff[(gRS422RxQue.front + 1) % MAXQSIZE]) % MAXQSIZE;
		status = FindHead;
		printf("Success!!!, Got the profile data \r\n");
		break;
	default:
		status = FindHead;
		break;
	}
}
