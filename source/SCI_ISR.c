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
	int x;
	int i;

	for(i = 0; i < len; ++i){
		x = ((crc >> 8) ^ buf[i]) & 0xff;
		x ^= x >> 4;
		crc = (crc << 8) ^ (x  << 12) ^ (x << 5) ^ 5;
		crc &= 0xffff;
	}
	return crc;
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
/***************************************************************
 *Name:						findhead
 *Function:
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.25
 ****************************************************************/
int findhead(void){
	char head1;
	char head2;
	while(1){

		head1 = gRS422RxQue.rxBuff[gRS422RxQue.front];
		head2 = gRS422RxQue.rxBuff[(gRS422RxQue.front + 1) % MAXQSIZE];

		if(head1 == HEAD1 && head2 == HEAD2){
			return SUCCESS;
		}

		if(DeQueue() == 0){
			//printf("接收缓冲区为空\r\n");
			return FAIL;
		}

	}
}
/***************************************************************
 *Name:						findtail
 *Function:
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.25
 ****************************************************************/
int findtail(int len){
	char tail1;
	char tail2;

	tail1 = gRS422RxQue.rxBuff[(gRS422RxQue.front + len - 1) % MAXQSIZE];
	tail2 = gRS422RxQue.rxBuff[(gRS422RxQue.front + len - 2) % MAXQSIZE];

	if(tail1 == TAIL1 && tail2 == TAIL2){
		return SUCCESS;
	}
	else{
		return FAIL;
	}
}
int checklength(void){
	if(gRS422RxQue.rxBuff[(gRS422RxQue.front + 2) % MAXQSIZE] < RS422RxQueLength()){
		return SUCCESS;
	}
	else
	{
		return FAIL;
	}
}

void saveprofile(int len){
	int i;

	for(i = 0; i < len; ++i){
		rs422rxPack[i] = gRS422RxQue.rxBuff[(gRS422RxQue.front + i) % MAXQSIZE];
	}
}
void unpack(){

}
void updatehead(int len){
	gRS422RxQue.front = (gRS422RxQue.front + len) % MAXQSIZE;
}

void UnpackRS422ANew(void){
	int length;

	if(findhead() == FAIL){
		printf("接收缓冲区为空\r\n");
		return;
	}
	else{
		printf("成功找到包头\r\n");
	}

	if(checklength() == FAIL){
		printf("缓冲区长度不够， len received =%d\r\n",gRS422RxQue.rxBuff[(gRS422RxQue.front + 2) % MAXQSIZE] );
		printf("缓冲区长度不够， len calculate =%d\r\n",RS422RxQueLength());
		return;
	}
	else{
		printf("成功：缓冲区长度满足解包条件\r\n");
	}

	length = gRS422RxQue.rxBuff[(gRS422RxQue.front + 2) % MAXQSIZE];

	if(findtail(length) == FAIL){
		printf("失败：包尾没有对应\r\n");
		if(DeQueue() == 0){
			printf("接收缓冲区为空\r\n");
		}

		return;
	}
	else{
		printf("成功找到包尾\r\n");
	}

	saveprofile(length);

	if(CalCrc(0, rs422rxPack + 3, length - 5) != 0){
		if(DeQueue() == 0){
			printf("接收缓冲区为空\r\n");
		}
		printf("失败：没有通过CRC校验\r\n");
		return;
	}
	else{
		printf("成功通过CRC校验\r\n");
	}

	unpack();
	updatehead(length);
	printf("update the front position----------------------------\r\n");
}
