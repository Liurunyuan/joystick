#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "SCI_ISR.h"


/***********globle variable define here***************/
int recievechar[RXBUGLEN]={0};
RS422RXQUE gRS422RxQue = {0};

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

int EnQueue(int e){
	if((gRS422RxQue.rear + 1)%MAXQSIZE == gRS422RxQue.front){
		return 0;
	}

	gRS422RxQue.rxBuff[gRS422RxQue.rear] = e;
	gRS422RxQue.rear = (gRS422RxQue.rear + 1)%MAXQSIZE;
	return 1;
}
//head:0x55
//length:0xXX
//data:.....
//CRC:0xXX
//tail:0xAA
int DeQueue()
{
	if(gRS422RxQue.front == gRS422RxQue.rear){
		return 0;
	}

	gRS422RxQue.front = (gRS422RxQue.front + 1)%MAXQSIZE;
	return 1;
}
/*************************************/
void RS422A_receive(void)
{
	//当接收fifo不为空时
	while(ScicRegs.SCIFFRX.bit.RXFFST != 0){// rs422 rx fifo is not empty
		if(EnQueue(ScicRegs.SCIRXBUF.all) == 0){
			//TODO update error msg
		}
	}
}

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


void UnpackRS422A(void)
{
	static int status = FindHead;
	switch (status)
	{
	case FindHead:
		while(gRS422RxQue.rxBuff[gRS422RxQue.front] != HEAD){
			if(DeQueue() == 0){
				//没有包可以解，接收缓冲区为空
				return;
			}
		}
		status = CheckLength;
	case CheckLength:
		if((gRS422RxQue.front + gRS422RxQue.rxBuff[gRS422RxQue.front + 1] + 3) < gRS422RxQue.rear)//接收缓冲区内数据长度大于一整包的长度
		{
			status = CheckTail;
		}
		else
		{
			break;
		}
	case CheckTail:
		if(gRS422RxQue.rxBuff[gRS422RxQue.front + gRS422RxQue.rxBuff[gRS422RxQue.front + 1] + 3] == TAIL)
		{
			status = CheckCRC;
		}
		else
		{
			gRS422RxQue.front += 1;
			status = FindHead;
			break;
		}
	case CheckCRC:
		// TODO CRC校验
		if(1){
			if(CalCrc(0, (gRS422RxQue.rxBuff)+2, gRS422RxQue.rxBuff[gRS422RxQue.front + 2]) == 0)
			status = Unpack;
		}
		else
		{
			gRS422RxQue.front += 1;
			status = FindHead;
			break;
		}
	case Unpack:
		//TODO
		status = UpdateHead;
	case UpdateHead:
		gRS422RxQue.front = gRS422RxQue.front + gRS422RxQue.rxBuff[gRS422RxQue.front + 1] + 4;
		status = FindHead;
		break;
	default:
		status = FindHead;
		break;
	}
	/*
	while(gRS422RxQue.rxBuff[gRS422RxQue.front] != HEAD){
		if(DeQueue() == 0){
			//没有包可以解，接收缓冲区为空
			return;
		}
	}

	if((gRS422RxQue.front + gRS422RxQue.rxBuff[gRS422RxQue.front + 1] + 3) < gRS422RxQue.rear)//接收缓冲区内数据长度大于一整包的长度
	{
		if(gRS422RxQue.rxBuff[gRS422RxQue.front + gRS422RxQue.rxBuff[gRS422RxQue.front + 1] + 3] == TAIL){

		}
	}
	else
	{

	}
	*/
}
