#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "SCI_ISR.h"
#include "SCI_TX.h"
#include <stdio.h>


/***********globle variable define here***************/
int recievechar[RXBUGLEN]={0};
RS422RXQUE gRS422RxQue = {0};
char rs422rxPack[16];

/***************************************************************
 *Name:						MsgStatusUnpack
 *Function:
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.25
 ****************************************************************/
static void MsgStatusUnpack(VAR16 a, int b, int c)
{
	//TODO just an example


}
/***************************************************************
 *Name:						WaveCommand
 *Function:
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.25
 ****************************************************************/
static void WaveCommand(VAR16 a, int b, int c)
{
	//TODO just an example
	int i;
	for(i = 0; i < 16; ++i){
		//unpack bit information
		if((a.value & (0x0001 << i)) >> i){
			//do something
			gRx422TxVar[i].isTx = 1;
		}
		else{
			//do something
			gRx422TxVar[i].isTx = 0;
		}
	}

}
/***************************************************************
 *Name:						WaveCommand
 *Function:
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.25
 ****************************************************************/
const functionMsgCodeUnpack msgInterface[] =
{
		0,
		MsgStatusUnpack,
		WaveCommand,
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
		crc = (crc << 8) ^ (x  << 12) ^ (x << 5) ^ x;
		crc &= 0xffff;
	}
	return crc;
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
/***************************************************************
 *Name:						checklength
 *Function:
 *Input:				    none
 *Output:					1 or 0, 1 means success, 0 means failed
 *Author:					Simon
 *Date:						2018.10.25
 ****************************************************************/
int checklength(void){
	if((gRS422RxQue.rxBuff[(gRS422RxQue.front + 2) % MAXQSIZE] * 3 + 7) < RS422RxQueLength()){
		return SUCCESS;
	}
	else
	{
		return FAIL;
	}
}
/***************************************************************
 *Name:						saveprofile
 *Function:
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.27
 ****************************************************************/
void saveprofile(int len){
	int i;

	for(i = 0; i < len; ++i){
		rs422rxPack[i] = gRS422RxQue.rxBuff[(gRS422RxQue.front + i) % MAXQSIZE];
	}
}
/***************************************************************
 *Name:						unpack
 *Function:					unpack profile data
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.27
 ****************************************************************/
void unpack(int len){
	int i;
	int msgCode;
	VAR16 var16;


	for(i = 0; i < len; ++i){

		msgCode = rs422rxPack[3 + 3*i];
		var16.datahl.h = rs422rxPack[3 + 3*i + 1];
		var16.datahl.l = rs422rxPack[3 + 3*i + 2];
		var16.value = var16.datahl.l + (var16.datahl.h << 8);

		if(msgCode < (sizeof(msgInterface)/sizeof(msgInterface[0]))){
			printf("msgCode = %d\r\n",msgCode);
			if(msgInterface[msgCode]){
				msgInterface[msgCode](var16,0,0);
			}
		}
		else{
			printf("unpack msg code is out of range\r\n");
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
void updatehead(int len){
	gRS422RxQue.front = (gRS422RxQue.front + len) % MAXQSIZE;
}
/***************************************************************
 *Name:						UnpackRS422ANew
 *Function:					unpack the hole data package
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.27
 ****************************************************************/
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
		printf("缓冲区长度不够， len received =%d\r\n",gRS422RxQue.rxBuff[(gRS422RxQue.front + 2) % MAXQSIZE] * 3 + 7 );
		printf("缓冲区长度不够， len calculate =%d\r\n",RS422RxQueLength());
		return;
	}
	else{
		printf("成功：缓冲区长度满足解包条件\r\n");
	}

	length = gRS422RxQue.rxBuff[(gRS422RxQue.front + 2) % MAXQSIZE] * 3 + 7;

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

	unpack(gRS422RxQue.rxBuff[(gRS422RxQue.front + 2) % MAXQSIZE]);
	updatehead(length);
	printf("update the front position----------------------------\r\n");
}
/***************************************************************
 *Name:						testwithlabview
 *Function:					just a test function to test with Labview
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.27
 ****************************************************************/
void testwithlabview(){

	int i;
	static int f = 0;
	int crc;
	static int data = 0;
	char buf[19]={
				0x55,
				0x5a,
				0x04,
				0x01,
				0x00,
				0x00,
				0x02,
				0x00,
				0x00,
				0x03,
				0x00,
				0x00,
				0x04,
				0x00,
				0x00,
				0xd7,
				0x32,
				0xbb,
				0xaa
	};
	buf[5] = (char)data;
	buf[8] = (char)(100 - data);
	if(f == 0){
		++data;
	}
	else{
		--data;
	}

	if(data == 100){
		//data = 0;
		f = 1;
	}
	if(data ==0){
		f = 0;
	}

	crc = CalCrc(0, buf+3, 12);
	buf[16] = (char)crc;
	buf[15] = (char)(crc >> 8);
	for(i = 0; i < 19; ++i){
		while(ScicRegs.SCIFFTX.bit.TXFFST != 0){

		}
		ScicRegs.SCITXBUF = buf[i];

	}
}
