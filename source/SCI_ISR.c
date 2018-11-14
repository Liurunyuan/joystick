#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "SCI_ISR.h"
#include "SCI_TX.h"
#include <stdio.h>

#define UNIT_LEN (3) 			//0x00(index 1 byte) + 0x00(high 8 bit) + 0x00(low 8 bit)
#define EXTRA_LEN  (9)          //head(2 bytes) + length(1 byte) + serial number(2 bytes) + crc(2 bytes) + tail(2 bytes)
#define OFFSET (5) 				//head(2 bytes) + length(1 byte) + serial number(2 bytes)
#define WAVE_AMOUNT (16)
#define ENABLE_TX (1)
#define DISABLE_TX (0)

#define COMPARE_A_AND_B (0)
/***********globle variable define here***************/

int recievechar[RXBUGLEN]={0};
RS422RXQUE gRS422RxQue = {0};
RS422RXQUE gRS422RxQueB = {0};
char rs422rxPack[16];
RS422STATUS gRS422Status = {0};

/***************************************************************
 *Name:						MsgStatusUnpack
 *Function:
 *Input:          			VAR16,int, int
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.25
 ****************************************************************/
static void MsgStatusUnpack(VAR16 a, int b, int c) {
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
static void WaveCommand(VAR16 a, int b, int c) {
	int i;

	for(i = 0; i < WAVE_AMOUNT; ++i){
		//unpack bit information
		if((a.value & (0x0001 << i)) >> i){
			gRx422TxVar[i].isTx = ENABLE_TX;
		}
		else{
			gRx422TxVar[i].isTx = DISABLE_TX;
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
const functionMsgCodeUnpack msgInterface[] = {
		0,
		MsgStatusUnpack,
		WaveCommand,
		0,
		0,
		0
};
/***************************************************************
 *Name:						EnQueueB
 *Function:					insert the element in the queue
 *Input:				    received data from SCIC
 *Output:					1 or 0, 1 means insert success, 0 means the queue is full
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
int EnQueueB(int e, RS422RXQUE *RS422RxQue){
	//TODO copy this function,  just in case two threads call this function

	//TODO, when queue is full, override the queue, do not return fail
	if((RS422RxQue->rear + 1) % MAXQSIZE == RS422RxQue->front){
		RS422RxQue->front += 1;
		//printf("EnQueue FULL \r\n");
		//return 0;
	}

	RS422RxQue->rxBuff[RS422RxQue->rear] = e;
	RS422RxQue->rear = (RS422RxQue->rear + 1) % MAXQSIZE;
	return 1;
}
/***************************************************************
 *Name:						EnQueue
 *Function:					insert the element in the queue
 *Input:				    received data from SCIC
 *Output:					1 or 0, 1 means insert success, 0 means the queue is full
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
int EnQueue(int e, RS422RXQUE *RS422RxQue){
	//TODO copy this function,  just in case two threads call this function

	//TODO, when queue is full, override the queue, do not return fail
	if((RS422RxQue->rear + 1) % MAXQSIZE == RS422RxQue->front){
		RS422RxQue->front += 1;
		//printf("EnQueue FULL \r\n");
		//return 0;
	}

	RS422RxQue->rxBuff[RS422RxQue->rear] = e;
	RS422RxQue->rear = (RS422RxQue->rear + 1) % MAXQSIZE;
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
int DeQueue(RS422RXQUE *RS422RxQue){
	if(RS422RxQue->front == RS422RxQue->rear){
		return 0;
	}

	RS422RxQue->front = (RS422RxQue->front + 1) % MAXQSIZE;
	return 1;
}
/**************************************************************
 *Name:		   IsQueueEmpty
 *Comment:
 *Input:
 *Output:	   Uint16
 *Author:	   Simon
 *Date:		   2018.11.14
 **************************************************************/
Uint16 IsQueueEmpty(RS422RXQUE *RS422RxQue){
	if(RS422RxQue->front == RS422RxQue->rear){
		return 0;
	}
	else{
		return 1;
	}
}
/***************************************************************
 *Name:						RS422RxQueLength
 *Function:
 *Input:				  	none
 *Output:					rx queue length
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
int RS422RxQueLength(RS422RXQUE *RS422RxQue){
	int length;
	length = (RS422RxQue->rear - RS422RxQue->front + MAXQSIZE) % MAXQSIZE;
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
void RS422A_receive(RS422RXQUE *RS422RxQue){
	int16 data;

	while(ScicRegs.SCIFFRX.bit.RXFFST != 0){// rs422 rx fifo is not empty
		data = ScicRegs.SCIRXBUF.all;
		if(EnQueue(data, RS422RxQue) == 0){
			printf("RS422 rx queue full\r\n");
			//TODO update error msg
		}
	}
}

void RS422B_receive(RS422RXQUE *RS422RxQue){
	int16 data;

	while(ScibRegs.SCIFFRX.bit.RXFFST != 0){// rs422 rx fifo is not empty
		data = ScibRegs.SCIRXBUF.all;
		if(EnQueueB(data, RS422RxQue) == 0){
			//printf("RS422 rx queue full\r\n");
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
int CalCrc(int crc, const char *buf, int len){
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
 *Output:					1 or 0, 1 means find the head , 0 means failed
 *Author:					Simon
 *Date:						2018.10.25
 ****************************************************************/
int findhead(RS422RXQUE *RS422RxQue){

	char head1;
	char head2;
#if COMPARE_A_AND_B
	while(1){

		head1 = RS422RxQue->rxBuff[RS422RxQue->front];
		head2 = RS422RxQue->rxBuff[(RS422RxQue->front + 1) % MAXQSIZE];

		if(head1 == HEAD1 && head2 == HEAD2){
			if(gRS422RxQueB.rxBuff[RS422RxQue->front] == HEAD1 && gRS422RxQueB.rxBuff[(RS422RxQue->front + 1) % MAXQSIZE] == HEAD2){
				gRS422RxQueB.front = RS422RxQue->front;
				return SUCCESS;
			}
			else{
				printf("RS422B NOT FIND HEAD\r\n");
				return FAIL;
			}
		}

		if(DeQueue(RS422RxQue) == 0){
			//printf("rs422 rx queue is empty\r\n");
			return FAIL;
		}
	}


#else

	while(1){

		//printf("front = %d\r\n",RS422RxQue->front );
		//printf("rear = %d\r\n",RS422RxQue->rear );
		if(RS422RxQueLength(RS422RxQue) < EXTRA_LEN){
			//printf("-------------------------------------data not enough to unpak, so do not find head\r\n");
			return FAIL;
		}
		head1 = RS422RxQue->rxBuff[RS422RxQue->front];
		head2 = RS422RxQue->rxBuff[(RS422RxQue->front + 1) % MAXQSIZE];

		if(head1 == HEAD1 && head2 == HEAD2){
			return SUCCESS;
		}

		if(DeQueue(RS422RxQue) == 0){
			//printf("rs422 rx queue is empty\r\n");
			return FAIL;
		}
	}
#endif
}
/***************************************************************
 *Name:						findtail
 *Function:
 *Input:				    length
 *Output:					1 or 0, 1 means find the head , 0 means failed
 *Author:					Simon
 *Date:						2018.10.25
 ****************************************************************/
int findtail(int len, RS422RXQUE *RS422RxQue){
	char tail1;
	char tail2;

	tail1 = RS422RxQue->rxBuff[(RS422RxQue->front + len - 1) % MAXQSIZE];
	tail2 = RS422RxQue->rxBuff[(RS422RxQue->front + len - 2) % MAXQSIZE];

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
int checklength(RS422RXQUE *RS422RxQue){
#if COMPARE_A_AND_B

	if((RS422RxQue->rxBuff[(RS422RxQue->front + 2) % MAXQSIZE] * UNIT_LEN + EXTRA_LEN) < RS422RxQueLength()){
		if((gRS422RxQueB.rxBuff[(RS422RxQue->front + 2) % MAXQSIZE] * UNIT_LEN + EXTRA_LEN) < RS422RxQueLengthB()){
			printf("RS422 A AND B CHANNEL LENGTH IS ENOUGH!!!!!\r\n");
			return SUCCESS;
		}
		else{
			printf("RS422 B channel length not enough\r\n");
			return FAIL;
		}
	}
	else{
		return FAIL;
	}

#else

	if((RS422RxQue->rxBuff[(RS422RxQue->front + 2) % MAXQSIZE] * UNIT_LEN + EXTRA_LEN) < RS422RxQueLength(RS422RxQue)){
		return SUCCESS;
	}
	else{
		return FAIL;
	}
#endif
}
/***************************************************************
 *Name:						saveprofile
 *Function:
 *Input:				    length
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.27
 ****************************************************************/
void saveprofile(int len, RS422RXQUE *RS422RxQue){
	int i;

	for(i = 0; i < len; ++i){
		rs422rxPack[i] = RS422RxQue->rxBuff[(RS422RxQue->front + i) % MAXQSIZE];
	}
}
/***************************************************************
 *Name:						unpack
 *Function:					unpack profile data
 *Input:				    length
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.27
 ****************************************************************/
void unpack(int len){
	int i;
	int msgCode;
	VAR16 var16;

	for(i = 0; i < len; ++i){

		msgCode = rs422rxPack[OFFSET + UNIT_LEN * i];
		var16.datahl.h = rs422rxPack[OFFSET + UNIT_LEN*i + 1];
		var16.datahl.l = rs422rxPack[OFFSET + UNIT_LEN*i + 2];
		var16.value = var16.datahl.l + (var16.datahl.h << 8);

		if(gRS422Status.rs422CurrentChannel == RS422_CHANNEL_A){
			gRS422Status.rs422A = 1;
		}
		else if(gRS422Status.rs422CurrentChannel == RS422_CHANNEL_B){
			gRS422Status.rs422B = 1;
		}

		if(msgCode < (sizeof(msgInterface) / sizeof(msgInterface[0]))){
			//printf("msgCode = %d\r\n",msgCode);
			if(msgInterface[msgCode]){
				msgInterface[msgCode](var16, 0, 0);
			}
		}
		else{
			//printf("unpack msg code is out of range\r\n");
		}
	}
}
/**************************************************************
 *Name:		   CompareRS422AandB
 *Comment:
 *Input:	   Uint16 length
 *Output:	   Uint16
 *Author:	   Simon
 *Date:		   2018.11.14
 **************************************************************/
Uint16 CompareRS422AandB(Uint16 len, RS422RXQUE *RS422RxQue){
	int16 i;

	for (i = 0; i < len; ++i) {
		printf("gRS422RxQue = %d\r\n",RS422RxQue->rxBuff[(RS422RxQue->front + i) % MAXQSIZE]);
		printf("gRS422RxQueB= %d\r\n",gRS422RxQueB.rxBuff[(RS422RxQue->front + i) % MAXQSIZE]);
		if(RS422RxQue->rxBuff[(RS422RxQue->front + i) % MAXQSIZE] != gRS422RxQueB.rxBuff[(RS422RxQue->front + i) % MAXQSIZE]){
			printf("position = %d\r\n", i);
			printf("RS422RxQue->rxBuff = %d\r\n",RS422RxQue->rxBuff[(RS422RxQue->front + i) % MAXQSIZE]);
			printf("gRS422RxQueB.rxBuff = %d\r\n",gRS422RxQueB.rxBuff[(RS422RxQue->front + i) % MAXQSIZE]);
			return FAIL;
		}
	}
	return SUCCESS;
}
/***************************************************************
 *Name:						updatehead
 *Function:					move the front head to another position
 *Input:				    length
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.27
 ****************************************************************/
void updatehead(int len, RS422RXQUE *RS422RxQue){
	RS422RxQue->front = (RS422RxQue->front + len) % MAXQSIZE;
}
/**************************************************************
 *Name:		   UpdateRS422RxSerialNumber
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018.11.14
 **************************************************************/
void UpdateRS422RxSerialNumber(void){

	gRS422Status.currentSerialNumber = (rs422rxPack[3] << 8) + rs422rxPack[4];
	//printf("currentSerialNumber = %d\r\n", gRS422Status.currentSerialNumber);

}
/***************************************************************
 *Name:						UnpackRS422ANew
 *Function:					unpack the hole data package
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.27
 ****************************************************************/
void UnpackRS422ANew(RS422RXQUE *RS422RxQue){
	int length;
	while(RS422RxQueLength(RS422RxQue) > EXTRA_LEN){
		if(findhead(RS422RxQue) == FAIL){
			//printf("find head failed\r\n");
			return;
		}
		else{
			//printf("find head succeed\r\n");
		}

		if(checklength(RS422RxQue) == FAIL){
			//printf("len received =%d\r\n", RS422RxQue->rxBuff[(RS422RxQue->front + 2) % MAXQSIZE] * UNIT_LEN + EXTRA_LEN );
			//printf("len calculate =%d\r\n", RS422RxQueLength(RS422RxQue));
			//printf("data length is not enough, waiting for more data\r\n");
			return;
		}
		else{
			//printf("Check data length succeed, begin to check tail\r\n");
		}

		length = RS422RxQue->rxBuff[(RS422RxQue->front + 2) % MAXQSIZE] * UNIT_LEN + EXTRA_LEN;

	#if COMPARE_A_AND_B

		if(CompareRS422AandB(length, RS422RxQue) == FAIL){
			//printf("Data in RS422 A Channel are not the same with B channel \r\n");
		}
		else{
			//printf("CompareRS422AandB SUCCESS\r\n");
		}
	#endif

		if(findtail(length,RS422RxQue) == FAIL){
			//printf("find tail failed\r\n");
			if(DeQueue(RS422RxQue) == 0){
				//printf("RS422 rx queue is empty\r\n");
			}
			return;
		}
		else{
			//printf("find tail succeed\r\n");
		}

		saveprofile(length,RS422RxQue);

		if(CalCrc(0, rs422rxPack + OFFSET, length - EXTRA_LEN + 2) != 0){
			if(DeQueue(RS422RxQue) == 0){
				//printf("RS422 rx queue is empty\r\n");
			}
			//printf("CRC check failed\r\n");
			return;
		}
		else{
			//printf("CRC check succeed\r\n");
		}

		unpack(RS422RxQue->rxBuff[(RS422RxQue->front + 2) % MAXQSIZE]);
		UpdateRS422RxSerialNumber();
		updatehead(length, RS422RxQue);
		printf("update the front position-------------\r\n");
	}
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
	char buf[21]={
				0x55,
				0x5a,
				0x04,
				0x00,//serial number high byte
				0x01,//serial number low byte
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

	crc = CalCrc(0, buf + OFFSET, 12);
	buf[16] = (char)crc;
	buf[15] = (char)(crc >> 8);
	for(i = 0; i < 21; ++i){
		while(ScicRegs.SCIFFTX.bit.TXFFST != 0){

		}
		ScicRegs.SCITXBUF = buf[i];

	}
}
