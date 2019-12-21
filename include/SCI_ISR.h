#ifndef _SCI_ISR_H
#define _SCI_ISR_H

#define MAXQSIZE 800
#define RXBUGLEN (16)

#define HEAD1 0x5a
#define HEAD2 0x5a
#define TAIL1 0xa5
#define TAIL2 0xa5


typedef struct _RS422RXQUE{
	char rxBuff[MAXQSIZE];
	int front;
	int rear;
}RS422RXQUE;

enum {
	FindHead,
	CheckLength,
	CheckTail,
	CheckCRC,
	Unpack,
	UpdateHead
};

enum{
	ACTION_COMMAND,
	DATA_FEEDBACK,
	SPEED_TARGET,
	RUNNING_TIME
};

typedef struct _DATA{
	Uint16 l : 8;
	Uint16 h : 8;
}DATA;

typedef union _VAR16{
	DATA datahl;
	int16 value;
}VAR16;



typedef void (*functionMsgCodeUnpack)(VAR16 a, int b,int c);

extern RS422RXQUE gRS422RxQue;
extern RS422RXQUE gRS422RxQueB;



void UnpackRS422A(void);
void RS422A_receive(RS422RXQUE *RS422RxQue);
void RS422B_receive(RS422RXQUE *RS422RxQue);
void UnpackRS422ANew(RS422RXQUE *RS422RxQue);
void ClearRS422RxOverFlow(void);

#endif
