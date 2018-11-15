#ifndef _SCI_ISR_H
#define _SCI_ISR_H

#define MAXQSIZE 800
#define RXBUGLEN (16)

#define HEAD1 0x5a
#define HEAD2 0x5a
#define TAIL1 0xa5
#define TAIL2 0xa5

#define FAIL (0)
#define SUCCESS (1)

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
	Uint16 l;
	Uint16 h;
}DATA;

typedef union _VAR16{
	DATA datahl;
	Uint16 value;
}VAR16;

typedef struct{
	Uint16 rs422A;
	Uint16 rs422B;
	Uint16 currentSerialNumber;
	Uint16 rs422CurrentChannel;
	Uint16 shakeHand;
}RS422STATUS;

typedef void (*functionMsgCodeUnpack)(VAR16 a, int b,int c);

extern RS422RXQUE gRS422RxQue;
extern RS422RXQUE gRS422RxQueB;
extern RS422STATUS gRS422Status;


void UnpackRS422A(void);
void RS422A_receive(RS422RXQUE *RS422RxQue);
void RS422B_receive(RS422RXQUE *RS422RxQue);
void UnpackRS422ANew(RS422RXQUE *RS422RxQue);
void testwithlabview(void);
void ClearRS422RxOverFlow(void);

#endif
