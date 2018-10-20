#ifndef _SCI_ISR_H
#define _SCI_ISR_H

#define MAXQSIZE 128
#define RXBUGLEN (16)
typedef void (*functionMsgCodeUnpack)(int a, int b,int c);

#define HEAD 0x55
#define TAIL 0xAA

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
extern RS422RXQUE gRS422RxQue;
void UnpackRS422A(void);
void RS422A_receive(void);

#endif
