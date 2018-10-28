#ifndef _SCI_ISR_H
#define _SCI_ISR_H

#define MAXQSIZE 128
#define RXBUGLEN (16)


#define HEAD1 0x5a
#define HEAD2 0x5a
#define TAIL1 0xa5
#define TAIL2 0xa5

#define HEAD 0x55
#define TAIL 0xAA



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
	unsigned char l;
	unsigned char h;
}DATA;

typedef union _VAR16{
	DATA datahl;
	Uint16 value;
}VAR16;

typedef void (*functionMsgCodeUnpack)(VAR16 a, int b,int c);
extern RS422RXQUE gRS422RxQue;
void UnpackRS422A(void);
void RS422A_receive(void);
void UnpackRS422ANew(void);
void testwithlabview();

#endif
