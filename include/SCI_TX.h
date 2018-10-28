#ifndef _SCI_TX_H
#define _SCI_TX_H
#define TXMAXQSIZE 900

typedef void (*updatevalue)(int a, int b,int c);
enum {
	TORQUE,
	SPEED,
	ROTOR_POSITION,
	MOTROR_CURRENT,
	DYNAMO_VOLTAGE,
	DYNAMO_CURRENT,
	TEMP,
	TXEND
};

enum{
	ALARM_STATE,
	WAITING,
	RUNNING
};

typedef struct _DATAHL{
	unsigned char l;
	unsigned char h;
}DATAHL;

typedef union _VAR{
	DATAHL datahl;
	Uint16 value;
}VAR;

typedef struct _GRX422TX{
	unsigned char index;
	VAR var;
	updatevalue updateValue;
	unsigned char isTx;

}GRX422TX;


typedef struct _RS422TXQUE{
	char txBuf[TXMAXQSIZE];
	int front;
	int rear;
}RS422TXQUE;

extern GRX422TX gRx422TxVar[20];
void testrs422tx(void);
extern char Rx4225TxBuf[900];
extern RS422TXQUE gRS422TxQue;
int RX422TXDeQueue(void);

#endif