#ifndef _SCI_TX_H
#define _SCI_TX_H




#define TXMAXQSIZE (1800)
#define TOTAL_TX_VAR (20)

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



typedef struct _GRX422TX{
	Uint16 value;
	unsigned char index;
	unsigned char isTx;
	updatevalue updateValue;
}GRX422TX;


typedef struct _RS422TXQUE{
	char txBuf[TXMAXQSIZE];
	int front;
	int rear;
}RS422TXQUE;

extern GRX422TX gRx422TxVar[TOTAL_TX_VAR];
extern Uint16 gRx422TxEnableFlag[TOTAL_TX_VAR];
extern char Rx4225TxBuf[900];
extern RS422TXQUE gRS422TxQue;

int RX422TXDeQueue(void);
void RS422A_Transmit(void);
void testrs422tx(void);

#endif
