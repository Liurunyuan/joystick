#ifndef _SCI_TX_H
#define _SCI_TX_H

typedef void (*updatevalue)(int a, int b,int c);
enum {
	TORQUE,
	SPEED,
	ROTOR_POSITION,
	MOTROR_CURRENT,
	TEMP,
	TXEND
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

extern GRX422TX gRx422TxVar[5];
void testrs422tx(void);
extern char Rx4225TxBuf[128];

#endif
