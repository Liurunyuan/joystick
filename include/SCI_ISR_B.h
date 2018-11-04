#ifndef _SCI_ISR_B_H
#define _SCI_ISR_B_H

#define MAXQSIZE_B 128
#define RXBUGLEN_B (16)

#define HEAD1_B 0x5a
#define HEAD2_B 0x5a
#define TAIL1_B 0xa5
#define TAIL2_B 0xa5

#define FAIL (0)
#define SUCCESS (1)

typedef struct _RS422RXQUEB{
	char rxBuff[MAXQSIZE_B];
	int front;
	int rear;
}RS422RXQUEB;

extern RS422RXQUEB gRS422RxQueB;


void RS422B_receive(void);

#endif
