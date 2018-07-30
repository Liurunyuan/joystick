#include "SCI_ISR.h"


/***********globle variable define here***************/
int recievechar[RXBUGLEN]={0};

static void MsgStatusUnpack(int a, int b, int c)
{
	//TODO just an example

}

const functionMsgCodeUnpack msgInterface[] =
{
		0,
		MsgStatusUnpack,
		0,
		0,
		0,
		0
};


/*************************************/
void RS422A_recieve(void)
{
	int index;
	for(index = 0; index < RXBUGLEN; ++index)
	{
		recievechar[index] = 1;

		if(1>=1)
		{

		}
	}
}
