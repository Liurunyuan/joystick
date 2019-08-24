#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "GlobalVarAndFunc.h"
#include "SCI_TX.h"
#include <stdio.h>
#include <string.h>
#include "ADprocessor.h"
#include "PWM_ISR.h"

GRX422TX gRx422TxVar[TOTAL_TX_VAR] = {0};
Uint16 gRx422TxEnableFlag[TOTAL_TX_VAR] = {0};
RS422TXQUE gRS422TxQue = {0};
#define S (0)
/***************************************************************/



void GetTorqueCurve(int a, int b, int c){
	//gRx422TxVar[0].value = gKeyValue.motorSpeed;
	//gRx422TxVar[0].value = (int16)(gExternalForceState.value*100);
    gRx422TxVar[0].value = (gStickState.value * 100) + 3000;
}
void GetMotorSpeedCurve(int a, int b, int c){
	//gRx422TxVar[1].value = (int)real2;
	//gRx422TxVar[1].value = (int)(gKeyValue.motorSpeed);
	//gRx422TxVar[1].value = gSysMonitorVar.anolog.AD_16bit.var[ForceValue_16bit].value;
    //gRx422TxVar[1].value = gExternalForceState.value;
    //gRx422TxVar[1].value = gStickState.value;
    gRx422TxVar[1].value = (gKeyValue.motorSpeed * 1000) + 3000;
}
void GetDisplacementCurve(int a, int b, int c){
	//gRx422TxVar[2].value = (int)(gKeyValue.displacement);
//    gRx422TxVar[2].value = (gSysInfo.duty)*50;
    //gRx422TxVar[2].value = (int16)(gKeyValue.motorSpeed*1000);
	//gRx422TxVar[2].value = (int16)(gExternalForceState.value*100);
    gRx422TxVar[2].value = (gKeyValue.motorAccel * 100) + 3000;
    //gRx422TxVar[2].value =gSysInfo.duty;
}
void GetMotorCurrentCurve(int a, int b, int c){
	gRx422TxVar[3].value = gSysMonitorVar.anolog.single.var[BusCurrentA].value;
	//gRx422TxVar[3].value = (gSysInfo.currentDuty)*10;
}
void GetDynamoVoltageCurve(int a, int b, int c){
	gRx422TxVar[4].value = 20000;
}
void GetDynamoCurrentCurve(int a, int b, int c){
	gRx422TxVar[4].value = 1;
}
void GetTemperatureCurve(int a, int b, int c){
	gRx422TxVar[5].value = 3000;
}
void GetMotorAccelCurve(int a, int b, int c){
	gRx422TxVar[6].value = (int)(gKeyValue.motorAccel * 500);;
}






void InitgRx422TxEnableFlag(void){
/**************************************************************
 *Name:		   InitgRx422TxEnableFlag
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2019��1��5������3:58:19
 **************************************************************/
	int index;

	memset(gRx422TxEnableFlag, 0, sizeof(gRx422TxEnableFlag));
	for (index = 0; index < 16; ++index) {

		gRx422TxEnableFlag[index] = 0;
	}
}
/**************************************************************
 *Name:		   InitgRx422TxVar
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2019��1��5������3:55:13
 **************************************************************/
void InitgRx422TxVar(void) {

	int index;

	memset(gRx422TxVar, 0, sizeof(gRx422TxVar));
	for (index = 0; index < 16; ++index) {

		gRx422TxVar[index].isTx = 0;
		gRx422TxVar[index].index = index;
	}
	gRx422TxVar[0].updateValue = GetTorqueCurve;
	gRx422TxVar[1].updateValue = GetMotorSpeedCurve;
	gRx422TxVar[2].updateValue = GetDisplacementCurve;
	gRx422TxVar[3].updateValue = GetMotorCurrentCurve;
	gRx422TxVar[4].updateValue = GetDynamoVoltageCurve;
	gRx422TxVar[5].updateValue = GetDynamoCurrentCurve;
	gRx422TxVar[6].updateValue = GetTemperatureCurve;
	gRx422TxVar[7].updateValue = GetMotorAccelCurve;
}

/***************************************************************
 *Name:						RX422TXEnQueue
 *Function:
 *Input:				    char e, come from tx queue
 *Output:					1 or 0, 1 means success, 0 means the queue is full already
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
int RX422TXEnQueue(char e){
	if((gRS422TxQue.rear + 1) % TXMAXQSIZE == gRS422TxQue.front){
		asm ("      ESTOP0");
		return 0;
	}

	gRS422TxQue.txBuf[gRS422TxQue.rear] = e;
	gRS422TxQue.rear = (gRS422TxQue.rear + 1) % TXMAXQSIZE;
	return 1;
}
/***************************************************************
 *Name:						RX422TXDeQueue
 *Function:
 *Input:				    none
 *Output:					1 or 0, 1 means success, 0 means the queue is empty already
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
int RX422TXDeQueue(void){
	if(gRS422TxQue.front == gRS422TxQue.rear){
		return 0;
	}

	gRS422TxQue.front = (gRS422TxQue.front + 1) % TXMAXQSIZE;
	return 1;
}
/***************************************************************
 *Name:						RS422RxQueLength
 *Function:
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
int RS422TxQueLength(){
	int length;
	length = (gRS422TxQue.rear - gRS422TxQue.front + TXMAXQSIZE) % TXMAXQSIZE;
	return length;
}
/***************************************************************
 *Name:						CalCrc
 *Function:
 *Input:				    rs422 rx data
 *Output:					int, should be zero
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
int calCrc(int crc, const char *buf, int len) {
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
/**************************************************************
 *Name:		   updateTxEnableFlag
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018.11.14
 **************************************************************/
void updateTxEnableFlag(void) {
	int i;
	for (i = 0; i < TOTAL_TX_VAR; ++i) {
		gRx422TxVar[i].isTx = gRx422TxEnableFlag[i];
	}
}

/***************************************************************
 *Name:						PackRS422TxData
 *Function:					pack the data that need to be sent
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
void PackRS422TxData(void){
	//TODO need do some test, because we sync the tx enable flag here
	int i;
	char crcl;
	char crch;
	static unsigned char count = 0;
	static int crc = 0;
	char tmp[3] = {0};
	int lenPosition = 0;
	Uint16 total =0;


	if(count == 0){
		if(RX422TXEnQueue(0x5a) == 0){
			asm ("      ESTOP0");
			return;
		}
		if(RX422TXEnQueue(0x5a) == 0){
			asm ("      ESTOP0");
			return;
		}
		lenPosition = gRS422TxQue.rear;
		if(RX422TXEnQueue(0x05) == 0){
			asm ("      ESTOP0");
			return;
		}
		if(RX422TXEnQueue(0xff) == 0){
			asm ("      ESTOP0");
			return;
		}
		if(RX422TXEnQueue(0xff) == 0){
			asm ("      ESTOP0");
			return;
		}
		updateTxEnableFlag();
	}

	for(i = 0; i < TOTAL_TX_VAR; ++i){
		if(gRx422TxVar[i].isTx){
			++total;

			gRx422TxVar[i].updateValue(0,0,0);
			tmp[0] = gRx422TxVar[i].index;
			tmp[1] = gRx422TxVar[i].value >> 8;
			tmp[2] = gRx422TxVar[i].value;
			if(RX422TXEnQueue(gRx422TxVar[i].index) == 0){
				asm ("      ESTOP0");
				return;
			}
			if(RX422TXEnQueue(gRx422TxVar[i].value >> 8) == 0){
				asm ("      ESTOP0");
				return;
			}
			if(RX422TXEnQueue(gRx422TxVar[i].value) == 0){
				asm ("      ESTOP0");
				return;
			}
			crc = calCrc(crc, tmp, 3);
		}

		//TODO ������Ϣ��������Ϣһֱ����
	}

	if(count == 0){
		gRS422TxQue.txBuf[lenPosition] = total * (S + 1);//timer0 interrupt isr can not be interrupted by TX, so we can set length value here
	}

	++count;

	if(count > S){

		crcl = (char)crc;
		crch = (char)(crc >> 8);
		crc = 0;
		count = 0;
		if(RX422TXEnQueue(crch) == 0){
			asm ("      ESTOP0");
			return;
		}
		if(RX422TXEnQueue(crcl) == 0){
			asm ("      ESTOP0");
			return;
		}
		if(RX422TXEnQueue(0xa5) == 0){
			asm ("      ESTOP0");
			return;
		}
		if(RX422TXEnQueue(0xa5) == 0){
			asm ("      ESTOP0");
			return;
		}
	}
}
/**************************************************************
 *Name:		   ScibTxByte
 *Comment:
 *Input:	   one byte to send by sci B
 *Output:	   none
 *Author:	   Simon
 *Date:		   2018.11.14
 **************************************************************/
void ScibTxByte(Uint16 t){

	ScibRegs.SCITXBUF = t;

}
/**************************************************************
 *Name:		   ScicTxByte
 *Comment:
 *Input:	   one byte to send by sci C
 *Output:	   none
 *Author:	   Simon
 *Date:		   2018.11.14
 **************************************************************/
void ScicTxByte(Uint16 t){

	ScicRegs.SCITXBUF = t;

}
/**************************************************************
 *Name:		   DisableScicTxInterrupt
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018.11.14
 **************************************************************/
void DisableScicTxInterrupt(void){

	ScicRegs.SCIFFTX.bit.TXFFIENA = 0;

}
/***************************************************************
 *Name:						rs422 tx interrupt isr
 *Function:			  		none
 *Input:				  	none
 *Output:					none
 *Author:					Simon
 *Date:						2018.11.3
 ****************************************************************/
void RS422A_Transmit(void){

	if(gRS422TxQue.front == gRS422TxQue.rear){
		DisableScicTxInterrupt();//disable the tx interrupt when tx fifo empty
		return;
	}

	//while((ScicRegs.SCIFFTX.bit.TXFFST != 16)
	//			&& (ScibRegs.SCIFFTX.bit.TXFFST != 16)){
	while((ScibRegs.SCIFFTX.bit.TXFFST != 16)){
		if(RS422TxQueLength() == 0)
		{
			return;
		}
		ScibTxByte(gRS422TxQue.txBuf[gRS422TxQue.front]);//printf by Scic
		ScicTxByte(gRS422TxQue.txBuf[gRS422TxQue.front]);

		if(RX422TXDeQueue() == 0){
			DisableScicTxInterrupt();
			return;
		}
	}
}
/**************************************************************
 *Name:		   TransmitRS422ShakeHandMsg
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018��11��15������9:02:53
 **************************************************************/
void TransmitRS422ShakeHandMsg(void){

	int len;
	int index;
	char rs422ShakeHandMsg[] = {
		//TODO use the real shake hand msg in the future
		0x5a,//head1
		0x5a,//head2
		0x01,//length
		0x00,//serial number
		0x00,//serial number
		0xff,//shake hand
		0xff,//shake hand
		0xff,//shake hand
		0x00,//crc1
		0x00,//crc2
		0xa5,//tail1
		0xa5 //tail2
	};

	len = sizeof(rs422ShakeHandMsg);

	for(index = 0; index < len; ++index){
		while(ScibRegs.SCIFFTX.bit.TXFFST != 0){

		}
		ScibRegs.SCITXBUF = rs422ShakeHandMsg[index];
	}
}
/**************************************************************
 *Name:		   ShakeHandWithUpperComputer
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018��11��15������9:03:08
 **************************************************************/
void ShakeHandWithUpperComputer(void){

	if(FAIL == gRS422Status.shakeHand){
		TransmitRS422ShakeHandMsg();
	}
}
