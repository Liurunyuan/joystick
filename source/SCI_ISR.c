#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "SCI_ISR.h"
#include "SCI_TX.h"
#include "GlobalVarAndFunc.h"
#include <stdio.h>
#include "PID.h"

#define UNIT_LEN (3) 			//0x00(index 1 byte) + 0x00(high 8 bit) + 0x00(low 8 bit)
#define EXTRA_LEN  (9)          //head(2 bytes) + length(1 byte) + serial number(2 bytes) + crc(2 bytes) + tail(2 bytes)
#define OFFSET (5) 				//head(2 bytes) + length(1 byte) + serial number(2 bytes)
#define WAVE_AMOUNT (16)
#define ENABLE_TX (1)
#define DISABLE_TX (0)

#define COMPARE_A_AND_B (0)
/***********globle variable define here***************/
RS422RXQUE gRS422RxQue = {0};
RS422RXQUE gRS422RxQueB = {0};
char rs422rxPack[100] = {0};


/**************************************************************
 *Name:		   TestDuty
 *Comment:
 *Input:	   VAR16, int, int
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018��11��15������9:15:42
 **************************************************************/
static void TestDuty(VAR16 a, int b, int c) {

	// gSysInfo.duty = (int)a.value;
    gSysInfo.RS422_Rx_Data = (int)a.value;

	//TODO just an example
}
/**************************************************************
 *Name:		   TestHallPosition
 *Comment:
 *Input:	   VAR16, int, int
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018��11��15������9:15:42
 **************************************************************/
static void TestHallPosition(VAR16 a, int b, int c) {

	//gSysInfo.currentHallPosition = a.value;
}

/**************************************************************
 *Name:		   ShakeHandMsg
 *Comment:
 *Input:	   VAR16, int, int
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018��11��15������9:15:42
 **************************************************************/
static void ShakeHandMsg(VAR16 a, int b, int c) {
//	gRS422Status.shakeHand = SUCCESS;
    UpdateForceDisplaceCurve();
}

/*******************************************************/
static void configPara1(VAR16 a, int b, int c) {
	gConfigPara.dampingFactor = ((double)(a.value)) / 100;
}
static void configPara2(VAR16 a, int b, int c) {
	gConfigPara.naturalVibrationFreq = ((double)(a.value)) / 100;
}
static void configPara3(VAR16 a, int b, int c) {
	gConfigPara.LF_FrontFriction = ((double)(a.value)) / 100;
}
static void configPara4(VAR16 a, int b, int c) {
	gConfigPara.LF_RearFriction = ((double)(a.value)) / 100;
}
static void configPara5(VAR16 a, int b, int c) {
	gConfigPara.RB_FrontFriction = ((double)(a.value)) / 100;
}
static void configPara6(VAR16 a, int b, int c) {
	gConfigPara.RB_RearFriction = ((double)(a.value)) / 100;
}
static void configPara7(VAR16 a, int b, int c) {
	gConfigPara.LF_EmptyDistance = ((double)(a.value)) / 100;
}
static void configPara8(VAR16 a, int b, int c) {
	gConfigPara.LF_StartForce = ((double)(a.value)) / 100;
}
static void configPara9(VAR16 a, int b, int c) {
	gConfigPara.LF_Distance1 = ((double)(a.value)) / 100;
}
static void configPara10(VAR16 a, int b, int c) {
	gConfigPara.LF_Force1 = ((double)(a.value)) / 100;
}
/*******************************************************/
static void configPara11(VAR16 a, int b, int c) {
	gConfigPara.LF_Distance2 = ((double)(a.value)) / 100;
}
static void configPara12(VAR16 a, int b, int c) {
	gConfigPara.LF_Force2 = ((double)(a.value)) / 100;
}
static void configPara13(VAR16 a, int b, int c) {
	gConfigPara.LF_Distance3 = ((double)(a.value)) / 100;
}
static void configPara14(VAR16 a, int b, int c) {
	gConfigPara.LF_Force3 = ((double)(a.value)) / 100;
}
static void configPara15(VAR16 a, int b, int c) {
	gConfigPara.LF_Distance4 = ((double)(a.value)) / 100;
}
static void configPara16(VAR16 a, int b, int c) {
	gConfigPara.LF_Force4 = ((double)(a.value)) / 100;
}
static void configPara17(VAR16 a, int b, int c) {
	gConfigPara.LF_Distance5 = ((double)(a.value)) / 100;
}
static void configPara18(VAR16 a, int b, int c) {
	gConfigPara.LF_Force5 = ((double)(a.value)) / 100;
}
static void configPara19(VAR16 a, int b, int c) {
	gConfigPara.LF_Distance6 = ((double)(a.value)) / 100;
}
static void configPara20(VAR16 a, int b, int c) {
	gConfigPara.LF_Force6 = ((double)(a.value)) / 100;
}
/*******************************************************/
static void configPara21(VAR16 a, int b, int c) {
	gConfigPara.LF_Distance7 = ((double)(a.value)) / 100;
}
static void configPara22(VAR16 a, int b, int c) {
	gConfigPara.LF_Force7 = ((double)(a.value)) / 100;
}
static void configPara23(VAR16 a, int b, int c) {
	gConfigPara.LF_Distance8 = ((double)(a.value)) / 100;
}
static void configPara24(VAR16 a, int b, int c) {
	gConfigPara.LF_Force8 = ((double)(a.value)) / 100;
}
static void configPara25(VAR16 a, int b, int c) {
	gConfigPara.LF_MaxDistance = ((double)(a.value)) / 100;
}
static void configPara26(VAR16 a, int b, int c) {
	gConfigPara.LF_MaxForce = ((double)(a.value)) / 100;
}
static void configPara27(VAR16 a, int b, int c) {
	gConfigPara.RB_EmptyDistance = ((double)(a.value)) / 100;
}
static void configPara28(VAR16 a, int b, int c) {
	gConfigPara.RB_StartForce = ((double)(a.value)) / 100;
}
static void configPara29(VAR16 a, int b, int c) {
	gConfigPara.RB_Distance1 = ((double)(a.value)) / 100;
}
static void configPara30(VAR16 a, int b, int c) {
	gConfigPara.RB_Force1 = ((double)(a.value)) / 100;
}
/**************************************************************/
static void configPara31(VAR16 a, int b, int c) {
	gConfigPara.RB_Distance2 = ((double)(a.value)) / 100;
}
static void configPara32(VAR16 a, int b, int c) {
	gConfigPara.RB_Force2 = ((double)(a.value)) / 100;
}
static void configPara33(VAR16 a, int b, int c) {
	gConfigPara.RB_Distance3 = ((double)(a.value)) / 100;
}
static void configPara34(VAR16 a, int b, int c) {
	gConfigPara.RB_Force3 = ((double)(a.value)) / 100;
}
static void configPara35(VAR16 a, int b, int c) {
	gConfigPara.RB_Distance4 = ((double)(a.value)) / 100;
}
static void configPara36(VAR16 a, int b, int c) {
	gConfigPara.RB_Force4 = ((double)(a.value)) / 100;
}
static void configPara37(VAR16 a, int b, int c) {
	gConfigPara.RB_Distance5 = ((double)(a.value)) / 100;
}
static void configPara38(VAR16 a, int b, int c) {
	gConfigPara.RB_Force5 = ((double)(a.value)) / 100;
}
static void configPara39(VAR16 a, int b, int c) {
	gConfigPara.RB_Distance6 = ((double)(a.value)) / 100;
}
static void configPara40(VAR16 a, int b, int c) {
	gConfigPara.RB_Force6 = ((double)(a.value)) / 100;
}
/*****************************************************************/
static void configPara41(VAR16 a, int b, int c) {
	gConfigPara.RB_Distance7 = ((double)(a.value)) / 100;
}
static void configPara42(VAR16 a, int b, int c) {
	gConfigPara.RB_Force7 = ((double)(a.value)) / 100;
}
/*****************************************************************/
static void configPara43(VAR16 a, int b, int c) {
	gConfigPara.RB_Distance8 = ((double)(a.value)) / 100;
}
static void configPara44(VAR16 a, int b, int c) {
	gConfigPara.RB_Force8 = ((double)(a.value)) / 100;
}
static void configPara45(VAR16 a, int b, int c) {
	gConfigPara.RB_MaxDistance = ((double)(a.value)) / 100;
}
static void configPara46(VAR16 a, int b, int c) {
	gConfigPara.RB_MaxForce = ((double)(a.value)) / 100;
}
/*****************************************************************/
static void configPara47(VAR16 a, int b, int c) {
	gConfigPara.Trim_StepSize = ((double)(a.value)) / 100;
}
static void configPara48(VAR16 a, int b, int c) {
	gConfigPara.Trim_Speed = ((double)(a.value)) / 100;
}
/*****************************************************************/
static void configPara49(VAR16 a, int b, int c) {
	gConfigPara.timeDelay = ((double)(a.value)) / 100;
}
static void configPara50(VAR16 a, int b, int c) {
    gSysInfo.Force_Init2Pos_Thr = ((double)(a.value)) / 100;
}
static void configPara51(VAR16 a, int b, int c) {
    gSysInfo.Force_Init2Neg_Thr = ((double)(a.value)) / 100;
}
static void configPara52(VAR16 a, int b, int c) {
	gSysInfo.Accel_Init2Pos_Thr = ((double)(a.value)) / 100;
}
static void configPara53(VAR16 a, int b, int c) {
	gSysInfo.Accel_Init2Neg_Thr = ((double)(a.value)) / 100;
}
static void configPara54(VAR16 a, int b, int c) {
	gSysInfo.Velocity_Init2Pos_Thr = ((double)(a.value)) / 1000000;
}
static void configPara55(VAR16 a, int b, int c) {
	gSysInfo.Velocity_Init2Neg_Thr = ((double)(a.value)) / 1000000;
}
static void configPara56(VAR16 a, int b, int c) {
    gSysInfo.Force_Pos_Thr = ((double)(a.value)) / 100;
}
static void configPara57(VAR16 a, int b, int c) {
    gSysInfo.Force_Neg_Thr = ((double)(a.value)) / 100;
}
/************************************************************/
static void configPara58(VAR16 a, int b, int c) {
    gSysInfo.Force_Hysteresis = ((double)(a.value)) / 100;
}
static void configPara59(VAR16 a, int b, int c) {
    gSysInfo.Accel_Pos_Thr = ((double)(a.value)) / 100;
}
static void configPara60(VAR16 a, int b, int c) {
    gSysInfo.Accel_Neg_Thr = ((double)(a.value)) / 100;
}
static void configPara61(VAR16 a, int b, int c) {
    gSysInfo.Accel_Zero2Pos_Thr = ((double)(a.value)) / 100;
}
static void configPara62(VAR16 a, int b, int c) {
    gSysInfo.Accel_Zero2Neg_Thr = ((double)(a.value)) / 100;
}
static void configPara63(VAR16 a, int b, int c) {
    gSysInfo.Accel_Hysteresis = ((double)(a.value)) / 100;
}
static void configPara64(VAR16 a, int b, int c) {
    gSysInfo.Accel_Debounce_Cnt_1 = ((double)(a.value)) / 100;
}
static void configPara65(VAR16 a, int b, int c) {
    gSysInfo.Accel_Debounce_Cnt_2 = ((double)(a.value)) / 100;
}
static void configPara66(VAR16 a, int b, int c) {
    gSysInfo.Velocity_Pos_Thr = ((double)(a.value)) / 100;
}
static void configPara67(VAR16 a, int b, int c) {
    gSysInfo.Velocity_Neg_Thr = ((double)(a.value)) / 100;
}
static void configPara68(VAR16 a, int b, int c) {
    gSysInfo.Velocity_Zero2Pos_Thr = ((double)(a.value)) / 100;
}
static void configPara69(VAR16 a, int b, int c) {
    gSysInfo.Velocity_Zero2Neg_Thr = ((double)(a.value)) / 100;
}
static void configPara70(VAR16 a, int b, int c) {
    gSysInfo.Velocity_Hysteresis = ((double)(a.value)) / 100;
}
static void configPara71(VAR16 a, int b, int c) {
    gSysInfo.Velocity_Debounce_Cnt_1 = ((double)(a.value)) / 100;
}
static void configPara72(VAR16 a, int b, int c) {
    gSysInfo.Velocity_Debounce_Cnt_2 = ((double)(a.value)) / 100;
}
static void configPara73(VAR16 a, int b, int c) {
    gPidPara.kp_force_ODE = ((double)(a.value)) / 100;
}
static void configPara74(VAR16 a, int b, int c) {
    gPidPara.ki_force_ODE = ((double)(a.value)) / 100;
}
static void configPara75(VAR16 a, int b, int c) {
    gPidPara.kp_force_NULL = ((double)(a.value)) / 100;
}
static void configPara76(VAR16 a, int b, int c) {
    gPidPara.ki_force_NULL = ((double)(a.value)) / 100;
}
static void configPara77(VAR16 a, int b, int c) {
    gPidPara.kp_velocity_ODE = ((double)(a.value)) / 100;
}
static void configPara78(VAR16 a, int b, int c) {
    gPidPara.ki_velocity_ODE = ((double)(a.value)) / 100;
}
static void configPara79(VAR16 a, int b, int c) {
    gPidPara.kp_velocity_NULL = ((double)(a.value)) / 100;
}
static void configPara80(VAR16 a, int b, int c) {
    gPidPara.ki_velocity_NULL = ((double)(a.value)) / 100;
}
static void configPara81(VAR16 a, int b, int c) {
    gPidPara.K_F_ODE = ((double)(a.value)) / 100;
}
static void configPara82(VAR16 a, int b, int c) {
    gPidPara.B_F_ODE = ((double)(a.value)) / 100;
}
static void configPara83(VAR16 a, int b, int c) {
    gPidPara.K_F_NULL = ((double)(a.value)) / 100;
}
static void configPara84(VAR16 a, int b, int c) {
    gPidPara.B_F_NULL = ((double)(a.value)) / 100;
}
static void configPara85(VAR16 a, int b, int c) {
    gPidPara.K_V_ODE = ((double)(a.value)) / 100;
}
static void configPara86(VAR16 a, int b, int c) {
    gPidPara.B_V_ODE = ((double)(a.value)) / 100;
}
static void configPara87(VAR16 a, int b, int c) {
    gPidPara.K_V_NULL = ((double)(a.value)) / 100;
}
static void configPara88(VAR16 a, int b, int c) {
    gPidPara.B_V_NULL = ((double)(a.value)) / 100;
}
static void configPara89(VAR16 a, int b, int c) {
    //gSysInfo.coe_Force_Max_ODE = ((double)(a.value)) / 100;
    gSysInfo.coe_Force = ((double)(a.value)) / 100;
}
static void configPara90(VAR16 a, int b, int c) {
    gSysInfo.coe_Force_Min_ODE = ((double)(a.value)) / 100;
}
static void configPara91(VAR16 a, int b, int c) {
    //gSysInfo.coe_Velocity_Max_ODE = ((double)(a.value)) / 100;
    gSysInfo.coe_Velocity = ((double)(a.value)) / 100;
}
static void configPara92(VAR16 a, int b, int c) {
    gSysInfo.coe_Velocity_Min_ODE = ((double)(a.value)) / 100;
}
static void systemStateCommand(VAR16 a, int b, int c){
	gConfigPara.stateCommand = (int)a.value;
}


/***************************************************************
 *Name:						WaveCommand
 *Function:
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.25
 ****************************************************************/
static void WaveCommand(VAR16 a, int b, int c) {
	int i;

	for(i = 0; i < 8; ++i){
		//unpack bit information
		if((a.value & (0x0001 << i)) >> i){
			gRx422TxEnableFlag[i] = ENABLE_TX;
		}
		else{
			gRx422TxEnableFlag[i] = DISABLE_TX;
		}
	}
}

static void clutchSlipSpeedTarget(VAR16 a, int b, int c) {

}
static void twinTrawlingSpeedTarget(VAR16 a, int b, int c) {

}
static void runningTime(VAR16 a, int b, int c) {

}
static void actionCommand(VAR16 a, int b, int c) {

}
/***************************************************************
 *Name:						functionMsgCodeUnpack
 *Function:					poniter of function list
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.25
 ****************************************************************/
const functionMsgCodeUnpack msgInterface[] = {
		ShakeHandMsg,						//0
		actionCommand,						//1
		WaveCommand,						//2
		clutchSlipSpeedTarget,				//3
		twinTrawlingSpeedTarget,			//4
		runningTime,						//5
		configPara1,						//6
		configPara2,						//7
		configPara3,						//8
		configPara4,						//9
		configPara5,						//10
		configPara6,						//11
		configPara7,						//12
		configPara8,						//13
		configPara9,						//14
		configPara10,						//15
		configPara11,						//16
		configPara12,						//17
		configPara13,						//18
		configPara14,						//19
		configPara15,						//20
		configPara16,						//21
		configPara17,						//22
		configPara18,						//23
		configPara19,						//24
		configPara20,						//25
		configPara21,						//26
		configPara22,						//27
		configPara23,						//28
		configPara24,						//29
		configPara25,						//30
		configPara26,						//31
		configPara27,						//32
		configPara28,						//33
		configPara29,						//34
		configPara30,						//35
		configPara31,						//36
		configPara32,						//37
		configPara33,						//38
		configPara34,						//39
		configPara35,						//40
		configPara36,						//41
		configPara37,						//42
		configPara38,						//43
		configPara39,						//44
		configPara40,						//45
		configPara41,						//46
		configPara42,						//47
		configPara43,						//48
		configPara44,						//49
		configPara45,						//50
		configPara46,						//51
		configPara47,						//52
		configPara48,						//53
		configPara49,						//54
		configPara50,						//55
		configPara51,						//56
		configPara52,						//57
		configPara53,						//58
		configPara54,						//59
		configPara55,						//60
		configPara56,						//61
		configPara57,						//62
		configPara58,						//63
		configPara59,						//64
		configPara60,						//65
		configPara61,						//66
		configPara62,						//67
		configPara63,						//68
		configPara64,						//69
		configPara65,						//70
		configPara66,						//71
		configPara67,						//72
		configPara68,						//73
		configPara69,						//74
		configPara70,						//75
		configPara71,						//76
		configPara72,
		configPara73,
		configPara74,
		configPara75,
		configPara76,
		configPara77,
		configPara78,
		configPara79,
		configPara80,
		configPara81,
		configPara82,
		configPara83,
		configPara84,
		configPara85,
		configPara86,
		configPara87,
		configPara88,
		configPara89,
		configPara90,
		configPara91,
		configPara92,
		systemStateCommand,
		TestDuty,
		TestHallPosition,
		0,
		0
};
/***************************************************************
 *Name:						EnQueueB
 *Function:					insert the element in the queue
 *Input:				    received data from SCIC
 *Output:					1 or 0, 1 means insert success, 0 means the queue is full
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
int EnQueueB(int e, RS422RXQUE *RS422RxQue){

	if((RS422RxQue->rear + 1) % MAXQSIZE == RS422RxQue->front){
		RS422RxQue->front = (RS422RxQue->front + 1) % MAXQSIZE;
	}

	RS422RxQue->rxBuff[RS422RxQue->rear] = e;
	RS422RxQue->rear = (RS422RxQue->rear + 1) % MAXQSIZE;
	return 1;
}
/***************************************************************
 *Name:						EnQueue
 *Function:					insert the element in the queue
 *Input:				    received data from SCIC
 *Output:					1 or 0, 1 means insert success, 0 means the queue is full
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
int EnQueue(int e, RS422RXQUE *RS422RxQue){

	if((RS422RxQue->rear + 1) % MAXQSIZE == RS422RxQue->front){
		RS422RxQue->front = (RS422RxQue->front + 1) % MAXQSIZE;
	}

	RS422RxQue->rxBuff[RS422RxQue->rear] = e;
	RS422RxQue->rear = (RS422RxQue->rear + 1) % MAXQSIZE;
	return 1;
}
/***************************************************************
 *Name:						DeQueue
 *Function:					remove the element in the queue
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
int DeQueue(RS422RXQUE *RS422RxQue){
	if(RS422RxQue->front == RS422RxQue->rear){
		return 0;
	}

	RS422RxQue->front = (RS422RxQue->front + 1) % MAXQSIZE;
	return 1;
}
/**************************************************************
 *Name:		   IsQueueEmpty
 *Comment:
 *Input:
 *Output:	   Uint16
 *Author:	   Simon
 *Date:		   2018.11.14
 **************************************************************/
Uint16 IsQueueEmpty(RS422RXQUE *RS422RxQue){
	if(RS422RxQue->front == RS422RxQue->rear){
		return 0;
	}
	else{
		return 1;
	}
}
/***************************************************************
 *Name:						RS422RxQueLength
 *Function:
 *Input:				  	none
 *Output:					rx queue length
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
int RS422RxQueLength(RS422RXQUE *RS422RxQue){
	int length;
	length = (RS422RxQue->rear - RS422RxQue->front + MAXQSIZE) % MAXQSIZE;
	return length;
}
/***************************************************************
 *Name:						RS422A_receive
 *Function:
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
void RS422A_receive(RS422RXQUE *RS422RxQue){
	int16 data;

	while(ScicRegs.SCIFFRX.bit.RXFFST != 0){// rs422 rx fifo is not empty
		data = ScicRegs.SCIRXBUF.all;
		if(EnQueue(data, RS422RxQue) == 0){
			gSysState.alarm.bit.rs422RxQFull = 1;
		}
	}
}

void RS422B_receive(RS422RXQUE *RS422RxQue){
	int16 data;

	while(ScibRegs.SCIFFRX.bit.RXFFST != 0){// rs422 rx fifo is not empty
		data = ScibRegs.SCIRXBUF.all;
		if(EnQueueB(data, RS422RxQue) == 0){
			gSysState.alarm.bit.rs422RxQFull = 1;
		}
	}

}
/***************************************************************
 *Name:						CalCrc
 *Function:
 *Input:				    rs422 rx data
 *Output:					int, should be zero
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
int CalCrc(int crc, const char *buf, int len){
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
/***************************************************************
 *Name:						findhead
 *Function:
 *Input:				    none
 *Output:					1 or 0, 1 means find the head , 0 means failed
 *Author:					Simon
 *Date:						2018.10.25
 ****************************************************************/
int findhead(RS422RXQUE *RS422RxQue){

	char head1;
	char head2;
#if COMPARE_A_AND_B
	while(1){

		head1 = RS422RxQue->rxBuff[RS422RxQue->front];
		head2 = RS422RxQue->rxBuff[(RS422RxQue->front + 1) % MAXQSIZE];

		if(head1 == HEAD1 && head2 == HEAD2){
			if(gRS422RxQueB.rxBuff[RS422RxQue->front] == HEAD1 && gRS422RxQueB.rxBuff[(RS422RxQue->front + 1) % MAXQSIZE] == HEAD2){
				gRS422RxQueB.front = RS422RxQue->front;
				return SUCCESS;
			}
			else{
				//printf("RS422B NOT FIND HEAD\r\n");
				return FAIL;
			}
		}

		if(DeQueue(RS422RxQue) == 0){
			//printf("rs422 rx queue is empty\r\n");
			return FAIL;
		}
	}


#else

	while(1){

		//printf("front = %d\r\n",RS422RxQue->front );
		//printf("rear = %d\r\n",RS422RxQue->rear );
//		if(RS422RxQueLength(RS422RxQue) < 1){
//			//printf("-------------------------------------data not enough to unpak, so do not find head\r\n");
//			return FAIL;
//		}
		head1 = RS422RxQue->rxBuff[RS422RxQue->front];
		head2 = RS422RxQue->rxBuff[(RS422RxQue->front + 1) % MAXQSIZE];

		if(head1 == HEAD1 && head2 == HEAD2){
			return SUCCESS;
		}

		if(DeQueue(RS422RxQue) == 0){
			//printf("rs422 rx queue is empty\r\n");
			return FAIL;
		}
	}
#endif
}
/***************************************************************
 *Name:						findtail
 *Function:
 *Input:				    length
 *Output:					1 or 0, 1 means find the head , 0 means failed
 *Author:					Simon
 *Date:						2018.10.25
 ****************************************************************/
int findtail(int len, RS422RXQUE *RS422RxQue){
	char tail1;
	char tail2;

	tail1 = RS422RxQue->rxBuff[(RS422RxQue->front + len - 1) % MAXQSIZE];
	tail2 = RS422RxQue->rxBuff[(RS422RxQue->front + len - 2) % MAXQSIZE];

	if(tail1 == TAIL1 && tail2 == TAIL2){
		return SUCCESS;
	}
	else{
		return FAIL;
	}
}
/***************************************************************
 *Name:						checklength
 *Function:
 *Input:				    none
 *Output:					1 or 0, 1 means success, 0 means failed
 *Author:					Simon
 *Date:						2018.10.25
 ****************************************************************/
int checklength(RS422RXQUE *RS422RxQue){
#if COMPARE_A_AND_B

	if((RS422RxQue->rxBuff[(RS422RxQue->front + 2) % MAXQSIZE] * UNIT_LEN + EXTRA_LEN) < RS422RxQueLength()){
		if((gRS422RxQueB.rxBuff[(RS422RxQue->front + 2) % MAXQSIZE] * UNIT_LEN + EXTRA_LEN) < RS422RxQueLengthB()){
			//printf("RS422 A AND B CHANNEL LENGTH IS ENOUGH!!!!!\r\n");
			return SUCCESS;
		}
		else{
			//printf("RS422 B channel length not enough\r\n");
			return FAIL;
		}
	}
	else{
		return FAIL;
	}

#else

	if((RS422RxQue->rxBuff[(RS422RxQue->front + 2) % MAXQSIZE] * UNIT_LEN + EXTRA_LEN) <= RS422RxQueLength(RS422RxQue)){
		return SUCCESS;
	}
	else{
		return FAIL;
	}
#endif
}
/***************************************************************
 *Name:						saveprofile
 *Function:
 *Input:				    length
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.27
 ****************************************************************/
void saveprofile(int len, RS422RXQUE *RS422RxQue){
	int i;

	for(i = 0; i < len; ++i){
		rs422rxPack[i] = RS422RxQue->rxBuff[(RS422RxQue->front + i) % MAXQSIZE];
	}
}
/***************************************************************
 *Name:						unpack
 *Function:					unpack profile data
 *Input:				    length
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.27
 ****************************************************************/
void unpack(int len){
	int i;
	int msgCode;
	VAR16 var16;

	for(i = 0; i < len; ++i){

		msgCode = rs422rxPack[OFFSET + UNIT_LEN * i];
		var16.datahl.h = rs422rxPack[OFFSET + UNIT_LEN*i + 1];
		var16.datahl.l = rs422rxPack[OFFSET + UNIT_LEN*i + 2];
		//var16.value = var16.datahl.l + (var16.datahl.h << 8);

		if(gRS422Status.rs422CurrentChannel == RS422_CHANNEL_A){
			gRS422Status.rs422A = 1;
		}
		else if(gRS422Status.rs422CurrentChannel == RS422_CHANNEL_B){
			gRS422Status.rs422B = 1;
		}

		if(msgCode < (sizeof(msgInterface) / sizeof(msgInterface[0]))){
			//printf("msgCode = %d\r\n",msgCode);
			if(msgInterface[msgCode]){
				msgInterface[msgCode](var16, 0, 0);
			}
		}
		else{
			//printf("unpack msg code is out of range\r\n");
		}
	}
}
/**************************************************************
 *Name:		   CompareRS422AandB
 *Comment:
 *Input:	   Uint16 length
 *Output:	   Uint16
 *Author:	   Simon
 *Date:		   2018.11.14
 **************************************************************/
Uint16 CompareRS422AandB(Uint16 len, RS422RXQUE *RS422RxQue){
	int16 i;

	for (i = 0; i < len; ++i) {
		//printf("gRS422RxQue = %d\r\n",RS422RxQue->rxBuff[(RS422RxQue->front + i) % MAXQSIZE]);
		//printf("gRS422RxQueB= %d\r\n",gRS422RxQueB.rxBuff[(RS422RxQue->front + i) % MAXQSIZE]);
		if(RS422RxQue->rxBuff[(RS422RxQue->front + i) % MAXQSIZE] != gRS422RxQueB.rxBuff[(RS422RxQue->front + i) % MAXQSIZE]){
			//printf("position = %d\r\n", i);
			//printf("RS422RxQue->rxBuff = %d\r\n",RS422RxQue->rxBuff[(RS422RxQue->front + i) % MAXQSIZE]);
			//printf("gRS422RxQueB.rxBuff = %d\r\n",gRS422RxQueB.rxBuff[(RS422RxQue->front + i) % MAXQSIZE]);
			return FAIL;
		}
	}
	return SUCCESS;
}
/***************************************************************
 *Name:						updatehead
 *Function:					move the front head to another position
 *Input:				    length
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.27
 ****************************************************************/
void updatehead(int len, RS422RXQUE *RS422RxQue){
	RS422RxQue->front = (RS422RxQue->front + len) % MAXQSIZE;
}
/**************************************************************
 *Name:		   UpdateRS422RxSerialNumber
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018.11.14
 **************************************************************/
void UpdateRS422RxSerialNumber(void){

	gRS422Status.currentSerialNumber = (rs422rxPack[3] << 8) + rs422rxPack[4];
	//printf("currentSerialNumber = %d\r\n", gRS422Status.currentSerialNumber);

}
/***************************************************************
 *Name:						UnpackRS422ANew
 *Function:					unpack the hole data package
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.27
 ****************************************************************/
void UnpackRS422ANew(RS422RXQUE *RS422RxQue){
	int length;
	while(RS422RxQueLength(RS422RxQue) > EXTRA_LEN){
		if(findhead(RS422RxQue) == FAIL){
			//printf("find head failed\r\n");
			return;
		}
		else{
			//printf("find head succeed\r\n");
		}

		if(checklength(RS422RxQue) == FAIL){
			//printf("len received =%d\r\n", RS422RxQue->rxBuff[(RS422RxQue->front + 2) % MAXQSIZE] * UNIT_LEN + EXTRA_LEN );
			//printf("len calculate =%d\r\n", RS422RxQueLength(RS422RxQue));
			//printf("data length is not enough, waiting for more data\r\n");
			return;
		}
		else{
			//printf("Check data length succeed, begin to check tail\r\n");
		}

		length = RS422RxQue->rxBuff[(RS422RxQue->front + 2) % MAXQSIZE] * UNIT_LEN + EXTRA_LEN;

	#if COMPARE_A_AND_B

		if(CompareRS422AandB(length, RS422RxQue) == FAIL){
			//printf("Data in RS422 A Channel are not the same with B channel \r\n");
		}
		else{
			//printf("CompareRS422AandB SUCCESS\r\n");
		}
	#endif

		if(findtail(length,RS422RxQue) == FAIL){
			//printf("find tail failed\r\n");
			if(DeQueue(RS422RxQue) == 0){
				//printf("RS422 rx queue is empty\r\n");
			}
			return;
		}
		else{
			//printf("find tail succeed\r\n");
		}

		saveprofile(length,RS422RxQue);

		if(CalCrc(0, rs422rxPack + OFFSET, length - EXTRA_LEN + 2) != 0){
			if(DeQueue(RS422RxQue) == 0){
				//printf("RS422 rx queue is empty\r\n");
			}
			//printf("CRC check failed\r\n");
			return;
		}
		else{
			//printf("CRC check succeed\r\n");
		}

		unpack(RS422RxQue->rxBuff[(RS422RxQue->front + 2) % MAXQSIZE]);
		UpdateRS422RxSerialNumber();
		updatehead(length, RS422RxQue);
//		printf("update the front position-------------\r\n");
	}
}
/**************************************************************
 *Name:		   ClearRS422RxOverFlow
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018.11.15
 **************************************************************/
void ClearRS422RxOverFlow(void) {
	if (ScibRegs.SCIFFRX.bit.RXFFOVF == 1) {
		// printf(">>>>>>scib rx fifo over flow\r\n");
		ScibRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
		ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
		if (ScibRegs.SCIFFRX.bit.RXFFOVF == 0) {
//			printf(">>scib clear fifo over flow flag\r\n");
		}
	}
}

/*************New Protocol for the Joystick********************/
#define HEAD1_NEW 0xAA
#define HEAD2_NEW 0x55
#define LENGHT_NEW 0x1a
#define EXTRA_LEN_NEW 0x0d
#define OFFSET_NEW 0x03
#define UNIT_LEN_NEW 0x02

void UpdateStartForce(VAR16 a)
{

}

void UpdateFriction(VAR16 a)
{

}

void UpdateEmptyDistance(VAR16 a)
{

}

void UpdateK(VAR16 a)
{

}

void UpdateTimeDelay(VAR16 a)
{

}

int FindHead_New(RS422RXQUE *RS422RxQue)
{
	char head1;
	char head2;


	while(1){

		head1 = RS422RxQue->rxBuff[RS422RxQue->front];
		head2 = RS422RxQue->rxBuff[(RS422RxQue->front + 1) % MAXQSIZE];

		if(head1 == HEAD1_NEW && head2 == HEAD2_NEW){

			return SUCCESS;
		}

		if(DeQueue(RS422RxQue) == 0){
			//printf("rs422 rx queue is empty\r\n");
			return FAIL;
		}
	}
}

int CheckLength_New(RS422RXQUE *RS422RxQue){

	if(LENGHT_NEW <= RS422RxQueLength(RS422RxQue)){
		return SUCCESS;
	}
	else{
		return FAIL;
	}
}


int CheckSum_New(const char *buf, int len){
	int i = 0;
	Uint16 sum = 0;
	Uint16 rxSum = 0;

	rxSum = buf[LENGHT_NEW - 2];
	rxSum = rxSum << 8;
	rxSum = rxSum | buf[LENGHT_NEW - 1];


	for(i = 0; i < len - 2; ++i)
	{
		sum += buf[i];
	}

	if(sum == rxSum)
	{
		return 0;
	}

	return 1;
}
Uint16 gTT[6] = {0};

void Unpack_New(int len){
// update the value from the host side

	int16 unitCode;
	int16 startForce;
	int16 friction;
	int16 emptyDistance;
	int16 k;
	int16 timeDelay;

	VAR16 var16;

	var16.datahl.h = rs422rxPack[OFFSET_NEW + UNIT_LEN_NEW*0 + 1];
	var16.datahl.l = rs422rxPack[OFFSET_NEW + UNIT_LEN_NEW*0 + 2];
	unitCode = var16.value;
	gTT[0] = unitCode;

	if(unitCode != 1)
	{
		return;
	}

	var16.datahl.h = rs422rxPack[OFFSET_NEW + UNIT_LEN_NEW*1 + 1];
	var16.datahl.l = rs422rxPack[OFFSET_NEW + UNIT_LEN_NEW*1 + 2];
	startForce = var16.value;
	gTT[1] = startForce;

	var16.datahl.h = rs422rxPack[OFFSET_NEW + UNIT_LEN_NEW*2 + 1];
	var16.datahl.l = rs422rxPack[OFFSET_NEW + UNIT_LEN_NEW*2 + 2];
	friction = var16.value;
	gTT[2] = friction;

	var16.datahl.h = rs422rxPack[OFFSET_NEW + UNIT_LEN_NEW*3 + 1];
	var16.datahl.l = rs422rxPack[OFFSET_NEW + UNIT_LEN_NEW*3 + 2];
	emptyDistance = var16.value;
	gTT[3] = emptyDistance;

	var16.datahl.h = rs422rxPack[OFFSET_NEW + UNIT_LEN_NEW*4 + 1];
	var16.datahl.l = rs422rxPack[OFFSET_NEW + UNIT_LEN_NEW*4 + 2];
	k = var16.value;
	gTT[4] = k;

	var16.datahl.h = rs422rxPack[OFFSET_NEW + UNIT_LEN_NEW*5 + 1];
	var16.datahl.l = rs422rxPack[OFFSET_NEW + UNIT_LEN_NEW*5 + 2];
	timeDelay = var16.value;
	gTT[5] = timeDelay;
}

void UnpackRS422A_New(RS422RXQUE *RS422RxQue){

	while(RS422RxQueLength(RS422RxQue) > EXTRA_LEN_NEW){
		if(FindHead_New(RS422RxQue) == FAIL){
			return;
		}


		if(CheckLength_New(RS422RxQue) == FAIL){
		    PieCtrlRegs.PIEIER9.bit.INTx3 = 0;
		    RS422B_receive(&gRS422RxQueB);
		    PieCtrlRegs.PIEIER9.bit.INTx3 = 1;
			return;
		}



		// length for the new protocol is a fixed value

		saveprofile(LENGHT_NEW, RS422RxQue);

		if(CheckSum_New(rs422rxPack, LENGHT_NEW) != 0){
			if(DeQueue(RS422RxQue) == 0){

			}
			return;
		}

		Unpack_New(LENGHT_NEW);

		updatehead(LENGHT_NEW, RS422RxQue);
	}
}
