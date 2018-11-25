/*
 * main.c
 *
 *
 * Author:					Simon
 * Date:					2018.6.7
 * Corporation:				RunZhang
 *
 * */
#include <string.h>
#include <stdio.h>
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "main.h"
#include "SCI_ISR.h"
#include "ADprocessor.h"
#include "SCI_TX.h"
#include "PWM_ISR.h"
#include "GlobalVarAndFunc.h"


#define UART_PRINTF

#ifdef UART_PRINTF

int fputc(int _c, register FILE *_fp);
int fputs(const char *_ptr, register FILE *_fp);

#endif

#ifdef UART_PRINTF

int fputc(int _c, register FILE *_fp){
	while(ScicRegs.SCIFFTX.bit.TXFFST != 0){
	}

	ScicRegs.SCITXBUF = (unsigned char) _c;

	return ((unsigned char) _c);
}

int fputs(const char *_ptr, register FILE *_fp){
	unsigned int i, len;

	len = strlen(_ptr);

	for(i = 0; i < len; ++i){
		while(ScicRegs.SCIFFTX.bit.TXFFST != 0){
		}
		ScicRegs.SCITXBUF =(unsigned char) _ptr[i];
	}

	return len;
}

#endif
/************************************************************
 *Name:						Init_Peripheral
 *Function:					Initialize all the peripherals,
							including ADC,SCI,PWM,GPIO initialization and configuration
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.6.10
 ************************************************************/
void Init_Peripheral(void){
	/*Init IO pin */
	Init_GPIO();
	/*Init and config ADC*/
	Init_ADC();
	/*Init and cnofig XINTF*/
	Init_XINTF();
	/*Init and config SCI RS422*/
	Init_SCI();
	/*Init and config SPI*/
	Init_SPI();
	/*Init and config A-CAN and B-CAN */
	Init_CAN();
	/*Init and config I2C*/
	Init_I2C();
	/*Init and config CAP4,CAP5,CAP6*/
	Init_CAP();
	/*Init and config QEP2*/
	Init_QEP();
	/*PWM IO init and config*/
	Init_PWM();
	/*DMA init and config*/
	Init_DMA();
}

/*************************************************************
 *Name:						FeedWatchDog
 *Function:					Feed the watch dog
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.6.10
 *************************************************************/
void FeedWatchDog(void){
	TOOGLE_WATCHDOG = TRUE;
}

void Delayfunc(Uint16 sec){
	Uint16 i;
	Uint16 j;

	for(i = 0; i < 10000; ++i){
		for(j = 0; j < sec; ++j){
			asm(" NOP");
		}
	}
}
int PowerOnBIT(void){
	//TODO   implement here, figure out what need to check, what to do if BIT fail.
	return 0;
}
/**************************************************************
 *Name:						Start_main_loop
 *Function:					Business logic
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.6.10
 **************************************************************/
void Start_main_loop(void){

	FeedWatchDog();
	//TODO need to implement
}
/**************************************************************
 *Name:						test_spi_tx
 *Function:					Business logic
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.28
 **************************************************************/
void test_spi_tx(void){
	int retry = 0;
	while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG == 1){
		retry ++;
		if(retry > 200){
				//return 0;
		}
	}
	SpiaRegs.SPITXBUF = 0x0001;
}
/**************************************************************
 *Name:						Init_gRS422RxQue
 *Function:
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.28
 **************************************************************/
void Init_gRS422RxQue(void) {
	gRS422RxQue.front = 0;
	gRS422RxQue.rear = 0;
	memset(gRS422RxQue.rxBuff, 0, sizeof(gRS422RxQue.rxBuff));

	gRS422RxQueB.front = 0;
	gRS422RxQueB.rear = 0;
	memset(gRS422RxQueB.rxBuff, 0, sizeof(gRS422RxQueB.rxBuff));
}
/**************************************************************
 *Name:						Init_gRS422TxQue
 *Function:
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.28
 **************************************************************/
void Init_gRS422TxQue(void) {
	gRS422TxQue.front = 0;
	gRS422TxQue.rear = 0;
	memset(gRS422TxQue.txBuf, 0, sizeof(gRS422TxQue.txBuf));
}
/**************************************************************
 *Name:						Init_gRx422TxVar
 *Function:
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.28
 **************************************************************/
void Init_gRx422TxVar(void) {
	int index;

	memset(gRx422TxVar, 0, sizeof(gRx422TxVar));
	memset(gRx422TxEnableFlag, 0, sizeof(gRx422TxEnableFlag));
	for (index = 0; index < 20; ++index) {

		gRx422TxVar[index].isTx = 1;
		gRx422TxEnableFlag[index] = 1;
		gRx422TxVar[index].index = index;

	}
}
/**************************************************************
 *Name:						Init_feedbackVarBuf
 *Function:
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.28
 **************************************************************/
void Init_feedbackVarBuf(void) {
	int index;

	feedbackVarBuf.maxDisplacement = 0;
	feedbackVarBuf.maxForce = 0;
	feedbackVarBuf.minDisplacement = 0;
	feedbackVarBuf.minForce = 0;
	feedbackVarBuf.sumDisplacement = 0;
	feedbackVarBuf.sumForce = 0;
	memset(feedbackVarBuf.displacementbuf, 0,sizeof(feedbackVarBuf.displacementbuf));
	memset(feedbackVarBuf.forcebuf, 0, sizeof(feedbackVarBuf.forcebuf));
	for (index = 0; index < 10; ++index) {
		feedbackVarBuf.displacementbuf[index] = index * index + 3 * index + 2;
		feedbackVarBuf.forcebuf[index] = index;
	}
}
/**************************************************************
 *Name:						Init_gSysMonitorVar
 *Function:
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.28
 **************************************************************/
void Init_gSysMonitorVar() {
	int index;
	for (index = 0; index < TotalChannel; ++index) {
		gSysMonitorVar.anolog.single.var[index].updateValue = funcptr[index];
		gSysMonitorVar.anolog.single.var[index].max =
				anologMaxMinInit[index][0];
		gSysMonitorVar.anolog.single.var[index].min =
				anologMaxMinInit[index][1];
	}
	for (index = 0; index < 12; ++index) {
		//gSysMonitorVar.digit.single.var[index].valueP = gSysMonitorVar.digit.single.var[index].updateValue();
	}
}
/**************************************************************
 *Name:		   Init_gRS422Status
 *Comment:
 *Input:	   none
 *Output:	   none
 *Author:	   Simon
 *Date:		   2018.11.14
 **************************************************************/
void Init_gRS422Status(void){
	gRS422Status.rs422A = 1;
	gRS422Status.rs422B = 1;
	gRS422Status.currentSerialNumber = 0;
	gRS422Status.rs422CurrentChannel = RS422_CHANNEL_A;
	gRS422Status.shakeHand = FAIL;
}
/***************************************************************
 *Name:						GlobleVarInit
 *Function:
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.20
 ****************************************************************/
void InitGlobalVar(void){

	Init_gRS422RxQue();
	Init_gRS422TxQue();
	Init_gRx422TxVar();
	Init_feedbackVarBuf();
	Init_gSysMonitorVar();
	Init_gRS422Status();
}
/**************************************************************
 *Name:		   RS422Unpack
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018.11.14
 **************************************************************/
void RS422Unpack(void) {
	if (gRS422Status.rs422CurrentChannel == RS422_CHANNEL_A) {
		UnpackRS422ANew(&gRS422RxQue);
	}
	else if (gRS422Status.rs422CurrentChannel == RS422_CHANNEL_B) {
		UnpackRS422ANew(&gRS422RxQueB);
	}
}
/***************************************************************
 *Name:						main
 *Function:
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.6.10
 ****************************************************************/
void main(void) {

	/*system init*/
	InitSysCtrl_M();
	/*peripheral init*/
	Init_Peripheral();

	InitGlobalVar();
	/*interrupt init*/
	Init_Interrupt();

	PowerOnBIT();

	gSysInfo.currentHallPosition = 6;
	gSysInfo.duty = 200;

	//GpioDataRegs.GPCCLEAR.bit.GPIO84 = 1;
	GpioDataRegs.GPCCLEAR.bit.GPIO84 = 1;

	while(1)
	{
#if TEST_TIME_MAIN_LOOP
		GpioDataRegs.GPCSET.bit.GPIO82 = 1;
#endif

		Start_main_loop();

		ShakeHandWithUpperComputer();

		//test_spi_tx();

		RS422Unpack();

		ClearRS422RxOverFlow();

#if TEST_TIME_MAIN_LOOP
		GpioDataRegs.GPCCLEAR.bit.GPIO82 = 1;
#endif
	}
}
