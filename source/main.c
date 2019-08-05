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

Uint16 currentRefCollect[100] = {0};
Uint16 voltageRefCollect[100] = {0};

enum FSM {
	INIT = 0,
	ZERO,
	PASSIVE,
	DAMP,
	SLAVE,
	FREEZE,
	ALARM
};

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
	POWER_BOARD_TOOGLE_WATCHDOG = TRUE;
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
        gSysMonitorVar.anolog.single.var[index].max2nd =
                anologMaxMinInit[index][1];
		gSysMonitorVar.anolog.single.var[index].min =
				anologMaxMinInit[index][2];
        gSysMonitorVar.anolog.single.var[index].min2nd =
                anologMaxMinInit[index][3];

		gSysMonitorVar.anolog.single.var[index].count_max = 0;
		gSysMonitorVar.anolog.single.var[index].count_min = 0;
	}
	gSysMonitorVar.digit.multi.var[8].valueN = 55;
	gSysMonitorVar.digit.multi.var[8].valueP = 55;

	for (index = 0; index < AD16bit_Total; ++index) {
	    gSysMonitorVar.anolog.AD_16bit.var[index].max =
	            AD16bitMaxMinInit[index][0];
	    gSysMonitorVar.anolog.AD_16bit.var[index].max2nd =
	            AD16bitMaxMinInit[index][1];
	    gSysMonitorVar.anolog.AD_16bit.var[index].min =
	            AD16bitMaxMinInit[index][2];
	    gSysMonitorVar.anolog.AD_16bit.var[index].min2nd =
	            AD16bitMaxMinInit[index][3];
	}

	for (index = 0; index < 12; ++index) {
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
	gRS422Status.rs422CurrentChannel = RS422_CHANNEL_B;
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
	Init_feedbackVarBuf();
	Init_gSysMonitorVar();
	Init_gRS422Status();
	InitConfigParameter();
	InitgRx422TxVar();
	InitgRx422TxEnableFlag();
	gKeyValue.displacement = 10;
	gKeyValue.lock = 0;
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

inline void Check_Power28V_M(){
	if(gSysMonitorVar.anolog.single.var[Power28V_M].value > gSysMonitorVar.anolog.single.var[Power28V_M].min) {
		++gSysMonitorVar.anolog.single.var[Power28V_M].count_min;
	}
	else{
		if(gSysMonitorVar.anolog.single.var[Power28V_M].count_min > 0)
			--gSysMonitorVar.anolog.single.var[Power28V_M].count_min;
		else{
			gSysMonitorVar.anolog.single.var[Power28V_M].count_min = 0;
		}
	}

	if(gSysMonitorVar.anolog.single.var[Power28V_M].count_min > VOLTAGE_ABNORMAL_COUNT){
		// TODO generate alarm message and open XIEFANG
		//GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;
	}
	else{
		gSysMonitorVar.anolog.single.var[Power28V_M].count_max = 0;
		return;
	}

	if(gSysMonitorVar.anolog.single.var[Power28V_M].value > gSysMonitorVar.anolog.single.var[Power28V_M].max) {
		++gSysMonitorVar.anolog.single.var[Power28V_M].count_max;
	}
	else{
		if(gSysMonitorVar.anolog.single.var[Power28V_M].count_max > 0)
			--gSysMonitorVar.anolog.single.var[Power28V_M].count_max;
		else{
			gSysMonitorVar.anolog.single.var[Power28V_M].count_max = 0;
		}
	}

	if(gSysMonitorVar.anolog.single.var[Power28V_M].count_max > VOLTAGE_ABNORMAL_COUNT){
		// TODO generate alarm message and open XIEFANG and diable output
		//GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;
		//EPwm1Regs.AQCSFRC.all = 0x0009; //DisablePwm1();
		//EPwm2Regs.AQCSFRC.all = 0x0009; //DisablePwm2();
		//EPwm3Regs.AQCSFRC.all = 0x0009; //DisablePwm3();
	}
}

inline void Check_Power28V(){
	if(gSysMonitorVar.anolog.single.var[Power28V].value > gSysMonitorVar.anolog.single.var[Power28V].min) {
		++gSysMonitorVar.anolog.single.var[Power28V].count_min;
	}
	else{
		if(gSysMonitorVar.anolog.single.var[Power28V].count_min > 0)
			--gSysMonitorVar.anolog.single.var[Power28V].count_min;
		else{
			gSysMonitorVar.anolog.single.var[Power28V].count_min = 0;
		}
	}

	if(gSysMonitorVar.anolog.single.var[Power28V].count_min > VOLTAGE_ABNORMAL_COUNT){
		// TODO generate alarm message and open XIEFANG
		//GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;
	}
	else{
		gSysMonitorVar.anolog.single.var[Power28V].count_max = 0;
		return;
	}

	if(gSysMonitorVar.anolog.single.var[Power28V].value > gSysMonitorVar.anolog.single.var[Power28V].max) {
		++gSysMonitorVar.anolog.single.var[Power28V].count_max;
	}
	else{
		if(gSysMonitorVar.anolog.single.var[Power28V].count_max > 0)
			--gSysMonitorVar.anolog.single.var[Power28V].count_max;
		else{
			gSysMonitorVar.anolog.single.var[Power28V].count_max = 0;
		}
	}

	if(gSysMonitorVar.anolog.single.var[Power28V].count_max > VOLTAGE_ABNORMAL_COUNT){
		// TODO generate alarm message and open XIEFANG and diable output
		//GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;
		//EPwm1Regs.AQCSFRC.all = 0x0009; //DisablePwm1();
		//EPwm2Regs.AQCSFRC.all = 0x0009; //DisablePwm2();
		//EPwm3Regs.AQCSFRC.all = 0x0009; //DisablePwm3();
	}
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
	StateMachine();
//	UpdateForceDisplaceCurve();

	Check_Power28V_M();

	Check_Power28V();

	if(IsCommonAnalogValueAbnormal() == TRUE){
		//TODO, generate alarm and notice uppper computer
	}

	RS422Unpack();

	ClearRS422RxOverFlow();
	//TODO need to implement

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
	DisablePwmOutput();
	SET_DIGIT_SER_LOAD_HIGH;
	SET_DIGIT_SER_CLK_LOW;
	//gSysInfo.currentHallPosition = 5;

	gSysInfo.currentHallPosition = GetCurrentHallValue();
	gSysInfo.lastTimeHalllPosition = gSysInfo.currentHallPosition;

	gSysState.erro.bit.software = 0;
	gConfigPara.stateCommand = 0;
	//gConfigPara.stateCommand = 1;
	gSysInfo.duty = 0;
	//gSysInfo.duty = 100;

	Init_Interrupt();
	ClearFault();
	PowerOnBIT();
//	for(i = 0; i<100; i++){
//		while(AdcRegs.ADCST.bit.INT_SEQ1==0){
//
//		}
//		currentRefCollect[i] = AdcRegs.ADCRESULT1;
//		voltageRefCollect[i] = AdcRegs.ADCRESULT2;
//		AdcRegs.ADCST.bit.INT_SEQ1_CLR=1;
//	}
	//GpioDataRegs.GPCCLEAR.bit.GPIO84 = 1;
	GpioDataRegs.GPCCLEAR.bit.GPIO84 = 1;
	//GpioDataRegs.GPASET.bit.GPIO6 = 1;

	while(1)
	{
#if TEST_TIME_MAIN_LOOP
		GpioDataRegs.GPCSET.bit.GPIO82 = 1;
#endif

		Start_main_loop();

#if TEST_TIME_MAIN_LOOP
		GpioDataRegs.GPCCLEAR.bit.GPIO82 = 1;
#endif
	}
}
