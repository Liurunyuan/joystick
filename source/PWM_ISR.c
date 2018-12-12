#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "PWM_ISR.h"
#include "ADprocessor.h"
#include "Filter_Alg.h"
#include "SPIprocess.h"
#include "GlobalVarAndFunc.h"


FeedbackVarBuf feedbackVarBuf;
void ForceAndDisplaceProcess(int count);
Uint16 test_data[160] = {0};
Uint16 al[160] = {0};
int alb[160] = {0};
Uint16 alc[160] = {0};
Uint16 kal[160] = {0};
int tmp = 0;

/**************************************************************
 *Name:						CalForceSpeedAccel
 *Function:
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.28
 **************************************************************/
void CalForceSpeedAccel(void) {
	static int countb = 0;
	static int count = 0;
	static int test = 0;
	static int testb = 10;
	static int next = 0;
	int i = 0;
	if(test >= 160){
		return;
		test = 0;
		next = 0;
		testb = 10;
	}

	//CalFuncPara(feedbackVarBuf.displacementbuf[count], feedbackVarBuf.forcebuf[count], count);
	CalFuncPara(gSysMonitorVar.anolog.single.var[ForceValue].value, gSysMonitorVar.anolog.single.var[DisplacementValue].value, count);
//	if(countb >= 10){
//		CalFuncParaB(gSysMonitorVar.anolog.single.var[DisplacementValue].value, countb);
//	}

//	++countb;
	++count;


//	if(countb >= 30){
//
//		for(i = 0; i < DATA_AMOUNT; ++i){
//			alc[testb + i] = ((funcParaDisplacementb.a * i * i) + (funcParaDisplacementb.b * i) + (funcParaDisplacementb.c))*1000;
//		}
//		for(i = 0; i < 10; ++i){
//			alb[next + i] = ((funcParaDisplacementb.a * i * i) + (funcParaDisplacementb.b * i) + (funcParaDisplacementb.c))*1000;
//
//		}
//		next = next + 10;
//		testb = testb + DATA_AMOUNT;
//		countb = 10;
//	}
	if(count >= DATA_AMOUNT){
//		gKeyValue.displacement = funcParaDisplacement.a * 121 + funcParaDisplacement.b * 11 + funcParaDisplacement.c;
//		gKeyValue.motorSpeed = (funcParaDisplacement.a * 22) + (funcParaDisplacement.b);
//		gKeyValue.motorAccel = 2 * funcParaDisplacement.a;

		gKeyValue.displacement = funcParaDisplacement.a * 100 + funcParaDisplacement.b * 10 + funcParaDisplacement.c;
		//gKeyValue.displacement = KalmanFilter(((funcParaDisplacement.a * 100 + funcParaDisplacement.b * 10 + funcParaDisplacement.c)), KALMAN_Q, KALMAN_R);
		gKeyValue.motorSpeed = ((funcParaDisplacement.a * 20) + (funcParaDisplacement.b));
		gKeyValue.motorSpeed = KalmanFilterSpeed(((funcParaDisplacement.a * 20) + (funcParaDisplacement.b)), KALMAN_Q, KALMAN_R);
		gKeyValue.motorAccel = 2 * funcParaDisplacement.a;

		gKeyValue.force = funcParaForce.a * 121 + funcParaForce.b * 11 + funcParaForce.c;

		al[test] = gKeyValue.displacement;
		alb[test] = gKeyValue.motorSpeed * 500;
		kal[test] = tmp;
		++test;
//		for(i = 0; i < DATA_AMOUNT; ++i){
//			al[test + i] = ((funcParaDisplacement.a * i * i) + (funcParaDisplacement.b * i) + (funcParaDisplacement.c))*1000;
//		}
//		//al[test] = gKeyValue.displacement * 100;
//		for(i = 0; i < 10; ++i){
//			alb[next + i] = ((funcParaDisplacement.a * i * i) + (funcParaDisplacement.b * i) + (funcParaDisplacement.c))*1000;
//
//		}
//		next = next + 10;
//
//		test += DATA_AMOUNT;
		count = 0;
	}
}
/**************************************************************
 *Name:						GetCurrentHallValue
 *Function:
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.31
 **************************************************************/
Uint16 GetCurrentHallValue(void){

	Uint16 temp;
	Uint16 a;
	Uint16 b;
	Uint16 c;

	c = GpioDataRegs.GPADAT.bit.GPIO27;
	b = GpioDataRegs.GPBDAT.bit.GPIO48;
	a = GpioDataRegs.GPBDAT.bit.GPIO49;

	temp = ((c << 2) + (b << 1) + a)^0x07;

	if(temp < 1 || temp >6){
		gSysState.erro.bit.software = 1;
	}
	return temp;
}
/**************************************************************
 *Name:		   CPositiveToBNegtive
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018年11月25日下午1:16:27
 **************************************************************/
inline void CPositiveToBNegtive(void) {

	EPwm3Regs.AQCSFRC.bit.CSFA = 0x01; //shutdown A phase
	EPwm3Regs.AQCSFRC.bit.CSFB = 0x01; //shutdown A phase
	EPwm1Regs.CMPA.half.CMPA = EPWM1_TIMER_HALF_TBPRD + gSysInfo.duty;
	EPwm2Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD - gSysInfo.duty;
	EPwm1Regs.AQCSFRC.bit.CSFA = 0x00;
	EPwm1Regs.AQCSFRC.bit.CSFB = 0x00;
	EPwm2Regs.AQCSFRC.bit.CSFA = 0x00;
	EPwm2Regs.AQCSFRC.bit.CSFB = 0x00;
}
/**************************************************************
 *Name:		   CPositiveToANegtive
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018年11月25日下午1:16:55
 **************************************************************/
inline void CPositiveToANegtive(void) {

	EPwm2Regs.AQCSFRC.bit.CSFA = 0x01; //shutdown B phase
	EPwm2Regs.AQCSFRC.bit.CSFB = 0x01; //shutdown B phase
	EPwm1Regs.CMPA.half.CMPA = EPWM1_TIMER_HALF_TBPRD + gSysInfo.duty;
	EPwm3Regs.CMPA.half.CMPA = EPWM1_TIMER_HALF_TBPRD - gSysInfo.duty;
	EPwm1Regs.AQCSFRC.bit.CSFA = 0x00;
	EPwm1Regs.AQCSFRC.bit.CSFB = 0x00;
	EPwm3Regs.AQCSFRC.bit.CSFA = 0x00;
	EPwm3Regs.AQCSFRC.bit.CSFB = 0x00;
}
/**************************************************************
 *Name:		   BPositiveToANegtive
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018年11月25日下午1:17:04
 **************************************************************/
inline void BPositiveToANegtive(void) {

	EPwm1Regs.AQCSFRC.bit.CSFA = 0x01; //shutdown C phase
	EPwm1Regs.AQCSFRC.bit.CSFB = 0x01; //shutdown C phase
	EPwm2Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD + gSysInfo.duty;
	EPwm3Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD - gSysInfo.duty;
	EPwm2Regs.AQCSFRC.bit.CSFA = 0x00;
	EPwm2Regs.AQCSFRC.bit.CSFB = 0x00;
	EPwm3Regs.AQCSFRC.bit.CSFA = 0x00;
	EPwm3Regs.AQCSFRC.bit.CSFB = 0x00;
}
/**************************************************************
 *Name:		   BPositiveToCNegtive
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018年11月25日下午1:17:14
 **************************************************************/
inline void BPositiveToCNegtive(void) {

	EPwm3Regs.AQCSFRC.bit.CSFA = 0x01; //shutdown A phase
	EPwm3Regs.AQCSFRC.bit.CSFB = 0x01; //shutdown A phase
	EPwm2Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD + gSysInfo.duty;
	EPwm1Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD - gSysInfo.duty;
	EPwm2Regs.AQCSFRC.bit.CSFA = 0x00;
	EPwm2Regs.AQCSFRC.bit.CSFB = 0x00;
	EPwm1Regs.AQCSFRC.bit.CSFA = 0x00;
}
/**************************************************************
 *Name:		   APositiveToCNegtive
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018年11月25日下午1:17:26
 **************************************************************/
inline void APositiveToCNegtive(void) {

	EPwm2Regs.AQCSFRC.bit.CSFA = 0x01; //shutdown B phase
	EPwm2Regs.AQCSFRC.bit.CSFB = 0x01; //shutdown B phase
	EPwm3Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD + gSysInfo.duty;
	EPwm1Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD - gSysInfo.duty;
	EPwm1Regs.AQCSFRC.bit.CSFA = 0x00;
	EPwm1Regs.AQCSFRC.bit.CSFB = 0x00;
	EPwm3Regs.AQCSFRC.bit.CSFA = 0x00;
	EPwm3Regs.AQCSFRC.bit.CSFB = 0x00;
}
/**************************************************************
 *Name:		   APositiveToBNegtive
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018年11月25日下午1:17:37
 **************************************************************/
inline void APositiveToBNegtive(void) {

	EPwm1Regs.AQCSFRC.bit.CSFA = 0x01; //shutdown C phase
	EPwm1Regs.AQCSFRC.bit.CSFB = 0x01; //shutdown C phase
	EPwm3Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD + gSysInfo.duty;
	EPwm2Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD - gSysInfo.duty;
	EPwm2Regs.AQCSFRC.bit.CSFA = 0x00;
	EPwm2Regs.AQCSFRC.bit.CSFB = 0x00;
	EPwm3Regs.AQCSFRC.bit.CSFA = 0x00;
	EPwm3Regs.AQCSFRC.bit.CSFB = 0x00;
}

/**************************************************************
 *Name:						SwitchDirection
 *Function:
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.31
 **************************************************************/
void SwitchDirection(void){
	gSysInfo.lastTimeHalllPosition = gSysInfo.currentHallPosition;
	//gSysInfo.currentHallPosition = GetCurrentHallValue();
	//3:A 2:B 1:C
	switch (gSysInfo.currentHallPosition) {
		case 4://C+ ---------------> B-
			//本项目电机会进行正转和反转。所以需要判断HALL相邻两个位置是否一样。
			if((4 == gSysInfo.lastTimeHalllPosition ) || (5 == gSysInfo.lastTimeHalllPosition)){
				CPositiveToBNegtive();
			}
			break;
		case 6://C+ ---------------> A-
			if((6 == gSysInfo.lastTimeHalllPosition ) || (4 == gSysInfo.lastTimeHalllPosition)){
				CPositiveToANegtive();
			}
			break;
		case 2://B+ ---------------> A-
			if((2 == gSysInfo.lastTimeHalllPosition ) || (6 == gSysInfo.lastTimeHalllPosition)){
				BPositiveToANegtive();
			}
			break;
		case 3://B+ ---------------> C-
			if((3 == gSysInfo.lastTimeHalllPosition ) || (2 == gSysInfo.lastTimeHalllPosition)){
				BPositiveToCNegtive();
			}
			break;
		case 1://A+ ---------------> C-
			if((1 == gSysInfo.lastTimeHalllPosition ) || (3 == gSysInfo.lastTimeHalllPosition)){
				APositiveToCNegtive();
			}
			break;
		case 5://A+ ---------------> B-
			if((5 == gSysInfo.lastTimeHalllPosition ) || (1 == gSysInfo.lastTimeHalllPosition)){
				APositiveToBNegtive();
			}
			break;
		default:
			gSysState.erro.bit.software = TRUE;
			break;
	}
}

/**************************************************************
 *Name:						Pwm_ISR_Thread
 *Function:					PWM interrupt function
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.6.10
 **************************************************************/
void Pwm_ISR_Thread(void)
{
	static int test = 0;
	static int testb = 0;

	if(test >= 3200){
		//return;
		test = 0;
	}

	StartGetADBySpi();
	//ReadAnalogValue();
	ReadDigitalValue();

	if(IsSingleAnalogValueAbnormal() == True){
		//TODO  不着急的量放进主循环，这里只判断电流以及高速
	}
	SwitchDirection();
	ReadADBySpi();

//	if(test > 15){
//		test_data[test] = gSysMonitorVar.anolog.single.var[DisplacementValue].value;
//		testb = 0;
//	}
//
	tmp = gSysMonitorVar.anolog.single.var[DisplacementValue].value;
	gSysMonitorVar.anolog.single.var[DisplacementValue].value =  KalmanFilter(gSysMonitorVar.anolog.single.var[DisplacementValue].value, KALMAN_Q, KALMAN_R);
//	kal[test] = gSysMonitorVar.anolog.single.var[DisplacementValue].value;
	//传入最小二乘法的值范围为-10 到 10
	CalForceSpeedAccel();
	++test;
}
/**************************************************************
 *Name:						forcebufProcess
 *Function:
 *Input:					none
 *Output:					force value
 *Author:					Simon
 *Date:						2018.10.28
 **************************************************************/
int32 forcebufProcess(void)
{
	return ((feedbackVarBuf.sumForce - feedbackVarBuf.maxForce - feedbackVarBuf.minForce) >> 3);
}
/**************************************************************
 *Name:						displacebufProcess
 *Function:
 *Input:					none
 *Output:					displacement value
 *Author:					Simon
 *Date:						2018.10.28
 **************************************************************/
int32 displacebufProcess(void)
{
	return ((feedbackVarBuf.sumDisplacement - feedbackVarBuf.maxDisplacement - feedbackVarBuf.minDisplacement) >> 3);
}
/**************************************************************
 *Name:						UpdateMaxAndMin
 *Function:
 *Input:					feedbackVarBuf
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.28
 **************************************************************/
void UpdateMaxAndMin(FeedbackVarBuf* feedbackVarBuf) {
	if (gSysMonitorVar.anolog.single.var[ForceValue].value
			>= feedbackVarBuf->maxForce) {
		feedbackVarBuf->maxForce =
				gSysMonitorVar.anolog.single.var[ForceValue].value;
	}
	if (gSysMonitorVar.anolog.single.var[ForceValue].value
			<= feedbackVarBuf->minForce) {
		feedbackVarBuf->minForce =
				gSysMonitorVar.anolog.single.var[ForceValue].value;
	}
	if (gSysMonitorVar.anolog.single.var[DisplacementValue].value
			>= feedbackVarBuf->maxDisplacement) {
		feedbackVarBuf->maxDisplacement =
				gSysMonitorVar.anolog.single.var[DisplacementValue].value;
	}
	if (gSysMonitorVar.anolog.single.var[DisplacementValue].value
			<= feedbackVarBuf->minDisplacement) {
		feedbackVarBuf->minDisplacement =
				gSysMonitorVar.anolog.single.var[DisplacementValue].value;
	}
}
/**************************************************************
 *Name:						ForceAndDisplaceProcess
 *Function:					PWM interrupt function
 *Input:					int count
 *Output:					none
 *Author:					Simon
 *Date:						2018.6.10
 **************************************************************/
void ForceAndDisplaceProcess(int count){
	/*
	feedbackVarBuf.forcebuf[count] = gSysMonitorVar.anolog.single.var[ForceValue].value;
	feedbackVarBuf.displacementbuf[count] = gSysMonitorVar.anolog.single.var[DisplacementValue].value;
	*/

	feedbackVarBuf.sumForce = feedbackVarBuf.sumForce + feedbackVarBuf.forcebuf[count];
	feedbackVarBuf.sumDisplacement = feedbackVarBuf.sumDisplacement + feedbackVarBuf.displacementbuf[count];

	//UpdateMaxAndMin(&feedbackVarBuf);
	if(count >= 9){
		gKeyValue.lock = 1;//need to remove after debug
		if(gKeyValue.lock == 0)
		{
			//TODO generate alarm;
			return;
		}
		gKeyValue.displacement = displacebufProcess();
		gKeyValue.force = forcebufProcess();
		gKeyValue.lock = 0;
		feedbackVarBuf.sumForce = 0;
		feedbackVarBuf.sumDisplacement =0;
	}
}
