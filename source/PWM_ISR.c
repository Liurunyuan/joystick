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

Uint16 real = 0;
Uint16 realbak = 0;
Uint16 real2 = 0;
Uint16 real3 = 0;

Uint16 real5 = 0;


int16 countreal = 0;
Uint16 zz[400] = {0};

void UpdateKeyValue(void) {

	funcParaDisplacement = calFuncPara(sumParaDisplacement);
	gKeyValue.displacement = funcParaDisplacement.a * 100 + funcParaDisplacement.b * 10 + funcParaDisplacement.c;

	gKeyValue.motorSpeed = KalmanFilterSpeed((funcParaDisplacement.a * 10 + funcParaDisplacement.b), KALMAN_Q, KALMAN_R);
	//gKeyValue.motorSpeed = (funcParaDisplacement.a * 40) + (funcParaDisplacement.b);
	gKeyValue.motorAccel = 2 * funcParaDisplacement.a;
}

/**************************************************************
 *Name:						CalForceSpeedAccel
 *Function:
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.28
 **************************************************************/
void CalForceSpeedAccel(void) {

	static int count = 0;

	if(gKeyValue.lock == 1){
		return;
	}
	CalFuncPara(gSysMonitorVar.anolog.single.var[ForceValue].value, gSysMonitorVar.anolog.single.var[DisplacementValue].value, count);
	++count;

	if(count >= DATA_AMOUNT){
		gKeyValue.lock = 1;
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

	temp = ((c << 2) + (b << 1) + a);

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
	gSysInfo.currentHallPosition = GetCurrentHallValue();
	//3:A 2:B 1:C
	switch (gSysInfo.currentHallPosition) {
		case 4://C+ ---------------> B-
			//本项目电机会进行正转和反转。所以需要判断HALL相邻两个位置是否一样。
			if((4 == gSysInfo.lastTimeHalllPosition )
				|| (5 == gSysInfo.lastTimeHalllPosition)
				|| (6 == gSysInfo.lastTimeHalllPosition)){

				CPositiveToBNegtive();
			}
			break;
		case 6://C+ ---------------> A-
			if((6 == gSysInfo.lastTimeHalllPosition )
				|| (4 == gSysInfo.lastTimeHalllPosition)
				|| (2 == gSysInfo.lastTimeHalllPosition)){

				CPositiveToANegtive();
			}
			break;
		case 2://B+ ---------------> A-
			if((2 == gSysInfo.lastTimeHalllPosition )
				|| (6 == gSysInfo.lastTimeHalllPosition)
				|| (3 == gSysInfo.lastTimeHalllPosition)){

				BPositiveToANegtive();
			}
			break;
		case 3://B+ ---------------> C-
			if((3 == gSysInfo.lastTimeHalllPosition )
				|| (2 == gSysInfo.lastTimeHalllPosition)
				|| (1 == gSysInfo.lastTimeHalllPosition)){
				BPositiveToCNegtive();
			}
			break;
		case 1://A+ ---------------> C-
			if((1 == gSysInfo.lastTimeHalllPosition )
				|| (3 == gSysInfo.lastTimeHalllPosition)
				|| (5 == gSysInfo.lastTimeHalllPosition)){

				APositiveToCNegtive();
			}
			break;
		case 5://A+ ---------------> B-
			if((5 == gSysInfo.lastTimeHalllPosition )
				|| (1 == gSysInfo.lastTimeHalllPosition)
				|| (4 == gSysInfo.lastTimeHalllPosition)){

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
	static int count = 0;

	StartGetADBySpi();

	if(count >= 400){
		count = 0;
	}

	ReadDigitalValue();


	ReadAnalogValue();

	if(IsSingleAnalogValueAbnormal() == True){
		//TODO  不着急的量放进主循环，这里只判断电流以及高速
	}
	SwitchDirection();
	ReadADBySpi();

	if(real2 > 400){
		++countreal;
		real5 = real2;
	}

	gSysMonitorVar.anolog.single.var[DisplacementValue].value = real;
	gSysMonitorVar.anolog.single.var[DisplacementValue].value = (int)(KalmanFilter(real, KALMAN_Q, KALMAN_R));

	CalForceSpeedAccel();
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
