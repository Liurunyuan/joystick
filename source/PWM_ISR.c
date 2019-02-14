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

void UpdateKeyValue(void) {

	funcParaDisplacement = calFuncPara(sumParaDisplacement);
	gKeyValue.displacement = funcParaDisplacement.a * 100 + funcParaDisplacement.b * 10 + funcParaDisplacement.c;

	gKeyValue.motorSpeed = KalmanFilterSpeed((funcParaDisplacement.a * 10 + funcParaDisplacement.b), KALMAN_Q, KALMAN_R);
	//gKeyValue.motorSpeed = (funcParaDisplacement.a * 40) + (funcParaDisplacement.b);
	gKeyValue.motorAccel = 2 * funcParaDisplacement.a;
}

/*
void DisablePwm1(void){
	//EALLOW;
	//EPwm1Regs.AQCSFRC.bit.CSFA = 1;
	//EPwm1Regs.AQCSFRC.bit.CSFB = 2;
	//EPwm1Regs.TZFRC.bit.OST = 1;
	//EDIS;

	EPwm1Regs.AQCSFRC.all = 0x0009;
}
void DisablePwm2(void){
	//EALLOW;
	//EPwm2Regs.AQCSFRC.bit.CSFA = 2;
	//EPwm2Regs.AQCSFRC.bit.CSFB = 1;
	//EPwm2Regs.TZFRC.bit.OST = 1;
	//EDIS;
	EPwm2Regs.AQCSFRC.all = 0x0009;
}
void DisablePwm3(void){
	//EALLOW;
	//EPwm3Regs.AQCSFRC.bit.CSFA = 2;
	//EPwm3Regs.AQCSFRC.bit.CSFB = 1;
	//EPwm3Regs.TZFRC.bit.OST = 1;
	//EDIS;

	EPwm3Regs.AQCSFRC.all = 0x0009;
}
void EnablePwm1(void){
	//EALLOW;
	//EPwm1Regs.AQCSFRC.bit.CSFA = 2;
	//EPwm1Regs.AQCSFRC.bit.CSFB = 3;
	//EPwm1Regs.TZCLR.all = 0x003f;
	//EDIS;
	EPwm1Regs.AQCSFRC.all = 0x000f;
}
void EnablePwm2(void){
	//EALLOW;
	//EPwm2Regs.AQCSFRC.bit.CSFA = 2;
	//EPwm2Regs.AQCSFRC.bit.CSFB = 3;
	//EPwm2Regs.TZCLR.all = 0x003f;
	//EDIS;
	EPwm2Regs.AQCSFRC.all = 0x000f;
}
void EnablePwm3(void){
	//EALLOW;
	//EPwm3Regs.AQCSFRC.bit.CSFA = 2;
	//EPwm3Regs.AQCSFRC.bit.CSFB = 3;
	//EPwm3Regs.TZCLR.all = 0x003f;
	//EDIS;
	EPwm3Regs.AQCSFRC.all = 0x000f;
}
*/
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

	a = GpioDataRegs.GPADAT.bit.GPIO27;
	b = GpioDataRegs.GPBDAT.bit.GPIO48;
	c = GpioDataRegs.GPBDAT.bit.GPIO49;

	temp = ((a << 2) + (b << 1) + c);

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
/*
inline void CPositiveToBNegtive(void) {

//	EPwm3Regs.AQCSFRC.bit.CSFA = 0x01; //shutdown A phase
//	EPwm3Regs.AQCSFRC.bit.CSFB = 0x01; //shutdown A phase
//	EPwm3Regs.AQCSFRC.all = 9;
//	EPwm1Regs.CMPA.half.CMPA = EPWM1_TIMER_HALF_TBPRD + gSysInfo.duty;
//	EPwm2Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD - gSysInfo.duty;
//	EPwm1Regs.AQCSFRC.bit.CSFA = 0x00;
//	EPwm1Regs.AQCSFRC.bit.CSFB = 0x00;
//	EPwm2Regs.AQCSFRC.bit.CSFA = 0x00;
//	EPwm2Regs.AQCSFRC.bit.CSFB = 0x00;

//	EPwm1Regs.AQCSFRC.all = 0x000f;
//	EPwm2Regs.AQCSFRC.all = 0x000f;
	DisablePwm1();
	EPwm3Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD + gSysInfo.duty;
	EPwm2Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD - gSysInfo.duty;
	EnablePwm2();
	EnablePwm3();
}
*/
/**************************************************************
 *Name:		   CPositiveToANegtive
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018年11月25日下午1:16:55
 **************************************************************/
/*
inline void CPositiveToANegtive(void) {

//	EPwm2Regs.AQCSFRC.bit.CSFA = 0x01; //shutdown B phase
//	EPwm2Regs.AQCSFRC.bit.CSFB = 0x01; //shutdown B phase
//	EPwm2Regs.AQCSFRC.all = 9;
//	EPwm1Regs.CMPA.half.CMPA = EPWM1_TIMER_HALF_TBPRD + gSysInfo.duty;
//	EPwm3Regs.CMPA.half.CMPA = EPWM1_TIMER_HALF_TBPRD - gSysInfo.duty;
//	EPwm1Regs.AQCSFRC.bit.CSFA = 0x00;
//	EPwm1Regs.AQCSFRC.bit.CSFB = 0x00;
//	EPwm3Regs.AQCSFRC.bit.CSFA = 0x00;
//	EPwm3Regs.AQCSFRC.bit.CSFB = 0x00;

//	EPwm1Regs.AQCSFRC.all = 0x000f;
//	EPwm3Regs.AQCSFRC.all = 0x000f;

	DisablePwm2();
	EPwm3Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD + gSysInfo.duty;
	EPwm1Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD - gSysInfo.duty;
	EnablePwm1();
	EnablePwm3();
}
*/
/**************************************************************
 *Name:		   BPositiveToANegtive
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018年11月25日下午1:17:04
 **************************************************************/
/*
inline void BPositiveToANegtive(void) {

//	EPwm1Regs.AQCSFRC.bit.CSFA = 0x01; //shutdown C phase
//	EPwm1Regs.AQCSFRC.bit.CSFB = 0x01; //shutdown C phase
//	EPwm1Regs.AQCSFRC.all = 9;
//	EPwm2Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD + gSysInfo.duty;
//	EPwm3Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD - gSysInfo.duty;
//	EPwm2Regs.AQCSFRC.bit.CSFA = 0x00;
//	EPwm2Regs.AQCSFRC.bit.CSFB = 0x00;
//	EPwm3Regs.AQCSFRC.bit.CSFA = 0x00;
//	EPwm3Regs.AQCSFRC.bit.CSFB = 0x00;

//	EPwm2Regs.AQCSFRC.all = 0x000f;
//	EPwm3Regs.AQCSFRC.all = 0x000f;
	DisablePwm3();
	//EPwm2Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD + gSysInfo.duty;
	//EPwm1Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD - gSysInfo.duty;
	EPwm2Regs.CMPA.half.CMPA = duty_p;
	EPwm1Regs.CMPA.half.CMPA = duty_n;
	EnablePwm2();
	EnablePwm1();
}
*/
/**************************************************************
 *Name:		   BPositiveToCNegtive
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018年11月25日下午1:17:14
 **************************************************************/
/*
inline void BPositiveToCNegtive(void) {

//	EPwm3Regs.AQCSFRC.bit.CSFA = 0x01; //shutdown A phase
//	EPwm3Regs.AQCSFRC.bit.CSFB = 0x01; //shutdown A phase
//	EPwm3Regs.AQCSFRC.all = 9;
//	EPwm2Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD + gSysInfo.duty;
//	EPwm1Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD - gSysInfo.duty;
//	EPwm2Regs.AQCSFRC.bit.CSFA = 0x00;
//	EPwm2Regs.AQCSFRC.bit.CSFB = 0x00;
//	EPwm1Regs.AQCSFRC.bit.CSFA = 0x00;
//	EPwm1Regs.AQCSFRC.bit.CSFB = 0x00;


//	EPwm1Regs.AQCSFRC.all = 0x000f;
//	EPwm2Regs.AQCSFRC.all = 0x000f;

	DisablePwm1();
	//EPwm2Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD + gSysInfo.duty;
	//EPwm3Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD - gSysInfo.duty;
	EPwm2Regs.CMPA.half.CMPA = duty_p;
	EPwm3Regs.CMPA.half.CMPA = duty_n;
	EnablePwm2();
	EnablePwm3();
}
*/
/**************************************************************
 *Name:		   APositiveToCNegtive
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018年11月25日下午1:17:26
 **************************************************************/
/*
inline void APositiveToCNegtive(void) {

//	EPwm2Regs.AQCSFRC.bit.CSFA = 0x01; //shutdown B phase
//	EPwm2Regs.AQCSFRC.bit.CSFB = 0x01; //shutdown B phase
//	EPwm2Regs.AQCSFRC.all = 9;
//	EPwm3Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD + gSysInfo.duty;
//	EPwm1Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD - gSysInfo.duty;
//	EPwm1Regs.AQCSFRC.bit.CSFA = 0x00;
//	EPwm1Regs.AQCSFRC.bit.CSFB = 0x00;
//	EPwm3Regs.AQCSFRC.bit.CSFA = 0x00;
//	EPwm3Regs.AQCSFRC.bit.CSFB = 0x00;

//	EPwm1Regs.AQCSFRC.all = 0x000f;
//	EPwm3Regs.AQCSFRC.all = 0x000f;
	DisablePwm2();
	//EPwm1Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD + gSysInfo.duty;
	//EPwm3Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD - gSysInfo.duty;
	EPwm1Regs.CMPA.half.CMPA = duty_p;
	EPwm3Regs.CMPA.half.CMPA = duty_n;
	EnablePwm1();
	EnablePwm3();
}
*/
/**************************************************************
 *Name:		   APositiveToBNegtive
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018年11月25日下午1:17:37
 **************************************************************/
/*
inline void APositiveToBNegtive(void) {

//	EPwm1Regs.AQCSFRC.bit.CSFA = 0x01; //shutdown C phase
//	EPwm1Regs.AQCSFRC.bit.CSFB = 0x01; //shutdown C phase
//	EPwm1Regs.AQCSFRC.all = 9;
//	EALLOW;
//	EPwm1Regs.TZFRC.bit.OST = 1;
//	EDIS;
//	EPwm3Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD + gSysInfo.duty;
//	EPwm2Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD - gSysInfo.duty;
//	EPwm2Regs.AQCSFRC.bit.CSFA = 0x00;
//	EPwm2Regs.AQCSFRC.bit.CSFB = 0x00;
//
//	EPwm3Regs.AQCSFRC.bit.CSFA = 0x00;
//	EPwm3Regs.AQCSFRC.bit.CSFB = 0x00;

//	EPwm2Regs.AQCSFRC.all = 0x000f;
//	EPwm3Regs.AQCSFRC.all = 0x000f;
	DisablePwm3();
	//EPwm1Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD + gSysInfo.duty;
	//EPwm2Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD - gSysInfo.duty;
	EPwm1Regs.CMPA.half.CMPA = duty_p;
	EPwm2Regs.CMPA.half.CMPA = duty_n;

	EnablePwm1();
	EnablePwm2();
}
*/
/**************************************************************
 *Name:						SwitchDirection
 *Function:
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.31
 **************************************************************/
void SwitchDirection(void){
	int t_duty_temp;
	Uint16 t_duty_p;
	Uint16 t_duty_n;
	gSysInfo.lastTimeHalllPosition = gSysInfo.currentHallPosition;
	gSysInfo.currentHallPosition = GetCurrentHallValue();

	t_duty_temp = gSysInfo.duty;
	if(t_duty_temp > EPWM2_TIMER_HALF_TBPRD) t_duty_temp = EPWM2_TIMER_HALF_TBPRD;
	else if(t_duty_temp < -( EPWM2_TIMER_HALF_TBPRD )) t_duty_temp = -( EPWM2_TIMER_HALF_TBPRD );
	t_duty_p = (Uint16)(EPWM2_TIMER_HALF_TBPRD + t_duty_temp);
	t_duty_n = (Uint16)(EPWM2_TIMER_HALF_TBPRD - t_duty_temp);

	//3:A 2:B 1:C
	switch (gSysInfo.currentHallPosition) {
		case 3://A+ ---------------> C-
			//本项目电机会进行正转和反转。所以需要判断HALL相邻两个位置是否一样。

			if((3 == gSysInfo.lastTimeHalllPosition )
				|| (2 == gSysInfo.lastTimeHalllPosition)
				|| (1 == gSysInfo.lastTimeHalllPosition)){

				//APositiveToCNegtive();
				EPwm2Regs.AQCSFRC.all = 0x0009; //DisablePwm2();
				EPwm1Regs.CMPA.half.CMPA = t_duty_p;
				EPwm3Regs.CMPA.half.CMPA = t_duty_n;
				EPwm1Regs.AQCSFRC.all = 0x000f; //EnablePwm1();
				EPwm3Regs.AQCSFRC.all = 0x000f; //EnablePwm3();
			}
			break;
		case 1://B+ ---------------> C-
			if((1 == gSysInfo.lastTimeHalllPosition )
				|| (3 == gSysInfo.lastTimeHalllPosition)
				|| (5 == gSysInfo.lastTimeHalllPosition)){

				//BPositiveToCNegtive();
				EPwm1Regs.AQCSFRC.all = 0x0009; //DisablePwm1();
				EPwm2Regs.CMPA.half.CMPA = t_duty_p;
				EPwm3Regs.CMPA.half.CMPA = t_duty_n;
				EPwm2Regs.AQCSFRC.all = 0x000f; //EnablePwm2();
				EPwm3Regs.AQCSFRC.all = 0x000f; //EnablePwm3();
			}
			break;
		case 5://B+ ---------------> A-
			if((5 == gSysInfo.lastTimeHalllPosition )
				|| (1 == gSysInfo.lastTimeHalllPosition)
				|| (4 == gSysInfo.lastTimeHalllPosition)){

				//BPositiveToANegtive();
				EPwm3Regs.AQCSFRC.all = 0x0009; //DisablePwm3();
				EPwm2Regs.CMPA.half.CMPA = t_duty_p;
				EPwm1Regs.CMPA.half.CMPA = t_duty_n;
				EPwm2Regs.AQCSFRC.all = 0x000f; //EnablePwm2();
				EPwm1Regs.AQCSFRC.all = 0x000f; //EnablePwm1();
			}
			break;
		case 4://C+ ---------------> A-
			if((4 == gSysInfo.lastTimeHalllPosition )
				|| (5 == gSysInfo.lastTimeHalllPosition)
				|| (6 == gSysInfo.lastTimeHalllPosition)){

				//CPositiveToANegtive();
				EPwm2Regs.AQCSFRC.all = 0x0009; //DisablePwm2();
				EPwm3Regs.CMPA.half.CMPA = t_duty_p;
				EPwm1Regs.CMPA.half.CMPA = t_duty_n;
				EPwm1Regs.AQCSFRC.all = 0x000f; //EnablePwm1();
				EPwm3Regs.AQCSFRC.all = 0x000f; //EnablePwm3();
			}
			break;
		case 6://C+ ---------------> B-
			if((6 == gSysInfo.lastTimeHalllPosition )
				|| (4 == gSysInfo.lastTimeHalllPosition)
				|| (2 == gSysInfo.lastTimeHalllPosition)){

				//CPositiveToBNegtive();
				EPwm1Regs.AQCSFRC.all = 0x0009; //DisablePwm1();
				EPwm3Regs.CMPA.half.CMPA = t_duty_p;
				EPwm2Regs.CMPA.half.CMPA = t_duty_n;
				EPwm2Regs.AQCSFRC.all = 0x000f; //EnablePwm2();
				EPwm3Regs.AQCSFRC.all = 0x000f; //EnablePwm3();
			}
			break;
		case 2://A+ ---------------> B-
			if((2 == gSysInfo.lastTimeHalllPosition )
				|| (6 == gSysInfo.lastTimeHalllPosition)
				|| (3 == gSysInfo.lastTimeHalllPosition)){

				//APositiveToBNegtive();
				EPwm3Regs.AQCSFRC.all = 0x0009; //DisablePwm3();
				EPwm1Regs.CMPA.half.CMPA = t_duty_p;
				EPwm2Regs.CMPA.half.CMPA = t_duty_n;
				EPwm1Regs.AQCSFRC.all = 0x000f; //EnablePwm1();
				EPwm2Regs.AQCSFRC.all = 0x000f; //EnablePwm2();
			}
			break;
		default:
			gSysState.erro.bit.software = TRUE;
			DisablePwmOutput();
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
	//return; //LRT For test use
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
	if(gConfigPara.stateCommand == 1){
		SwitchDirection();
	}
	else{
		DisablePwmOutput();
	}

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
