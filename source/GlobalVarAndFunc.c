#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "GlobalVarAndFunc.h"
#include <string.h>
#include "PWM_ISR.h"

Uint32 gECapCount = 0;
RS422STATUS gRS422Status = {0};
KeyValue gKeyValue = {0};
SYSINFO gSysInfo = {0};
SYSSTATE gSysState = {0};
SYSPARA gSysPara = {0};
SYSCURRENTSTATE gSysCurrentState = {0};
CONFIGPARA gConfigPara = {0};
FORCE_DISPLACE_CURVE gForceAndDisplaceCurve  = {0};

ANOLOG16BIT gAnalog16bit = {0};

int gforwardOverLimit = 0;
int gbackwardOverLimit = 0;
int gforwardForce = 0;
int gbackwardForce = 0;
int gNoExternalForce = 0;

void InitForceDisplaceCurve(void){

}

void UpdateForceDisplaceCurve(void){
	int index;

	gForceAndDisplaceCurve.springForceP[0] = gConfigPara.LF_Force1;
	gForceAndDisplaceCurve.springForceP[1] = gConfigPara.LF_Force2;
	gForceAndDisplaceCurve.springForceP[2] = gConfigPara.LF_Force3;
	gForceAndDisplaceCurve.springForceP[3] = gConfigPara.LF_Force4;
	gForceAndDisplaceCurve.springForceP[4] = gConfigPara.LF_Force5;
	gForceAndDisplaceCurve.springForceP[5] = gConfigPara.LF_Force6;
	gForceAndDisplaceCurve.springForceP[6] = gConfigPara.LF_Force7;
	gForceAndDisplaceCurve.springForceP[7] = gConfigPara.LF_Force8;
	gForceAndDisplaceCurve.springForceP[8] = gConfigPara.LF_Force9;
	gForceAndDisplaceCurve.springForceP[9] = gConfigPara.LF_MaxForce;

	gForceAndDisplaceCurve.springForceN[0] = gConfigPara.RB_Force1;
	gForceAndDisplaceCurve.springForceN[1] = gConfigPara.RB_Force2;
	gForceAndDisplaceCurve.springForceN[2] = gConfigPara.RB_Force3;
	gForceAndDisplaceCurve.springForceN[3] = gConfigPara.RB_Force4;
	gForceAndDisplaceCurve.springForceN[4] = gConfigPara.RB_Force5;
	gForceAndDisplaceCurve.springForceN[5] = gConfigPara.RB_Force6;
	gForceAndDisplaceCurve.springForceN[6] = gConfigPara.RB_Force7;
	gForceAndDisplaceCurve.springForceN[7] = gConfigPara.RB_Force8;
	gForceAndDisplaceCurve.springForceN[8] = gConfigPara.RB_Force9;
	gForceAndDisplaceCurve.springForceN[9] = gConfigPara.RB_MaxForce;

	gForceAndDisplaceCurve.displacementP[0] = gConfigPara.LF_Distance1;
	gForceAndDisplaceCurve.displacementP[1] = gConfigPara.LF_Distance2;
	gForceAndDisplaceCurve.displacementP[2] = gConfigPara.LF_Distance3;
	gForceAndDisplaceCurve.displacementP[3] = gConfigPara.LF_Distance4;
	gForceAndDisplaceCurve.displacementP[4] = gConfigPara.LF_Distance5;
	gForceAndDisplaceCurve.displacementP[5] = gConfigPara.LF_Distance6;
	gForceAndDisplaceCurve.displacementP[6] = gConfigPara.LF_Distance7;
	gForceAndDisplaceCurve.displacementP[7] = gConfigPara.LF_Distance8;
	gForceAndDisplaceCurve.displacementP[8] = gConfigPara.LF_Distance9;
	gForceAndDisplaceCurve.displacementP[9] = gConfigPara.LF_MaxDistance;


	gForceAndDisplaceCurve.displacementN[0] = gConfigPara.RB_Distance1;
	gForceAndDisplaceCurve.displacementN[1] = gConfigPara.RB_Distance2;
	gForceAndDisplaceCurve.displacementN[2] = gConfigPara.RB_Distance3;
	gForceAndDisplaceCurve.displacementN[3] = gConfigPara.RB_Distance4;
	gForceAndDisplaceCurve.displacementN[4] = gConfigPara.RB_Distance5;
	gForceAndDisplaceCurve.displacementN[5] = gConfigPara.RB_Distance6;
	gForceAndDisplaceCurve.displacementN[6] = gConfigPara.RB_Distance7;
	gForceAndDisplaceCurve.displacementN[7] = gConfigPara.RB_Distance8;
	gForceAndDisplaceCurve.displacementN[8] = gConfigPara.RB_Distance9;
	gForceAndDisplaceCurve.displacementN[9] = gConfigPara.RB_MaxDistance;

	gForceAndDisplaceCurve.maxPoints = 10;

	for(index = 1; index <gForceAndDisplaceCurve.maxPoints; ++index){
		gForceAndDisplaceCurve.K_spring_forceP[index] =
				(gForceAndDisplaceCurve.springForceP[index] - gForceAndDisplaceCurve.springForceP[index - 1]) /
				(gForceAndDisplaceCurve.displacementP[index] - gForceAndDisplaceCurve.displacementP[index - 1]);

		gForceAndDisplaceCurve.b_P[index] =
				gForceAndDisplaceCurve.springForceP[index] - gForceAndDisplaceCurve.K_spring_forceP[index] * gForceAndDisplaceCurve.displacementP[index];
	}

	for(index = 1; index <gForceAndDisplaceCurve.maxPoints; ++index){
		gForceAndDisplaceCurve.K_spring_forceN[index] =
				(gForceAndDisplaceCurve.springForceN[index] - gForceAndDisplaceCurve.springForceN[index - 1]) /
				(gForceAndDisplaceCurve.displacementN[index] - gForceAndDisplaceCurve.displacementN[index - 1]);

		gForceAndDisplaceCurve.b_N[index] =
				gForceAndDisplaceCurve.springForceN[index] - gForceAndDisplaceCurve.K_spring_forceN[index] * gForceAndDisplaceCurve.displacementN[index];
	}
}

void InitConfigParameter(void){
	gConfigPara.LF_MaxForce = 0;
	gConfigPara.LF_Force1 = 0;
	gConfigPara.LF_Force2 = 0;
	gConfigPara.LF_Force3 = 0;
	gConfigPara.LF_Force4 = 0;
	gConfigPara.LF_Force5 = 0;
	gConfigPara.LF_Force6 = 0;
	gConfigPara.LF_Force7 = 0;
	gConfigPara.LF_Force8 = 0;
	gConfigPara.LF_Force9 = 0;

	gConfigPara.RB_Force1 = 0;
	gConfigPara.RB_Force2 = 0;
	gConfigPara.RB_Force3 = 0;
	gConfigPara.RB_Force4 = 0;
	gConfigPara.RB_Force5 = 0;
	gConfigPara.RB_Force6 = 0;
	gConfigPara.RB_Force7 = 0;
	gConfigPara.RB_Force8 = 0;
	gConfigPara.RB_Force9 = 0;
	gConfigPara.RB_MaxForce = 0;

	gConfigPara.LF_MaxDistance = 0;
	gConfigPara.LF_Distance1 = 0;
	gConfigPara.LF_Distance2 = 0;
	gConfigPara.LF_Distance3 = 0;
	gConfigPara.LF_Distance4 = 0;
	gConfigPara.LF_Distance5 = 0;
	gConfigPara.LF_Distance6 = 0;
	gConfigPara.LF_Distance7 = 0;
	gConfigPara.LF_Distance8 = 0;
	gConfigPara.LF_Distance9 = 0;

	gConfigPara.RB_Distance1 = 0;
	gConfigPara.RB_Distance2 = 0;
	gConfigPara.RB_Distance3 = 0;
	gConfigPara.RB_Distance4 = 0;
	gConfigPara.RB_Distance5 = 0;
	gConfigPara.RB_Distance6 = 0;
	gConfigPara.RB_Distance7 = 0;
	gConfigPara.RB_Distance8 = 0;
	gConfigPara.RB_Distance9 = 0;
	gConfigPara.RB_MaxDistance = 0;

	gConfigPara.LF_StartForce = 0;
	gConfigPara.RB_StartForce = 0;

	gConfigPara.LF_FrontFriction = 0;
	gConfigPara.LF_RearFriction = 0;
	gConfigPara.RB_FrontFriction = 0;
	gConfigPara.RB_RearFriction = 0;

	gConfigPara.LF_EmptyDistance = 0;
	gConfigPara.RB_EmptyDistance = 0;

	gConfigPara.dampingFactor = 0;

	gConfigPara.naturalVibrationFreq = 0;

	gConfigPara.equivalentMass = 0;

	gConfigPara.LF_TrimRange = 0;
	gConfigPara.RB_TrimRange = 0;

	gConfigPara.trimTarget = 0;

	gConfigPara.trimCommand = 0;

	gConfigPara.timeDelay = 0;

	gConfigPara.stateCommand = 0;

	gConfigPara.innerErrorThresholdWithInnerMaxKp = 0;
	gConfigPara.innerMaxKp = 0;
	gConfigPara.innerKi = 0;
	gConfigPara.innerKd = 0;
	gConfigPara.innerFeedForward = 0;
	gConfigPara.innerMaxStartError = 0;
	gConfigPara.innerMinStartError = 0;
	gConfigPara.innerMaxIntergralSaturation = 0;
	gConfigPara.innerMinIntergralSaturation = 0;

	gConfigPara.middleKd = 0;
	gConfigPara.middleKi = 0;
	gConfigPara.middleKp = 0;

	gConfigPara.outerKd = 0;
	gConfigPara.outerKi = 0;
	gConfigPara.outerKp = 0;
}
/**************************************************************
 *Name:		   InitSysState
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018��11��25������12:40:11
 **************************************************************/
void InitSysState(void){
	gSysState.alarm.all 	= 0;
	gSysState.erro.all 		= 0;
	gSysState.warning.all 	= 0;

	gSysPara.k_dampForce = 0;
	gSysPara.k_springForce = 0;
	gSysPara.mass = 0;

	gSysCurrentState.accTarget = 0;
	gSysCurrentState.dampForce = 0;
	gSysCurrentState.displaceTarget = 0;
	gSysCurrentState.speedTarget = 0;
	gSysCurrentState.springForce = 0;
}
/**************************************************************
 *Name:		   KalmanFilter
 *Comment:
 *Input:	   const double, double, double
 *Output:	   double
 *Author:	   Simon
 *Date:		   2019��1��2������9:57:12
 **************************************************************/
double KalmanFilter(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R)
{

	double R = MeasureNoise_R;
	double Q = ProcessNiose_Q;

	static double x_last = 0;
	double x_mid = x_last;
	double x_now;

	static double p_last = 0;
	double p_mid;
	double p_now;

	double kg;

	x_mid = x_last;
	p_mid = p_last + Q;


	kg = p_mid / (p_mid + R);
	x_now = x_mid + kg * (ResrcData - x_mid);
	p_now = (1 - kg) * p_mid;
	p_last = p_now;
	x_last = x_now;

	return x_now;
}
double KalmanFilterSpeed(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R)
{
	double R = MeasureNoise_R;
	double Q = ProcessNiose_Q;

	static double x_last = 0;
	double x_mid = x_last;
	double x_now;

	static double p_last = 0;
	double p_mid;
	double p_now;

	double kg;

	x_mid = x_last;
	p_mid = p_last + Q;


	kg = p_mid / (p_mid + R);
	x_now = x_mid + kg * (ResrcData - x_mid);
	p_now = (1 - kg) * p_mid;
	p_last = p_now;
	x_last = x_now;

	return x_now;
}


void Enable_KZ_P_DSP(void){
	GpioDataRegs.GPASET.bit.GPIO31 = 1;
}
void Disable_KZ_P_DSP(void){
	GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;
}

void Enable_KZ_N_DSP(void){
	GpioDataRegs.GPCCLEAR.bit.GPIO86 = 1;
}
void Disable_KZ_N_DSP(void){
	GpioDataRegs.GPCSET.bit.GPIO86 = 1;
}

void EnablePwmOutput(void){
	//gSysInfo.currentHallPosition = GetCurrentHallValue();
	//gSysInfo.lastTimeHalllPosition = gSysInfo.currentHallPosition;

	GpioDataRegs.GPCCLEAR.bit.GPIO87 = 1;
	Enable_KZ_P_DSP();
	Enable_KZ_N_DSP();
}
void DisablePwmOutput(void){
	GpioDataRegs.GPCSET.bit.GPIO87 = 1;
	//GpioDataRegs.GPCCLEAR.bit.GPIO87 = 1; //For temp use
	Disable_KZ_P_DSP();
	Disable_KZ_N_DSP();
	EPwm1Regs.AQCSFRC.all = 0x0009; //DisablePwm1();
	EPwm2Regs.AQCSFRC.all = 0x0009; //DisablePwm2();
	EPwm3Regs.AQCSFRC.all = 0x0009; //DisablePwm3();
}
void StateMachine(void){
	//gConfigPara.stateCommand = 1; // For TEMP Test
	if(gConfigPara.stateCommand == 1 && gSysState.erro.bit.software != 1){
		EnablePwmOutput();
	}
	else{
		DisablePwmOutput();
	}
}

void ClearFault(void){
	int i = 0;

	GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
	//TODO delay for some time, need to verify
	for(i = 0; i < 1000; ++i){
		asm(" NOP");
	}
	GpioDataRegs.GPASET.bit.GPIO9 = 1;
}

void Enable_PWMD_BK(void){
	GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;
}
void Disable_PWMD_BK(void){
	GpioDataRegs.GPASET.bit.GPIO9 = 1;
}



