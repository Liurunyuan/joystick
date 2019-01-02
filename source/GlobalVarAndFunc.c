#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "GlobalVarAndFunc.h"


Uint32 gECapCount;
RS422STATUS gRS422Status = {0};
KeyValue gKeyValue = {0};
SYSINFO gSysInfo = {0};
SYSSTATE gSysState = {0};
SYSPARA gSysPara = {0};
SYSCURRENTSTATE gSysCurrentState = {0};
CONFIGPARA gConfigPara = {0};


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
 *Date:		   2018年11月25日下午12:40:11
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
 *Date:		   2019年1月2日下午9:57:12
 **************************************************************/
double KalmanFilter(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R)
{

	double R = MeasureNoise_R;
	double Q = ProcessNiose_Q;

	static double x_last;
	double x_mid = x_last;
	double x_now;

	static double p_last;
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

	static double x_last;
	double x_mid = x_last;
	double x_now;

	static double p_last;
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
