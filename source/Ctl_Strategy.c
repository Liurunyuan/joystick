#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "Ctl_Strategy.h"
#include "GlobalVarAndFunc.h"
#include "ADprocessor.h"
#include <math.h>

#define ITEGRATION_TIMES (6)



double findSpringForceK(double displace){
	double springForce = -1;
	int index;
	if(displace >= 0){
		for(index = 1; index < gForceAndDisplaceCurve.maxPoints; ++index){

			if((displace <= gForceAndDisplaceCurve.displacementP[index]) && (displace >= gForceAndDisplaceCurve.displacementP[index - 1])){
				springForce = gForceAndDisplaceCurve.K_spring_forceP[index];
				return springForce;
			}
		}
	}
	else
	{
		for(index = 1; index < gForceAndDisplaceCurve.maxPoints; ++index){

			if((displace >= gForceAndDisplaceCurve.displacementN[index]) && (displace <= gForceAndDisplaceCurve.displacementN[index - 1])){
				springForce = gForceAndDisplaceCurve.K_spring_forceN[index];
				return springForce;
			}
		}
	}

	//TODO generate alarm, because the displacement is out of range
	return springForce;
}
double findSpringForceB(double displace){
	double springForceB = -1;
	int index;
	if(displace >= 0){
		for(index = 1; index < gForceAndDisplaceCurve.maxPoints; ++index){

			if((displace <= gForceAndDisplaceCurve.displacementP[index]) && (displace >= gForceAndDisplaceCurve.displacementP[index - 1])){
				springForceB = gForceAndDisplaceCurve.b_P[index];
				return springForceB;
			}
		}
	}
	else
	{
		for(index = 1; index < gForceAndDisplaceCurve.maxPoints; ++index){

			if((displace >= gForceAndDisplaceCurve.displacementN[index]) && (displace <= gForceAndDisplaceCurve.displacementN[index - 1])){
				springForceB = gForceAndDisplaceCurve.b_N[index];
				return springForceB;
			}
		}
	}

	//TODO generate alarm, because the displacement is out of range
	return springForceB;
}
/**************************************************************
 *Name:		   CalculateSpringForce
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018��11��25������9:31:25
 **************************************************************/
inline void CalculateSpringForce(void) {
	//SpringForce = k * s
	gSysCurrentState.springForce = gSysPara.k_springForce * gKeyValue.displacement;
}
/**************************************************************
 *Name:		   CalculateDampForce
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018��11��25������9:31:25
 **************************************************************/
inline void CalculateDampForce(void) {
	//DampForce = k * v
	gSysCurrentState.dampForce = gSysPara.k_dampForce *  gKeyValue.motorSpeed;
}
/**************************************************************
 *Name:		   CalculateTargetAcc
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018��11��25������9:31:48
 **************************************************************/
inline void CalculateTargetAcc(void) {
	//a = (F - springForce - dampForce) / m
	gSysCurrentState.accTarget = (gKeyValue.force - gSysCurrentState.springForce - gSysCurrentState.dampForce) / gSysPara.mass;

}
/**************************************************************
 *Name:		   CalculateTargetSpeed
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018��11��25������9:31:59
 **************************************************************/
inline void CalculateTargetSpeed(void) {

	gSysCurrentState.speedTarget = gKeyValue.motorSpeed + gSysCurrentState.accTarget * 1;

}
/**************************************************************
 *Name:		   CalculateTargeDisplace
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018��11��25������9:32:11
 **************************************************************/
inline void CalculateTargeDisplace(void) {
//TODO need to find a method to get the right target of displacement

}
/**************************************************************
 *Name:		   UpdateAccErr
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018��11��25������9:32:46
 **************************************************************/
inline void UpdateAccErr(void) {
	gSysCurrentState.errAcc = gSysCurrentState.accTarget - gKeyValue.motorAccel;
}
/**************************************************************
 *Name:		   UpdateSpeedErr
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018��11��25������9:32:57
 **************************************************************/
inline void UpdateSpeedErr(void) {
	gSysCurrentState.errSpeed = gSysCurrentState.speedTarget - gKeyValue.motorSpeed;
}
/**************************************************************
 *Name:		   UpdateDisplacementErr
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018��11��25������9:33:07
 **************************************************************/
inline void UpdateDisplacementErr(void) {
	gSysCurrentState.errDisplacement = gSysCurrentState.displaceTarget - gKeyValue.displacement;
}
void OnlyWithSpringRear(void){
	double k;
	double kb;
	double y;
	int tmp;

	k = findSpringForceK(gStickState.value);
	kb = findSpringForceB(gStickState.value);

	y =  k * gStickState.value + kb;

	tmp = (int32)((y - gExternalForceState.value) * 10);
	tmp = -tmp;
	gSysInfo.targetDuty = y + tmp;
}

void OnlyWithSpringFront(void){
	double k;
	double kb;
	double y;
	int tmp;

	k = findSpringForceK(gStickState.value);
	kb = findSpringForceB(gStickState.value);

	y = k * gStickState.value + kb;

	tmp = (int32)((y - gExternalForceState.value) * 10);
	tmp = -tmp;
	gSysInfo.targetDuty = y + tmp;
	
}
/**************************************************************
 *Name:		   PidProcess
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018��11��25������9:33:16
 **************************************************************/
void PidProcess(void){

	//CalculateSpringForce();
	//CalculateDampForce();
	//CalculateTargetAcc();

	//CalculateTargetAcc();
	//CalculateTargetSpeed();
	//CalculateTargeDisplace();

	RKT(0,gKeyValue.displacement,gKeyValue.motorSpeed,0.01);

	//UpdateAccErr();
	//UpdateSpeedErr();
	//UpdateDisplacementErr();
}
//F - K1 * dy/dt - K2 * y = m * dy2/dt2
//F�� - K���� * dy/dt - K�� * y = m * dy2/dt2
//��z = dy/dt
//=====>
double function(double x0, double y0, double z0, double h){
	double K11;
	double K12;
	double K13;
	double K14;

	double K21;
	double K22;
	double K23;
	double K24;

	double a = 0;//a = f����/m
	double b = 0;//b = f����/m
	double c = 0;//c = -f����/m


	double y1;
	double z1;
	double a1;

	double k;
	double kb;


//	a = gSysPara.k_dampForce/gSysPara.mass;
//	b = gSysPara.k_springForce / gSysPara.mass;
//	c = gKeyValue.force / gSysPara.mass;

	//k = findSpringForceK(y0);
	//kb = findSpringForceB(y0);
	//gSysPara.k_springForce = k;
	//gSysPara.k_dampForce = kb;
	//gSysPara.mass = k/(4); 

	//a = 2 * gConfigPara.dampingFactor * sqrt(k / gSysPara.mass);
	a = 2 * gConfigPara.dampingFactor * 2;
	b =	((k * y0) + kb)/gSysPara.mass;
	c =	(-gKeyValue.force) / gSysPara.mass;

	K11 = z0;
	K21 = c - (a * z0) - (b * y0);

	K12 = z0 + h/2 * K21;
	K22 = c - b * (y0 + h/2 * K11) - a * (z0 + h/2 * K21);

	K13 = z0 + h/2 * K22;
	K23 = c - b * (y0 + h/2 * K12) - a * (z0 + h/2 * K22);

	K14 = z0 + h * K23;
	K24 = c - b * (y0 + h/2 * K13) - a * (z0 + h * K23);

	y1 = y0 + h/6 *(K11 + 2 * K12 + 2 * K13 + K14);
	z1 = z0 + h/6 *(K21 + 2 * K22 + 2 * K23 + K24);
	a1 = c - a * z1 - b * y1;


	gSysCurrentState.displaceTarget = y1;
	gSysCurrentState.speedTarget = z1;
	gSysCurrentState.accTarget = a1;

	return y1;
}
/**************************************************************
 *Name:		   RKT
 *Comment:
 *Input:	   void
 *Output:	   int
 *Author:	   Simon
 *Date:		   2018��12��18������9:05:27
 **************************************************************/
int RKT(double x, double y, double z, double h){
	int ret = 0;
	int i;

	double x0 = x;
	double y0 = y;
	double z0 = z;
	double h0 = h;

	for(i = 0; i < 2; ++i){
		function(x0, y0, z0, h0);
		//TODO update value of y
		//TODO update value of z
		x0 = x0 + h0;
		y0 = gSysCurrentState.displaceTarget;
		z0 = gSysCurrentState.speedTarget;
	}
	return ret;
}
