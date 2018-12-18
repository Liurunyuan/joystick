#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "Ctl_Strategy.h"
#include "GlobalVarAndFunc.h"

#define ITEGRATION_TIMES (6)

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
/**************************************************************
 *Name:		   PidProcess
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018��11��25������9:33:16
 **************************************************************/
void PidProcess(void){

	CalculateSpringForce();
	CalculateDampForce();
	CalculateTargetAcc();

	CalculateTargetAcc();
	CalculateTargetSpeed();
	CalculateTargeDisplace();


	UpdateAccErr();
	UpdateSpeedErr();
	UpdateDisplacementErr();
}
//F - K1 * dy/dt - K2 * y = m * dy2/dt2
//F�� - K���� * dy/dt - K�� * y = m * dy2/dt2
//��z = dy/dt
//=====>
double function(double x0, double y0, double z0, double h){
	int K11;
	int K12;
	int K13;
	int K14;

	int K21;
	int K22;
	int K23;
	int K24;

	int a = 0;//a = f����/m
	int b = 0;//b = f����/m
	int c = 0;//c = -f����/m

	a = gSysPara.k_dampForce/gSysPara.mass;
	b = gSysPara.k_springForce / gSysPara.mass;
	c = gKeyValue.force / gSysPara.mass;

	double y1;
	double z1;

	K11 = z0;
	K21 = c - (a * z0) - (b * y0);

	K12 = z0 + h/2 * K21;
	K22 =   -2 * (y0 + h/2 * K11) + 2 * (z0 + h/2 * K21);

	K13 = z0 + h/2 * K22;
	K23 =   -2 * (y0 + h/2 * K12) + 2 * (z0 + h/2 * K22);

	K14 = z0 + h * K23;
	K24 =   -2 * (y0 + h/2 * K13) + 2 * (z0 + h * K23);

	y1 = y0 + h/6 *(K11 + 2 * K12 + 2 * K13 + K14);
	z1 = z0 + h/6 *(K21 + 2 * K22 + 2 * K23 + K24);


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

	for(i = 0; i < 10; ++i){
		function(x, y, z, h);
		//TODO update value of y
		//TODO update value of z
		x = x + h;
	}
	return ret;
}
