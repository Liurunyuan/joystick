#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "Ctl_Strategy.h"
#include "GlobalVarAndFunc.h"

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
