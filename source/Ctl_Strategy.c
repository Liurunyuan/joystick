#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "Ctl_Strategy.h"
#include "GlobalVarAndFunc.h"
#include "ADprocessor.h"
#include <math.h>
#include "PID.h"

#define ITEGRATION_TIMES (6)

#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(findSpringForceK, "ramfuncs")
#endif
void findSpringForceK(double displace){

    switch (gSysInfo.currentStickDisSection)
    {
    case INIT_SECTION:
    gSysInfo.currentStickDisSection = CheckStickSetion(displace);
        break;
    case 0:
        if(gStickState.value  > (gConfigPara.RB_MaxDistance + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 1;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[11];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[11];
            gSysInfo.soft_break_flag = 0;
        }
        else{
            gSysInfo.currentStickDisSection = 0;
            gSysInfo.soft_break_flag = 1;
            gSysInfo.targetDuty = 40;
        }
        break;
    case 1:
        if(gStickState.value  > (gConfigPara.RB_Distance9 + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 2;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[10];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[10];
            gSysInfo.soft_break_flag = 0;
        }
        else if(gStickState.value < (gConfigPara.RB_MaxDistance - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 0;
            gSysInfo.soft_break_flag = 1;
            gSysInfo.targetDuty = 40;
        }
        else{
            gSysInfo.currentStickDisSection = 1;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[11];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[11];
            gSysInfo.soft_break_flag = 0;
        }
        break;
    case 2:
        if(gStickState.value  > (gConfigPara.RB_Distance8 + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 3;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[9];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[9];
        }
        else if(gStickState.value < (gConfigPara.RB_Distance9 - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 1;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[11];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[11];
        }
        else{
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[10];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[10];
        }
        break;
    case 3:
        if(gStickState.value  > (gConfigPara.RB_Distance7 + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 4;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[8];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[8];
        }
        else if(gStickState.value < (gConfigPara.RB_Distance8 - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 2;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[10];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[10];
        }
        else{
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[9];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[9];
        }
        break;
    case 4:
        if(gStickState.value  > (gConfigPara.RB_Distance6 + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 5;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[7];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[7];
        }
        else if(gStickState.value < (gConfigPara.RB_Distance7 - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 3;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[9];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[9];
        }
        else{
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[8];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[8];
        }
        break;
    case 5:
        if(gStickState.value  > (gConfigPara.RB_Distance5 + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 6;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[6];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[6];
        }
        else if(gStickState.value < (gConfigPara.RB_Distance6 - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 4;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[8];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[8];
        }
        else{
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[7];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[7];
        }
        break;
    case 6:
        if(gStickState.value  > (gConfigPara.RB_Distance4 + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 7;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[5];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[5];
        }
        else if(gStickState.value < (gConfigPara.RB_Distance5 - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 5;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[7];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[7];
        }
        else{
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[6];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[6];
        }
        break;
    case 7:
        if(gStickState.value  > (gConfigPara.RB_Distance3 + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 8;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[4];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[4];
        }
        else if(gStickState.value < (gConfigPara.RB_Distance4 - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 6;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[6];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[6];
        }
        else{
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[5];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[5];
        }
        break;
    case 8:
        if(gStickState.value  > (gConfigPara.RB_Distance2 + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 9;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[3];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[3];
        }
        else if(gStickState.value < (gConfigPara.RB_Distance3 - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 7;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[5];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[5];
        }
        else{
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[4];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[4];
        }
        break;
    case 9:
        if(gStickState.value  > (gConfigPara.RB_EmptyDistance + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 10;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[2];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[2];
        }
        else if(gStickState.value < (gConfigPara.RB_Distance2 - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 8;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[4];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[4];
        }
        else{
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[3];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[3];
        }
        break;
    case 10:
        if(gStickState.value  > (gConfigPara.RB_Distance1 + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 11;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[1];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[1];
        }
        else if(gStickState.value < (gConfigPara.RB_EmptyDistance - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 9;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[3];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[3];
        }
        else{
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[2];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[2];
        }
        break;
    case 11:
        if(gStickState.value  > (gConfigPara.RB_Distance0 + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 12;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[1];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[1];
        }
        else if(gStickState.value < (gConfigPara.RB_Distance1 - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 10;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[2];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[2];
        }
        else{
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[1];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[1];
        }
        break;
    case 12:
        if(gStickState.value  > (gConfigPara.LF_Distance1 + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 13;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[2];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[2];
        }
        else if(gStickState.value < (gConfigPara.LF_Distance0 - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 11;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[1];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[1];
        }
        else{
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[1];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[1];
        }
        break;
    case 13:
        if(gStickState.value  > (gConfigPara.LF_EmptyDistance + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 14;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[3];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[3];
        }
        else if(gStickState.value < (gConfigPara.LF_Distance1 - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 12;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[1];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[1];
        }
        else{
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[2];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[2];
        }
        break;
    case 14:
        if(gStickState.value  > (gConfigPara.LF_Distance2 + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 15;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[4];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[4];
        }
        else if(gStickState.value < (gConfigPara.LF_EmptyDistance - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 13;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[2];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[2];
        }
        else{
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[3];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[3];
        }
        break;
    case 15:
        if(gStickState.value  > (gConfigPara.LF_Distance3 + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 16;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[5];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[5];
        }
        else if(gStickState.value < (gConfigPara.LF_Distance2 - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 14;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[3];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[3];
        }
        else{
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[4];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[4];
        }
        break;
    case 16:
        if(gStickState.value  > (gConfigPara.LF_Distance4 + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 17;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[6];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[6];
        }
        else if(gStickState.value < (gConfigPara.LF_Distance3 - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 15;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[4];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[4];
        }
        else{
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[5];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[5];
        }
        break;
    case 17:
        if(gStickState.value  > (gConfigPara.LF_Distance5 + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 18;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[7];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[7];
        }
        else if(gStickState.value < (gConfigPara.LF_Distance4 - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 16;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[5];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[5];
        }
        else{
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[6];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[6];
        }
        break;
    case 18:
        if(gStickState.value  > (gConfigPara.LF_Distance6 + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 19;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[8];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[8];
        }
        else if(gStickState.value < (gConfigPara.LF_Distance5 - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 17;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[6];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[6];
        }
        else{
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[7];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[7];
        }
        break;
    case 19:
        if(gStickState.value  > (gConfigPara.LF_Distance7 + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 20;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[9];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[9];
        }
        else if(gStickState.value < (gConfigPara.LF_Distance6 - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 18;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[7];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[7];
        }
        else{
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[8];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[8];
        }
        break;
    case 20:
        if(gStickState.value  > (gConfigPara.LF_Distance8 + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 21;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[10];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[10];
        }
        else if(gStickState.value < (gConfigPara.LF_Distance7 - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 19;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[8];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[8];
        }
        else{
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[9];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[9];
        }
        break;
    case 21:
        if(gStickState.value  > (gConfigPara.LF_Distance9 + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 22;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[11];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[11];
        }
        else if(gStickState.value < (gConfigPara.LF_Distance8 - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 20;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[9];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[9];
        }
        else{
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[10];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[10];
        }
        break;
    case 22:
        if(gStickState.value  > (gConfigPara.LF_MaxDistance + DEBOUNCE)){
            gSysInfo.currentStickDisSection = 23;
            gSysInfo.soft_break_flag = 1;
            gSysInfo.targetDuty = -40;
        }
        else if(gStickState.value < (gConfigPara.LF_Distance9 - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 21;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[10];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[10];
            gSysInfo.soft_break_flag = 0;
        }
        else{
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[11];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[11];
            gSysInfo.soft_break_flag = 0;
        }
        break;
    case 23:
        if(gStickState.value  < (gConfigPara.LF_MaxDistance - DEBOUNCE)){
            gSysInfo.currentStickDisSection = 22;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[11];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[11];
            gSysInfo.soft_break_flag = 0;
        }
        else{
            gSysInfo.currentStickDisSection = 23;
            gSysInfo.soft_break_flag = 1;
            gSysInfo.targetDuty = -40;
        }
        break;
    default:
        break;
    }
}
#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(OnlyWithSpringFront, "ramfuncs")
#endif
void OnlyWithSpringFront(void){
	double k;
	double kb;
	double force_openLoop;
	int force_closeLoop;
	double damp_force;
	double spring_force;
	double mass;
	double inertial_force;
	double B_F = 0;
#if(TARGET_DUTY_GRADUAL_CHANGE == INCLUDE_FEATURE)
    int tempDuty = 0;
#endif

	findSpringForceK(gStickState.value);

    if(gSysInfo.soft_break_flag == 1){
        return;
    }
    k = gSysInfo.springForceK;
	kb = gSysInfo.springForceB;

	mass = (k * 1000) / (gConfigPara.naturalVibrationFreq * gConfigPara.naturalVibrationFreq);
    gSysPara.mass = mass;
//	if(mass > 1){
//	    gSysState.warning.bit.a = 0;
//	}
//	else{
//	    gSysState.warning.bit.a = 1;
//	}

	spring_force = k * gStickState.value + kb;
	damp_force = 2 * gConfigPara.dampingFactor * mass * gKeyValue.motorSpeed * gConfigPara.naturalVibrationFreq;
	inertial_force = mass * gKeyValue.motorAccel;

	if(gAccelDirection.accelDirection == STOP_DIRECTION){
	    inertial_force = 0;
	}
	else{
	    inertial_force = -inertial_force;
	}

	force_openLoop = spring_force + damp_force - gSysInfo.friction + inertial_force;
	force_closeLoop = force_PidOutput(force_openLoop, gExternalForceState.value);
	force_closeLoop = -force_closeLoop;

	gSysInfo.ob_velocityOpenLoop = force_openLoop;

    if(gRotateDirection.rotateDirection == FORWARD_DIRECTION){
        B_F = gSysInfo.openLoop_Force_front_B;
    }
    else if(gRotateDirection.rotateDirection == BACKWARD_DIRECTION){
        B_F = gSysInfo.openLoop_Force_rear_B;
    }

	gSysInfo.targetDuty_F = (int16)((gPidPara.K_F_ODE * force_openLoop + B_F) + force_closeLoop);
#if(TARGET_DUTY_GRADUAL_CHANGE == INCLUDE_FEATURE)
	tempDuty = (int16)(gSysInfo.coe_Velocity * gSysInfo.targetDuty_V + gSysInfo.coe_Force * gSysInfo.targetDuty_F);

    if(tempDuty > gSysInfo.targetDuty)
    {
        gSysInfo.targetDuty++;
    }
    else if(tempDuty < gSysInfo.targetDuty)
    {
        gSysInfo.targetDuty--;
    }
    else{
         gSysInfo.targetDuty = tempDuty;
    }
#else
	gSysInfo.targetDuty = (int16)(gSysInfo.coe_Velocity * gSysInfo.targetDuty_V + gSysInfo.coe_Force * gSysInfo.targetDuty_F);
#endif

    if(gSysInfo.targetDuty > DUTY_LIMIT_P){
        gSysInfo.targetDuty = DUTY_LIMIT_P;
    }
    else if(gSysInfo.targetDuty < DUTY_LIMIT_N){
        gSysInfo.targetDuty = DUTY_LIMIT_N;
    }
}
