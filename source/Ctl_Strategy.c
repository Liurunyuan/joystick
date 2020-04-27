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
    int32 tmp;
    gSysInfo.currentStickDisSection = CheckStickSetion(displace);
    switch (gSysInfo.currentStickDisSection)
    {
    case INIT_SECTION:
    gSysInfo.currentStickDisSection = CheckStickSetion(displace);
        break;
    case 0:
            gSysInfo.soft_break_flag = 1;
            if(gSysInfo.board_type == ROLL){
                gSysInfo.targetDuty = 50;
            }
            else{
                gSysInfo.targetDuty = 50;
            }
        break;
    case 1:
            gSysInfo.currentStickDisSection = 1;
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[11];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[11];
            gSysInfo.soft_break_flag = 0;
        break;
    case 2:
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[10];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[10];
        break;
    case 3:
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[9];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[9];
        break;
    case 4:
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[8];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[8];
        break;
    case 5:
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[7];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[7];
        break;
    case 6:
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[6];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[6];
        break;
    case 7:
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[5];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[5];
        break;
    case 8:
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[4];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[4];
        break;
    case 9:
            //rear ode
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[3];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[3];
            gSysInfo.soft_break_flag = 0;
        break;
    case 10:
            //rear start force
            if(gExternalForceState.value < (gConfigPara.RB_StartForce)){
                gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[3];
                gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[3];
                gSysInfo.soft_break_flag = 0;
//                gSysInfo.targetDuty = -38;
//                gSysInfo.soft_break_flag = 1;
            }
            else if(gExternalForceState.value > 0){
                tmp = (int32)((0.5 - gExternalForceState.value)* 5);
                tmp = -tmp;
                gSysInfo.targetDuty = tmp;
                gSysInfo.soft_break_flag = 1;
//                gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[3];
//                gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[3];
//                gSysInfo.soft_break_flag = 0;
            }
            else if(gExternalForceState.ForceState == NO_FORCE){
                gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[3];
                gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[3];
                gSysInfo.soft_break_flag = 0;
            }
            else{
//                gSysInfo.soft_break_flag = 1;
//                gSysInfo.targetDuty = 0;
                gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[3];
                gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[3];
                gSysInfo.soft_break_flag = 0;
//                gSysInfo.targetDuty = (gStickState.value * -50) + displace_PidOutput(gConfigPara.RB_EmptyDistance,gStickState.value);
//                gSysInfo.soft_break_flag = 1;
            }
        break;
    case 11:
            //rear null distance
            if(gExternalForceState.ForceState == FORWARD_FORCE){
                tmp = (int32)((0.5 - gExternalForceState.value)* 10);
                tmp = -tmp;
                gSysInfo.targetDuty = tmp;
                gSysInfo.soft_break_flag = 1;

            }
            else if(gExternalForceState.ForceState == BACKWARD_FORCE){
                tmp = (int32)(((-0.5) - gExternalForceState.value)* 10);
                tmp = -tmp;
                gSysInfo.targetDuty = tmp;
                gSysInfo.soft_break_flag = 1;
            }
            else{
                gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceN[1];
                gSysInfo.springForceB = gForceAndDisplaceCurve.b_N[1];
//                gSysInfo.targetDuty = 0;
                gSysInfo.soft_break_flag = 0;
            }
        break;
    case 12:
            //front null distance
            if(gExternalForceState.ForceState == FORWARD_FORCE){
                tmp = (int32)((0.5 - gExternalForceState.value)* 10);
                tmp = -tmp;
                gSysInfo.targetDuty = tmp;
                gSysInfo.soft_break_flag = 1;

            }
            else if(gExternalForceState.ForceState == BACKWARD_FORCE){
                tmp = (int32)(((-0.5) - gExternalForceState.value)* 10);
                tmp = -tmp;
                gSysInfo.targetDuty = tmp;
                gSysInfo.soft_break_flag = 1;
            }
            else{
                gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[1];
                gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[1];
//                gSysInfo.targetDuty = 0;
                gSysInfo.soft_break_flag = 0;
            }
        break;
    case 13:
            //front start force
            if(gExternalForceState.value > (gConfigPara.LF_StartForce)){
                gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[3];
                gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[3];
                gSysInfo.soft_break_flag = 0;
//                gSysInfo.targetDuty = 35;
//                gSysInfo.soft_break_flag = 1;
            }
            else if(gExternalForceState.value < 0){
                tmp = (int32)(((-0.5) - gExternalForceState.value)* 5);
                tmp = -tmp;
                gSysInfo.targetDuty = tmp;
                gSysInfo.soft_break_flag = 1;
//                gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[3];
//                gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[3];
//                gSysInfo.soft_break_flag = 0;
            }
            else if(gExternalForceState.ForceState == NO_FORCE){
                gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[3];
                gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[3];
                gSysInfo.soft_break_flag = 0;
            }
            else{
//                gSysInfo.soft_break_flag = 1;
//                gSysInfo.targetDuty = 0;
                gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[3];
                gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[3];
                gSysInfo.soft_break_flag = 0;
//                gSysInfo.targetDuty = (gStickState.value * -50) + displace_PidOutput(gConfigPara.LF_EmptyDistance,gStickState.value);
//                gSysInfo.soft_break_flag = 1;
            }
        break;
    case 14:
            //front ode
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[3];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[3];
            gSysInfo.soft_break_flag = 0;
        break;
    case 15:
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[4];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[4];
        break;
    case 16:
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[5];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[5];
        break;
    case 17:
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[6];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[6];
        break;
    case 18:
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[7];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[7];
        break;
    case 19:
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[8];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[8];
        break;
    case 20:
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[9];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[9];
        break;
    case 21:
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[10];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[10];
        break;
    case 22:
            gSysInfo.springForceK = gForceAndDisplaceCurve.K_spring_forceP[11];
            gSysInfo.springForceB = gForceAndDisplaceCurve.b_P[11];
            gSysInfo.soft_break_flag = 0;
        break;
    case 23:
            gSysInfo.currentStickDisSection = 23;
            gSysInfo.soft_break_flag = 1;
            if(gSysInfo.board_type == ROLL){
                gSysInfo.targetDuty = -40;
            }
            else{
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
	double friction;
#if(TARGET_DUTY_GRADUAL_CHANGE == INCLUDE_FEATURE)
    int tempDuty = 0;
#endif

	findSpringForceK(gStickState.value);

    if(gSysInfo.soft_break_flag == 1){
        return;
    }
    k = gSysInfo.springForceK;
	kb = gSysInfo.springForceB;

    if((gSysInfo.currentStickDisSection > 11) && (gSysInfo.currentStickDisSection < 14)){
        mass = (gForceAndDisplaceCurve.K_spring_forceP[3] * 1000) / (gConfigPara.naturalVibrationFreq * gConfigPara.naturalVibrationFreq);
    }
    else if((gSysInfo.currentStickDisSection > 9) && (gSysInfo.currentStickDisSection < 12)){
        mass = (gForceAndDisplaceCurve.K_spring_forceN[3] * 1000) / (gConfigPara.naturalVibrationFreq * gConfigPara.naturalVibrationFreq);
    }
    else{
        mass = (k * 1000) / (gConfigPara.naturalVibrationFreq * gConfigPara.naturalVibrationFreq);
    }

//	mass = (k * 1000) / (gConfigPara.naturalVibrationFreq * gConfigPara.naturalVibrationFreq);
    gSysPara.mass = mass;

	spring_force = k * gStickState.value + kb;
//	damp_force = 2 * gConfigPara.dampingFactor * mass * gKeyValue.motorSpeed * gConfigPara.naturalVibrationFreq;
	inertial_force = mass * gKeyValue.motorAccel;

//	if(gAccelDirection.accelDirection == STOP_DIRECTION){
//	    inertial_force = 0;
//	}
//	else{
//	    inertial_force = -inertial_force;
//	}

//	friction = gSysInfo.friction;
	if((gSysInfo.currentStickDisSection > 9) && (gSysInfo.currentStickDisSection < 14)){
	    friction = 0;
	    damp_force = 0;
	}
	else{
	    friction = gSysInfo.friction;
	    damp_force = 2 * gConfigPara.dampingFactor * mass * gKeyValue.motorSpeed * gConfigPara.naturalVibrationFreq;
	}


	force_openLoop = spring_force + damp_force - friction + inertial_force;
	force_closeLoop = force_PidOutput(force_openLoop, gExternalForceState.value);
	force_closeLoop = -force_closeLoop;

//	gSysInfo.ob_velocityOpenLoop = inertial_force;
//	gSysInfo.ob_velocityOpenLoop2 = damp_force;

    if(force_closeLoop >= 0){
        B_F = 12;
//        if((gSysInfo.currentStickDisSection > 9) && (gSysInfo.currentStickDisSection < 14)){
//            B_F = 0;
//        }
//        else{
//            B_F = gSysInfo.openLoop_Force_front_B;
//        }
    }
    else{
        B_F = -12;
    }
//    else if(gRotateDirection.rotateDirection == BACKWARD_DIRECTION){
//        if((gSysInfo.currentStickDisSection > 9) && (gSysInfo.currentStickDisSection < 14)){
//            B_F = -0;
//        }
//        else{
//            B_F = gSysInfo.openLoop_Force_rear_B;
//        }
//    }

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
