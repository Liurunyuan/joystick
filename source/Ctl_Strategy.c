#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "Ctl_Strategy.h"
#include "GlobalVarAndFunc.h"
#include "ADprocessor.h"
#include <math.h>
#include "PID.h"

#define ITEGRATION_TIMES (6)

#pragma CODE_SECTION(findSpringForceK, "ramfuncs")
void findSpringForceK(double displace){
//	double springForce = -1;
//	int index;

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
/*
	if(displace >= 0){
	    if((displace > gForceAndDisplaceCurve.displacementP[1]) && (displace < gForceAndDisplaceCurve.displacementP[2])){
	        if(gExternalForceState.value >= gForceAndDisplaceCurve.springForceP[2]){
	            springForce = gForceAndDisplaceCurve.K_spring_forceP[3];
	            return springForce;
	        }
	        else{
	            gSysInfo.soft_break_flag = 1;
	            gSysInfo.targetDuty = -40;
	            return 0;
	        }
	    }
		for(index = 1; index < gForceAndDisplaceCurve.maxPoints; ++index){
			if((displace <= gForceAndDisplaceCurve.displacementP[index]) && (displace >= gForceAndDisplaceCurve.displacementP[index - 1])){
				springForce = gForceAndDisplaceCurve.K_spring_forceP[index];
				gSysInfo.soft_break_flag = 0;
				return springForce;
			}
			else if(displace > gForceAndDisplaceCurve.displacementP[(gForceAndDisplaceCurve.maxPoints)-1]){
			    gSysInfo.soft_break_flag = 1;
			    gSysInfo.targetDuty = -40;
                return 0;
			}
		}
	}
	else
	{
        if((displace < gForceAndDisplaceCurve.displacementP[1]) && (displace > gForceAndDisplaceCurve.displacementP[2])){
            if(gExternalForceState.value <= gForceAndDisplaceCurve.springForceN[2]){
                springForce = gForceAndDisplaceCurve.K_spring_forceN[3];
                return springForce;
            }
            else{
                gSysInfo.soft_break_flag = 1;
                gSysInfo.targetDuty = 40;
                return 0;
            }
        }
		for(index = 1; index < gForceAndDisplaceCurve.maxPoints; ++index){

			if((displace >= gForceAndDisplaceCurve.displacementN[index]) && (displace <= gForceAndDisplaceCurve.displacementN[index - 1])){
				springForce = gForceAndDisplaceCurve.K_spring_forceN[index];
				gSysInfo.soft_break_flag = 0;
				return springForce;
			}
            else if(displace < gForceAndDisplaceCurve.displacementN[(gForceAndDisplaceCurve.maxPoints)-1]){
                gSysInfo.soft_break_flag = 1;
                gSysInfo.targetDuty = 40;
                return 0;
            }
		}
	}
*/
	//TODO generate alarm, because the displacement is out of range
//	return springForce;
}
/*
#pragma CODE_SECTION(findSpringForceB, "ramfuncs")
double findSpringForceB(double displace){
	double springForceB = -1;
	int index;
	if(displace >= 0){
		for(index = 1; index < gForceAndDisplaceCurve.maxPoints; ++index){

			if((displace <= gForceAndDisplaceCurve.displacementP[index]) && (displace >= gForceAndDisplaceCurve.displacementP[index - 1])){
				springForceB = gForceAndDisplaceCurve.b_P[index];
				return springForceB;
			}
            else if(displace > gForceAndDisplaceCurve.displacementP[(gForceAndDisplaceCurve.maxPoints)-1]){
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
            else if(displace < gForceAndDisplaceCurve.displacementN[(gForceAndDisplaceCurve.maxPoints)-1]){
                springForceB = gForceAndDisplaceCurve.b_N[index];
                return springForceB;
            }
		}
	}

	//TODO generate alarm, because the displacement is out of range
	return springForceB;
}
*/
//#pragma CODE_SECTION(OnlyWithSpringRear, "ramfuncs")
//void OnlyWithSpringRear(void){
//    double k;
//    double kb;
//    double force_openLoop;
//    int force_closeLoop;
//    double damp_force;
//    double spring_force;
//    double mass;
//    double inertial_force;
//    double B_F = 0;
//    double velocity_openLoop;
//    int velocity_closeLoop;
//    double B_V = 0;

//    k = findSpringForceK(gStickState.value);
//    if(gSysInfo.soft_break_flag == 1){
//        return;
//    }
//    kb = findSpringForceB(gStickState.value);
//
//    mass = (k * 1000) / (gConfigPara.naturalVibrationFreq * gConfigPara.naturalVibrationFreq);
//
//    if(mass > 1){
//        gSysState.warning.bit.a = 0;
//    }
//    else{
//        gSysState.warning.bit.a = 1;
//    }
//
//    spring_force = k * gStickState.value + kb;
//    damp_force = 2 * gConfigPara.dampingFactor * mass * gKeyValue.motorSpeed * gConfigPara.naturalVibrationFreq;
//    inertial_force = mass * gKeyValue.motorAccel;
//
//    if(gAccelDirection.accelDirection == STOP_DIRECTION){
//        inertial_force = 0;
//    }
//    else{
//        inertial_force = -inertial_force;
//    }
//
//    force_openLoop = spring_force + damp_force - gSysInfo.friction + inertial_force;
//    force_closeLoop = force_PidOutput(force_openLoop, gExternalForceState.value);
//    force_closeLoop = -force_closeLoop;
//
//    gSysInfo.ob_velocityOpenLoop = force_openLoop;
//
//    if(gRotateDirection.rotateDirection == FORWARD_DIRECTION){
//        B_F = 10;
//    }
//    else if(gRotateDirection.rotateDirection == BACKWARD_DIRECTION){
//        B_F = -20;
//    }

//    if(gRotateDirection.rotateDirection == STOP_DIRECTION){
//        gSysInfo.velocity_last = 0;
//    }
//    else{
//        gSysInfo.velocity_last = gSysInfo.velocity_last;
//    }
//
//    velocity_openLoop = gSysInfo.velocity_last + ((gExternalForceState.value - damp_force + gSysInfo.friction - spring_force) / mass);
//    gSysInfo.velocity_last = gSysInfo.velocity_last + ((gExternalForceState.value - damp_force + gSysInfo.friction - spring_force) / mass);
//
//    if(velocity_openLoop > 20){
//        velocity_openLoop = 20;
//    }
//    else if(velocity_openLoop < -20){
//        velocity_openLoop = -20;
//    }
//    else{
//        velocity_openLoop = velocity_openLoop;
//    }
//
//    velocity_closeLoop = velocity_PidOutput(velocity_openLoop, gKeyValue.motorSpeed);
//
//    gSysInfo.targetDuty_V = (int16)((gPidPara.K_V_ODE * velocity_openLoop + B_V) + velocity_closeLoop);
//    gSysInfo.targetDuty_F = (int16)((gPidPara.K_F_ODE * force_openLoop + B_F) + force_closeLoop);
//    gSysInfo.targetDuty = (int16)(gSysInfo.coe_Velocity * gSysInfo.targetDuty_V + gSysInfo.coe_Force * gSysInfo.targetDuty_F);
//    if(gSysInfo.targetDuty > DUTY_LIMIT_P){
//        gSysInfo.targetDuty = DUTY_LIMIT_P;
//    }
//    else if(gSysInfo.targetDuty < DUTY_LIMIT_N){
//        gSysInfo.targetDuty = DUTY_LIMIT_N;
//    }
//}

#pragma CODE_SECTION(OnlyWithSpringFront, "ramfuncs")
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
//	double velocity_openLoop;
//	int velocity_closeLoop;
//	double B_V = 0;

	findSpringForceK(gStickState.value);
	k = gSysInfo.springForceK;
    if(gSysInfo.soft_break_flag == 1){
        return;
    }
	kb = gSysInfo.springForceB;

	mass = (k * 1000) / (gConfigPara.naturalVibrationFreq * gConfigPara.naturalVibrationFreq);

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
        B_F = 20;
    }
    else if(gRotateDirection.rotateDirection == BACKWARD_DIRECTION){
        B_F = -10;
    }


//    if(gRotateDirection.rotateDirection == STOP_DIRECTION){
//        gSysInfo.velocity_last = 0;
//    }
//    else{
//        gSysInfo.velocity_last = gSysInfo.velocity_last;
//    }
//
//	velocity_openLoop = gSysInfo.velocity_last + ((gExternalForceState.value - damp_force + gSysInfo.friction - spring_force) / mass);
//	gSysInfo.velocity_last = gSysInfo.velocity_last + ((gExternalForceState.value - damp_force + gSysInfo.friction - spring_force) / mass);
//
//	if(velocity_openLoop > 20){
//	    velocity_openLoop = 20;
//	}
//    else if(velocity_openLoop < -20){
//        velocity_openLoop = -20;
//    }
//    else{
//        velocity_openLoop = velocity_openLoop;
//    }
//
//	velocity_closeLoop = velocity_PidOutput(velocity_openLoop, gKeyValue.motorSpeed);

//	gSysInfo.targetDuty_V = (int16)((gPidPara.K_V_ODE * velocity_openLoop + B_V) + velocity_closeLoop);
	gSysInfo.targetDuty_F = (int16)((gPidPara.K_F_ODE * force_openLoop + B_F) + force_closeLoop);
	gSysInfo.targetDuty = (int16)(gSysInfo.coe_Velocity * gSysInfo.targetDuty_V + gSysInfo.coe_Force * gSysInfo.targetDuty_F);
    if(gSysInfo.targetDuty > DUTY_LIMIT_P){
        gSysInfo.targetDuty = DUTY_LIMIT_P;
    }
    else if(gSysInfo.targetDuty < DUTY_LIMIT_N){
        gSysInfo.targetDuty = DUTY_LIMIT_N;
    }
}

/**************************************************************
 *Name:		   PidProcess
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018��11��25������9:33:16
 **************************************************************/
//void PidProcess(void){

	//CalculateSpringForce();
	//CalculateDampForce();
	//CalculateTargetAcc();

	//CalculateTargetAcc();
	//CalculateTargetSpeed();
	//CalculateTargeDisplace();

//	RKT(0,gKeyValue.displacement,gKeyValue.motorSpeed,0.01);

	//UpdateAccErr();
	//UpdateSpeedErr();
	//UpdateDisplacementErr();
//}
//F - K1 * dy/dt - K2 * y = m * dy2/dt2
//F�� - K���� * dy/dt - K�� * y = m * dy2/dt2
//��z = dy/dt
//=====>
//double function(double x0, double y0, double z0, double h){
//	double K11;
//	double K12;
//	double K13;
//	double K14;
//
//	double K21;
//	double K22;
//	double K23;
//	double K24;
//
//	double a = 0;//a = f����/m
//	double b = 0;//b = f����/m
//	double c = 0;//c = -f����/m
//
//
//	double y1;
//	double z1;
//	double a1;
//
//	double k;
//	double kb;


//	a = gSysPara.k_dampForce/gSysPara.mass;
//	b = gSysPara.k_springForce / gSysPara.mass;
//	c = gKeyValue.force / gSysPara.mass;

//	k = findSpringForceK(y0);
//	kb = findSpringForceB(y0);
	//gSysPara.k_springForce = k;
	//gSysPara.k_dampForce = kb;
	//gSysPara.mass = k/(4); 

	//a = 2 * gConfigPara.dampingFactor * sqrt(k / gSysPara.mass);
//	a = 2 * gConfigPara.dampingFactor * 2;
//	b =	((k * y0) + kb)/gSysPara.mass;
//	c =	(-gKeyValue.force) / gSysPara.mass;
//
//	K11 = z0;
//	K21 = c - (a * z0) - (b * y0);
//
//	K12 = z0 + h/2 * K21;
//	K22 = c - b * (y0 + h/2 * K11) - a * (z0 + h/2 * K21);
//
//	K13 = z0 + h/2 * K22;
//	K23 = c - b * (y0 + h/2 * K12) - a * (z0 + h/2 * K22);
//
//	K14 = z0 + h * K23;
//	K24 = c - b * (y0 + h/2 * K13) - a * (z0 + h * K23);
//
//	y1 = y0 + h/6 *(K11 + 2 * K12 + 2 * K13 + K14);
//	z1 = z0 + h/6 *(K21 + 2 * K22 + 2 * K23 + K24);
//	a1 = c - a * z1 - b * y1;
//
//
//	gSysCurrentState.displaceTarget = y1;
//	gSysCurrentState.speedTarget = z1;
//	gSysCurrentState.accTarget = a1;
//
//	return y1;
//}
/**************************************************************
 *Name:		   RKT
 *Comment:
 *Input:	   void
 *Output:	   int
 *Author:	   Simon
 *Date:		   2018��12��18������9:05:27
 **************************************************************/
//int RKT(double x, double y, double z, double h){
//	int ret = 0;
//	int i;
//
//	double x0 = x;
//	double y0 = y;
//	double z0 = z;
//	double h0 = h;
//
//	for(i = 0; i < 2; ++i){
//		function(x0, y0, z0, h0);
//		//TODO update value of y
//		//TODO update value of z
//		x0 = x0 + h0;
//		y0 = gSysCurrentState.displaceTarget;
//		z0 = gSysCurrentState.speedTarget;
//	}
//	return ret;
//}
