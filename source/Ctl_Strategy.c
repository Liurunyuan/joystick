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
double findSpringForceK(double displace){
	double springForce = -1;
	int index;
	if(displace >= 0){
		for(index = 1; index < gForceAndDisplaceCurve.maxPoints; ++index){

			if((displace <= gForceAndDisplaceCurve.displacementP[index]) && (displace >= gForceAndDisplaceCurve.displacementP[index - 1])){
				springForce = gForceAndDisplaceCurve.K_spring_forceP[index];
				return springForce;
			}
			else if(displace > gForceAndDisplaceCurve.displacementP[(gForceAndDisplaceCurve.maxPoints)-1]){
                springForce = 10;
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
            else if(displace < gForceAndDisplaceCurve.displacementN[(gForceAndDisplaceCurve.maxPoints)-1]){
                springForce = 6;
                return springForce;
            }
		}
	}

	//TODO generate alarm, because the displacement is out of range
	return springForce;
}

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

#pragma CODE_SECTION(OnlyWithSpringRear, "ramfuncs")
void OnlyWithSpringRear(void){
    double k;
    double kb;
    double force_openLoop;
    int force_closeLoop;
    double friction;
    double damp_force;
    double spring_force;
    double mass;
    double inertial_force;
    double velocity_force;
    double B_F = 0;

    double velocity_openLoop;
    int velocity_closeLoop;
    double B_V = 0;

    if(gExternalForceState.ForceState == BACKWARD_FORCE){
        B_F = -gPidPara.B_F_ODE;
    }
    else if(gExternalForceState.ForceState == FORWARD_FORCE){
        B_F = gPidPara.B_F_ODE;
    }
    else{
        B_F = 0;
    }

    k = findSpringForceK(gStickState.value);
    kb = findSpringForceB(gStickState.value);

    mass = (k * 1000) / (gConfigPara.naturalVibrationFreq * gConfigPara.naturalVibrationFreq);

    if(mass > 1){
        gSysState.warning.bit.a = 0;
    }
    else{
        gSysState.warning.bit.a = 1;
    }

    friction = gConfigPara.LF_FrontFriction;

    spring_force = k * gStickState.value + kb;
    damp_force = 2 * gConfigPara.dampingFactor * mass * gKeyValue.motorSpeed * gConfigPara.naturalVibrationFreq;
    inertial_force = mass * gKeyValue.motorAccel;

    if(gRotateDirection.rotateDirection == FORWARD_DIRECTION){
        velocity_force = 0 - friction - damp_force;
        B_V = gPidPara.B_V_ODE;
    }
    else if(gRotateDirection.rotateDirection == BACKWARD_DIRECTION){
        velocity_force = friction - damp_force;
        B_V = -gPidPara.B_V_ODE;
    }
    else{
        velocity_force = 0;
        friction = 0;
        B_V = 0;
    }

    if(gAccelDirection.accelDirection == STOP_DIRECTION){
        inertial_force = 0;
    }
    else{
        inertial_force = -inertial_force;
    }

    force_openLoop = spring_force + velocity_force + inertial_force;
    force_closeLoop = force_PidOutput(force_openLoop, gExternalForceState.value);
    force_closeLoop = -force_closeLoop;

    velocity_openLoop = gSysInfo.velocity_last + ((gExternalForceState.value - velocity_force - spring_force) / mass) * (0.25/1000);
    gSysInfo.velocity_last = gSysInfo.velocity_last + ((gExternalForceState.value - velocity_force - spring_force) / mass) * (0.25/1000);

    if(velocity_openLoop > 20){
        velocity_openLoop = 20;
    }
    else if(velocity_openLoop < -20){
        velocity_openLoop = -20;
    }
    else{
        velocity_openLoop = velocity_openLoop;
    }

    velocity_closeLoop = velocity_PidOutput(velocity_openLoop, gKeyValue.motorSpeed);

    gSysInfo.targetDuty_V = (int16)((gPidPara.K_V_ODE * velocity_openLoop + B_V) + velocity_closeLoop);
    gSysInfo.targetDuty_F = (int16)((gPidPara.K_F_ODE * force_openLoop + B_F) + force_closeLoop);
    gSysInfo.targetDuty = (int16)(gSysInfo.coe_Velocity * gSysInfo.targetDuty_V + gSysInfo.coe_Force * gSysInfo.targetDuty_F);
    if(gSysInfo.targetDuty > DUTY_LIMIT_P){
        gSysInfo.targetDuty = DUTY_LIMIT_P;
    }
    else if(gSysInfo.targetDuty < DUTY_LIMIT_N){
        gSysInfo.targetDuty = DUTY_LIMIT_N;
    }
}

#pragma CODE_SECTION(OnlyWithSpringFront, "ramfuncs")
void OnlyWithSpringFront(void){
	double k;
	double kb;
	double force_openLoop;
	int force_closeLoop;
	double friction;
	double damp_force;
	double spring_force;
	double mass;
	double inertial_force;
	double velocity_force;
	double B_F = 0;

	double velocity_openLoop;
	int velocity_closeLoop;
	double B_V = 0;

	if(gExternalForceState.ForceState == BACKWARD_FORCE){
		B_F = -gPidPara.B_F_ODE;
	}
	else if(gExternalForceState.ForceState == FORWARD_FORCE){
		B_F = gPidPara.B_F_ODE;
	}
	else{
		B_F = 0;
	}

	k = findSpringForceK(gStickState.value);
	kb = findSpringForceB(gStickState.value);

	mass = (k * 1000) / (gConfigPara.naturalVibrationFreq * gConfigPara.naturalVibrationFreq);

	if(mass > 1){
	    gSysState.warning.bit.a = 0;
	}
	else{
	    gSysState.warning.bit.a = 1;
	}

    friction = gConfigPara.LF_FrontFriction;

	spring_force = k * gStickState.value + kb;
	damp_force = 2 * gConfigPara.dampingFactor * mass * gKeyValue.motorSpeed * gConfigPara.naturalVibrationFreq;
	inertial_force = mass * gKeyValue.motorAccel;

	if(gRotateDirection.rotateDirection == FORWARD_DIRECTION){
	    velocity_force = 0 - friction - damp_force;
	    B_V = gPidPara.B_V_ODE;
	}
	else if(gRotateDirection.rotateDirection == BACKWARD_DIRECTION){
	    velocity_force = friction - damp_force;
	    B_V = -gPidPara.B_V_ODE;
	}
	else{
	    velocity_force = 0;
        friction = 0;
        B_V = 0;
	}

	if(gAccelDirection.accelDirection == STOP_DIRECTION){
	    inertial_force = 0;
	}
	else{
	    inertial_force = -inertial_force;
	}

	force_openLoop = spring_force + velocity_force + inertial_force;
	force_closeLoop = force_PidOutput(force_openLoop, gExternalForceState.value);
	force_closeLoop = -force_closeLoop;

	velocity_openLoop = gSysInfo.velocity_last + ((gExternalForceState.value - velocity_force - spring_force) / mass) * (0.25/1000);
	gSysInfo.velocity_last = gSysInfo.velocity_last + ((gExternalForceState.value - velocity_force - spring_force) / mass) * (0.25/1000);

	if(velocity_openLoop > 20){
	    velocity_openLoop = 20;
	}
    else if(velocity_openLoop < -20){
        velocity_openLoop = -20;
    }
    else{
        velocity_openLoop = velocity_openLoop;
    }

	velocity_closeLoop = velocity_PidOutput(velocity_openLoop, gKeyValue.motorSpeed);

	gSysInfo.targetDuty_V = (int16)((gPidPara.K_V_ODE * velocity_openLoop + B_V) + velocity_closeLoop);
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

	k = findSpringForceK(y0);
	kb = findSpringForceB(y0);
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
