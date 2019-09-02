#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "GlobalVarAndFunc.h"
#include "PID.h"

volatile PIDPARA gPidPara = {0};
volatile int gTargetSpeed = 500;

void InitPidVar(void){
    gPidPara.kp_velocity = 1000;
    gPidPara.ki_velocity = 250;

    gPidPara.kp_force = 5;
    gPidPara.ki_force = 0;

    gTargetSpeed = 500;
}


int16 velocity_PidOutput(double targetVal, double controlVar){
    int16 pidOutput = 0;
    double ek1;

    ek1 = (targetVal - controlVar);
    if((ek1 > -gSysInfo.Ki_Threshold_v) && (ek1 < gSysInfo.Ki_Threshold_v))
    {
        if(((ek1 > 0) && (gSysInfo.sek_v < 0.05)) || ((ek1 < 0) && (gSysInfo.sek_v > -0.05)))
        {
            gSysInfo.sek_v = gSysInfo.sek_v + ek1;
        }
    }
    else
    {
        gSysInfo.sek_v = 0;
    }
    pidOutput = (int16)(ek1 * gPidPara.kp_velocity) + (int16)(gSysInfo.sek_v * gPidPara.ki_velocity);

    if(pidOutput > 750){
        pidOutput = 750;
    }
    else if(pidOutput < -750){
        pidOutput = -750;
    }

    return pidOutput;
}

int16 force_PidOutput(double targetVal, double controlVar){
    int16 pidOutput = 0;
    double ek1;

    ek1 = (targetVal - controlVar);
    if((ek1 > -gSysInfo.Ki_Threshold_f) && (ek1 < gSysInfo.Ki_Threshold_f))
    {
        if(((ek1 > 0) && (gSysInfo.sek_f < 60)) || ((ek1 < 0) && (gSysInfo.sek_f > -60)))
        {
            gSysInfo.sek_f = gSysInfo.sek_f + ek1;
        }
    }
    else
    {
        gSysInfo.sek_f = 0;
    }
    pidOutput = (int16)(ek1 * gPidPara.kp_force) + (int16)(gSysInfo.sek_f * gPidPara.ki_force);

    if(pidOutput > 750){
        pidOutput = 750;
    }
    else if(pidOutput < -750){
        pidOutput = -750;
    }

    return pidOutput;
}
