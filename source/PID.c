#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "GlobalVarAndFunc.h"
#include "PID.h"

volatile PIDPARA gPidPara = {0};
volatile int gTargetSpeed = 500;

void InitPidVar(void){
//  gPidPara.kp = 450;
//  gPidPara.ki = 500;
    //gPidPara.kp_displace = 100;
    gPidPara.kp_displace = 100;
    gPidPara.ki_displace = 0;
    gPidPara.kd_displace = 0;
    //gPidPara.targetPid_displace = 0;  //é”Ÿæ–¤æ‹·é”Ÿæ–¤æ‹·ç‰¡é”Ÿè§’ç™™IDé”Ÿæ–¤æ‹·é”Ÿè¡—ï¿½  ä»€ä¹ˆé”Ÿæ–¤æ‹·æ€�é”Ÿæ–¤æ‹·LUG

    gPidPara.kp_force = 10;
    gPidPara.ki_force = 1.2;
    gPidPara.kd_force = 0;
    //gPidPara.targetPid_force = 0;

    gTargetSpeed = 500;
}


//int32 displace_PidOutput(double targetVal, double controlVar){
//    int32 pidOutput = 0;
//    double ek1;
//
//    ek1 = (targetVal - controlVar);
////    if((ek1 > -gSysInfo.Ki_Threshold) && (ek1 < gSysInfo.Ki_Threshold))
////    {
////        if(((ek1 > 0) && (gSysInfo.sek < 1171)) || ((ek1 < 0) && (gSysInfo.sek > -1171)))
////        {
////            gSysInfo.sek = gSysInfo.sek + ek1;
////        }
////    }
////    else
////    {
////        gSysInfo.sek = 0;
////    }
//    //pidOutput = (int32)(ek1 * gPidPara.kp_displace) + (int16)(((gSysInfo.sek >> 8) * gPidPara.ki_displace) >> 11);
//    pidOutput = (int32)((ek1 * gPidPara.kp_displace) + (gExternalForceState.value * 3));
//    //pidOutput = (int32)(ek1 * gPidPara.kp_displace);
//
//    if(pidOutput > 750){
//        pidOutput = 750;
//    }
//    else if(pidOutput < -750){
//        pidOutput = -750;
//    }
//    //gPidPara.targetPid_displace = pidOutput;
//
//    return pidOutput;
//}

int16 force_PidOutput(double targetVal, double controlVar){
    int16 pidOutput = 0;
    double ek1;

    ek1 = (targetVal - controlVar);
    if((ek1 > -gSysInfo.Ki_Threshold) && (ek1 < gSysInfo.Ki_Threshold))
    {
        if(((ek1 > 0) && (gSysInfo.sek < 60)) || ((ek1 < 0) && (gSysInfo.sek > -60)))
        {
            gSysInfo.sek = gSysInfo.sek + ek1;
        }
    }
    else
    {
        gSysInfo.sek = 0;
    }
    pidOutput = (int16)(ek1 * gPidPara.kp_force) + (int16)(gSysInfo.sek * gPidPara.ki_force);
    //pidOutput = (int32)((ek1 * gPidPara.kp_force) + (gExternalForceState.value * 2));
    //pidOutput = (int32)(ek1 * gPidPara.kp_displace);

    if(pidOutput > 750){
        pidOutput = 750;
    }
    else if(pidOutput < -750){
        pidOutput = -750;
    }
    //gPidPara.targetPid_displace = pidOutput;

    return pidOutput;
}
