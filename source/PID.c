#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "GlobalVarAndFunc.h"
#include "PID.h"

volatile PIDPARA gPidPara = {0};

void InitPidVar(void){
    gPidPara.kp_velocity_ODE = 200;
    gPidPara.ki_velocity_ODE = 50;

    gPidPara.kp_displace_ODE = 200;
    gPidPara.ki_displace_ODE = 60;

    if(gSysInfo.board_type == PITCH){
        gPidPara.kp_force_ODE = 5.5;
        gPidPara.ki_force_ODE = 0.4;

        gPidPara.K_F_ODE = -0.7;
    }
    else if(gSysInfo.board_type == ROLL){
        gPidPara.kp_force_ODE = 5.5;
        gPidPara.ki_force_ODE = 0.3;

        gPidPara.K_F_ODE = -0.7;
    }
    else{
        gPidPara.kp_force_ODE = 0;
        gPidPara.ki_force_ODE = 0;

        gPidPara.K_F_ODE = 0;
    }

    gPidPara.B_F_ODE = 40;
    gPidPara.K_F_NULL = 0;
    gPidPara.B_F_NULL = 0;
    gPidPara.K_V_ODE = 40;
    gPidPara.B_V_ODE = 0;
    gPidPara.K_V_NULL = 0;
    gPidPara.B_V_NULL = 0;
}

#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(velocity_PidOutput, "ramfuncs")
#endif
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
    pidOutput = (int16)(ek1 * gPidPara.kp_velocity_ODE) + (int16)(gSysInfo.sek_v * gPidPara.ki_velocity_ODE);

    if(pidOutput > 750){
        pidOutput = 750;
    }
    else if(pidOutput < -750){
        pidOutput = -750;
    }

    return pidOutput;
}
#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(force_PidOutput, "ramfuncs")
#endif
int16 force_PidOutput(double targetVal, double controlVar){
    int16 pidOutput = 0;
    double ek1;

    ek1 = (targetVal - controlVar);
    if((ek1 > -gSysInfo.Ki_Threshold_f) && (ek1 < gSysInfo.Ki_Threshold_f))
    {
        if(((ek1 > 0) && (gSysInfo.sek_f < 70)) || ((ek1 < 0) && (gSysInfo.sek_f > -70)))
        {
            gSysInfo.sek_f = gSysInfo.sek_f + ek1;
        }
    }
    else
    {
        gSysInfo.sek_f = 0;
    }
    pidOutput = (int16)(ek1 * gPidPara.kp_force_ODE) + (int16)(gSysInfo.sek_f * gPidPara.ki_force_ODE);
//    gSysInfo.ob_velocityOpenLoop = ek1;
    if(pidOutput > 750){
        pidOutput = 750;
    }
    else if(pidOutput < -750){
        pidOutput = -750;
    }

    return pidOutput;
}


int16 displace_PidOutput(double targetVal, double controlVar){
    int16 pidOutput = 0;
    double ek1;

    ek1 = (targetVal - controlVar);
    if((ek1 > -gSysInfo.Ki_Threshold_d) && (ek1 < gSysInfo.Ki_Threshold_d))
    {
        if(((ek1 > 0) && (gSysInfo.sek_d < 0.3625)) || ((ek1 < 0) && (gSysInfo.sek_d > -0.3625)))
        {
            gSysInfo.sek_d = gSysInfo.sek_d + ek1;
        }
    }
    else
    {
        gSysInfo.sek_d = 0;
    }
    pidOutput = (int16)(ek1 * gPidPara.kp_displace_ODE) + (int16)(gSysInfo.sek_d * gPidPara.ki_displace_ODE);
//    gSysInfo.ob_velocityOpenLoop = ek1;
    if(pidOutput > 750){
        pidOutput = 750;
    }
    else if(pidOutput < -750){
        pidOutput = -750;
    }

    return pidOutput;
}
