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
    gPidPara.kp = 300;
    gPidPara.ki = 400;
    gPidPara.kd = 0;
    gPidPara.targetPid = 0;  //é”Ÿæ–¤æ‹·é”Ÿæ–¤æ‹·ç‰¡é”Ÿè§’ç™™IDé”Ÿæ–¤æ‹·é”Ÿè¡—ï¿½  ä»€ä¹ˆé”Ÿæ–¤æ‹·æ€�é”Ÿæ–¤æ‹·LUG

    gTargetSpeed = 500;
}
