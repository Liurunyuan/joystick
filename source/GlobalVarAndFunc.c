#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "GlobalVarAndFunc.h"
#include <string.h>
#include "PWM_ISR.h"
#include "PID.h"

#define  DEBOUNCE (0.10)

Uint32 gECapCount = 0;
RS422STATUS gRS422Status = {0};
KeyValue gKeyValue = {0};
SYSINFO gSysInfo = {0};
SYSSTATE gSysState = {0};
SYSPARA gSysPara = {0};
SYSCURRENTSTATE gSysCurrentState = {0};
CONFIGPARA gConfigPara = {0};
FORCE_DISPLACE_CURVE gForceAndDisplaceCurve  = {0};

ANOLOG16BIT gAnalog16bit = {0};


STICKSTATE gStickState = {0};

EXTFORCESTATE gExternalForceState = {0};

int gforwardOverLimit = 0;
int gbackwardOverLimit = 0;
int gCheckStartForceForwardMargin = 0;
int gCheckStartForceBackwardMargin = 0;
int gforwardForce = 0;
int gbackwardForce = 0;
int gNoExternalForce = 0;

typedef void (*CONTROLSTATEMACHINE)(int a,int b);
void InitStickState(void);

void InitGlobalVarAndFunc(void){
	gSysInfo.ddtmax = 1;
	gSysInfo.dutyAddInterval = 2;
	gSysInfo.targetDuty = 0;
	gSysInfo.controlFuncIndex = 0;
	gSysInfo.currentStickDisSection = INIT_SECTION;
	gSysInfo.TH0 = -9.2;
	gSysInfo.TH1 = -2.0;
	gSysInfo.TH2 = -1.0;
	gSysInfo.TH3 = 0.0;
	gSysInfo.TH4 = 1.0;
	gSysInfo.TH5 = 2.0;
	gSysInfo.TH6 = 10.2;
	gSysInfo.Ki_Threshold = 100;

	InitSysState();
	InitStickState();
}

void IRNullDisAndNoForce(int a,  int b){
	/*stick is in the range of the null displacement and no external force on the it */
	/*so decide what we should do */
	gSysInfo.targetDuty = 0;

} 

void IRNullDisAndForwardForce(int a, int b){
	/*stick is in the range of the null displacement and the external force is forward */
	/*so decidde what we should do here */
	int32 tmp;
	tmp = (int32)((FORWARD_FORCE_VALUE - gExternalForceState.value)* 500);
	gSysInfo.targetDuty = tmp; 
}

void IRNullDisAndBackwardForce(int a, int b){
	/*stick is in the range of the null displacement and the external force is backward */
	/*so decidde what we should do here */
	int32 tmp;
	tmp = (int32)((BACKWARD_FORCE_VALUE - gExternalForceState.value)* 500);
	gSysInfo.targetDuty = tmp; 
}


void OORThresholdDisBackward(int a, int b){
	/*stick is out of the range of the bakcward threshold displacement*/
	/*so decidde what we should do here */
	gSysInfo.targetDuty = 0;

}

void OORThresholdDisForward(int a, int b){
	/*stick is out of the range of the forward threshold displacement*/
	/*so decidde what we should do here */
	gSysInfo.targetDuty = 0;

}

void OORThresholdDis(int a, int b){
	/*stick is out of range of the threshold displacement */
	/*this functin may not need to use */
	gSysInfo.targetDuty = 0;

}
/* 
* -20mm                                                     0mm                                                      12mm 
*  |<--------------------------Backwards--------------------->|<------------------------Forward------------------------->| 
*  |                                                          |
*  |Threshold|        ODE      | StartForce   |     Null      |     Null      | StartForce    |      ODE       |Threshold|
*  |--Sec0---|-------Sec1------|----Sec2------|----Sec3-------|------Sec4-----|-----Sec5------|-----Sec6-------|---Sec7--|
*  |--------TH0---------------TH1------------TH2-------------TH3-------------TH4-------------TH5---------------TH6-------|
*  |----- -18mm ----------- -15mm -------- -10mm ----------- 0mm ----------  8mm ----------  9mm ------------ 10mm ------|
*/

void IRStartForceSecAndNoForce_sec2(int a,  int b){
    /*stick is in the range of the null displacement and no external force on the it */
    /*so decide what we should do */
    int32 tmp;
    tmp = (int32)((gSysInfo.TH2 - gStickState.value)* 100);
    //tmp = (int32)(displace_PidOutput(gSysInfo.TH2, gStickState.value));
    //tmp = -tmp;
    gSysInfo.targetDuty = tmp;

}

void IRStartForceSecAndNoForce_sec5(int a,  int b){
    /*stick is in the range of the null displacement and no external force on the it */
    /*so decide what we should do */
    int32 tmp;
    tmp = (int32)((gSysInfo.TH4 - gStickState.value)* 100);
    //tmp = (int32)(displace_PidOutput(gSysInfo.TH4, gStickState.value));
    //tmp = -tmp;
    gSysInfo.targetDuty = tmp;

}

void IRStartForceSecAndForwardForce_sec5(int a, int b){
    /*stick is in the range of the null displacement and the external force is forward */
    /*so decidde what we should do here */
    int32 tmp;
    if(gExternalForceState.value < FOWARD_START_FORCE){
        IRNullDisAndForwardForce(0,0);
    }
    else{
        //tmp = (int32)((gSysInfo.TH4 - gStickState.value)* 100);

		// if(gStickState.value > (gSysInfo.TH4 + ((gSysInfo.TH5 - gSysInfo.TH4)/2))){
        // 	tmp = (int32)((gSysInfo.TH4 - gStickState.value)* 100);
		// }
		// else{
        // 	tmp = displace_PidOutput(gSysInfo.TH4, gStickState.value);
		// }
#if(MACHINE_FRICTION == INCLUDE_FEATURE)
        tmp = -10;//if duty set to 0, you need 22N to push the stick move 
#elif
        tmp = displace_PidOutput(gSysInfo.TH4, gStickState.value);
#endif
        //tmp = -tmp;
        gSysInfo.targetDuty = tmp;
    }
}

void IRStartForceSecAndBackwardForce_sec2(int a, int b){
    /*stick is in the range of the null displacement and the external force is backward */
    /*so decidde what we should do here */
    int32 tmp;
    if(gExternalForceState.value > BACKWARD_START_FORCE){
        IRNullDisAndBackwardForce(0,0);
    }
    else{
        //tmp = (int32)((gSysInfo.TH2 - gStickState.value)* 100);
        //tmp = (int32)(displace_PidOutput(gSysInfo.TH2, gStickState.value));
        //tmp = -tmp;
		tmp = 10;
        gSysInfo.targetDuty = tmp;
    }
}

void sec0_threshold_rear(int a, int b){
    /*stick is out of the range of the bakcward threshold displacement*/
    /*so decidde what we should do here */
    //gSysInfo.sek = 0;
    gSysInfo.targetDuty = 0;

}

void sec1_ODE_rear(int a, int b){
    /*stick is out of the range of the bakcward threshold displacement*/
    /*so decidde what we should do here */
    //gSysInfo.sek = 0;
    gSysInfo.targetDuty = 0;

}

void sec2_StartForce_rear(int a, int b){
    /*stick is out of the range of the bakcward threshold displacement*/
    /*so decidde what we should do here */
    //gSysInfo.sek = 0;
    switch (gExternalForceState.ForceState)
    {
    case NO_FORCE:
		//gSysInfo.targetDuty = 10; 
        IRStartForceSecAndNoForce_sec2(0,0);
        break;

    case BACKWARD_FORCE:
        IRStartForceSecAndBackwardForce_sec2(0,0);
        break;

    case FORWARD_FORCE:
        IRNullDisAndForwardForce(0,0);
        break;

    default:
        break;
    }

}

void sec3_Null_rear(int a, int b){
    /*stick is out of the range of the bakcward threshold displacement*/
    /*so decidde what we should do here */
   // gSysInfo.sek = 0;
    switch (gExternalForceState.ForceState)
    {
    case NO_FORCE:
        IRNullDisAndNoForce(0,0);
        break;

    case BACKWARD_FORCE:
        IRNullDisAndBackwardForce(0,0);
        break;

    case FORWARD_FORCE:
        IRNullDisAndForwardForce(0,0);
        break;

    default:
        break;
    }

}
void sec4_Null_front(int a, int b){
    /*stick is out of the range of the bakcward threshold displacement*/
    /*so decidde what we should do here */
    //gSysInfo.sek = 0;
    switch (gExternalForceState.ForceState)
    {
    case NO_FORCE:
        IRNullDisAndNoForce(0,0);
        break;

    case BACKWARD_FORCE:
        IRNullDisAndBackwardForce(0,0);
        break;

    case FORWARD_FORCE:
        IRNullDisAndForwardForce(0,0);
        break;

    default:
        break;
    }
}
void sec5_StartForce_front(int a, int b){
    /*stick is out of the range of the bakcward threshold displacement*/
    /*so decidde what we should do here */
    //gSysInfo.sek = 0;
    switch (gExternalForceState.ForceState)
    {
    case NO_FORCE:
        //gSysInfo.targetDuty = -10;
        IRStartForceSecAndNoForce_sec5(0,0);
        break;

    case BACKWARD_FORCE:
        //gSysInfo.targetDuty = -10;
        IRNullDisAndBackwardForce(0,0);
        break;

    case FORWARD_FORCE:
        IRStartForceSecAndForwardForce_sec5(0,0);
        break;

    default:
        break;
    }

}
void sec6_ODE_front(int a, int b){
    /*stick is out of the range of the bakcward threshold displacement*/
    /*so decidde what we should do here */
    //gSysInfo.sek = 0;
    gSysInfo.targetDuty = 0;

}
void sec7_threshold_front(int a, int b){
    /*stick is out of the range of the bakcward threshold displacement*/
    /*so decidde what we should do here */
    //gSysInfo.sek = 0;
    gSysInfo.targetDuty = 0;

}

const CONTROLSTATEMACHINE controlStateMahchineInterface[] = {
    sec0_threshold_rear,              	//0:	Rear OOR
	sec3_Null_rear, 					//1:	Rear ODE 
	sec2_StartForce_rear,              	//2:	Rear Start force
	sec3_Null_rear, 					//3:	Rear Null displacement
	sec4_Null_front,					//4:	Front Null displacement
	sec5_StartForce_front,              //5:	Front Start force
	sec4_Null_front,					//6:	Front ODE
	sec7_threshold_front               	//7:	Front OOR
};

void ControleStateMachineSwitch(int value){

	if(value < (sizeof(controlStateMahchineInterface) / sizeof(controlStateMahchineInterface[0]))){
		if(controlStateMahchineInterface[value]){
			controlStateMahchineInterface[value](0, 0);
		}
	}
	else{
		//TODO generate alarm msg, something wrong
	}
}

void checkExternalForce(int value){
	/*need to decide if need to enable debouce feature */
	switch (gExternalForceState.ForceState)
	{
	case INIT_FORCE:

		if(gExternalForceState.value > BACKWARD_FORCE_VALUE){
			gExternalForceState.ForceState = BACKWARD_FORCE;
		}
		else if (gExternalForceState.value  < FORWARD_FORCE_VALUE){
			gExternalForceState.ForceState = FORWARD_FORCE;
		}
		else{
			gExternalForceState.ForceState = NO_FORCE;
		}
		break;
	case FORWARD_FORCE:
		if(gExternalForceState.value > BACKWARD_FORCE_VALUE){
			gExternalForceState.ForceState = BACKWARD_FORCE;
		}
		else if(gExternalForceState.value < FORWARD_FORCE_VALUE){
			gExternalForceState.ForceState = FORWARD_FORCE;
		}
		else{
			gExternalForceState.ForceState = NO_FORCE;
		}
		break;
	case BACKWARD_FORCE:
		if(gExternalForceState.value > BACKWARD_FORCE_VALUE){
			gExternalForceState.ForceState = BACKWARD_FORCE;
		}
		else if(gExternalForceState.value < FORWARD_FORCE_VALUE){
			gExternalForceState.ForceState = FORWARD_FORCE;
		}
		else{
			gExternalForceState.ForceState = NO_FORCE;
		}
		break;
	case NO_FORCE:
		if(gExternalForceState.value > BACKWARD_FORCE_VALUE){
			gExternalForceState.ForceState = BACKWARD_FORCE;
		}
		else if(gExternalForceState.value < FORWARD_FORCE_VALUE){
			gExternalForceState.ForceState = FORWARD_FORCE;
		}
		else{
			gExternalForceState.ForceState = NO_FORCE;
		}
		break;
	default:
		break;
	}
}

void InitStickState(void){

	gExternalForceState.ForceState = INIT_FORCE;
	gExternalForceState.updateForceState = checkExternalForce;

	gStickState.value = 0;
	gExternalForceState.value = 0;
}

void InitForceDisplaceCurve(void){

}

void UpdateForceDisplaceCurve(void){
	int index;

	gForceAndDisplaceCurve.springForceP[0] = gConfigPara.LF_Force1;
	gForceAndDisplaceCurve.springForceP[1] = gConfigPara.LF_Force2;
	gForceAndDisplaceCurve.springForceP[2] = gConfigPara.LF_Force3;
	gForceAndDisplaceCurve.springForceP[3] = gConfigPara.LF_Force4;
	gForceAndDisplaceCurve.springForceP[4] = gConfigPara.LF_Force5;
	gForceAndDisplaceCurve.springForceP[5] = gConfigPara.LF_Force6;
	gForceAndDisplaceCurve.springForceP[6] = gConfigPara.LF_Force7;
	gForceAndDisplaceCurve.springForceP[7] = gConfigPara.LF_Force8;
	gForceAndDisplaceCurve.springForceP[8] = gConfigPara.LF_Force9;
	gForceAndDisplaceCurve.springForceP[9] = gConfigPara.LF_MaxForce;

	gForceAndDisplaceCurve.springForceN[0] = gConfigPara.RB_Force1;
	gForceAndDisplaceCurve.springForceN[1] = gConfigPara.RB_Force2;
	gForceAndDisplaceCurve.springForceN[2] = gConfigPara.RB_Force3;
	gForceAndDisplaceCurve.springForceN[3] = gConfigPara.RB_Force4;
	gForceAndDisplaceCurve.springForceN[4] = gConfigPara.RB_Force5;
	gForceAndDisplaceCurve.springForceN[5] = gConfigPara.RB_Force6;
	gForceAndDisplaceCurve.springForceN[6] = gConfigPara.RB_Force7;
	gForceAndDisplaceCurve.springForceN[7] = gConfigPara.RB_Force8;
	gForceAndDisplaceCurve.springForceN[8] = gConfigPara.RB_Force9;
	gForceAndDisplaceCurve.springForceN[9] = gConfigPara.RB_MaxForce;

	gForceAndDisplaceCurve.displacementP[0] = gConfigPara.LF_Distance1;
	gForceAndDisplaceCurve.displacementP[1] = gConfigPara.LF_Distance2;
	gForceAndDisplaceCurve.displacementP[2] = gConfigPara.LF_Distance3;
	gForceAndDisplaceCurve.displacementP[3] = gConfigPara.LF_Distance4;
	gForceAndDisplaceCurve.displacementP[4] = gConfigPara.LF_Distance5;
	gForceAndDisplaceCurve.displacementP[5] = gConfigPara.LF_Distance6;
	gForceAndDisplaceCurve.displacementP[6] = gConfigPara.LF_Distance7;
	gForceAndDisplaceCurve.displacementP[7] = gConfigPara.LF_Distance8;
	gForceAndDisplaceCurve.displacementP[8] = gConfigPara.LF_Distance9;
	gForceAndDisplaceCurve.displacementP[9] = gConfigPara.LF_MaxDistance;


	gForceAndDisplaceCurve.displacementN[0] = gConfigPara.RB_Distance1;
	gForceAndDisplaceCurve.displacementN[1] = gConfigPara.RB_Distance2;
	gForceAndDisplaceCurve.displacementN[2] = gConfigPara.RB_Distance3;
	gForceAndDisplaceCurve.displacementN[3] = gConfigPara.RB_Distance4;
	gForceAndDisplaceCurve.displacementN[4] = gConfigPara.RB_Distance5;
	gForceAndDisplaceCurve.displacementN[5] = gConfigPara.RB_Distance6;
	gForceAndDisplaceCurve.displacementN[6] = gConfigPara.RB_Distance7;
	gForceAndDisplaceCurve.displacementN[7] = gConfigPara.RB_Distance8;
	gForceAndDisplaceCurve.displacementN[8] = gConfigPara.RB_Distance9;
	gForceAndDisplaceCurve.displacementN[9] = gConfigPara.RB_MaxDistance;

	gForceAndDisplaceCurve.maxPoints = 10;

	for(index = 1; index <gForceAndDisplaceCurve.maxPoints; ++index){
		gForceAndDisplaceCurve.K_spring_forceP[index] =
				(gForceAndDisplaceCurve.springForceP[index] - gForceAndDisplaceCurve.springForceP[index - 1]) /
				(gForceAndDisplaceCurve.displacementP[index] - gForceAndDisplaceCurve.displacementP[index - 1]);

		gForceAndDisplaceCurve.b_P[index] =
				gForceAndDisplaceCurve.springForceP[index] - gForceAndDisplaceCurve.K_spring_forceP[index] * gForceAndDisplaceCurve.displacementP[index];
	}

	for(index = 1; index <gForceAndDisplaceCurve.maxPoints; ++index){
		gForceAndDisplaceCurve.K_spring_forceN[index] =
				(gForceAndDisplaceCurve.springForceN[index] - gForceAndDisplaceCurve.springForceN[index - 1]) /
				(gForceAndDisplaceCurve.displacementN[index] - gForceAndDisplaceCurve.displacementN[index - 1]);

		gForceAndDisplaceCurve.b_N[index] =
				gForceAndDisplaceCurve.springForceN[index] - gForceAndDisplaceCurve.K_spring_forceN[index] * gForceAndDisplaceCurve.displacementN[index];
	}
}

void InitConfigParameter(void){
	gConfigPara.LF_Force1 = 5;
	gConfigPara.LF_Force2 = 10;
	gConfigPara.LF_Force3 = 15;
	gConfigPara.LF_Force4 = 20;
	gConfigPara.LF_Force5 = 25;
	gConfigPara.LF_Force6 = 30;
	gConfigPara.LF_Force7 = 35;
	gConfigPara.LF_Force8 = 40;
	gConfigPara.LF_Force9 = 45;
	gConfigPara.LF_MaxForce = 50;

	gConfigPara.RB_Force1 = -5;
	gConfigPara.RB_Force2 = -10;
	gConfigPara.RB_Force3 = -15;
	gConfigPara.RB_Force4 = -20;
	gConfigPara.RB_Force5 = -25;
	gConfigPara.RB_Force6 = -30;
	gConfigPara.RB_Force7 = -35;
	gConfigPara.RB_Force8 = -40;
	gConfigPara.RB_Force9 = -45;
	gConfigPara.RB_MaxForce = -50;

	gConfigPara.LF_Distance1 = 2;
	gConfigPara.LF_Distance2 = 3;
	gConfigPara.LF_Distance3 = 4;
	gConfigPara.LF_Distance4 = 5;
	gConfigPara.LF_Distance5 = 6;
	gConfigPara.LF_Distance6 = 7;
	gConfigPara.LF_Distance7 = 8;
	gConfigPara.LF_Distance8 = 9;
	gConfigPara.LF_Distance9 = 9.5;
	gConfigPara.LF_MaxDistance = 10;

	gConfigPara.RB_Distance1 = -2;
	gConfigPara.RB_Distance2 = -3;
	gConfigPara.RB_Distance3 = -4;
	gConfigPara.RB_Distance4 = -5;
	gConfigPara.RB_Distance5 = -6;
	gConfigPara.RB_Distance6 = -7;
	gConfigPara.RB_Distance7 = -7.5;
	gConfigPara.RB_Distance8 = -8;
	gConfigPara.RB_Distance9 = -8.5;
	gConfigPara.RB_MaxDistance = -9;

	gConfigPara.LF_StartForce = 0;
	gConfigPara.RB_StartForce = 0;

	gConfigPara.LF_FrontFriction = 0;
	gConfigPara.LF_RearFriction = 0;
	gConfigPara.RB_FrontFriction = 0;
	gConfigPara.RB_RearFriction = 0;

	gConfigPara.LF_EmptyDistance = 0;
	gConfigPara.RB_EmptyDistance = 0;

	gConfigPara.dampingFactor = 0;

	gConfigPara.naturalVibrationFreq = 0;

	gConfigPara.equivalentMass = 0;

	gConfigPara.LF_TrimRange = 0;
	gConfigPara.RB_TrimRange = 0;

	gConfigPara.trimTarget = 0;

	gConfigPara.trimCommand = 0;

	gConfigPara.timeDelay = 0;

	gConfigPara.stateCommand = 0;

	gConfigPara.innerErrorThresholdWithInnerMaxKp = 0;
	gConfigPara.innerMaxKp = 0;
	gConfigPara.innerKi = 0;
	gConfigPara.innerKd = 0;
	gConfigPara.innerFeedForward = 0;
	gConfigPara.innerMaxStartError = 0;
	gConfigPara.innerMinStartError = 0;
	gConfigPara.innerMaxIntergralSaturation = 0;
	gConfigPara.innerMinIntergralSaturation = 0;

	gConfigPara.middleKd = 0;
	gConfigPara.middleKi = 0;
	gConfigPara.middleKp = 0;

	gConfigPara.outerKd = 0;
	gConfigPara.outerKi = 0;
	gConfigPara.outerKp = 0;
}
/**************************************************************
 *Name:		   InitSysState
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018��11��25������12:40:11
 **************************************************************/
void InitSysState(void){
	gSysState.alarm.all 	= 0;
	gSysState.erro.all 		= 0;
	gSysState.warning.all 	= 0;

	gSysPara.k_dampForce = 0;
	gSysPara.k_springForce = 0;
	gSysPara.mass = 0;

	gSysCurrentState.accTarget = 0;
	gSysCurrentState.dampForce = 0;
	gSysCurrentState.displaceTarget = 0;
	gSysCurrentState.speedTarget = 0;
	gSysCurrentState.springForce = 0;
}
/**************************************************************
 *Name:		   KalmanFilter
 *Comment:
 *Input:	   const double, double, double
 *Output:	   double
 *Author:	   Simon
 *Date:		   2019��1��2������9:57:12
 **************************************************************/
double KalmanFilter(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R)
{

	double R = MeasureNoise_R;
	double Q = ProcessNiose_Q;

	static double x_last = 0;
	double x_mid = x_last;
	double x_now;

	static double p_last = 0;
	double p_mid;
	double p_now;

	double kg;

	x_mid = x_last;
	p_mid = p_last + Q;


	kg = p_mid / (p_mid + R);
	x_now = x_mid + kg * (ResrcData - x_mid);
	p_now = (1 - kg) * p_mid;
	p_last = p_now;
	x_last = x_now;

	return x_now;
}
double KalmanFilterSpeed(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R)
{
	double R = MeasureNoise_R;
	double Q = ProcessNiose_Q;

	static double x_last = 0;
	double x_mid = x_last;
	double x_now;

	static double p_last = 0;
	double p_mid;
	double p_now;

	double kg;

	x_mid = x_last;
	p_mid = p_last + Q;

	kg = p_mid / (p_mid + R);
	x_now = x_mid + kg * (ResrcData - x_mid);
	p_now = (1 - kg) * p_mid;
	p_last = p_now;
	x_last = x_now;

	return x_now;
}


void Enable_KZ_P_DSP(void){
	GpioDataRegs.GPASET.bit.GPIO31 = 1;
}
void Disable_KZ_P_DSP(void){
	GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;
}

void Enable_KZ_N_DSP(void){
	GpioDataRegs.GPCCLEAR.bit.GPIO86 = 1;
}
void Disable_KZ_N_DSP(void){
	GpioDataRegs.GPCSET.bit.GPIO86 = 1;
}

void EnablePwmOutput(void){
	//gSysInfo.currentHallPosition = GetCurrentHallValue();
	//gSysInfo.lastTimeHalllPosition = gSysInfo.currentHallPosition;

	GpioDataRegs.GPCCLEAR.bit.GPIO87 = 1;
	Enable_KZ_P_DSP();
	Enable_KZ_N_DSP();
}
void DisablePwmOutput(void){
	GpioDataRegs.GPCSET.bit.GPIO87 = 1;
	//GpioDataRegs.GPCCLEAR.bit.GPIO87 = 1; //For temp use
	Disable_KZ_P_DSP();
	Disable_KZ_N_DSP();
	EPwm1Regs.AQCSFRC.all = 0x0009; //DisablePwm1();
	EPwm2Regs.AQCSFRC.all = 0x0009; //DisablePwm2();
	EPwm3Regs.AQCSFRC.all = 0x0009; //DisablePwm3();
}
void StateMachine(void){
	//gConfigPara.stateCommand = 1; // For TEMP Test
	if(gConfigPara.stateCommand == 1 && gSysState.erro.bit.software != 1){
		EnablePwmOutput();
	}
	else{
		DisablePwmOutput();
	}
}

void ClearFault(void){
	int i = 0;

	GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
	//TODO delay for some time, need to verify
	for(i = 0; i < 1000; ++i){
		asm(" NOP");
	}
	GpioDataRegs.GPASET.bit.GPIO9 = 1;
}

void Enable_PWMD_BK(void){
	GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;
}
void Disable_PWMD_BK(void){
	GpioDataRegs.GPASET.bit.GPIO9 = 1;
}

/* 
* -20mm                                                     0mm                                                      12mm 
*  |<--------------------------Backwards--------------------->|<------------------------Forward------------------------->| 
*  |                                                          |
*  |Threshold|        ODE      | StartForce   |     Null      |     Null      | StartForce    |      ODE       |Threshold|
*  |--Sec0---|-------Sec1------|----Sec2------|----Sec3-------|------Sec4-----|-----Sec5------|-----Sec6-------|---Sec7--|
*  |--------TH0---------------TH1------------TH2-------------TH3-------------TH4-------------TH5---------------TH6-------|
*  |----- -18mm ----------- -15mm -------- -10mm ----------- 0mm ----------  8mm ----------  9mm ------------ 10mm ------|
*/

int CheckStickSetion(double val){
	if(val <= gSysInfo.TH0){
		return 0;
	}
	else if(val <= gSysInfo.TH1){
		return 1;
	}
	else if(val <= gSysInfo.TH2){
		return 2;
	}
	else if(val <= gSysInfo.TH3){
		return 3;
	}
	else if(val <= gSysInfo.TH4){
		return 4;
	}
	else if(val <= gSysInfo.TH5){
		return 5;
	}
	else if(val <= gSysInfo.TH6){
		return 6;
	}
	else if(val > gSysInfo.TH6){
		return 7;
	}
	else{
		return 8;
	}
}
/* 
* -20mm                                                      0mm                                                      12mm 
*  |<--------------------------Backwards--------------------->|<------------------------Forward------------------------->| 
*  |                                                          |
*  |Threshold|        ODE      | StartForce   |     Null      |     Null      | StartForce    |      ODE       |Threshold|
*  |--Sec0---|-------Sec1------|----Sec2------|----Sec3-------|------Sec4-----|-----Sec5------|-----Sec6-------|---Sec7--|
*  |--------TH0---------------TH1------------TH2-------------TH3-------------TH4-------------TH5---------------TH6-------|
*  |----- -18mm ----------- -15mm -------- -10mm ----------- 0mm ----------  8mm ----------  9mm ------------ 10mm ------|
*/

int LocateStickDisSection(void){
	switch (gSysInfo.currentStickDisSection)
	{
	case INIT_SECTION: 
		gSysInfo.currentStickDisSection = CheckStickSetion(gStickState.value);
		break;
	case 0:
		if(gStickState.value  > (gSysInfo.TH0 + DEBOUNCE)){
			gSysInfo.currentStickDisSection = CheckStickSetion(gStickState.value);
		}
		else{
		    gSysInfo.currentStickDisSection = CheckStickSetion(gStickState.value);
		}
		break;
	case 1:
		if((gStickState.value  > (gSysInfo.TH1 + DEBOUNCE)) || (gStickState.value < (gSysInfo.TH0 - DEBOUNCE))){
			gSysInfo.currentStickDisSection = CheckStickSetion(gStickState.value);
		}
		break;
	case 2:
		if((gStickState.value  > (gSysInfo.TH2 + DEBOUNCE)) || (gStickState.value < (gSysInfo.TH1 - DEBOUNCE))){
			gSysInfo.currentStickDisSection = CheckStickSetion(gStickState.value);
		}
		break;
	case 3:
		if((gStickState.value  > (gSysInfo.TH3 + DEBOUNCE)) || (gStickState.value < (gSysInfo.TH2 - DEBOUNCE))){
			gSysInfo.currentStickDisSection = CheckStickSetion(gStickState.value);
		}
		break;
	case 4:
		if((gStickState.value  > (gSysInfo.TH4 + DEBOUNCE)) || (gStickState.value < (gSysInfo.TH3 - DEBOUNCE))){
			gSysInfo.currentStickDisSection = CheckStickSetion(gStickState.value);
		}
		break;
	case 5:
		if((gStickState.value  > (gSysInfo.TH5 + DEBOUNCE)) || (gStickState.value < (gSysInfo.TH4 - DEBOUNCE))){
			gSysInfo.currentStickDisSection = CheckStickSetion(gStickState.value);
		}
		break;
	case 6:
		if((gStickState.value  > (gSysInfo.TH6 + DEBOUNCE)) || (gStickState.value < (gSysInfo.TH5 - DEBOUNCE))){
			gSysInfo.currentStickDisSection = CheckStickSetion(gStickState.value);
		}
		break;
	case 7:
		if(gStickState.value < (gSysInfo.TH6 - DEBOUNCE)){
			gSysInfo.currentStickDisSection = CheckStickSetion(gStickState.value);
		}
		break;
	
	default:
		break;
	}
	return gSysInfo.currentStickDisSection;
}



