#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "GlobalVarAndFunc.h"
#include <string.h>
#include "PWM_ISR.h"




Uint32 gECapCount = 0;
RS422STATUS gRS422Status = {0};
KeyValue gKeyValue = {0};
SYSINFO gSysInfo = {0};
SYSSTATE gSysState = {0};
SYSPARA gSysPara = {0};
SYSCURRENTSTATE gSysCurrentState = {0};
CONFIGPARA gConfigPara = {0};
FORCE_DISPLACE_CURVE gForceAndDisplaceCurve  = {0};

Uint32 gSysStateMachineNumber = 0;

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
	tmp = (int32)((30587.0 - gExternalForceState.value)* 0.0085);
	gSysInfo.targetDuty = tmp; 
}

void IRNullDisAndBackwardForce(int a, int b){
	/*stick is in the range of the null displacement and the external force is backward */
	/*so decidde what we should do here */
	int32 tmp;
	tmp = (int32)((30787.0 - gExternalForceState.value)* 0.0085);
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

const int controlIndexFuncMap[] = {
	0,//0
	2,//1
	1,//2
	0,//3
	0,//4
	0,//5
	0,//6
	0,//7
	0,//8
	0,//9
	0,//10
	0,//11
	3,//12
	3,//13
	3,//14
	0//15
};


const CONTROLSTATEMACHINE controlStateMahchineInterface[] = {
	IRNullDisAndNoForce,
	IRNullDisAndForwardForce,
	IRNullDisAndBackwardForce,
	OORThresholdDis,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0
};

void ControleStateMachineSwitch(int value){
	int mapValue = controlIndexFuncMap[value];

	if(mapValue < (sizeof(controlStateMahchineInterface) / sizeof(controlStateMahchineInterface[0]))){
		if(controlStateMahchineInterface[mapValue]){
			controlStateMahchineInterface[mapValue](0, 0);
		}
	}
	else{
		//TODO generate alarm msg, something wrong
	}
}
/******************************
 * 		bit6	0:									1: 	
 * 		bit5	0: Backward Threshold displacement in range								1: Backward Threshold displacement out of range 	
 * 		bit4	0: Forward Threshold displacement in range								1: Forward Threshold displacement out of range 	
 * 		bit3	0: Backward Null displacement in range									1: Backward Null displacement out of range 	
 * 		bit2	0: Forward Null displacement in ragne									1: Forward Null displacement out of range 	
 * 		bit1	0:									1: 	
 * 		bit0	0:									1: 	
 * 
 * 		bit1 and bit0 --> 0: No external force		1: Backward external force		2: Forward external force
 */

void checkNullDisBack(int value){

	switch (gStickState.NullDistanceBackwardState)
	{
	case INIT_NULL_DIS:
		if(gStickState.value >= OOR_BACKWARD_NULL_DIS_VAL){
			gStickState.NullDistanceBackwardState = OOR_NULL_DIS;
			gSysStateMachineNumber |= BIT_3;
		}
		else if(gStickState.value < OOR_BACKWARD_NULL_DIS_VAL){
			gStickState.NullDistanceBackwardState = IR_NULL_DIS;
			gSysStateMachineNumber &= ~BIT_3;
		}
		else{

		}
		break;

	case OOR_NULL_DIS:
		if(gStickState.value < IR_BACKWARD_NULL_DIS_VAL){
			gStickState.NullDistanceBackwardState = IR_NULL_DIS;
			gSysStateMachineNumber &= ~BIT_3;
		}
		break;

	case IR_NULL_DIS:
		if(gStickState.value > OOR_BACKWARD_NULL_DIS_VAL){
			gStickState.NullDistanceBackwardState = OOR_NULL_DIS;
			gSysStateMachineNumber |= BIT_3;
		}
		break;

	default:
		break;
	}
}

void checkNullDisFor(int value){

	switch (gStickState.NullDistanceForwardState)
	{
	case INIT_NULL_DIS:
		if(gStickState.value >= OOR_FORWARD_NULL_DIS_VAL){
			gStickState.NullDistanceForwardState = IR_NULL_DIS;
			gSysStateMachineNumber &= ~BIT_2;
		}
		else if(gStickState.value < OOR_FORWARD_NULL_DIS_VAL){
			gStickState.NullDistanceForwardState = OOR_NULL_DIS;
			gSysStateMachineNumber |= BIT_2;
		}
		else{

		}
		break;

	case OOR_NULL_DIS:
		if(gStickState.value > IR_FORWARD_NULL_DIS_VAL){
			gStickState.NullDistanceForwardState = IR_NULL_DIS;
			gSysStateMachineNumber &= ~BIT_2;
		}
		break;

	case IR_NULL_DIS:
		if(gStickState.value < OOR_FORWARD_NULL_DIS_VAL){
			gStickState.NullDistanceForwardState = OOR_NULL_DIS;
			gSysStateMachineNumber |= BIT_2;
		}
		break;
	
	default:
		break;
	}
}

void checkThresholdDisBack(int value){
	switch (gStickState.ThresholdBackwardState)
	{
	case INIT_THRESHOLD_DIS:
		if(gStickState.value >= OOR_BACKWARD_THRESHOLD_DIS_VAL){
			gStickState.ThresholdBackwardState = OOR_THRESHOLD_DIS;
			gSysStateMachineNumber |= BIT_5;
		}
		else if(gStickState.value < OOR_BACKWARD_THRESHOLD_DIS_VAL){
			gStickState.ThresholdBackwardState = IR_THRESHOLD_DIS;
			gSysStateMachineNumber &= ~BIT_5;
		}
		break;
	case OOR_THRESHOLD_DIS:
		if(gStickState.value < IR_BACKWARD_THRESHOLD_DIS_VAL){
			gStickState.ThresholdBackwardState = IR_THRESHOLD_DIS;
			gSysStateMachineNumber &= ~BIT_5;
		}
		break;

	case IR_THRESHOLD_DIS:
		if(gStickState.value > OOR_BACKWARD_THRESHOLD_DIS_VAL){
			gStickState.ThresholdBackwardState = OOR_THRESHOLD_DIS;
			gSysStateMachineNumber |= BIT_5;
		}
		break;
	default:
		break;
	}
}

void checkThresholdDisFor(int value){
	switch (gStickState.ThresholdForwaredState)
	{
	case INIT_THRESHOLD_DIS:
		if(gStickState.value >= OOR_FORWARD_THRESHOLD_DIS_VAL){
			gStickState.ThresholdForwaredState = IR_THRESHOLD_DIS;
			gSysStateMachineNumber &= ~BIT_4;
		}
		else if(gStickState.value < OOR_FORWARD_THRESHOLD_DIS_VAL){
			gStickState.ThresholdForwaredState = OOR_THRESHOLD_DIS;
			gSysStateMachineNumber |= BIT_4;
		}
		break;
	case OOR_THRESHOLD_DIS:
		if(gStickState.value > IR_FORWARD_THRESHOLD_DIS_VAL){
			gStickState.ThresholdForwaredState = IR_THRESHOLD_DIS;
			gSysStateMachineNumber &= ~BIT_4;
		}
		break;

	case IR_THRESHOLD_DIS:
		if(gStickState.value < OOR_FORWARD_THRESHOLD_DIS_VAL){
			gStickState.ThresholdForwaredState = OOR_THRESHOLD_DIS;
			gSysStateMachineNumber |= BIT_4;
		}
		break;
	default:
		break;
	}
}



void checkExternalForce(int value){
	/*need to decide if need to enable debouce feature */
	switch (gExternalForceState.ForceState)
	{
	case INIT_FORCE:

		if(gExternalForceState.value > BACKWARD_FORCE_VALUE){
			gExternalForceState.ForceState = BACKWARD_FORCE;
			gSysStateMachineNumber |= BIT_0;
			gSysStateMachineNumber &= ~BIT_1;
		}
		else if (gExternalForceState.value  < FORWARD_FORCE_VALUE){
			gExternalForceState.ForceState = FORWARD_FORCE;
			gSysStateMachineNumber &= ~BIT_0;
			gSysStateMachineNumber |= BIT_1;
		}
		else{
			gExternalForceState.ForceState = NO_FORCE;
			gSysStateMachineNumber &= ~BIT_0;
			gSysStateMachineNumber &= ~BIT_1;
		}
		break;
	case FORWARD_FORCE:
		if(gExternalForceState.value > BACKWARD_FORCE_VALUE){
			gExternalForceState.ForceState = BACKWARD_FORCE;
			gSysStateMachineNumber |= BIT_0;
			gSysStateMachineNumber &= ~BIT_1;
		}
		else if(gExternalForceState.value < FORWARD_FORCE_VALUE){
			gExternalForceState.ForceState = FORWARD_FORCE;
			gSysStateMachineNumber &= ~BIT_0;
			gSysStateMachineNumber |= BIT_1;
		}
		else{
			gExternalForceState.ForceState = NO_FORCE;
			gSysStateMachineNumber &= ~BIT_0;
			gSysStateMachineNumber &= ~BIT_1;
		}
		break;
	case BACKWARD_FORCE:
		if(gExternalForceState.value > BACKWARD_FORCE_VALUE){
			gExternalForceState.ForceState = BACKWARD_FORCE;
			gSysStateMachineNumber |= BIT_0;
			gSysStateMachineNumber &= ~BIT_1;
		}
		else if(gExternalForceState.value < FORWARD_FORCE_VALUE){
			gExternalForceState.ForceState = FORWARD_FORCE;
			gSysStateMachineNumber &= ~BIT_0;
			gSysStateMachineNumber |= BIT_1;
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
			gSysStateMachineNumber &= ~BIT_0;
			gSysStateMachineNumber |= BIT_1;
		}
		else{
			gExternalForceState.ForceState = NO_FORCE;
			gSysStateMachineNumber &= ~BIT_0;
			gSysStateMachineNumber &= ~BIT_1;
		}
		break;
	default:
		break;
	}
}

void InitStickState(void){
	gStickState.NullDistanceBackwardState = INIT_NULL_DIS;
	gStickState.NullDistanceForwardState  = INIT_NULL_DIS;

	gStickState.ThresholdBackwardState = INIT_THRESHOLD_DIS;
	gStickState.ThresholdForwaredState = INIT_THRESHOLD_DIS;

	gStickState.updateNullDisBackwardState = checkNullDisBack;
	gStickState.updateNullDisForwardState = checkNullDisFor;

	gStickState.updateThresholdDisBackwardState = checkThresholdDisBack;
	gStickState.updateThresholdDisForwardState = checkThresholdDisFor;

	gExternalForceState.ForceState = INIT_FORCE;
	gExternalForceState.updateForceState = checkExternalForce;

	gSysStateMachineNumber = 0;

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
	gConfigPara.LF_MaxForce = 0;
	gConfigPara.LF_Force1 = 0;
	gConfigPara.LF_Force2 = 0;
	gConfigPara.LF_Force3 = 0;
	gConfigPara.LF_Force4 = 0;
	gConfigPara.LF_Force5 = 0;
	gConfigPara.LF_Force6 = 0;
	gConfigPara.LF_Force7 = 0;
	gConfigPara.LF_Force8 = 0;
	gConfigPara.LF_Force9 = 0;

	gConfigPara.RB_Force1 = 0;
	gConfigPara.RB_Force2 = 0;
	gConfigPara.RB_Force3 = 0;
	gConfigPara.RB_Force4 = 0;
	gConfigPara.RB_Force5 = 0;
	gConfigPara.RB_Force6 = 0;
	gConfigPara.RB_Force7 = 0;
	gConfigPara.RB_Force8 = 0;
	gConfigPara.RB_Force9 = 0;
	gConfigPara.RB_MaxForce = 0;

	gConfigPara.LF_MaxDistance = 0;
	gConfigPara.LF_Distance1 = 0;
	gConfigPara.LF_Distance2 = 0;
	gConfigPara.LF_Distance3 = 0;
	gConfigPara.LF_Distance4 = 0;
	gConfigPara.LF_Distance5 = 0;
	gConfigPara.LF_Distance6 = 0;
	gConfigPara.LF_Distance7 = 0;
	gConfigPara.LF_Distance8 = 0;
	gConfigPara.LF_Distance9 = 0;

	gConfigPara.RB_Distance1 = 0;
	gConfigPara.RB_Distance2 = 0;
	gConfigPara.RB_Distance3 = 0;
	gConfigPara.RB_Distance4 = 0;
	gConfigPara.RB_Distance5 = 0;
	gConfigPara.RB_Distance6 = 0;
	gConfigPara.RB_Distance7 = 0;
	gConfigPara.RB_Distance8 = 0;
	gConfigPara.RB_Distance9 = 0;
	gConfigPara.RB_MaxDistance = 0;

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



