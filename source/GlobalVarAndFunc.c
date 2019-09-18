#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "GlobalVarAndFunc.h"
#include <string.h>
#include "PWM_ISR.h"
#include "PID.h"
#include "Ctl_Strategy.h"

#define  DEBOUNCE (0.10)

void InitStickState(void);
void checkRotateDirection(int value);
void checkAcceleration(int value);

Uint32 gECapCount = 0;
RS422STATUS gRS422Status = {0};
KeyValue gKeyValue = {0};
SYSINFO gSysInfo = {0};
SYSSTATE gSysState = {0};
SYSPARA gSysPara = {0};
SYSCURRENTSTATE gSysCurrentState = {0};
CONFIGPARA gConfigPara = {0};
FORCE_DISPLACE_CURVE gForceAndDisplaceCurve  = {0};
TENAVE gTenAverageArray = {0};
ANOLOG16BIT gAnalog16bit = {0};
STICKSTATE gStickState = {0};
EXTFORCESTATE gExternalForceState = {0};
ROTATEDIRECTION gRotateDirection = {0};
ACCELDIRECTION gAccelDirection = {0};
double gDebug[3] = {0};
int gPISO_165[8] = {0};
int gButtonCmd[6] = {0};

typedef void (*CONTROLSTATEMACHINE)(int a,int b);

void InitGlobalVarAndFunc(void){
    // PITCH
    if(checkPitchOrRoll() == PITCH){
        gSysInfo.DimL_K = -0.001559;
        gSysInfo.DimL_B = 59.6805;
        gSysInfo.TH0 = -19.2;
        gSysInfo.TH6 = 11.8;
    }
    //ROLL
    else if(checkPitchOrRoll() == ROLL){
        gSysInfo.DimL_K = -0.0017467;
        gSysInfo.DimL_B = 60.9135;
        gSysInfo.TH0 = -17.8;
        gSysInfo.TH6 = 17.8;
    }
    else{
        gSysInfo.DimL_K = 0;
        gSysInfo.DimL_B = 0;
        gSysInfo.TH0 = 0;
        gSysInfo.TH6 = 0;
        gSysState.warning.bit.b = 1;
    }
	gSysInfo.ddtmax = 1;
	gSysInfo.dutyAddInterval = 2;
	gSysInfo.targetDuty = 0;
	gSysInfo.targetDuty_F = 0;
	gSysInfo.targetDuty_V = 0;
	gSysInfo.coe_Force = 0.6;
	gSysInfo.coe_Velocity = 0.4;
	gSysInfo.controlFuncIndex = 0;
	gSysInfo.currentStickDisSection = INIT_SECTION;
	//gSysInfo.TH0 = -19.2; //-17.8
	gSysInfo.TH1 = -2.0;
	gSysInfo.TH2 = -1.5;
	gSysInfo.TH3 = 0.0;
	gSysInfo.TH4 = 1.5;
	gSysInfo.TH5 = 2.0;
	//gSysInfo.TH6 = 11.8; //17.8
	gSysInfo.Ki_Threshold_f = 6;
	gSysInfo.Ki_Threshold_v = 0.1;
	gSysInfo.velocity_last = 0;

	InitSysState();
	InitStickState();

	gRotateDirection.rotateDirection = INIT_DIRECTION;
	gRotateDirection.updateRotateDirection = checkRotateDirection;
	gAccelDirection.accelDirection = INIT_DIRECTION;
	gAccelDirection.updateAccelDirection = checkAcceleration;
    gAccelDirection.debounceCount_1 = 0;
    gAccelDirection.debounceCount_2 = 0;
	gRotateDirection.debounceCount_1 = 0;
	gRotateDirection.debounceCount_2 = 0;
	gSysInfo.Force_Init2Pos_Thr = 3;
	gSysInfo.Force_Init2Pos_Thr = -3;
	gSysInfo.Accel_Init2Pos_Thr = 0.1;
	gSysInfo.Accel_Init2Neg_Thr = -0.1;
	gSysInfo.Velocity_Init2Pos_Thr = 0.0025;
	gSysInfo.Velocity_Init2Neg_Thr = -0.0025;
	gSysInfo.Force_Pos_Thr = 3;
	gSysInfo.Force_Neg_Thr = -3;
	gSysInfo.Force_Hysteresis = 0.15;
	gSysInfo.Accel_Pos_Thr = 0.04;
    gSysInfo.Accel_Neg_Thr = -0.04;
    gSysInfo.Accel_Zero2Pos_Thr = 0.4;
    gSysInfo.Accel_Zero2Neg_Thr = -0.4;
    gSysInfo.Accel_Hysteresis = 0;
    gSysInfo.Accel_Debounce_Cnt_1 = 4;
    gSysInfo.Accel_Debounce_Cnt_2 = 8;
    gSysInfo.Velocity_Pos_Thr = 0.02;
    gSysInfo.Velocity_Neg_Thr = -0.02;
    gSysInfo.Velocity_Zero2Pos_Thr = 0.03;
    gSysInfo.Velocity_Zero2Neg_Thr = -0.03;
    gSysInfo.Velocity_Hysteresis = 0;
    gSysInfo.Velocity_Debounce_Cnt_1 = 10;
    gSysInfo.Velocity_Debounce_Cnt_2 = 10;
    gSysInfo.coe_Force_Max_ODE = 0;
    gSysInfo.coe_Force_Min_ODE = 0;
    gSysInfo.coe_Velocity_Max_ODE = 0;
    gSysInfo.coe_Velocity_Min_ODE = 0;
    gButtonCmd[1] = 0;
}

int checkPitchOrRoll(void){
    int board_type;
    // PITCH
    if((GpioDataRegs.GPBDAT.bit.GPIO61 == 1) && (GpioDataRegs.GPBDAT.bit.GPIO35 == 0)){
        board_type = PITCH;
    }
    //ROLL
    else if((GpioDataRegs.GPBDAT.bit.GPIO35 == 1) && (GpioDataRegs.GPBDAT.bit.GPIO61 == 0)){
        board_type = ROLL;
    }
    else{
        board_type = -1;
        gSysState.warning.bit.b = 1;
    }
    return board_type;
}

void IRNullDisAndNoForce(int a,  int b){
	/*stick is in the range of the null displacement and no external force on the it */
	/*so decide what we should do */
	int32 tmp;
	tmp = (int32)((-3 * gExternalForceState.value)* 250);
	tmp = -tmp;
	gSysInfo.targetDuty = tmp; 
	gSysInfo.targetDuty = 0;

} 

void IRNullDisAndForwardForce(int a, int b){
	/*stick is in the range of the null displacement and the external force is forward */
	/*so decidde what we should do here */
	int32 tmp;
	tmp = (int32)((gSysInfo.Force_Pos_Thr - gExternalForceState.value)* 250);
	tmp = -tmp;
	//tmp = tmp + 20;
	gSysInfo.targetDuty = tmp; 
	//gSysInfo.targetDuty = 100; 
}

void IRNullDisAndBackwardForce(int a, int b){
	/*stick is in the range of the null displacement and the external force is backward */
	/*so decidde what we should do here */
	int32 tmp;
	tmp = (int32)((gSysInfo.Force_Neg_Thr - gExternalForceState.value)* 250);
	tmp = -tmp;
	//tmp = tmp - 20;
	gSysInfo.targetDuty = tmp; 
	//gSysInfo.targetDuty = -100; 
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
    if(gExternalForceState.value > gConfigPara.LF_StartForce){
        IRNullDisAndForwardForce(0,0);
    }
    else{
#if(MACHINE_FRICTION == INCLUDE_FEATURE)
        tmp = -10;//if duty set to 0, you need 22N to push the stick move 
#elif
        tmp = displace_PidOutput(gSysInfo.TH4, gStickState.value);
#endif
        gSysInfo.targetDuty = tmp;
    }
}

void IRStartForceSecAndBackwardForce_sec2(int a, int b){
    /*stick is in the range of the null displacement and the external force is backward */
    /*so decidde what we should do here */
    int32 tmp;
    if(gExternalForceState.value < gConfigPara.RB_StartForce){
        IRNullDisAndBackwardForce(0,0);
    }
    else{
		tmp = 10;
        gSysInfo.targetDuty = tmp;
    }
}

void sec0_threshold_rear(int a, int b){
    /*stick is out of the range of the bakcward threshold displacement*/
	/*just output a force to let the stick go to zero state */
        gSysInfo.targetDuty = 80;
}

void sec1_ODE_rear(int a, int b){
    /*stick is out of the range of the bakcward threshold displacement*/
    /*so decidde what we should do here */
    //gSysInfo.sek = 0;
    //PidProcess();

#if(ONLY_SPRING == INCLUDE_FEATURE)
	OnlyWithSpringRear();
#else
    gSysInfo.targetDuty = 0;
#endif
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
#if(ONLY_SPRING == INCLUDE_FEATURE)
	OnlyWithSpringFront();
#else
    //PidProcess();
#endif
}
void sec7_threshold_front(int a, int b){
    /*stick is out of the range of the bakcward threshold displacement*/
	/*just output a force to let the stick go to zero state */
        gSysInfo.targetDuty = -80;
}

const CONTROLSTATEMACHINE controlStateMahchineInterface[] = {
    sec0_threshold_rear,              	//0:	Rear OOR
	sec1_ODE_rear, 					//1:	Rear ODE 
	sec2_StartForce_rear,              	//2:	Rear Start force
	sec3_Null_rear, 					//3:	Rear Null displacement
	sec4_Null_front,					//4:	Front Null displacement
	sec5_StartForce_front,              //5:	Front Start force
	sec6_ODE_front,					//6:	Front ODE
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

void checkRotateDirection(int value){
	switch(gRotateDirection.rotateDirection)
	{
		case INIT_DIRECTION:
			if(gKeyValue.motorSpeed > gSysInfo.Velocity_Init2Pos_Thr){
				gRotateDirection.rotateDirection = FORWARD_DIRECTION;
			}
			else if(gKeyValue.motorSpeed < gSysInfo.Velocity_Init2Neg_Thr){
				gRotateDirection.rotateDirection = BACKWARD_DIRECTION;
			}
			else{
				gRotateDirection.rotateDirection = STOP_DIRECTION;
			}
			break;
		case BACKWARD_DIRECTION:
			if(gKeyValue.motorSpeed > gSysInfo.Velocity_Neg_Thr){
			    gRotateDirection.debounceCount_1++;
			    gRotateDirection.debounceCount_2 = 0;
			    if(gRotateDirection.debounceCount_1 > gSysInfo.Velocity_Debounce_Cnt_1){
			        gRotateDirection.rotateDirection = STOP_DIRECTION;
			        gRotateDirection.debounceCount_1 = 0;
			    }
			}
            else{
                gRotateDirection.debounceCount_1 = 0;
                gRotateDirection.debounceCount_2 = 0;

            }
			break;
		case FORWARD_DIRECTION:
			if(gKeyValue.motorSpeed < gSysInfo.Velocity_Pos_Thr){
                gRotateDirection.debounceCount_1++;
                gRotateDirection.debounceCount_2 = 0;
                if(gRotateDirection.debounceCount_1 > gSysInfo.Velocity_Debounce_Cnt_1){
                    gRotateDirection.rotateDirection = STOP_DIRECTION;
                    gRotateDirection.debounceCount_1 = 0;
                }
			}
            else{
                gRotateDirection.debounceCount_1 = 0;
                gRotateDirection.debounceCount_2 = 0;

            }
			break;
		case STOP_DIRECTION:
			if(gKeyValue.motorSpeed > gSysInfo.Velocity_Init2Pos_Thr){
                gRotateDirection.debounceCount_1++;
                gRotateDirection.debounceCount_2 = 0;
                if(gRotateDirection.debounceCount_1 > gSysInfo.Velocity_Debounce_Cnt_2){
                    gRotateDirection.rotateDirection = FORWARD_DIRECTION;
                    gRotateDirection.debounceCount_1 = 0;
                }
			}
			else if(gKeyValue.motorSpeed < gSysInfo.Velocity_Init2Neg_Thr){
                gRotateDirection.debounceCount_2++;
                gRotateDirection.debounceCount_1 = 0;
                if(gRotateDirection.debounceCount_2 > gSysInfo.Velocity_Debounce_Cnt_2){
                    gRotateDirection.rotateDirection = BACKWARD_DIRECTION;
                    gRotateDirection.debounceCount_2 = 0;
                }
			}
			else{
				gRotateDirection.rotateDirection = STOP_DIRECTION;
				gRotateDirection.debounceCount_1 = 0;
				gRotateDirection.debounceCount_2 = 0;
			}

			break;
		default:
			break;
	}
}

void checkAcceleration(int value){
    switch(gAccelDirection.accelDirection)
    {
        case INIT_DIRECTION:
            if(gKeyValue.motorAccel > gSysInfo.Accel_Init2Pos_Thr){
                gAccelDirection.accelDirection = FORWARD_DIRECTION;
            }
            else if(gAccelDirection.accelDirection < gSysInfo.Accel_Init2Neg_Thr){
                gAccelDirection.accelDirection = BACKWARD_DIRECTION;
            }
            else{
                gAccelDirection.accelDirection = STOP_DIRECTION;
            }
            break;
        case BACKWARD_DIRECTION:
            if(gKeyValue.motorAccel > gSysInfo.Accel_Neg_Thr){
                gAccelDirection.debounceCount_1++;
                gAccelDirection.debounceCount_2 = 0;
                if(gAccelDirection.debounceCount_1 > gSysInfo.Accel_Debounce_Cnt_1){
                    gAccelDirection.accelDirection = STOP_DIRECTION;
                    gAccelDirection.debounceCount_1 = 0;
                }
            }
			else{
                gAccelDirection.debounceCount_1 = 0;
                gAccelDirection.debounceCount_2 = 0;

			}
            break;
        case FORWARD_DIRECTION:
            if(gKeyValue.motorAccel < gSysInfo.Accel_Pos_Thr){
                gAccelDirection.debounceCount_1++;
                gAccelDirection.debounceCount_2 = 0;
                if(gAccelDirection.debounceCount_1 > gSysInfo.Accel_Debounce_Cnt_1){
                    gAccelDirection.accelDirection = STOP_DIRECTION;
                    gAccelDirection.debounceCount_1 = 0;
                }
            }
			else{
                gAccelDirection.debounceCount_1 = 0;
                gAccelDirection.debounceCount_2 = 0;
			}
            break;
        case STOP_DIRECTION:
            if(gKeyValue.motorAccel > gSysInfo.Accel_Zero2Pos_Thr){
                gAccelDirection.debounceCount_1++;
                gAccelDirection.debounceCount_2 = 0;
                if(gAccelDirection.debounceCount_1 > gSysInfo.Accel_Debounce_Cnt_2){
                    gAccelDirection.accelDirection = FORWARD_DIRECTION;
                    gAccelDirection.debounceCount_1 = 0;
                }
            }
            else if(gKeyValue.motorAccel < gSysInfo.Accel_Zero2Neg_Thr){
                gAccelDirection.debounceCount_2++;
                gAccelDirection.debounceCount_1 = 0;
                if(gAccelDirection.debounceCount_2 > gSysInfo.Accel_Debounce_Cnt_2){
                    gAccelDirection.accelDirection = BACKWARD_DIRECTION;
                    gAccelDirection.debounceCount_2 = 0;
                }
            }
            else{
                gAccelDirection.accelDirection = STOP_DIRECTION;
                gAccelDirection.debounceCount_1 = 0;
                gAccelDirection.debounceCount_2 = 0;
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

		if(gExternalForceState.value < gSysInfo.Force_Init2Neg_Thr){
			gExternalForceState.ForceState = BACKWARD_FORCE;
		}
		else if (gExternalForceState.value  > gSysInfo.Force_Init2Pos_Thr){
			gExternalForceState.ForceState = FORWARD_FORCE;
		}
		else{
			gExternalForceState.ForceState = NO_FORCE;
		}
		break;
	case FORWARD_FORCE:
		if(gExternalForceState.value > (gSysInfo.Force_Pos_Thr - gSysInfo.Force_Hysteresis)){
			gExternalForceState.ForceState = FORWARD_FORCE;
		}
		else if(gExternalForceState.value < gSysInfo.Force_Neg_Thr - gSysInfo.Force_Hysteresis){
			gExternalForceState.ForceState = BACKWARD_FORCE;
		}
		else{
			gExternalForceState.ForceState = NO_FORCE;
		}
		break;
	case BACKWARD_FORCE:
		if(gExternalForceState.value < (gSysInfo.Force_Neg_Thr + gSysInfo.Force_Hysteresis)){
			gExternalForceState.ForceState = BACKWARD_FORCE;
		}
		else if(gExternalForceState.value > (gSysInfo.Force_Pos_Thr + gSysInfo.Force_Hysteresis)){
			gExternalForceState.ForceState = FORWARD_FORCE;
		}
		else{
			gExternalForceState.ForceState = NO_FORCE;
		}
		break;
	case NO_FORCE:
		if(gExternalForceState.value < gSysInfo.Force_Neg_Thr - gSysInfo.Force_Hysteresis){
			gExternalForceState.ForceState = BACKWARD_FORCE;
		}
		else if(gExternalForceState.value > gSysInfo.Force_Pos_Thr + gSysInfo.Force_Hysteresis){
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

	gForceAndDisplaceCurve.springForceP[0] = gConfigPara.LF_StartForce;
	gForceAndDisplaceCurve.springForceP[1] = gConfigPara.LF_Force2;
	gForceAndDisplaceCurve.springForceP[2] = gConfigPara.LF_Force3;
	gForceAndDisplaceCurve.springForceP[3] = gConfigPara.LF_Force4;
	gForceAndDisplaceCurve.springForceP[4] = gConfigPara.LF_Force5;
	gForceAndDisplaceCurve.springForceP[5] = gConfigPara.LF_Force6;
	gForceAndDisplaceCurve.springForceP[6] = gConfigPara.LF_Force7;
	gForceAndDisplaceCurve.springForceP[7] = gConfigPara.LF_Force8;
	gForceAndDisplaceCurve.springForceP[8] = gConfigPara.LF_Force9;
	gForceAndDisplaceCurve.springForceP[9] = gConfigPara.LF_MaxForce;

	gForceAndDisplaceCurve.springForceN[0] = gConfigPara.RB_StartForce;
	gForceAndDisplaceCurve.springForceN[1] = gConfigPara.RB_Force2;
	gForceAndDisplaceCurve.springForceN[2] = gConfigPara.RB_Force3;
	gForceAndDisplaceCurve.springForceN[3] = gConfigPara.RB_Force4;
	gForceAndDisplaceCurve.springForceN[4] = gConfigPara.RB_Force5;
	gForceAndDisplaceCurve.springForceN[5] = gConfigPara.RB_Force6;
	gForceAndDisplaceCurve.springForceN[6] = gConfigPara.RB_Force7;
	gForceAndDisplaceCurve.springForceN[7] = gConfigPara.RB_Force8;
	gForceAndDisplaceCurve.springForceN[8] = gConfigPara.RB_Force9;
	gForceAndDisplaceCurve.springForceN[9] = gConfigPara.RB_MaxForce;

	gForceAndDisplaceCurve.displacementP[0] = gConfigPara.LF_EmptyDistance;
	gForceAndDisplaceCurve.displacementP[1] = gConfigPara.LF_Distance2;
	gForceAndDisplaceCurve.displacementP[2] = gConfigPara.LF_Distance3;
	gForceAndDisplaceCurve.displacementP[3] = gConfigPara.LF_Distance4;
	gForceAndDisplaceCurve.displacementP[4] = gConfigPara.LF_Distance5;
	gForceAndDisplaceCurve.displacementP[5] = gConfigPara.LF_Distance6;
	gForceAndDisplaceCurve.displacementP[6] = gConfigPara.LF_Distance7;
	gForceAndDisplaceCurve.displacementP[7] = gConfigPara.LF_Distance8;
	gForceAndDisplaceCurve.displacementP[8] = gConfigPara.LF_Distance9;
	gForceAndDisplaceCurve.displacementP[9] = gConfigPara.LF_MaxDistance;


	gForceAndDisplaceCurve.displacementN[0] = gConfigPara.RB_EmptyDistance;
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

	gConfigPara.LF_Distance1 = 1.5;
	gConfigPara.LF_Distance2 = 3;
	gConfigPara.LF_Distance3 = 4;
	gConfigPara.LF_Distance4 = 5;
	gConfigPara.LF_Distance5 = 6;
	gConfigPara.LF_Distance6 = 7;
	gConfigPara.LF_Distance7 = 8;
	gConfigPara.LF_Distance8 = 9;
	gConfigPara.LF_Distance9 = 9.5;
	gConfigPara.LF_MaxDistance = 12;

	gConfigPara.RB_Distance1 = -1.5;
	gConfigPara.RB_Distance2 = -3;
	gConfigPara.RB_Distance3 = -4;
	gConfigPara.RB_Distance4 = -5;
	gConfigPara.RB_Distance5 = -6;
	gConfigPara.RB_Distance6 = -7;
	gConfigPara.RB_Distance7 = -7.5;
	gConfigPara.RB_Distance8 = -8;
	gConfigPara.RB_Distance9 = -8.5;
	gConfigPara.RB_MaxDistance = -20;

	gConfigPara.LF_StartForce = 5;
	gConfigPara.RB_StartForce = -5;

	gConfigPara.LF_FrontFriction = 3;
	gConfigPara.LF_RearFriction = 3;
	gConfigPara.RB_FrontFriction = 3;
	gConfigPara.RB_RearFriction = 3;

	gConfigPara.LF_EmptyDistance = 0;
	gConfigPara.RB_EmptyDistance = 0;

	gConfigPara.dampingFactor = 0.5;

	gConfigPara.naturalVibrationFreq = 20.0;

	gConfigPara.equivalentMass = 0;

	gConfigPara.LF_TrimRange = 0;
	gConfigPara.RB_TrimRange = 0;

	gConfigPara.Trim_StepSize = 0;

	gConfigPara.Trim_Speed = 0;

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
	gSysPara.mass = 1;

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
double KalmanFilterForce(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R)
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

double KalmanFilterAccel(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R)
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
	else{
		return 7;
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
	    gSysInfo.sek_v = 0;
	    gSysInfo.velocity_last = 0;
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
	    gSysInfo.sek_v = 0;
	    gSysInfo.velocity_last = 0;
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
	    gSysInfo.sek_v = 0;
	    gSysInfo.velocity_last = 0;
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
	    gSysInfo.sek_v = 0;
	    gSysInfo.velocity_last = 0;
		if(gStickState.value < (gSysInfo.TH6 - DEBOUNCE)){
			gSysInfo.currentStickDisSection = CheckStickSetion(gStickState.value);
		}
		break;
	
	default:
		break;
	}
	return gSysInfo.currentStickDisSection;
}

double TenDisplaceElemntAverage(void){
	double ret = 0;
	int i;
	double sum = 0;

	double k[10];


	for(i = 0; i < 10; ++i){
		k[i] = (gTenAverageArray.displaceArray[i] - gTenAverageArray.displaceArrayBak[i]) / 0.25;
		sum += k[i];
	}

	ret = sum / 10;
	if(ret < 0.02684 && ret > -0.02684){
		ret = 0;
	}

	for(i = 0; i < 10; ++i){
		gTenAverageArray.displaceArrayBak[i] = gTenAverageArray.displaceArray[i];
	}
	return ret;
}

void DigitalSignalPISO(void){
    int i = 0;

    //GpioDataRegs.GPBDAT.bit.GPIO53 = 1;
    GpioDataRegs.GPBDAT.bit.GPIO53 = 0;
    asm(" NOP");
    GpioDataRegs.GPBDAT.bit.GPIO53 = 1;

    for(i=0; i<8; i++){
        GpioDataRegs.GPBDAT.bit.GPIO52 = 1;
        asm(" NOP");
        GpioDataRegs.GPBDAT.bit.GPIO52 = 0;
        asm(" NOP");

        if(GpioDataRegs.GPBDAT.bit.GPIO59 == 1){
            //gPISO_165[i] |= (0x01<<(7-i));
            gPISO_165[i] = 1;
        }
        else{
            //gPISO_165[i] &= ~(0x01<<(7-i));
            gPISO_165[i] = 0;
        }
    }
}

void Button_Debounce(void){
   // int j = 1;
    static int count1 = 0;
    static int count2 = 0;
    static int count3 = 0;
    static int count4 = 0;
    static int count5 = 0;
    static int count6 = 0;

    if(gPISO_165[1] == 1){
        count1 ++;
    }
    else{
        count1 = 0;
    }
    if(count1 > 500){
        gButtonCmd[TK9_TRIGGER] += 1;
        count1 = 0;
    }
    else{
        //gButtonCmd[TK9_TRIGGER] = 0;
    }

    if(gPISO_165[2] == 1){
        count2 ++;
    }
    else{
        if(count2 > 700){
            gButtonCmd[AK29_BUTTON] += 1;
            count2 = 0;
        }
        else{
            //gButtonCmd[AK29_BUTTON] = gButtonCmd[AK29_BUTTON];
           // gButtonCmd[AK29_BUTTON] = 0;
        }
        count2 = 0;
    }

    if(gPISO_165[3] == 1){
        count3 ++;
    }
    else{
        count3 = 0;
    }
    if(count3 == 700){
        gButtonCmd[FWRD_SWITCH] += 1;
        count3 = 0;
    }
    else{
        gButtonCmd[FWRD_SWITCH] = gButtonCmd[FWRD_SWITCH];
        //gButtonCmd[FWRD_SWITCH] = 0;
    }

    if(gPISO_165[4] == 1){
        count4 ++;
    }
    else{
        count4 = 0;
    }
    if(count4 > 300){
        gButtonCmd[RGHT_SWITCH] += 1;
        count4 = 0;
    }
    else{
       // gButtonCmd[RGHT_SWITCH] = 0;
    }

    if(gPISO_165[5] == 1){
        count5 ++;
    }
    else{
        count5 = 0;
    }
    if(count5 > 300){
        gButtonCmd[REAR_SWITCH] += 1;
        count5 = 0;
    }
    else{
        //gButtonCmd[REAR_SWITCH] = 0;
    }

    if(gPISO_165[6] == 1){
        count6 ++;
    }
    else{
        count6 = 0;
    }
    if(count6 > 300){
        gButtonCmd[LEFT_SWITCH] += 1;
        count6 = 0;
    }
    else{
        //gButtonCmd[LEFT_SWITCH] = 0;
    }
}
