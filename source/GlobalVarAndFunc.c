#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "GlobalVarAndFunc.h"
#include <string.h>
#include "PWM_ISR.h"
#include "PID.h"
#include "Ctl_Strategy.h"

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
int gButtonStatus[6] = {0};


void InitGlobalVarAndFunc(void){
    // PITCH
//    checkPitchOrRoll();
    if(gSysInfo.board_type == PITCH){
        gSysInfo.DimL_K = -0.001683;
        gSysInfo.DimL_B = 63.2728;
        gSysInfo.Force_K = 0.014027;
        gSysInfo.Force_B = -459.6276;
        gSysInfo.openLoop_Force_front_B = 0;
        gSysInfo.openLoop_Force_rear_B = 0;
    }
    //ROLL
    else if(gSysInfo.board_type == ROLL){
        gSysInfo.DimL_K = -0.0017467;
        gSysInfo.DimL_B = 57.9135;
        gSysInfo.Force_K = -0.014027;
        gSysInfo.Force_B = 459.6276;
        gSysInfo.openLoop_Force_front_B = 5;
        gSysInfo.openLoop_Force_rear_B = -5;
    }
    else{
        gSysInfo.DimL_K = 0;
        gSysInfo.DimL_B = 0;
        gSysState.warning.bit.b = 1;
        gSysInfo.openLoop_Force_front_B = 0;
        gSysInfo.openLoop_Force_rear_B = 0;
    }
    gSysInfo.sek_v = 0;
    gSysInfo.sek_f = 0;
    gSysInfo.sek_d = 0;
	gSysInfo.ddtmax = 5;
	gSysInfo.dutyAddInterval = 2;
	gSysInfo.targetDuty = 0;
	gSysInfo.currentDuty = 0;
	gSysInfo.targetDuty_F = 0;
	gSysInfo.targetDuty_V = 0;
	gSysInfo.coe_Force = 1;
	gSysInfo.coe_Velocity = 0;
	gSysInfo.controlFuncIndex = 0;
	gSysInfo.currentStickDisSection = INIT_SECTION;
	gSysInfo.Ki_Threshold_f = 5;
	gSysInfo.Ki_Threshold_d = 0.0725;
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
	gSysInfo.Force_Init2Neg_Thr = -3;
	gSysInfo.Accel_Init2Pos_Thr = 0.1;
	gSysInfo.Accel_Init2Neg_Thr = -0.1;
	gSysInfo.Velocity_Init2Pos_Thr = 0.0001;
	gSysInfo.Velocity_Init2Neg_Thr = -0.0001;
	gSysInfo.Force_Pos_Thr = 0.9;
	gSysInfo.Force_Neg_Thr = -0.9;
	gSysInfo.Force_Hysteresis = 0.15;
	gSysInfo.Accel_Pos_Thr = 0.04;
    gSysInfo.Accel_Neg_Thr = -0.04;
    gSysInfo.Accel_Zero2Pos_Thr = 0.4;
    gSysInfo.Accel_Zero2Neg_Thr = -0.4;
    gSysInfo.Accel_Hysteresis = 0;
    gSysInfo.Accel_Debounce_Cnt_1 = 4;
    gSysInfo.Accel_Debounce_Cnt_2 = 8;
    gSysInfo.Velocity_Pos_Thr = 0.0001;
    gSysInfo.Velocity_Neg_Thr = -0.0001;
    gSysInfo.Velocity_Zero2Pos_Thr = 0.03;
    gSysInfo.Velocity_Zero2Neg_Thr = -0.03;
    gSysInfo.Velocity_Hysteresis = 0;
    gSysInfo.Velocity_Debounce_Cnt_1 = 10;
    gSysInfo.Velocity_Debounce_Cnt_2 = 10;
    gSysInfo.coe_Force_Max_ODE = 0;
    gSysInfo.coe_Force_Min_ODE = 0;
    gSysInfo.coe_Velocity_Max_ODE = 0;
    gSysInfo.coe_Velocity_Min_ODE = 0;
    gButtonStatus[0] = 2;
    gButtonStatus[1] = 2;
    gButtonStatus[2] = 2;
    gButtonStatus[3] = 2;
    gButtonStatus[4] = 2;
    gButtonStatus[5] = 2;
    gButtonCmd[0] = 0;
    gButtonCmd[1] = 0;
    gButtonCmd[2] = 0;
    gButtonCmd[3] = 0;
    gButtonCmd[4] = 0;
    gButtonCmd[5] = 0;
    gSysInfo.ob_velocityOpenLoop = 0;
    gSysInfo.ob_velocityOpenLoop2 = 0;
    gKeyValue.motorAccel = 0;
    gKeyValue.motorSpeed = 0;
    gKeyValue.displacement = 0;
    gKeyValue.force = 0;
    gKeyValue.lock = 0;


    gSysInfo.friction = 0;
    gSysInfo.soft_break_flag = 0;
    gSysInfo.springForceK = 0;
    gSysInfo.springForceB = 0;
    gSysInfo.zeroForce = 0;

    gSysInfo.isEcapRefresh = 0;
    gSysInfo.JoyStickSpeed = 0;
    gSysInfo.rotateDirection = 0;

    gSysInfo.sixButtons = 0;
    gSysInfo.RS422_Rx_Data = 0;
    gSysInfo.software_version = 7;
}

void checkPitchOrRoll(void){

    // PITCH
    if((GpioDataRegs.GPBDAT.bit.GPIO61 == 1) && (GpioDataRegs.GPBDAT.bit.GPIO35 == 0)){
        gSysInfo.board_type = PITCH;
    }
    //ROLL
    else if((GpioDataRegs.GPBDAT.bit.GPIO35 == 1) && (GpioDataRegs.GPBDAT.bit.GPIO61 == 0)){
        gSysInfo.board_type = ROLL;
    }
    else{
        gSysInfo.board_type = -1;
        gSysState.warning.bit.b = 1;
    }
}

void checkRotateDirection(int value){
    static int last_state = 2;
	switch(gRotateDirection.rotateDirection)
	{
		case INIT_DIRECTION:
			if(gKeyValue.motorSpeed > gSysInfo.Velocity_Init2Pos_Thr){
				gRotateDirection.rotateDirection = FORWARD_DIRECTION;
				gSysInfo.friction = - gConfigPara.LF_FrontFriction;
				last_state = 1;
			}
			else if(gKeyValue.motorSpeed < gSysInfo.Velocity_Init2Neg_Thr){
				gRotateDirection.rotateDirection = BACKWARD_DIRECTION;
				gSysInfo.friction = gConfigPara.LF_FrontFriction;
				last_state = 0;
			}
			else{
				gRotateDirection.rotateDirection = STOP_DIRECTION;
				gSysInfo.friction = 0;
				last_state = 2;
			}
			break;
		case BACKWARD_DIRECTION:
			if(gKeyValue.motorSpeed > gSysInfo.Velocity_Neg_Thr){
			    gRotateDirection.debounceCount_1++;
			    gRotateDirection.debounceCount_2 = 0;
			    if(gRotateDirection.debounceCount_1 > gSysInfo.Velocity_Debounce_Cnt_1){
			        gRotateDirection.rotateDirection = STOP_DIRECTION;
			        gRotateDirection.debounceCount_1 = 0;
                    gSysInfo.friction = gConfigPara.LF_FrontFriction;
                    last_state = 0;
			    }
			}
            else{
                gRotateDirection.debounceCount_1 = 0;
                gRotateDirection.debounceCount_2 = 0;
                gSysInfo.friction = gConfigPara.LF_FrontFriction;
                last_state = 0;

            }
			break;
		case FORWARD_DIRECTION:
			if(gKeyValue.motorSpeed < gSysInfo.Velocity_Pos_Thr){
                gRotateDirection.debounceCount_1++;
                gRotateDirection.debounceCount_2 = 0;
                if(gRotateDirection.debounceCount_1 > gSysInfo.Velocity_Debounce_Cnt_1){
                    gRotateDirection.rotateDirection = STOP_DIRECTION;
                    gRotateDirection.debounceCount_1 = 0;
                    gSysInfo.friction = - gConfigPara.LF_FrontFriction;
                    last_state = 1;
                }
			}
            else{
                gRotateDirection.debounceCount_1 = 0;
                gRotateDirection.debounceCount_2 = 0;
                gSysInfo.friction = - gConfigPara.LF_FrontFriction;
                last_state = 1;
            }
			break;
		case STOP_DIRECTION:
			if(gKeyValue.motorSpeed > gSysInfo.Velocity_Init2Pos_Thr){
                gRotateDirection.debounceCount_1++;
                gRotateDirection.debounceCount_2 = 0;
                if(gRotateDirection.debounceCount_1 > gSysInfo.Velocity_Debounce_Cnt_2){
                    gRotateDirection.rotateDirection = FORWARD_DIRECTION;
                    gRotateDirection.debounceCount_1 = 0;
                    gSysInfo.friction = - gConfigPara.LF_FrontFriction;
                    last_state = 1;
                }
			}
			else if(gKeyValue.motorSpeed < gSysInfo.Velocity_Init2Neg_Thr){
                gRotateDirection.debounceCount_2++;
                gRotateDirection.debounceCount_1 = 0;
                if(gRotateDirection.debounceCount_2 > gSysInfo.Velocity_Debounce_Cnt_2){
                    gRotateDirection.rotateDirection = BACKWARD_DIRECTION;
                    gRotateDirection.debounceCount_2 = 0;
                    gSysInfo.friction = gConfigPara.LF_FrontFriction;
                    last_state = 0;
                }
			}
			else{
				gRotateDirection.rotateDirection = STOP_DIRECTION;
				gRotateDirection.debounceCount_1 = 0;
				gRotateDirection.debounceCount_2 = 0;
				if(last_state == 1){
				    gSysInfo.friction = - gConfigPara.LF_FrontFriction;
				}
				else if(last_state == 0){
				    gSysInfo.friction = gConfigPara.LF_FrontFriction;
				}
				else{
				    gSysInfo.friction = 0;
				}
			}

			break;
		default:
			break;
	}
}
#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(checkAcceleration, "ramfuncs")
#endif
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
#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(checkExternalForce, "ramfuncs")
#endif
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

void UpdateForceDisplaceCurve(void){
	int index;
    gForceAndDisplaceCurve.springForceP[0] = gConfigPara.LF_Force0;
    gForceAndDisplaceCurve.springForceP[1] = gConfigPara.LF_Force1;
	gForceAndDisplaceCurve.springForceP[2] = gConfigPara.LF_StartForce;
	gForceAndDisplaceCurve.springForceP[3] = gConfigPara.LF_Force2;
	gForceAndDisplaceCurve.springForceP[4] = gConfigPara.LF_Force3;
	gForceAndDisplaceCurve.springForceP[5] = gConfigPara.LF_Force4;
	gForceAndDisplaceCurve.springForceP[6] = gConfigPara.LF_Force5;
	gForceAndDisplaceCurve.springForceP[7] = gConfigPara.LF_Force6;
	gForceAndDisplaceCurve.springForceP[8] = gConfigPara.LF_Force7;
	gForceAndDisplaceCurve.springForceP[9] = gConfigPara.LF_Force8;
	gForceAndDisplaceCurve.springForceP[10] = gConfigPara.LF_Force9;
	gForceAndDisplaceCurve.springForceP[11] = gConfigPara.LF_MaxForce;

    gForceAndDisplaceCurve.springForceN[0] = gConfigPara.RB_Force0;
    gForceAndDisplaceCurve.springForceN[1] = gConfigPara.RB_Force1;
	gForceAndDisplaceCurve.springForceN[2] = gConfigPara.RB_StartForce;
	gForceAndDisplaceCurve.springForceN[3] = gConfigPara.RB_Force2;
	gForceAndDisplaceCurve.springForceN[4] = gConfigPara.RB_Force3;
	gForceAndDisplaceCurve.springForceN[5] = gConfigPara.RB_Force4;
	gForceAndDisplaceCurve.springForceN[6] = gConfigPara.RB_Force5;
	gForceAndDisplaceCurve.springForceN[7] = gConfigPara.RB_Force6;
	gForceAndDisplaceCurve.springForceN[8] = gConfigPara.RB_Force7;
	gForceAndDisplaceCurve.springForceN[9] = gConfigPara.RB_Force8;
	gForceAndDisplaceCurve.springForceN[10] = gConfigPara.RB_Force9;
	gForceAndDisplaceCurve.springForceN[11] = gConfigPara.RB_MaxForce;

    gForceAndDisplaceCurve.displacementP[0] = gConfigPara.LF_Distance0;
    gForceAndDisplaceCurve.displacementP[1] = gConfigPara.LF_Distance1;
	gForceAndDisplaceCurve.displacementP[2] = gConfigPara.LF_EmptyDistance;
	gForceAndDisplaceCurve.displacementP[3] = gConfigPara.LF_Distance2;
	gForceAndDisplaceCurve.displacementP[4] = gConfigPara.LF_Distance3;
	gForceAndDisplaceCurve.displacementP[5] = gConfigPara.LF_Distance4;
	gForceAndDisplaceCurve.displacementP[6] = gConfigPara.LF_Distance5;
	gForceAndDisplaceCurve.displacementP[7] = gConfigPara.LF_Distance6;
	gForceAndDisplaceCurve.displacementP[8] = gConfigPara.LF_Distance7;
	gForceAndDisplaceCurve.displacementP[9] = gConfigPara.LF_Distance8;
	gForceAndDisplaceCurve.displacementP[10] = gConfigPara.LF_Distance9;
	gForceAndDisplaceCurve.displacementP[11] = gConfigPara.LF_MaxDistance;

    gForceAndDisplaceCurve.displacementN[0] = gConfigPara.RB_Distance0;
    gForceAndDisplaceCurve.displacementN[1] = gConfigPara.RB_Distance1;
	gForceAndDisplaceCurve.displacementN[2] = gConfigPara.RB_EmptyDistance;
	gForceAndDisplaceCurve.displacementN[3] = gConfigPara.RB_Distance2;
	gForceAndDisplaceCurve.displacementN[4] = gConfigPara.RB_Distance3;
	gForceAndDisplaceCurve.displacementN[5] = gConfigPara.RB_Distance4;
	gForceAndDisplaceCurve.displacementN[6] = gConfigPara.RB_Distance5;
	gForceAndDisplaceCurve.displacementN[7] = gConfigPara.RB_Distance6;
	gForceAndDisplaceCurve.displacementN[8] = gConfigPara.RB_Distance7;
	gForceAndDisplaceCurve.displacementN[9] = gConfigPara.RB_Distance8;
	gForceAndDisplaceCurve.displacementN[10] = gConfigPara.RB_Distance9;
	gForceAndDisplaceCurve.displacementN[11] = gConfigPara.RB_MaxDistance;

	gForceAndDisplaceCurve.maxPoints = 12;

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
    if(gSysInfo.board_type == PITCH){
        gConfigPara.LF_Force0 = 0;
        gConfigPara.LF_Force1 = 0;
        gConfigPara.LF_StartForce = 5;
        gConfigPara.LF_Force2 = 9.4348;
        gConfigPara.LF_Force3 = 12.3913;
        gConfigPara.LF_Force4 = 15.3478;
        gConfigPara.LF_Force5 = 18.3044;
        gConfigPara.LF_Force6 = 21.2609;
        gConfigPara.LF_Force7 = 24.2174;
        gConfigPara.LF_Force8 = 27.1739;
        gConfigPara.LF_Force9 = 33.0870;
        gConfigPara.LF_MaxForce = 39;

        gConfigPara.RB_Force0 = 0;
        gConfigPara.RB_Force1 = 0;
        gConfigPara.RB_StartForce = -5;
        gConfigPara.RB_Force2 = -9.6154;
        gConfigPara.RB_Force3 = -12.6923;
        gConfigPara.RB_Force4 = -18.8462;
        gConfigPara.RB_Force5 = -25;
        gConfigPara.RB_Force6 = -31.1539;
        gConfigPara.RB_Force7 = -37.3077;
        gConfigPara.RB_Force8 = -43.4615;
        gConfigPara.RB_Force9 = -49.6154;
        gConfigPara.RB_MaxForce = -65;

        gConfigPara.LF_Distance0 = 0;
        gConfigPara.LF_Distance1 = 0.25;
        gConfigPara.LF_EmptyDistance = 0.5;
        gConfigPara.LF_Distance2 = 2;
        gConfigPara.LF_Distance3 = 3;
        gConfigPara.LF_Distance4 = 4;
        gConfigPara.LF_Distance5 = 5;
        gConfigPara.LF_Distance6 = 6;
        gConfigPara.LF_Distance7 = 7;
        gConfigPara.LF_Distance8 = 8;
        gConfigPara.LF_Distance9 = 10;
        gConfigPara.LF_MaxDistance = 12;

        gConfigPara.RB_Distance0 = 0;
        gConfigPara.RB_Distance1 = -0.25;
        gConfigPara.RB_EmptyDistance = -0.5;
        gConfigPara.RB_Distance2 = -2;
        gConfigPara.RB_Distance3 = -3;
        gConfigPara.RB_Distance4 = -5;
        gConfigPara.RB_Distance5 = -7;
        gConfigPara.RB_Distance6 = -9;
        gConfigPara.RB_Distance7 = -11;
        gConfigPara.RB_Distance8 = -13;
        gConfigPara.RB_Distance9 = -15;
        gConfigPara.RB_MaxDistance =-20;
     }
     //ROLL
     else if(gSysInfo.board_type == ROLL){
         gConfigPara.LF_Force0 = 0;
         gConfigPara.LF_Force1 = 0;
         gConfigPara.LF_StartForce = 5;
         gConfigPara.LF_Force2 = 6.143;
         gConfigPara.LF_Force3 = 8.429;
         gConfigPara.LF_Force4 = 13;
         gConfigPara.LF_Force5 = 17.571;
         gConfigPara.LF_Force6 = 22.143;
         gConfigPara.LF_Force7 = 26.714;
         gConfigPara.LF_Force8 = 35.857;
         gConfigPara.LF_Force9 = 40.429;
         gConfigPara.LF_MaxForce = 45;

         gConfigPara.RB_Force0 = 0;
         gConfigPara.RB_Force1 = 0;
         gConfigPara.RB_StartForce = -5;
         gConfigPara.RB_Force2 = -6.143;
         gConfigPara.RB_Force3 = -8.429;
         gConfigPara.RB_Force4 = -13;
         gConfigPara.RB_Force5 = -17.571;
         gConfigPara.RB_Force6 = -22.143;
         gConfigPara.RB_Force7 = -26.714;
         gConfigPara.RB_Force8 = -35.857;
         gConfigPara.RB_Force9 = -40.429;
         gConfigPara.RB_MaxForce = -45;

         gConfigPara.LF_Distance0 = 0;
         gConfigPara.LF_Distance1 = 0.25;
         gConfigPara.LF_EmptyDistance = 0.5;
         gConfigPara.LF_Distance2 = 1;
         gConfigPara.LF_Distance3 = 2;
         gConfigPara.LF_Distance4 = 4;
         gConfigPara.LF_Distance5 = 6;
         gConfigPara.LF_Distance6 = 8;
         gConfigPara.LF_Distance7 = 10;
         gConfigPara.LF_Distance8 = 14;
         gConfigPara.LF_Distance9 = 16;
         gConfigPara.LF_MaxDistance = 18;

         gConfigPara.RB_Distance0 = 0;
         gConfigPara.RB_Distance1 = -0.25;
         gConfigPara.RB_EmptyDistance = -0.5;
         gConfigPara.RB_Distance2 = -1;
         gConfigPara.RB_Distance3 = -2;
         gConfigPara.RB_Distance4 = -4;
         gConfigPara.RB_Distance5 = -6;
         gConfigPara.RB_Distance6 = -8;
         gConfigPara.RB_Distance7 = -10;
         gConfigPara.RB_Distance8 = -14;
         gConfigPara.RB_Distance9 = -16;
         gConfigPara.RB_MaxDistance = -18;

     }
     else{
         gSysState.warning.bit.b = 1;
     }

	gConfigPara.LF_FrontFriction = 1;
	gConfigPara.LF_RearFriction = 1;
	gConfigPara.RB_FrontFriction = 1;
	gConfigPara.RB_RearFriction = 1;

	gConfigPara.dampingFactor = 0.1;

	gConfigPara.naturalVibrationFreq = 80.0;

	gConfigPara.equivalentMass = 0;

	gConfigPara.LF_TrimRange = 0;
	gConfigPara.RB_TrimRange = 0;

	gConfigPara.Trim_StepSize = 0;

	gConfigPara.Trim_Speed = 1; //unit mm/s

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

double KalmanFilterRodSpeed(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R)
{
    static int isFirstTimeExcuted = 1;

    double R = MeasureNoise_R;
    double Q = ProcessNiose_Q;

    static double x_last = 0;
    double x_mid = x_last;
    double x_now;

    static double p_last = 0;
    double p_mid;
    double p_now;

    double kg;


    if(isFirstTimeExcuted){
        isFirstTimeExcuted = 0;
        x_last = ResrcData;
        p_last = ResrcData;
        return ResrcData;
    }

    x_mid = x_last;
    p_mid = p_last + Q;


    kg = p_mid / (p_mid + R);
    x_now = x_mid + kg * (ResrcData - x_mid);
    p_now = (1 - kg) * p_mid;
    p_last = p_now;
    x_last = x_now;

    return x_now;
}


/**************************************************************
 *Name:		   KalmanFilter
 *Comment:
 *Input:	   const double, double, double
 *Output:	   double
 *Author:	   Simon
 *Date:		   2019��1��2������9:57:12
 **************************************************************/
#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(KalmanFilter, "ramfuncs")
#endif
double KalmanFilter(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R)
{
    static int isFirstTimeExcuted = 1;

	double R = MeasureNoise_R;
	double Q = ProcessNiose_Q;

	static double x_last = 0;
	double x_mid = x_last;
	double x_now;

	static double p_last = 0;
	double p_mid;
	double p_now;

	double kg;


    if(isFirstTimeExcuted){
        isFirstTimeExcuted = 0;
        x_last = ResrcData;
        p_last = ResrcData;
        return ResrcData;
    }

	x_mid = x_last;
	p_mid = p_last + Q;


	kg = p_mid / (p_mid + R);
	x_now = x_mid + kg * (ResrcData - x_mid);
	p_now = (1 - kg) * p_mid;
	p_last = p_now;
	x_last = x_now;

	return x_now;
}
#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(KalmanFilterSpeed, "ramfuncs")
#endif
double KalmanFilterSpeed(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R)
{
    static int isFirstTimeExcuted = 1;

	double R = MeasureNoise_R;
	double Q = ProcessNiose_Q;

	static double x_last = 0;
	double x_mid = x_last;
	double x_now;

	static double p_last = 0;
	double p_mid;
	double p_now;

	double kg;

    if(isFirstTimeExcuted){
        isFirstTimeExcuted = 0;
        x_last = ResrcData;
        p_last = ResrcData;
        return ResrcData;
    }

	x_mid = x_last;
	p_mid = p_last + Q;

	kg = p_mid / (p_mid + R);
	x_now = x_mid + kg * (ResrcData - x_mid);
	p_now = (1 - kg) * p_mid;
	p_last = p_now;
	x_last = x_now;

	return x_now;
}
#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(KalmanFilterForce, "ramfuncs")
#endif
double KalmanFilterForce(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R)
{

    static int isFirstTimeExcuted = 1;

	double R = MeasureNoise_R;
	double Q = ProcessNiose_Q;

	static double x_last = 0;
	double x_mid = x_last;
	double x_now;

	static double p_last = 0;
	double p_mid;
	double p_now;

	double kg;

    if(isFirstTimeExcuted){
        isFirstTimeExcuted = 0;
        x_last = ResrcData;
        p_last = ResrcData;
        return ResrcData;
    }

	x_mid = x_last;
	p_mid = p_last + Q;


	kg = p_mid / (p_mid + R);
	x_now = x_mid + kg * (ResrcData - x_mid);
	p_now = (1 - kg) * p_mid;
	p_last = p_now;
	x_last = x_now;

	return x_now;
}
#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(KalmanFilterAccel, "ramfuncs")
#endif
double KalmanFilterAccel(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R)
{
    static int isFirstTimeExcuted = 1;

	double R = MeasureNoise_R;
	double Q = ProcessNiose_Q;

	static double x_last = 0;
	double x_mid = x_last;
	double x_now;

	static double p_last = 0;
	double p_mid;
	double p_now;

	double kg;

    if(isFirstTimeExcuted){
        isFirstTimeExcuted = 0;
        x_last = ResrcData;
        p_last = ResrcData;
        return ResrcData;
    }

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
#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(DisablePwmOutput, "ramfuncs")
#endif
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
	if(gSysState.erro.bit.software != 1){
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

#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(CheckStickSetion, "ramfuncs")
#endif
int CheckStickSetion(double val){
	if(val <= gConfigPara.RB_MaxDistance){
		return 0;
	}
	else if(val <= gConfigPara.RB_Distance9){
		return 1;
	}
	else if(val <= gConfigPara.RB_Distance8){
		return 2;
	}
	else if(val <= gConfigPara.RB_Distance7){
		return 3;
	}
	else if(val <= gConfigPara.RB_Distance6){
		return 4;
	}
	else if(val <= gConfigPara.RB_Distance5){
		return 5;
	}
	else if(val <= gConfigPara.RB_Distance4){
		return 6;
	}
    else if(val <= gConfigPara.RB_Distance3){
        return 7;
    }
    else if(val <= gConfigPara.RB_Distance2){
        return 8;
    }
    else if(val <= gConfigPara.RB_EmptyDistance){
        return 9;
    }
    else if(val <= gConfigPara.RB_Distance1){
        return 10;
    }
    else if(val <= gConfigPara.RB_Distance0){
        return 11;
    }
    else if(val <= gConfigPara.LF_Distance1){
        return 12;
    }
    else if(val <= gConfigPara.LF_EmptyDistance){
        return 13;
    }
    else if(val <= gConfigPara.LF_Distance2){
        return 14;
    }
    else if(val <= gConfigPara.LF_Distance3){
        return 15;
    }
    else if(val <= gConfigPara.LF_Distance4){
        return 16;
    }
    else if(val <= gConfigPara.LF_Distance5){
        return 17;
    }
    else if(val <= gConfigPara.LF_Distance6){
        return 18;
    }
    else if(val <= gConfigPara.LF_Distance7){
        return 19;
    }
    else if(val <= gConfigPara.LF_Distance8){
        return 20;
    }
    else if(val <= gConfigPara.LF_Distance9){
        return 21;
    }
    else if(val <= gConfigPara.LF_MaxDistance){
        return 22;
    }
	else{
		return 23;
	}
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

void Button_Debounce1(void){
    static int count_pressed = 0;
    static int count_release = 0;

    switch(gButtonStatus[TK9_TRIGGER]){
        case BTN_INIT:
            if(gPISO_165[TK9_TRIGGER+1] == 1){
                gButtonStatus[TK9_TRIGGER] = BTN_PRESSED;
            }
            else if(gPISO_165[TK9_TRIGGER+1] == 0){
                gButtonStatus[TK9_TRIGGER] = BTN_RELEASE;
            }
            else{
                gButtonStatus[TK9_TRIGGER] = BTN_INIT;
            }
            break;
        case BTN_RELEASE:
        	gSysInfo.sixButtons &= (~(0x10));
            if(gPISO_165[TK9_TRIGGER+1] == 1){
                count_pressed ++;
            }
            else{
                count_pressed = 0;
                gButtonStatus[TK9_TRIGGER] = BTN_RELEASE;
            }
            if(count_pressed > 20){
                gButtonStatus[TK9_TRIGGER] = BTN_PRESSED;
            }
            else{
                gButtonStatus[TK9_TRIGGER] = BTN_RELEASE;
            }

            break;
        case BTN_PRESSED:
        	gSysInfo.sixButtons |= 0x10;
            if(gPISO_165[TK9_TRIGGER+1] == 0){
                count_release ++;
            }
            else{
                count_release = 0;
                gButtonStatus[TK9_TRIGGER] = BTN_PRESSED;
            }
            if(count_release > 20){
                gButtonCmd[TK9_TRIGGER] += 1;
                gButtonStatus[TK9_TRIGGER] = BTN_RELEASE;
            }
            else{
                gButtonStatus[TK9_TRIGGER] = BTN_PRESSED;
            }
            break;
        default:
            break;
    }
}

void Button_Debounce2(void){
    static int count_pressed = 0;
    static int count_release = 0;

    switch(gButtonStatus[AK29_BUTTON]){
        case BTN_INIT:
            if(gPISO_165[AK29_BUTTON+1] == 1){
                gButtonStatus[AK29_BUTTON] = BTN_PRESSED;
            }
            else if(gPISO_165[AK29_BUTTON+1] == 0){
                gButtonStatus[AK29_BUTTON] = BTN_RELEASE;
            }
            else{
                gButtonStatus[AK29_BUTTON] = BTN_INIT;
            }
            break;
        case BTN_RELEASE:
        	gSysInfo.sixButtons &= (~(0x20));
            if(gPISO_165[AK29_BUTTON+1] == 1){
                count_pressed ++;
            }
            else{
                count_pressed = 0;
                gButtonStatus[AK29_BUTTON] = BTN_RELEASE;
            }
            if(count_pressed > 20){
                gButtonCmd[AK29_BUTTON] = 1;
                gButtonStatus[AK29_BUTTON] = BTN_PRESSED;
            }
            else{
                gButtonStatus[AK29_BUTTON] = BTN_RELEASE;
            }

            break;
        case BTN_PRESSED:
        	gSysInfo.sixButtons |= 0x20;
            if(gPISO_165[AK29_BUTTON+1] == 0){
                count_release ++;
            }
            else{
                count_release = 0;
                gButtonStatus[AK29_BUTTON] = BTN_PRESSED;
            }
            if(count_release > 20){
                gButtonCmd[AK29_BUTTON] = 0;
                gButtonStatus[AK29_BUTTON] = BTN_RELEASE;
            }
            else{
                gButtonStatus[AK29_BUTTON] = BTN_PRESSED;
            }
            break;
        default:
            break;
    }
}

void Button_Debounce3(void){
    static int count_pressed = 0;
    static int count_release = 0;

    switch(gButtonStatus[FWRD_SWITCH]){
        case BTN_INIT:
            if(gPISO_165[FWRD_SWITCH+1] == 1){
                gButtonStatus[FWRD_SWITCH] = BTN_PRESSED;
            }
            else if(gPISO_165[FWRD_SWITCH+1] == 0){
                gButtonStatus[FWRD_SWITCH] = BTN_RELEASE;
            }
            else{
                gButtonStatus[FWRD_SWITCH] = BTN_INIT;
            }
            break;
        case BTN_RELEASE:
            gSysInfo.sixButtons &= (~(0x01));
            if(gPISO_165[FWRD_SWITCH+1] == 1){
                count_pressed ++;
            }
            else{
                count_pressed = 0;
                gButtonStatus[FWRD_SWITCH] = BTN_RELEASE;
            }
            if(count_pressed > 30){
                gButtonStatus[FWRD_SWITCH] = BTN_PRESSED;
            }
            else{
                gButtonStatus[FWRD_SWITCH] = BTN_RELEASE;
            }

            break;
        case BTN_PRESSED:
            gSysInfo.sixButtons |= 0x01;
            if(gPISO_165[FWRD_SWITCH+1] == 0){
                count_release ++;
            }
            else{
                count_release = 0;
                gButtonStatus[FWRD_SWITCH] = BTN_PRESSED;
            }
            if(count_release > 30){
                gButtonCmd[FWRD_SWITCH] += 1;
                gButtonStatus[FWRD_SWITCH] = BTN_RELEASE;
            }
            else{
                gButtonStatus[FWRD_SWITCH] = BTN_PRESSED;
            }
            break;
        default:
            break;
    }
    if(gButtonCmd[AK29_BUTTON] == 1){
        gButtonCmd[FWRD_SWITCH] = 0;
    }
}

void Button_Debounce4(void){
    static int count_pressed = 0;
    static int count_release = 0;

    switch(gButtonStatus[RGHT_SWITCH]){
        case BTN_INIT:
            if(gPISO_165[RGHT_SWITCH+1] == 1){
                gButtonStatus[RGHT_SWITCH] = BTN_PRESSED;
            }
            else if(gPISO_165[RGHT_SWITCH+1] == 0){
                gButtonStatus[RGHT_SWITCH] = BTN_RELEASE;
            }
            else{
                gButtonStatus[RGHT_SWITCH] = BTN_INIT;
            }
            break;
        case BTN_RELEASE:
            gSysInfo.sixButtons &= (~(0x08));
            if(gPISO_165[RGHT_SWITCH+1] == 1){
                count_pressed ++;
            }
            else{
                count_pressed = 0;
                gButtonStatus[RGHT_SWITCH] = BTN_RELEASE;
            }
            if(count_pressed > 30){
                gButtonStatus[RGHT_SWITCH] = BTN_PRESSED;
            }
            else{
                gButtonStatus[RGHT_SWITCH] = BTN_RELEASE;
            }

            break;
        case BTN_PRESSED:
            gSysInfo.sixButtons |= 0x08;
            if(gPISO_165[RGHT_SWITCH+1] == 0){
                count_release ++;
            }
            else{
                count_release = 0;
                gButtonStatus[RGHT_SWITCH] = BTN_PRESSED;
            }
            if(count_release > 30){
                gButtonCmd[RGHT_SWITCH] += 1;
                gButtonStatus[RGHT_SWITCH] = BTN_RELEASE;
            }
            else{
                gButtonStatus[RGHT_SWITCH] = BTN_PRESSED;
            }
            break;
        default:
            break;
    }
    if(gButtonCmd[AK29_BUTTON] == 1){
        gButtonCmd[RGHT_SWITCH] = 0;
    }
}

void Button_Debounce5(void){
    static int count_pressed = 0;
    static int count_release = 0;

    switch(gButtonStatus[REAR_SWITCH]){
        case BTN_INIT:
            if(gPISO_165[REAR_SWITCH+1] == 1){
                gButtonStatus[REAR_SWITCH] = BTN_PRESSED;
            }
            else if(gPISO_165[REAR_SWITCH+1] == 0){
                gButtonStatus[REAR_SWITCH] = BTN_RELEASE;
            }
            else{
                gButtonStatus[REAR_SWITCH] = BTN_INIT;
            }
            break;
        case BTN_RELEASE:
            gSysInfo.sixButtons &= (~(0x02));
            if(gPISO_165[REAR_SWITCH+1] == 1){
                count_pressed ++;
            }
            else{
                count_pressed = 0;
                gButtonStatus[REAR_SWITCH] = BTN_RELEASE;
            }
            if(count_pressed > 30){
                gButtonStatus[REAR_SWITCH] = BTN_PRESSED;
            }
            else{
                gButtonStatus[REAR_SWITCH] = BTN_RELEASE;
            }

            break;
        case BTN_PRESSED:
            gSysInfo.sixButtons |= 0x02;
            if(gPISO_165[REAR_SWITCH+1] == 0){
                count_release ++;
            }
            else{
                count_release = 0;
                gButtonStatus[REAR_SWITCH] = BTN_PRESSED;
            }
            if(count_release > 30){
                gButtonCmd[REAR_SWITCH] += 1;
                gButtonStatus[REAR_SWITCH] = BTN_RELEASE;
            }
            else{
                gButtonStatus[REAR_SWITCH] = BTN_PRESSED;
            }
            break;
        default:
            break;
    }
    if(gButtonCmd[AK29_BUTTON] == 1){
        gButtonCmd[REAR_SWITCH] = 0;
    }
}

void Button_Debounce6(void){
    static int count_pressed = 0;
    static int count_release = 0;

    switch(gButtonStatus[LEFT_SWITCH]){
        case BTN_INIT:
            if(gPISO_165[LEFT_SWITCH+1] == 1){
                gButtonStatus[LEFT_SWITCH] = BTN_PRESSED;
            }
            else if(gPISO_165[LEFT_SWITCH+1] == 0){
                gButtonStatus[LEFT_SWITCH] = BTN_RELEASE;
            }
            else{
                gButtonStatus[LEFT_SWITCH] = BTN_INIT;
            }
            break;
        case BTN_RELEASE:
            gSysInfo.sixButtons &= (~(0x04));
            if(gPISO_165[LEFT_SWITCH+1] == 1){
                count_pressed ++;
            }
            else{
                count_pressed = 0;
                gButtonStatus[LEFT_SWITCH] = BTN_RELEASE;
            }
            if(count_pressed > 30){
                gButtonStatus[LEFT_SWITCH] = BTN_PRESSED;
            }
            else{
                gButtonStatus[LEFT_SWITCH] = BTN_RELEASE;
            }

            break;
        case BTN_PRESSED:
            gSysInfo.sixButtons |= 0x04;
            if(gPISO_165[LEFT_SWITCH+1] == 0){
                count_release ++;
            }
            else{
                count_release = 0;
                gButtonStatus[LEFT_SWITCH] = BTN_PRESSED;
            }
            if(count_release > 30){
                gButtonCmd[LEFT_SWITCH] += 1;
                gButtonStatus[LEFT_SWITCH] = BTN_RELEASE;
            }
            else{
                gButtonStatus[LEFT_SWITCH] = BTN_PRESSED;
            }
            break;
        default:
            break;
    }
    if(gButtonCmd[AK29_BUTTON] == 1){
        gButtonCmd[LEFT_SWITCH] = 0;
    }
}

void Null_Displacement_Trim(void){
    double trim_sum;
    if(gSysInfo.board_type == PITCH){
        trim_sum = (gButtonCmd[FWRD_SWITCH] - gButtonCmd[REAR_SWITCH]) * gConfigPara.Trim_StepSize;
        if(trim_sum > 7){
            trim_sum = 7;
        }
        else if(trim_sum < -12){
            trim_sum = -12;
        }
        else{
            trim_sum = trim_sum;
        }
        gSysInfo.DimL_B = 63.2728 - trim_sum;
    }
    else if(gSysInfo.board_type == ROLL){
        trim_sum = (gButtonCmd[LEFT_SWITCH] - gButtonCmd[RGHT_SWITCH]) * gConfigPara.Trim_StepSize;
        if(trim_sum > 11){
            trim_sum = 11;
        }
        else if(trim_sum < -11){
            trim_sum = -11;
        }
        else{
            trim_sum = trim_sum;
        }
        gSysInfo.DimL_B = 57.9135 - trim_sum;
    }
}
