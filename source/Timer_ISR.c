#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "GlobalVarAndFunc.h"
#include "public.h"
#include "Timer_ISR.h"
#include "SCI_ISR.h"
#include "SCI_TX.h"
#include "PWM_ISR.h"
#include "Filter_Alg.h"
#include "ADprocessor.h"
#include "Ctl_Strategy.h"
#include "ECap_ISR.h"
#include <stdio.h>
#include <math.h>

#define N (300)
#define RS422STATUSCHECK (1000)

/***************************************************************
 *Name:						Timer0_ISR_Thread
 *Function:					period = 0.2ms, pack the data
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(Timer0_ISR_Thread, "ramfuncs")
#endif
void Timer0_ISR_Thread(void){

	static unsigned char count = 0;
	static unsigned char trim_time_count = 0;
	static double zero_force_SUM = 0;
	static int zero_count = 0;
	static int flag = 0;
	static int timer0_interrupt_cnt = 0;
	static int first_time_run = 0;
	static int first_time_back_to_mid = 0;
	static int first_time_to_front = 0;
	static int first_time_to_back = 0;
	static int location = 0;
	static int touch_positive_edge = 0;

    double force_Joystick;

    if(timer0_interrupt_cnt == 0){
        ++timer0_interrupt_cnt;
        CpuTimer0Regs.TCR.bit.TSS = 1;
        ConfigCpuTimer(&CpuTimer0, 120, 200);//t = freq * priod/150000000,0.2ms
        CpuTimer0Regs.TCR.bit.TSS = 0;
    }

	++count;

	if(count > N){
		PackRS422TxData();
		count = 0;
	}

    if((gSysInfo.displace_LSM_buffer == 0) || (gSysInfo.displace_LSM_buffer == 1)){
        UpdateKeyValue();
        gAccelDirection.updateAccelDirection(0);
//        gKeyValue.lock = 0;
    }
    else{
//        ++ gSysInfo.ob_velocityOpenLoop;
        return;
    }

    gStickState.value = gKeyValue.displacement;
    gRotateDirection.updateRotateDirection(0);

    //calculate function parameter
    force_Joystick = (gSysMonitorVar.anolog.AD_16bit.var[ForceValue_16bit].value * gSysInfo.Force_K + gSysInfo.Force_B)*0.32143;

    if(zero_count < 10){
        zero_force_SUM = zero_force_SUM + force_Joystick;
        ++zero_count;

//        gSysInfo.currentStickDisSection = CheckStickSetion(gStickState.value);
//          clearSum();
//        gKeyValue.lock = 0;
        return;
    }
    else{
        if(flag == 0)
        {
            gSysInfo.zeroForce = zero_force_SUM/10;
            flag = 1;
        }
    }
    gExternalForceState.value = force_Joystick - gSysInfo.zeroForce;
    gExternalForceState.updateForceState(0);
    gSysInfo.currentStickDisSection = CheckStickSetion(gStickState.value);

    if(touch_positive_edge == 0){
        if(gSysInfo.currentStickDisSection < 23){
            gSysInfo.targetDuty = 50;
        }
        else{
            gSysInfo.targetDuty = -50;
            touch_positive_edge = 1 - touch_positive_edge;
        }
    }
    else{
            if(gSysInfo.currentStickDisSection > 0){
                gSysInfo.targetDuty = -50;
            }
            else{
                gSysInfo.targetDuty = 50;
                touch_positive_edge = 1 - touch_positive_edge;
            }
    }


//    if(first_time_run == 0){
//        if(first_time_back_to_mid == 0){
//            if((gSysInfo.currentStickDisSection == 11) || (gSysInfo.currentStickDisSection == 12)){
//                first_time_back_to_mid = 1;
//            }
//            else{
//                ++gSysInfo.ob_velocityOpenLoop;
//                first_time_back_to_mid = 0;
//                OnlyWithSpringFront();
//            }
//        }
//        else if(first_time_back_to_mid == 1){
//            if(first_time_to_front == 0){
//                if(gSysInfo.currentStickDisSection < 23){
//                    gSysInfo.targetDuty = 50;
//                }
//                else{
////                    gSysInfo.targetDuty = -60;
//                    first_time_to_front = 1;
//                    return;
//                }
//            }
//            else{
//                if(first_time_to_back == 0){
//                    if(gSysInfo.currentStickDisSection > 0){
//                        gSysInfo.targetDuty = -50;
//                    }
//                    else{
////                        gSysInfo.targetDuty = 60;
//                        first_time_to_back = 1;
//                        return;
//                    }
//                }
//                else{
//                    if((gSysInfo.currentStickDisSection == 11) || (gSysInfo.currentStickDisSection == 12)){
//                        first_time_run = 1;
//                        return;
//                    }
//                    else{
//                        gSysInfo.targetDuty = 50;
//                    }
//                }
//            }
//        }
//        else{
//            ++gSysInfo.ob_velocityOpenLoop;
//        }
//    }
//    else{
//        OnlyWithSpringFront();
//    }

	if(trim_time_count == 1000){
	    trim_time_count = 0;
	    if(gSysInfo.board_type == PITCH){
	        if(gButtonStatus[FWRD_SWITCH] == BTN_PRESSED){
	            if(gSysInfo.DimL_B < 56.2728){
	                gSysInfo.DimL_B = 56.2728;
	            }
	            else{
	                gSysInfo.DimL_B = 63.2728 - gConfigPara.Trim_Speed * 0.2;
	            }
	        }
	        else if(gButtonStatus[REAR_SWITCH] == BTN_PRESSED){
	            if(gSysInfo.DimL_B > 75.2728){
	                gSysInfo.DimL_B = 75.2728;
	            }
	            else{
	                gSysInfo.DimL_B = 63.2728 + gConfigPara.Trim_Speed * 0.2;
	            }
	        }
	        else{
	            gSysInfo.DimL_B = gSysInfo.DimL_B;
	        }
	    }
	    else{
	        if(gButtonStatus[LEFT_SWITCH] == BTN_PRESSED){
	            if(gSysInfo.DimL_B < 46.9135){
	                gSysInfo.DimL_B = 46.9135;
	            }
	            else{
	                gSysInfo.DimL_B = 57.9135 - gConfigPara.Trim_Speed * 0.2;
	            }
	        }
	        else if(gButtonStatus[RGHT_SWITCH] == BTN_PRESSED){
	            if(gSysInfo.DimL_B > 68.9135){
	                gSysInfo.DimL_B = 68.9135;
	            }
	            else{
	                gSysInfo.DimL_B = 57.9135 + gConfigPara.Trim_Speed * 0.2;
	            }
	        }
	        else{
	            gSysInfo.DimL_B = gSysInfo.DimL_B;
	        }
	    }
	}
	else{
	    trim_time_count ++;
	    gSysInfo.DimL_B = gSysInfo.DimL_B;
	}
}
/**************************************************************
 *Name:		   EnableScicTxInterrupt
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018.11.14
 **************************************************************/
void EnableScicTxInterrupt(void){
	ScicRegs.SCIFFTX.bit.TXFFINTCLR = 1;
	ScicRegs.SCIFFTX.bit.TXFFIENA = 1;
}
/***************************************************************
 *Name:						Timer1_ISR_Thread
 *Function:					priod = 10ms, transmit datat on rs422
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
void Timer1_ISR_Thread(void){

	if(gRS422TxQue.front != gRS422TxQue.rear
			&& ScicRegs.SCIFFTX.bit.TXFFST == 0){

		 EnableScicTxInterrupt();
	}
}
