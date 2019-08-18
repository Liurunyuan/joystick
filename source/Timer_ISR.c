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
#include <stdio.h>

#define N (300)
#define RS422STATUSCHECK (1000)

int checkForceDirection(){
            if(gSysMonitorVar.anolog.AD_16bit.var[ForceValue_16bit].value > 32810){
                gforwardForce = 0;
                gbackwardForce = 1;
                gNoExternalForce = 0;
                return gSysMonitorVar.anolog.AD_16bit.var[ForceValue_16bit].value - 32810;
            }
            else if(gSysMonitorVar.anolog.AD_16bit.var[ForceValue_16bit].value < 32707){
                gforwardForce = 1;
                gbackwardForce = 0;
                gNoExternalForce = 0;
                return gSysMonitorVar.anolog.AD_16bit.var[ForceValue_16bit].value - 32707;
            }
            else{
                gforwardForce = 0;
                gbackwardForce = 0;
                gNoExternalForce = 1;
                return 0;
            }
}

void ForceCloseLoop(double forceKp){
    int forceCloseLoopPWM;

    forceCloseLoopPWM = forceKp * checkForceDirection();
    if(gNoExternalForce == 1){
        gSysInfo.currentDuty = 0;
    }
    else if(gforwardForce == 1){
        gSysInfo.currentDuty = forceCloseLoopPWM;
    }
    else if(gbackwardForce == 1){
        gSysInfo.currentDuty = forceCloseLoopPWM;

    }
    else{

    }
    if (gSysInfo.currentDuty > 750) {
        gSysInfo.currentDuty = 750;
    }
    else if (gSysInfo.currentDuty < -750) {
        gSysInfo.currentDuty = -750;
    }
    gSysInfo.duty = gSysInfo.currentDuty;
}


/***************************************************************
 *Name:						Timer0_ISR_Thread
 *Function:					period = 0.2ms, pack the data
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
void Timer0_ISR_Thread(void){

	static unsigned char count = 0;

	++count;

	if(count > N){
		PackRS422TxData();
		count = 0;
	}

	if(gKeyValue.lock == 1){
		//calculate function parameter
		UpdateKeyValue();

        /*type may need to conversion here */
        gStickState.value = gKeyValue.displacement;
        //gStickState.updateNullDisBackwardState(0);
        //gStickState.updateNullDisForwardState(0);
        //gStickState.updateThresholdDisBackwardState(0);
        //gStickState.updateThresholdDisForwardState(0);

        gExternalForceState.value = gSysMonitorVar.anolog.AD_16bit.var[ForceValue_16bit].value;
        gExternalForceState.updateForceState(0);
        /******************************
         * 
         * 40000                                                     30000                                                       10000
         *  |<--------------------------Backwards--------------------->|<------------------------Forward------------------------->| 
         *  |                                                          |
         *  |Threshold|         ODE     | StartForce   | Null          | Null          | StartForce    |    ODE         |Threshold|
         *  |---------|-----------------|--------------|---------------|---------------|---------------|----------------|---------|
         *  |--bit0---|-------bit1------|--bit2--------|----bit3-------|------bit4-----|-----bit5------|-----bit6-------|---bit7--|
         * 
         * 1:OOR    0:IR
         * 
         * I want to delete the external force state in the bit0 adn bit1.
         * just use the controlfuncindex to record the displacement state.
         * Foward state:
         * Use bit0 to indicate if the displacement is out of range of Threshold section
         * Use bit1 to indicate if the displacement is out of range of ODE section
         * Use bit2 to indicate if the displacement is out of range of StartForce section 
         * Use bit3 to indicate if the displacement is out of range of Null section
         * 
         * we wil check bit0 first then bit1.....when we meet the first value 1 which means that the stick displacement is in the bitx section
         */

        //gSysInfo.controlFuncIndex |= gExternalForceState.ForceState;
        //gSysInfo.controlFuncIndex |= (gStickState.NullDistanceForwardState || gStickState.NullDistanceBackwardState) << 2; 
        //gSysInfo.controlFuncIndex |= (gStickState.ThresholdForwaredState || gStickState.ThresholdForwaredState) << 3; 
        gSysInfo.controlFuncIndex = LocateStickDisSection();

        ControleStateMachineSwitch(gSysInfo.controlFuncIndex); 

        if(gSysInfo.controlFuncIndex == SECTION0
        || gSysInfo.controlFuncIndex == SECTION7){
            gSysState.warning.bit.a = 1;
        }
        else{
            gSysState.warning.bit.a = 0;
        }

		clearSum();
		gKeyValue.lock = 0;
        gSysInfo.controlFuncIndex = 0;
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
	static Uint16 count = 0;
	++count;

	if(gRS422TxQue.front != gRS422TxQue.rear
			&& ScicRegs.SCIFFTX.bit.TXFFST == 0){

		 EnableScicTxInterrupt();
	}

	if(count >= RS422STATUSCHECK){
		count = 0;
//		printf(">>>>>>>>>Check RS422 channel\r\n");
//
//
//		if(gRS422Status.rs422A == 0 && gRS422Status.rs422B == 0){
//			printf(">>>>>>>>>>RS422A and RS422B both failed to connect\r\n");
//			return;
//		}
//
//		if(RS422_CHANNEL_A == gRS422Status.rs422CurrentChannel){
//			if(gRS422Status.rs422A){
//				gRS422Status.rs422A = 0;
//			}
//			else{
//				printf(">>>>>>>>>>Switch to RS422BBBBBBBBBB channel\r\n");
//				gRS422Status.rs422CurrentChannel = RS422_CHANNEL_B;
//				//ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;
//			}
//		}
//		else if(RS422_CHANNEL_B == gRS422Status.rs422CurrentChannel){
//			if(gRS422Status.rs422B){
//				gRS422Status.rs422B = 0;
//			}
//			else{
//				//TODO need to switch rs422B to rs422A.
//				printf(">>>Switch to RS422AAAAAAAAAAAAAAA channel\r\n");
//				gRS422Status.rs422CurrentChannel = RS422_CHANNEL_A;
//			}
//		}
//		else{
//			printf(">>>>>>>>>>>>>>>>>>>>Unknow RS422 channel\r\n");
//		}
	}
}
