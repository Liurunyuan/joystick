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
void Timer0_ISR_Thread(void){

	static unsigned char count = 0;
	static double zero_force_SUM = 0;
	static int zero_count = 0;

    double force_Joystick;
    double cos_value;
    double angle;

	++count;

	if(count > N){
		PackRS422TxData();
		count = 0;
	}

	if(gKeyValue.lock == 1){
		//calculate function parameter
		UpdateKeyValue();

        gRotateDirection.updateRotateDirection(0);
        gStickState.value = gKeyValue.displacement;

        angle = (abs(gSysMonitorVar.anolog.AD_16bit.var[DisplacementValue_16bit].value - 26288))*0.00030821;
        cos_value = cos(angle*PI/180.0);
       // cos_value = 1;
        force_Joystick = -1*(((gSysMonitorVar.anolog.AD_16bit.var[ForceValue_16bit].value * FORCE_DIMENSION_K + FORCE_DIMENSION_B)*(0.045/0.14))/cos_value);

        if(zero_count < 10){
            zero_force_SUM = zero_force_SUM + force_Joystick;
            ++zero_count;
		    clearSum();
		    gKeyValue.lock = 0;
            return;
        }
        else{
            gSysInfo.zeroForce = zero_force_SUM/10;
        }

        gExternalForceState.value = force_Joystick - gSysInfo.zeroForce;
        gExternalForceState.updateForceState(0);
/******************************
* -20mm                                                     0mm                                                      12mm 
*  |<--------------------------Backwards--------------------->|<------------------------Forward------------------------->| 
*  |                                                          |
*  |Threshold|        ODE      | StartForce   |     Null      |     Null      | StartForce    |      ODE       |Threshold|
*  |--Sec0---|-------Sec1------|----Sec2------|----Sec3-------|------Sec4-----|-----Sec5------|-----Sec6-------|---Sec7--|
*  |--------TH0---------------TH1------------TH2-------------TH3-------------TH4-------------TH5---------------TH6-------|
*  |----- -18mm ----------- -15mm -------- -10mm ----------- 0mm ----------  8mm ----------  9mm ------------ 10mm ------|
* 
* 
* we wil check bit0 first then bit1.....when we meet the first value 1 which means that the stick displacement is in the bitx section
*/
        gSysInfo.controlFuncIndex = LocateStickDisSection();

        ControleStateMachineSwitch(gSysInfo.controlFuncIndex); 

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
