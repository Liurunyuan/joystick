#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "GlobalVarAndFunc.h"
#include "ECap_ISR.h"
#include "public.h"


//volatile Uint64  gECapCount = 0;
volatile double gMotorSpeedEcap = 0;

void InitEcapVar(void){
    gMotorSpeedEcap = 0;
//    gECapCount = 0;
}

/**************************************************************
 *Name:		   GetECap4Count
 *Comment:
 *Input:	   void
 *Output:	   motor speed(int)
 *Author:	   Simon
 *Date:		   2018.11.14
 **************************************************************/
int GetECap4Count(void){

	if(ECap4Regs.ECFLG.bit.CEVT1){
	    gSysInfo.isEcapRefresh = 1;
		gECapCount = ECap4Regs.CAP1;
	}
	else if(ECap4Regs.ECFLG.bit.CEVT2){
	    gSysInfo.isEcapRefresh = 1;
		gECapCount = ECap4Regs.CAP2 - ECap4Regs.CAP1;
	}
	else if(ECap4Regs.ECFLG.bit.CEVT3){
	    gSysInfo.isEcapRefresh = 1;
		gECapCount = ECap4Regs.CAP3 - ECap4Regs.CAP2;
	}
	else if(ECap4Regs.ECFLG.bit.CEVT4){
	    gSysInfo.isEcapRefresh = 1;
		gECapCount = ECap4Regs.CAP4 - ECap4Regs.CAP3;
	}
	else{

	}
	return gECapCount;
}
/**************************************************************
 *Name:		   GetECap5Count
 *Comment:
 *Input:	   void
 *Output:	   motor speed(int)
 *Author:	   Simon
 *Date:		   2018.11.14
 **************************************************************/
int GetECap5Count(void){

	if(ECap5Regs.ECFLG.bit.CEVT1){
	    gSysInfo.isEcapRefresh = 1;
		gECapCount = ECap5Regs.CAP1;
	}
	else if(ECap5Regs.ECFLG.bit.CEVT2){
	    gSysInfo.isEcapRefresh = 1;
		gECapCount = ECap5Regs.CAP2 - ECap5Regs.CAP1;
	}
	else if(ECap5Regs.ECFLG.bit.CEVT3){
	    gSysInfo.isEcapRefresh = 1;
		gECapCount = ECap5Regs.CAP3 - ECap5Regs.CAP2;
	}
	else if(ECap4Regs.ECFLG.bit.CEVT4){
	    gSysInfo.isEcapRefresh = 1;
		gECapCount = ECap5Regs.CAP4 - ECap5Regs.CAP3;
	}
	else{

	}
	return gECapCount;
}
/**************************************************************
 *Name:		   GetECap6Count
 *Comment:
 *Input:	   void
 *Output:	   motor speed(int)
 *Author:	   Simon
 *Date:		   2018.11.14
 **************************************************************/
int GetECap6Count(void){

	if(ECap6Regs.ECFLG.bit.CEVT1){
	    gSysInfo.isEcapRefresh = 1;
		gECapCount = ECap6Regs.CAP1;
	}
	else if(ECap6Regs.ECFLG.bit.CEVT2){
	    gSysInfo.isEcapRefresh = 1;
		gECapCount = ECap6Regs.CAP2 - ECap6Regs.CAP1;
	}
	else if(ECap6Regs.ECFLG.bit.CEVT3){
	    gSysInfo.isEcapRefresh = 1;
		gECapCount = ECap6Regs.CAP3 - ECap6Regs.CAP2;
	}
	else if(ECap6Regs.ECFLG.bit.CEVT4){
	    gSysInfo.isEcapRefresh = 1;
		gECapCount = ECap6Regs.CAP4 - ECap6Regs.CAP3;
	}
	else{

	}
	return gECapCount;
}
/**************************************************************
 *Name:		   CalculateSpeed
 *Comment:
 *Input:	   void
 *Output:	   motor speed(int)
 *Author:	   Simon
 *Date:		   2018��11��12������10:27:17
 **************************************************************/
double CalculateSpeed(Uint32 capCount){
	//TODO calculate the motor speed
	double speed32 = 0;
//	if(capCount <= 0){
//		return -1;
//	}

	//unit of speed32 = degree/sec
	//6171428571.42857
	//1436156.641641
	speed32 = ((201061.9298297)/(float)capCount);//17142857.142857 = 120000000/7 120000000=120MHz clk, 7=the nunber of pairs of poles of the motor
	return speed32;
//	if(speed32 < 19200){
//		return speed32;
//	}
//	else{
//		return -1;
//	}
}
/**************************************************************
 *Name:		   ECap4_Isr
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018��11��12������10:24:47
 **************************************************************/
void ECap4_Isr(void){
	GetECap4Count();
}
/**************************************************************
 *Name:		   ECap5_Isr
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018��11��12������10:24:47
 **************************************************************/
void ECap5_Isr(void){
	GetECap5Count();
}
/**************************************************************
 *Name:		   ECap6_Isr
 *Comment:
 *Input:	   void
 *Output:	   void
 *Author:	   Simon
 *Date:		   2018��11��12������10:24:47
 **************************************************************/
void ECap6_Isr(void){
	GetECap6Count();
}
