#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "GlobalVarAndFunc.h"
#include "public.h"


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
		gECapCount = ECap4Regs.CAP1;
	}
	else if(ECap4Regs.ECFLG.bit.CEVT2){
		gECapCount = ECap4Regs.CAP2 - ECap4Regs.CAP1;
	}
	else if(ECap4Regs.ECFLG.bit.CEVT3){
		gECapCount = ECap4Regs.CAP3 - ECap4Regs.CAP2;
	}
	else if(ECap4Regs.ECFLG.bit.CEVT4){
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
		gECapCount = ECap5Regs.CAP1;
	}
	else if(ECap5Regs.ECFLG.bit.CEVT2){
		gECapCount = ECap5Regs.CAP2 - ECap5Regs.CAP1;
	}
	else if(ECap5Regs.ECFLG.bit.CEVT3){
		gECapCount = ECap5Regs.CAP3 - ECap5Regs.CAP2;
	}
	else if(ECap4Regs.ECFLG.bit.CEVT4){
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
		gECapCount = ECap6Regs.CAP1;
	}
	else if(ECap6Regs.ECFLG.bit.CEVT2){
		gECapCount = ECap6Regs.CAP2 - ECap6Regs.CAP1;
	}
	else if(ECap6Regs.ECFLG.bit.CEVT3){
		gECapCount = ECap5Regs.CAP3 - ECap5Regs.CAP2;
	}
	else if(ECap6Regs.ECFLG.bit.CEVT4){
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
int32 CalculateSpeed(Uint32 capCount){
	//TODO calculate the motor speed
	int32 speed32 = 0;
	if(capCount){
		return -1;
	}

	speed32 = ((4500000000.0)/(float)capCount);//4500000000 = 75000000*60

	if(speed32 < 19200){
		return speed32;
	}
	else{
		return -1;
	}
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
