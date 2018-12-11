#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "SPIprocess.h"
#include "ADprocessor.h"
#include "GlobalVarAndFunc.h"
#include "public.h"


#define ENABLE_CNV_AD    	GpioDataRegs.GPCDAT.bit.GPIO84 = 1;
#define DISABLE_CNV_AD 		GpioDataRegs.GPCDAT.bit.GPIO84 = 0;


inline void Send16Clocks(void){
	SpiaRegs.SPICCR.bit.SPICHAR = 0xf;
	SpiaRegs.SPITXBUF = 0x1;
}
inline void Send1Clock(void) {
	SpiaRegs.SPICCR.bit.SPICHAR = 0x0;
	SpiaRegs.SPITXBUF = 1;
}
/***************************************************************
 *Name:						StartGetADBySpi
 *Function:
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
void StartGetADBySpi(void)
{
	ENABLE_CNV_AD;
}
/***************************************************************
 *Name:						ReadADBySpi
 *Function:
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.21
 ****************************************************************/
void ReadADBySpi(void)
{
	int retry;
	/**********************************************/
	Send16Clocks();
	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {
	}
	gSysMonitorVar.anolog.single.var[DisplacementValue].value = SpiaRegs.SPIRXBUF;
	//gSysMonitorVar.anolog.single.var[DisplacementValue].value = KalmanFilter(SpiaRegs.SPIRXBUF, KALMAN_Q, KALMAN_R);
	/**********************************************/
	Send16Clocks();
	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {
	}
	gSysMonitorVar.anolog.single.var[ForceValue].value = SpiaRegs.SPIRXBUF;
	//gSysMonitorVar.anolog.single.var[ForceValue].value = KalmanFilter(SpiaRegs.SPIRXBUF, KALMAN_Q, KALMAN_R);
	/**********************************************/
	Send1Clock();
	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {
	}
	retry = SpiaRegs.SPIRXBUF;
	/**********************************************/
	DISABLE_CNV_AD;
}
