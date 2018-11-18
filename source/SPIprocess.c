#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "SPIprocess.h"
#include "ADprocessor.h"
#include "GlobalVarAndFunc.h"




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
	//TODO
	int retry = 0;
	GpioDataRegs.GPCSET.bit.GPIO84 = 1;
	/**********************************************/
	while(gSysInfo.sdoStatus == 0){
		gSysInfo.sdoStatus = GpioDataRegs.GPBDAT.bit.GPIO55;
	}
	/**********************************************/

	SpiaRegs.SPICCR.bit.SPICHAR = 0x0;
	SpiaRegs.SPIDAT = 1;
	while(SpiaRegs.SPISTS.bit.INT_FLAG == 0){
		retry ++;
	}
	retry = SpiaRegs.SPIRXBUF;
	/**********************************************/
	SpiaRegs.SPICCR.bit.SPICHAR = 0xf;
	retry = 0;
	while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG == 1)
	{
		retry ++;
		if(retry > 200){
			//return 0;
		}
	}
	//SpiaRegs.SPICCR.bit.SPICHAR = 0x1;
	SpiaRegs.SPIDAT = 0x1;
	retry = 0;
	while(SpiaRegs.SPISTS.bit.INT_FLAG == 0){
		retry ++;
		if(retry > 200){
			return;
		}
	}

	gSysMonitorVar.anolog.single.var[DisplacementValue].value = SpiaRegs.SPIRXBUF;
	/**********************************************/
	retry = 0;
	while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG == 1)
	{
		retry ++;
		if(retry > 200){
			return;
		}
	}
	SpiaRegs.SPIDAT = 0x1;

	retry = 0;
	while(SpiaRegs.SPISTS.bit.INT_FLAG == 0){
		retry ++;
		if(retry > 200){
			return;
		}
	}
	gSysMonitorVar.anolog.single.var[ForceValue].value = SpiaRegs.SPIRXBUF;
	/**********************************************/
	GpioDataRegs.GPCCLEAR.bit.GPIO84 = 1;
	//GpioDataRegs.GPBSET.bit.GPIO56 = 1;
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
	//TODO
}
