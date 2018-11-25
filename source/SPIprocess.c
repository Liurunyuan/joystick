#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "SPIprocess.h"
#include "ADprocessor.h"
#include "GlobalVarAndFunc.h"
#include "public.h"




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
	//GpioDataRegs.GPCSET.bit.GPIO84 = 1;
	GpioDataRegs.GPCDAT.bit.GPIO84 = 1;
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
	int retry;



	/**********************************************/

	SpiaRegs.SPICCR.bit.SPICHAR = 0xf;
	//while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG == 1)
	//{
	//	retry ++;
	//	if(retry > 200){
	//		//return 0;
	//	}
	//}
	//SpiaRegs.SPICCR.bit.SPICHAR = 0x1;
	SpiaRegs.SPITXBUF = 0x1;
	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
	gSysMonitorVar.anolog.single.var[ForceValue].value = SpiaRegs.SPIRXBUF;

	/**********************************************/
	SpiaRegs.SPITXBUF = 0x1;
	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
	gSysMonitorVar.anolog.single.var[DisplacementValue].value = SpiaRegs.SPIRXBUF;
	/**********************************************/

	SpiaRegs.SPICCR.bit.SPICHAR = 0x0;
	SpiaRegs.SPITXBUF = 1;
	//while(SpiaRegs.SPISTS.bit.INT_FLAG == 0){
	//	retry ++;
	//}
	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
	retry = SpiaRegs.SPIRXBUF;
	GpioDataRegs.GPCDAT.bit.GPIO84 = 0;
}
