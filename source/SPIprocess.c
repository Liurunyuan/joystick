#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "SPIprocess.h"
#include "ADprocessor.h"




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
	while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG == 1)
	{
		retry ++;
		if(retry > 200){
			//return 0;
		}
	}
	SpiaRegs.SPITXBUF = 0xffff;
	retry = 0;
	while(SpiaRegs.SPISTS.bit.INT_FLAG == 0){
		retry ++;
			if(retry > 200){
				//return 0;
			}
	}
	gSysMonitorVar.anolog.single.var[DisplacementValue].value = SpiaRegs.SPIRXBUF;

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
