#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "ADprocessor.h"

SysMonitorVar gSysMonitorVar;
Uint16 real4 = 0;
Uint16 real6 = 0;

/********update anolog variable value******************************/
//int updateForceValue(void){return GET_FORCE_SGN;}
//int updateBusCurrentP(void){return GET_BUS_CURRENT_P;}
//int updatePower28V_M(void){return GET_28V_M;}
//int updateBridgeCurrentB(void){return GET_B_BRIDGE_CURRENT;}
//int updateBusCurrentB(void){return GET_B_BUS_CURRENT;}
//int updatePower28V(void){return GET_28V;}
//int updateBridgeCurrentA(void){return GET_A_BRIDGE_CURRENT;}
//int updateBusCurrentA(void){return GET_A_BUS_CURRENT;}
//int updateDisplacementValue(void){return GET_DISPLACEMENT_SGN;}
//int updateBridgeCurrentC(void){return GET_C_BRIDGE_CURRENT;}
//int updateBusCurrentC(void){return GET_C_BUS_CURRENT;}
/******************************************************************/


/********update anolog variable value******************************/
int updateForceValue(void){return DMABuf1[0];}
int updateBusCurrentP(void){return DMABuf1[1];}
int updatePower28V_M(void){return DMABuf1[2];}
int updateBridgeCurrentB(void){return DMABuf1[3];}
int updateBusCurrentB(void){return DMABuf1[4];}
int updatePower28V(void){return DMABuf1[5];}
int updateBridgeCurrentA(void){return DMABuf1[6];}
int updateBusCurrentA(void){return DMABuf1[7];}
int updateDisplacementValue(void){return DMABuf1[8];}
int updateBridgeCurrentC(void){return DMABuf1[9];}
int updateBusCurrentC(void){return DMABuf1[10];}
/******************************************************************/

const UV funcptr[] = {

	updateForceValue,
	updateBusCurrentP,
	updatePower28V_M,
	updateBridgeCurrentB,
	updateBusCurrentB,
	updatePower28V,
	updateBridgeCurrentA,
	updateBusCurrentA,
	updateDisplacementValue,
	updateBridgeCurrentC,
	updateBusCurrentC

};

const int anologMaxMinInit[][2] = {
		{0,0},
		{1,0},
		{2,0},
		{3,0},
		{4,0},
		{5,0},
		{6,0},
		{7,0},
		{8,0},
		{9,0},
		{10,0}
};
/**************************************************************
 *Name:						UpdatePowerBoardAnalogInput
 *Function:					更新功率板输入模拟量
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.8.2
 **************************************************************/
void UpdateSingleAnalogInput(void){
	int index;

	for(index = 0; index < TotalChannel; ++index){
		gSysMonitorVar.anolog.single.var[index].value = gSysMonitorVar.anolog.single.var[index].updateValue();
		//gSysMonitorVar.anolog.single.var[index].value = DMABuf1[index];
	}

}
/**************************************************************
 *Name:						IsSingleAnalogValueAbnormal
 *Function:					判定单通道模拟量是否越界
 *Input:					none
 *Output:					return 1表明越界 return 0表明没有越界
 *Author:					Simon
 *Date:						2018.8.2
 **************************************************************/
int IsSingleAnalogValueAbnormal(void){
	int index;
	int ret = 1;
	for(index = 0; index <= TotalChannel; ++index){
		if((gSysMonitorVar.anolog.single.var[index].value > gSysMonitorVar.anolog.single.var[index].max) ||
				(gSysMonitorVar.anolog.single.var[index].value < gSysMonitorVar.anolog.single.var[index].min)) {
			ret = 0;
		}
	}
	return ret;
}
/**************************************************************
 *Name:		   IsCommonAnalogValueAbnormal
 *Comment:
 *Input:	   void
 *Output:	   int
 *Author:	   Simon
 *Date:		   2018年12月18日下午9:02:38
 **************************************************************/
int IsCommonAnalogValueAbnormal(void){

	//TODO need to impletment later
	int ret = 0;

	return ret;
}
/**************************************************************
 *Name:						AdcConversionUnStable
 *Function:					判定模拟量多通道切换以及转换是否稳定
 *Input:					none
 *Output:					return 1表明本次不稳定 return 0表明转换已经稳定
 *Author:					Simon
 *Date:						2018.7.30
 **************************************************************/
int AdcConversionUnStable() {
	static Uint16 count = 0;
	++count;
	if (count > WAIT_TIMES) {
		count = 0;
		return 0;
	}
	else
	{
		return 1;
	}
}
/**************************************************************
 *Name:						AnologChannelChange
 *Function:					模拟量多通道地址改变
 *Input:					上次多通道模拟量地址值
 *Output:					返回多通道模拟量地址值
 *Author:					Simon
 *Date:						2018.7.30
 **************************************************************/
Uint16 AnalogChannelChange(Uint16 address){

	++address;
	if (address >= MAX_CHANNEL) {
		address = 0;
	}
	return address;
}
/**************************************************************
 *Name:						ReadChannelAdcValue
 *Function:					读取DSP的ADC转换结果
 *Input:					通道值
 *Output:					none
 *Author:					Simon
 *Date:						2018.7.30
 **************************************************************/
void ReadChannelAdcValue(Uint16 index){
	gSysMonitorVar.anolog.multi[0].var[index].value = GET_ADCINB7;
	gSysMonitorVar.anolog.multi[1].var[index].value = GET_ADCINB1;
}
/**************************************************************
 *Name:						SwitchAnalogChannel
 *Function:					切换多通道模拟量的地址值
 *Input:					本次要设置的多通道模拟量的地址值
 *Output:					none
 *Author:					Simon
 *Date:						2018.7.30
 **************************************************************/
void SwitchAnalogChannel(Uint16 address){
	/*
	 * GPIO30->AD1K
	 * GPIO29->AD2K
	 * GPIO85->AD3K
	 * GPIO39->AD4K
	 *
	 * */
	int result;
	result = address & 0x0001;
	if(result == 0 ){
		GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;
	}
	else{
		GpioDataRegs.GPASET.bit.GPIO30 = 1;
	}

	result = (address & 0x0004) >> 2;
	if(result == 0 ){
		GpioDataRegs.GPCCLEAR.bit.GPIO85 = 1;
	}
	else{
		GpioDataRegs.GPCSET.bit.GPIO85 = 1;
	}

	result = (address & 0x0008) >> 3;
	if(result == 0 ){
		GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;
	}
	else{
		GpioDataRegs.GPBSET.bit.GPIO39 = 1;
	}

	result = (address & 0x0002) >> 1;
	if(result == 0 ){
		GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;
	}
	else{
		GpioDataRegs.GPASET.bit.GPIO29 = 1;
	}
//	SET_AD1K = address & 0x0001;
//
//	SET_AD3K = (address & 0x0004) >> 2;
//
//    SET_AD4K = (address & 0x0008) >> 3;
//
//	SET_AD2K = (address & 0x0002) >> 1;
}
/**************************************************************
 *Name:						AnalogValueInspect
 *Function:					模拟量多通道巡检函数
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.7.30
 **************************************************************/
void AnalogValueInspect(void){
    static Uint16 address = 0;
    if(AdcConversionUnStable()){
    	return;
    }
    else{
        ReadChannelAdcValue(address);
    	++address;
    	if (address >= TOTAL_CTRLBRD_MULTI_DIGIT) {
    		address = 0;
    	}
//    	address = AnalogChannelChange(address);
        SwitchAnalogChannel(address);
    }

}
/**************************************************************
 *Name:						DigitalValueInspect
 *Function:					数字量多通道巡检函数
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.7.31
 **************************************************************/
void DigitalValueInspect(void){
	static int status = REFRESH;
	static int channel = 0;

	switch(status)
	{
		case REFRESH:
			SET_DIGIT_SER_LOAD_LOW;
			SET_DIGIT_SER_CLK_LOW;
			channel = 0;
			status = LOCK;
			break;
		case LOCK:
			SET_DIGIT_SER_LOAD_HIGH;
			status = TRIGGER;
			break;
		case TRIGGER:
			gSysMonitorVar.digit.multi.var[channel].valueP = GpioDataRegs.GPBDAT.bit.GPIO59;
			gSysMonitorVar.digit.multi.var[channel].valueN = GpioDataRegs.GPBDAT.bit.GPIO60;
			SET_DIGIT_SER_CLK_HIGH;
			status = GETDATA;
			break;
		case GETDATA:
			if(gSysMonitorVar.digit.multi.var[channel].valueP !=0 && gSysMonitorVar.digit.multi.var[channel].valueP != 1){
				real4++;
			}
			if(	gSysMonitorVar.digit.multi.var[channel].valueN !=0 && 	gSysMonitorVar.digit.multi.var[channel].valueN !=1){
				real6++;
			}
//			gSysMonitorVar.digit.multi.var[channel].valueP = GpioDataRegs.GPBDAT.bit.GPIO59;
//			gSysMonitorVar.digit.multi.var[channel].valueN = GpioDataRegs.GPBDAT.bit.GPIO60;

			SET_DIGIT_SER_CLK_LOW;

			++channel;
			if(channel >= TOTAL_CTRLBRD_MULTI_DIGIT)
			{
				channel = 0;
				status = REFRESH;
			}
			else{
				status = TRIGGER;
			}
			break;
		default:
			status = REFRESH;
			break;
	}
}
/**************************************************************
 *Name:						UpdateSingleDigitInput
 *Function:
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.8.6
 **************************************************************/
void UpdateSingleDigitInput(void){
	int index;

	for(index=0;index<12;++index){
	}

}
/**************************************************************
 *Name:						ReadAnalogValue
 *Function:					更新系统模拟量的转换值
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.8.6
 **************************************************************/
void ReadAnalogValue(void){

    if((AdcRegs.ADCASEQSR.bit.SEQ_CNTR==0)&&
              (AdcRegs.ADCST.bit.SEQ1_BSY==0)){

    }
    else{

    }

//	UpdateSingleAnalogInput();
	AnalogValueInspect();
}
/**************************************************************
 *Name:						ReadDigitalValue
 *Function:					更新系统数字量的转换值
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.8.6
 **************************************************************/
void ReadDigitalValue(void){
	DigitalValueInspect();
	//UpdateSingleDigitInput();
}
