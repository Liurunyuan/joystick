#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "ADprocessor.h"

SysMonitorVar gSysMonitorVar;

/********update anolog variable value******************************/
int updateForceValue(void){return GET_FORCE_SGN;}
int updateBusCurrentP(void){return GET_BUS_CURRENT_P;}
int updatePower28V_M(void){return GET_28V_M;}
int updateBridgeCurrentB(void){return GET_B_BRIDGE_CURRENT;}
int updateBusCurrentB(void){return GET_B_BUS_CURRENT;}
int updatePower28V(void){return GET_28V;}
int updateBridgeCurrentA(void){return GET_A_BRIDGE_CURRENT;}
int updateBusCurrentA(void){return GET_A_BUS_CURRENT;}
int updateDisplacementValue(void){return GET_DISPLACEMENT_SGN;}
int updateBridgeCurrentC(void){return GET_C_BRIDGE_CURRENT;}
int updateBusCurrentC(void){return GET_C_BUS_CURRENT;}
/******************************************************************/


/********update anolog variable value******************************/
//int updateForceValue(void){return DMABuf1[0];}
//int updateBusCurrentP(void){return DMABuf1[1];}
//int updatePower28V_M(void){return DMABuf1[4];}
//int updateBridgeCurrentB(void){return DMABuf1[3];}
//int updateBusCurrentB(void){return DMABuf1[2];}
//int updatePower28V(void){return DMABuf1[10];}
//int updateBridgeCurrentA(void){return DMABuf1[6];}
//int updateBusCurrentA(void){return DMABuf1[7];}
//int updateDisplacementValue(void){return DMABuf1[8];}
//int updateBridgeCurrentC(void){return DMABuf1[9];}
//int updateBusCurrentC(void){return DMABuf1[5];}
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

const Uint16 anologMaxMinInit[][4] = {
        //{max,2ndmax,min,2ndmin}
		{0,0,0,0},
		{1,0,0,0},
		{2870,0,2548,0},
		{3,0,0,0},
		{4,0,0,0},
		{3500,0,3000,0},
		{6,0,0,0},
		{7,0,0,0},
		{2690,2400,861,1000},
		{9,0,0,0},
		{10,0,0,0},
		{11,0,0,0}
};

const Uint16 AD16bitMaxMinInit[][4] = {
        //{max,2ndmax,min,2ndmin}
        {0,0,0,0},
        {42294,40000,13378,14000},
        {2,0,0,0}
};

/**************************************************************
 *Name:						UpdatePowerBoardAnalogInput
 *Function:					���¹��ʰ�����ģ����
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
 *Function:					�ж���ͨ��ģ�����Ƿ�Խ��
 *Input:					none
 *Output:					return 1����Խ�� return 0����û��Խ��
 *Author:					Simon
 *Date:						2018.8.2
 **************************************************************/
int IsSingleAnalogValueAbnormal(void){
	int index;
	int ret = 1;
	for(index = 0; index < TotalChannel; ++index){
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
 *Date:		   2018��12��18������9:02:38
 **************************************************************/
int IsCommonAnalogValueAbnormal(void){

	//TODO need to impletment later
	int ret = 0;

	return ret;
}
/**************************************************************
 *Name:						AdcConversionUnStable
 *Function:					�ж�ģ������ͨ���л��Լ�ת���Ƿ��ȶ�
 *Input:					none
 *Output:					return 1�������β��ȶ� return 0����ת���Ѿ��ȶ�
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
 *Function:					ģ������ͨ����ַ�ı�
 *Input:					�ϴζ�ͨ��ģ������ֵַ
 *Output:					���ض�ͨ��ģ������ֵַ
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
 *Function:					��ȡDSP��ADCת�����
 *Input:					ͨ��ֵ
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
 *Function:					�л���ͨ��ģ�����ĵ�ֵַ
 *Input:					����Ҫ���õĶ�ͨ��ģ�����ĵ�ֵַ
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
 *Function:					ģ������ͨ��Ѳ�캯��
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
 *Function:					��������ͨ��Ѳ�캯��
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
			}
			if(	gSysMonitorVar.digit.multi.var[channel].valueN !=0 && 	gSysMonitorVar.digit.multi.var[channel].valueN !=1){
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
 *Function:					����ϵͳģ������ת��ֵ
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

	//UpdateSingleAnalogInput();
	AnalogValueInspect();
}
/**************************************************************
 *Name:						ReadDigitalValue
 *Function:					����ϵͳ��������ת��ֵ
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.8.6
 **************************************************************/
void ReadDigitalValue(void){
	DigitalValueInspect();
	//UpdateSingleDigitInput();
}
