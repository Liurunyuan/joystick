#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "ADprocessor.h"



int test = 0;

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

struct MULTCH multiCHInspect = {0};



SysMonitorVar gSysMonitorVar;
/**************************************************************
 *Name:						UpdatePowerBoardAnalogInput
 *Function:					���¹��ʰ�����ģ����
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.8.2
 **************************************************************/
void UpdateSingleAnalogInput(void)
{
	int index;
	for(index = 0; index < TotalChannel; ++index)
	{
		gSysMonitorVar.anolog.single.var[index].value = gSysMonitorVar.anolog.single.var[index].updateValue();
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
int IsSingleAnalogValueAbnormal(void)
{
	int index;
	int ret = 1;
	for(index = 0; index <= TotalChannel; ++index)
	{
		if((gSysMonitorVar.anolog.single.var[index].value > gSysMonitorVar.anolog.single.var[index].max) ||
				(gSysMonitorVar.anolog.single.var[index].value < gSysMonitorVar.anolog.single.var[index].min))
		{
			ret = 0;
		}
	}
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
Uint16 AnalogChannelChange(Uint16 address)
{

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
void ReadChannelAdcValue(Uint16 index)
{
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
void SwitchAnalogChannel(Uint16 address)
{
	/*
	 * GPIO30->AD1K
	 * GPIO29->AD2K
	 * GPIO85->AD3K
	 * GPIO39->AD4K
	 *
	 * */
	SET_AD1K = address & 0x0001;
	asm(" nop");


	SET_AD2K = (address & 0x0002) >> 1;
	//SET_AD3K = (address & 0x0004) >> 2;
    //SET_AD4K = (address & 0x0008) >> 3;
}
/**************************************************************
 *Name:						AnalogValueInspect
 *Function:					ģ������ͨ��Ѳ�캯��
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.7.30
 **************************************************************/
void AnalogValueInspect(void)
{
    static Uint16 address = 0;
    if(AdcConversionUnStable())
    {
    	return;
    }
    else
    {
        ReadChannelAdcValue(address);
    	address = AnalogChannelChange(address);
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
void DigitalValueInspect(void)
{
	static int status = 1;
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
			SET_DIGIT_SER_CLK_HIGH;
			status = GETDATA;
			break;
		case GETDATA:
			gSysMonitorVar.digit.multi.var[channel].valueP = GET_DIGIT_SERIAL_P;
			gSysMonitorVar.digit.multi.var[channel].valueN = GET_DIGIT_SERIAL_N;

			SET_DIGIT_SER_CLK_LOW;
			++channel;
			if(channel >= 9)
			{
				status = REFRESH;
			}
			else
				status = TRIGGER;
			break;
		default:
			status = REFRESH;
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
void UpdateSingleDigitInput(void)
{
	int index;
	for(index=0;index<12;++index)
	{
		gSysMonitorVar.digit.single.var[index].valueP = gSysMonitorVar.digit.single.var[index].updateValue();
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
void ReadAnalogValue(void)
{
	UpdateSingleAnalogInput();
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
void ReadDigitalValue(void)
{
	DigitalValueInspect();
	UpdateSingleDigitInput();
	//read single digital channel value
}
