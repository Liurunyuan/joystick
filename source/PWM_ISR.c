#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "PWM_ISR.h"



struct AnalogAndDigitalInspect gAnalogAndDigitalInspect = {0};
struct PowerBoardAnalogInput gPowerBoardAnalogInput = {0};


void UpdatePowerBoardAnalogInput(void)
{
	gPowerBoardAnalogInput.forceValue 		= GET_FORCE_SGN;
	gPowerBoardAnalogInput.busCurrentPos 	= GET_BUS_CURRENT_P;
	gPowerBoardAnalogInput.power28V_M 		= GET_28V_M;
	gPowerBoardAnalogInput.bridgeCurrentB 	= GET_B_BRIDGE_CURRENT;
	gPowerBoardAnalogInput.busCurrentB 		= GET_B_BUS_CURRENT;
	gPowerBoardAnalogInput.power28V 		= GET_28V;
	gPowerBoardAnalogInput.bridgeCurrentA 	= GET_A_BRIDGE_CURRENT;
	gPowerBoardAnalogInput.busCurrentA 		= GET_A_BUS_CURRENT;
	gPowerBoardAnalogInput.displacementValue= GET_DISPLACEMENT_SGN;
	gPowerBoardAnalogInput.bridgeCurrentC 	= GET_C_BRIDGE_CURRENT;
	gPowerBoardAnalogInput.bridgeCurrentC 	= GET_C_BUS_CURRENT;
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
int AnologChannelChange(int address)
{

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
void ReadChannelAdcValue(int index)
{
	gAnalogAndDigitalInspect.MultiAnalogValue.controlBoardBIT[index] = GET_ADCINB7;
	gAnalogAndDigitalInspect.MultiAnalogValue.powerBoardBIT[index] = GET_ADCINB1;
}
/**************************************************************
 *Name:						SwitchAnalogChannel
 *Function:					切换多通道模拟量的地址值
 *Input:					本次要设置的多通道模拟量的地址值
 *Output:					none
 *Author:					Simon
 *Date:						2018.7.30
 **************************************************************/
void SwitchAnalogChannel(int address)
{
	/*
	 * GPIO30->AD1K
	 * GPIO29->AD2K
	 * GPIO85->AD3K
	 * GPIO39->AD4K
	 *
	 * */
	SET_AD1K = address & 0x0001;
	SET_AD2K = address & 0x0002;
	SET_AD3K = address & 0x0003;
    SET_AD4K = address & 0x0004;
}
/**************************************************************
 *Name:						AnalogValueInspect
 *Function:					模拟量多通道巡检函数
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.7.30
 **************************************************************/
void AnalogValueInspect(void)
{
    static int address = 0;
    if(AdcConversionUnStable())
    {
    	return;
    }
    ReadChannelAdcValue(address);
	address = AnologChannelChange(address);
    SwitchAnalogChannel(address);
}





/**************************************************************
 *Name:						DigitalValueInspect
 *Function:					数字量多通道巡检函数
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
			gAnalogAndDigitalInspect.DigitalParaToSerial_N[channel] = GET_DIGIT_SERIAL_N;
			gAnalogAndDigitalInspect.DigitalParaToSerial_P[channel] = GET_DIGIT_SERIAL_P;
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
 *Name:						Pwm_ISR_Thread
 *Function:					PWM interrupt function
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.6.10
 **************************************************************/
void Pwm_ISR_Thread(void)
{
	//TODO
	AnalogValueInspect();
	UpdatePowerBoardAnalogInput();
}
