#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "PWM_ISR.h"


struct BIT_AUX2_AN bit_aux2_an = {0};
Uint16 address   = 0;

void Pwm_isr(void)
{
	static Uint16 count = 0;

	count++;
	if(count > WAIT_TIMES)
	{
		SwitchAnalogChannel();
		ReadChannelAdcValue();
		IsAdcValueNormal();

	}
}

void ReadChannelAdcValue(void)
{
	bit_aux2_an.X[address] = (AdcRegs.ADCRESULT15) >> 4;//A通道A相电流
}
/*switch analog channel, plus 1 every time*/
void SwitchAnalogChannel(void)
{
	/*
	 * GPIO30->AD1K
	 * GPIO29->AD2K
	 * GPIO85->AD3K
	 * GPIO39->AD4K
	 *
	 * */

	GpioDataRegs.GPASET.bit.GPIO30			= address & 0x0001;
	GpioDataRegs.GPASET.bit.GPIO29			= address & 0x0002;
	GpioDataRegs.GPCSET.bit.GPIO85			= address & 0x0003;
	GpioDataRegs.GPBSET.bit.GPIO39			= address & 0x0004;

	address++;
	if(address >= MAX_CHANNEL)
	{
		address = 0;
	}
}


int	 IsAdcValueNormal(void)
{
	/*tbd****************************************/
	return 0;
}
