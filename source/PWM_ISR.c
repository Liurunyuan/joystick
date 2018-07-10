#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "PWM_ISR.h"



struct BIT_AUX2_AN bit_aux2_an = {0};
const Uint16 threshold[20] = {0};
Uint16 address = 0;



void Pwm_ISR_Thread(void)
{
	static Uint16 count = 0;

	++count;
	if(count > WAIT_TIMES)
	{
		count = 0;

		ReadChannelAdcValue(address);
		IsAdcValueNormal(address);
		SwitchAnalogChannel();

	}
}

void ReadChannelAdcValue(int index)
{
	bit_aux2_an.X[index] = CAL_ADCINB7;
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
	SET_AD1K = address & 0x0001;
	SET_AD2K = address & 0x0002;
	SET_AD3K = address & 0x0003;
    SET_AD4K = address & 0x0004;

	address++;

	if(address >= MAX_CHANNEL)
	{
		address = 0;
	}
}


int	 IsAdcValueNormal(int index)
{
	/*TODO realize this function****************************************/
	if(bit_aux2_an.X[index] > threshold[index])
	{
		/*set error flag*/
	}
	return 0;
}
