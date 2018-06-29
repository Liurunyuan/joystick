
#ifndef _PWM_ISR_H
#define _PWM_ISR_H


#define WAIT_TIMES 		(20)
#define MAX_CHANNEL		(16)

/********calculate the ADC result of ADCINB7**************/
#define CAL_ADCINB7   	((AdcRegs.ADCRESULT15) >> 4)
/*****************多通道BIT检测地址位设置******************/
#define SET_AD1K		(GpioDataRegs.GPASET.bit.GPIO30)
#define SET_AD2K		(GpioDataRegs.GPASET.bit.GPIO29)
#define SET_AD3K		(GpioDataRegs.GPCSET.bit.GPIO85)
#define SET_AD4K		(GpioDataRegs.GPBSET.bit.GPIO39)
/********************************************************/
struct BIT_AUX2_AN
{
	Uint16 	index;
	int		X[15];

};
/*Switch the analog channel, plus 1 every time*/
void Pwm_ISR_Thread(void);
void SwitchAnalogChannel(void);
void ReadChannelAdcValue(void);
int	 IsAdcValueNormal(void);


#endif
