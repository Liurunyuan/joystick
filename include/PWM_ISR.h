
#ifndef _PWM_ISR_H
#define _PWM_ISR_H


#define WAIT_TIMES 		(20)
#define MAX_CHANNEL		(16)

/********calculate the ADC result of ADCINB7**************/
#define CAL_ADCINB7   	((AdcRegs.ADCRESULT15) >> 4)
#define CAL_ADCINB1	    ((AdcRegs.ADCRESULT9) >> 4)

/*****************模拟量多通道BIT检测地址位设置************/
#define SET_AD1K		(GpioDataRegs.GPASET.bit.GPIO30)
#define SET_AD2K		(GpioDataRegs.GPASET.bit.GPIO29)
#define SET_AD3K		(GpioDataRegs.GPCSET.bit.GPIO85)
#define SET_AD4K		(GpioDataRegs.GPBSET.bit.GPIO39)
/***************************************************** ***/

/*****************数字量多通道BIT检测设置******************/
#define GET_DIGIT_SERIAL_N   (GpioDataRegs.GPBDAT.bit.GPIO60)
#define GET_DIGIT_SERIAL_P   (GpioDataRegs.GPBDAT.bit.GPIO59)

#define SET_DIGIT_SER_CLK_HIGH (GpioDataRegs.GPBSET.bit.GPIO52)
#define SET_DIGIT_SER_CLK_LOW  (GpioDataRegs.GPBCLEAR.bit.GPIO52)

#define SET_DIGIT_SER_LOAD_HIGH (GpioDataRegs.GPBSET.bit.GPIO53)
#define SET_DIGIT_SER_LOAD_LOW  (GpioDataRegs.GPBCLEAR.bit.GPIO53)

#define REFRESH (1)
#define LOCK	(2)
#define TRIGGER (3)
#define GETDATA (4)


struct MultiAnalogValue
{
	Uint16 	index;
	int		controlBoardBIT[15];
	int     powerBoardBIT[15];

};
struct AnalogAndDigitalInspect
{
	struct MultiAnalogValue gMultiAnalogValue;
	int DigitalParaToSerial_N[9];
	int DigitalParaToSerial_P[9];
};


/*Switch the analog channel, plus 1 every time*/
void Pwm_ISR_Thread(void);
void SwitchAnalogChannel(int address);

void ReadChannelAdcValue(int index);
int	 IsAdcValueNormal(int index);


#endif
