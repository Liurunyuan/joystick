



#ifndef _PWM_ISR_H
#define _PWM_ISR_H


#define WAIT_TIMES 		(20)
#define MAX_CHANNEL		(16)


struct BIT_AUX2_AN
{
	Uint16 	index;
	int		X[15];

};
/*Switch the analog channel, plus 1 every time*/
void Pwm_isr(void);
void SwitchAnalogChannel(void);
void ReadChannelAdcValue(void);
int	 IsAdcValueNormal(void);


#endif
