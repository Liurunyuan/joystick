#ifndef _PWM_ISR_H
#define _PWM_ISR_H


/*Switch the analog channel, plus 1 every time*/
void Pwm_ISR_Thread(void);

enum COMPARE
{
    LESS_THAN = 0,
    LARGE_THAN
};

extern Uint16 real3;
extern Uint16 GetCurrentHallValue(void);

void UpdateKeyValue(void);
#endif
