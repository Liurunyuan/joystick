#ifndef _ECAP_ISR_H
#define _ECAP_ISR_H



void ECap4_Isr(void);
void ECap5_Isr(void);
void ECap6_Isr(void);
double CalculateSpeed(Uint32 capCount);
void InitEcapVar(void);
extern volatile double gMotorSpeedEcap;
#endif
