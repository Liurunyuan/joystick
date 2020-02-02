#ifndef _TIMER_ISR_H
#define _TIMER_ISR_H

void rs422aPack();
void rs422bPack();
void rs422aTx();
void rs422bTx();

void MotorSpeed();

void Timer0_ISR_Thread(void);
void Timer1_ISR_Thread(void);
#endif
