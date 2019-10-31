#ifndef _CTLSTRATEGY_
#define _CTLSTRATEGY_


void PidProcess(void);
int RKT(double x, double y, double z, double h);
void OnlyWithSpringRear(void);
void OnlyWithSpringFront(void);
void BounceBackFront (void);

#endif
