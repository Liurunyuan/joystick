#ifndef _GLOBAL_VAR_AND_FUNC_H
#define _GLOBAL_VAR_AND_FUNC_H

typedef Uint16 bool;

typedef struct{
	Uint16 rs422A;
	Uint16 rs422B;
	Uint16 currentSerialNumber;
	Uint16 rs422CurrentChannel;
	Uint16 shakeHand;
}RS422STATUS;

extern Uint32 gECapCount;
extern RS422STATUS gRS422Status;


#endif
