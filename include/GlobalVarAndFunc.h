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

typedef struct _KeyValue{
	int32 force;
	int32 displacement;
	int32 motorSpeed;
	int32 motorAccel;
	int32 lock;
}KeyValue;

typedef struct{
	Uint16 currentHallPosition;
	Uint16 lastTimeHalllPosition;
	int16 duty;
}SYSINFO;

extern Uint32 gECapCount;
extern RS422STATUS gRS422Status;
extern KeyValue gKeyValue;
extern SYSINFO gSysInfo;


#endif
