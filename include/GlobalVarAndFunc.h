#ifndef _GLOBAL_VAR_AND_FUNC_H
#define _GLOBAL_VAR_AND_FUNC_H

#define KALMAN_Q  (1.1)
#define KALMAN_R  (57.2)

typedef Uint16 bool;

typedef struct{
	Uint16 high8bit : 8;
	Uint16 low8bit	: 8;
}VAR16BIT;
typedef union{
	VAR16BIT data;
	Uint16 all;
}PACK;
typedef struct{
	Uint16 rs422A;
	Uint16 rs422B;
	Uint16 currentSerialNumber;
	Uint16 rs422CurrentChannel;
	Uint16 shakeHand;
}RS422STATUS;

typedef struct _KeyValue{
	double force;
	double displacement;
	double motorSpeed;
	double motorAccel;
	int32 lock;
}KeyValue;

typedef struct{
	Uint16 currentHallPosition;
	Uint16 lastTimeHalllPosition;
	Uint16 sdoStatus;
	Uint16 duty;
}SYSINFO;




typedef struct{
	Uint16 software : 1;
	Uint16 b : 1;
	Uint16 c : 1;
	Uint16 d : 1;
	Uint16 e : 1;
	Uint16 f : 1;
	Uint16 g : 1;
	Uint16 h : 1;
	Uint16 i : 1;
	Uint16 j : 1;
	Uint16 k : 1;
	Uint16 l : 1;
	Uint16 m : 1;
	Uint16 n : 1;
	Uint16 o : 1;
	Uint16 p : 1;
}ERROBIT;


typedef union{
	Uint16 all;
	VAR16BIT data;
	ERROBIT bit;
}SYSERRO;
/*************************************/
typedef struct{
	Uint16 a : 1;
	Uint16 b : 1;
	Uint16 c : 1;
	Uint16 d : 1;
	Uint16 e : 1;
	Uint16 f : 1;
	Uint16 g : 1;
	Uint16 h : 1;
	Uint16 i : 1;
	Uint16 j : 1;
	Uint16 k : 1;
	Uint16 l : 1;
	Uint16 m : 1;
	Uint16 n : 1;
	Uint16 o : 1;
	Uint16 p : 1;
}WARNINGBIT;

typedef union{
	Uint16 all;
	VAR16BIT data;
	WARNINGBIT bit;
}SYSWARNING;
/*************************************/
typedef struct{
	Uint16 a : 1;
	Uint16 rs422RxQFull : 1;
	Uint16 c : 1;
	Uint16 d : 1;
	Uint16 e : 1;
	Uint16 f : 1;
	Uint16 g : 1;
	Uint16 h : 1;
	Uint16 i : 1;
	Uint16 j : 1;
	Uint16 k : 1;
	Uint16 l : 1;
	Uint16 m : 1;
	Uint16 n : 1;
	Uint16 o : 1;
	Uint16 p : 1;
}ALARMBIT;

typedef union{
	Uint16 all;
	VAR16BIT data;
	ALARMBIT bit;
}SYSALARM;
/*************************************/

typedef struct{
	SYSALARM alarm;
	SYSWARNING warning;
	SYSERRO erro;
}SYSSTATE;


typedef struct{
	double k_springForce;
	double k_dampForce;
	double mass;

}SYSPARA;

typedef struct{
	double springForce;
	double dampForce;
	double accTarget;
	double speedTarget;
	double displaceTarget;
	double errAcc;
	double errSpeed;
	double errDisplacement;
}SYSCURRENTSTATE;

extern Uint32 gECapCount;
extern RS422STATUS gRS422Status;
extern KeyValue gKeyValue;
extern SYSINFO gSysInfo;
extern SYSSTATE gSysState;
extern SYSPARA gSysPara;
extern SYSCURRENTSTATE gSysCurrentState;


void InitSysState(void);
double KalmanFilter(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R);


#endif
