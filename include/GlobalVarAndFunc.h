#ifndef _GLOBAL_VAR_AND_FUNC_H
#define _GLOBAL_VAR_AND_FUNC_H

#define KALMAN_Q  (1.1)
#define KALMAN_R  (157.2)

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
	int16 duty;
	int16 currentDuty;
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
	Uint16 overCurrent : 1;
	Uint16 rs422RxQFull : 1;
	Uint16 overBusVoltage : 1;
	Uint16 overTemperature : 1;
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

typedef struct{
	int LF_MaxForce;
	int LF_Force1;
	int LF_Force2;
	int LF_Force3;
	int LF_Force4;
	int LF_Force5;
	int LF_Force6;
	int LF_Force7;
	int LF_Force8;
	int LF_Force9;


	int RB_Force1;
	int RB_Force2;
	int RB_Force3;
	int RB_Force4;
	int RB_Force5;
	int RB_Force6;
	int RB_Force7;
	int RB_Force8;
	int RB_Force9;
	int RB_MaxForce;

	int LF_MaxDistance;
	int LF_Distance1;
	int LF_Distance2;
	int LF_Distance3;
	int LF_Distance4;
	int LF_Distance5;
	int LF_Distance6;
	int LF_Distance7;
	int LF_Distance8;
	int LF_Distance9;


	int RB_Distance1;
	int RB_Distance2;
	int RB_Distance3;
	int RB_Distance4;
	int RB_Distance5;
	int RB_Distance6;
	int RB_Distance7;
	int RB_Distance8;
	int RB_Distance9;
	int RB_MaxDistance;

	int LF_StartForce;
	int RB_StartForce;

	int LF_FrontFriction;
	int LF_RearFriction;
	int RB_FrontFriction;
	int RB_RearFriction;

	int LF_EmptyDistance;
	int RB_EmptyDistance;

	int dampingFactor;
	int naturalVibrationFreq;

	int equivalentMass;
	int LF_TrimRange;
	int RB_TrimRange;

	int trimTarget;
	int trimCommand;

	int timeDelay;
	int stateCommand;

	int innerMaxKp;
	int innerErrorThresholdWithInnerMaxKp;
	int innerKi;
	int innerKd;
	int innerFeedForward;
	int innerMaxStartError;
	int innerMinStartError;
	int innerMaxIntergralSaturation;
	int innerMinIntergralSaturation;

	int middleKp;
	int middleKi;
	int middleKd;

	int outerKp;
	int outerKi;
	int outerKd;


}CONFIGPARA;


typedef struct{
	int springForceP[10];
	int springForceN[10];
	int displacementP[10];
	int displacementN[10];

	double K_spring_forceP[10];
	double b_P[10];
	double K_spring_forceN[10];
	double b_N[10];

	int maxPoints;


}FORCE_DISPLACE_CURVE;

typedef struct{
    Uint16 displace;
    Uint16 force;
}ANOLOG16BIT;



/******joystick displacement state **********************/

#define OOR_FORWARD_NULL_DIS_VAL 			(1)
#define IR_FORWARD_NULL_DIS_VAL				(1)

#define OOR_BACKWARD_NULL_DIS_VAL			(1)
#define IR_BACKWARD_NULL_DIS_VAL			(1)

#define OOR_FORWARD_THRESHOLD_DIS_VAL		(1)
#define IR_FORWARD_THRESHOLD_DIS_VAL		(1)

#define OOR_BACKWARD_THRESHOLD_DIS_VAL		(1)
#define IR_BACKWARD_THRESHOLD_DIS_VAL		(1)

typedef void (*UPDATESTATE)(int value);

typedef struct _STICKSTATE{
	int NullDistanceForwardState;
	int NullDistanceBackwardState;
	int ThresholdForwaredState;
	int ThresholdBackwardState;
	UPDATESTATE updateNullDisForwardState;
	UPDATESTATE updateNullDisBackwardState;
	UPDATESTATE updateThresholdDisForwardState;
	UPDATESTATE updateThresholdDisBackwardState;
	Uint16 value;
}STICKSTATE;

enum eNullDistancedState{
	IR_NULL_DIS = 0, 
	OOR_NULL_DIS = 1,
	INIT_NULL_DIS
};
enum eThreasholdDistancedState{
	IR_THRESHOLD_DIS = 0,
	OOR_THRESHOLD_DIS = 1,
	INIT_THRESHOLD_DIS
};





/*******************************************************/
extern STICKSTATE gStickState;

extern Uint32 gECapCount;
extern RS422STATUS gRS422Status;
extern KeyValue gKeyValue;
extern SYSINFO gSysInfo;
extern SYSSTATE gSysState;
extern SYSPARA gSysPara;
extern SYSCURRENTSTATE gSysCurrentState;
extern CONFIGPARA gConfigPara;
extern FORCE_DISPLACE_CURVE gForceAndDisplaceCurve;

extern ANOLOG16BIT gAnalog16bit;

extern int gforwardOverLimit;
extern int gbackwardOverLimit;
extern int gforwardForce;
extern int gbackwardForce;
extern int gNoExternalForce;




extern int gCheckStartForceForwardMargin;
extern int gCheckStartForceBackwardMargin;



void InitSysState(void);
void InitConfigParameter(void);
double KalmanFilter(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R);
double KalmanFilterSpeed(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R);
void UpdateForceDisplaceCurve(void);
void EnablePwmOutput(void);
void DisablePwmOutput(void);

void StateMachine(void);
void ClearFault(void);
void Enable_PWMD_BK(void);
void Disable_PWMD_BK(void);


#endif
