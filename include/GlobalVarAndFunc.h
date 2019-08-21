#ifndef _GLOBAL_VAR_AND_FUNC_H
#define _GLOBAL_VAR_AND_FUNC_H

#define KALMAN_Q  (1.1)
#define KALMAN_R  (157.2)

#define DIS_DIMENSION_K (0.0007527)
#define DIS_DIMENSION_B (-7.813)
#define FORCE_DIMENSION_K (0.014027)
#define FORCE_DIMENSION_B (-459.6276)
#define PI (3.14149265)


#define BIT_0 (0x00000001)
#define BIT_1 (0x00000002)
#define BIT_2 (0x00000004)
#define BIT_3 (0x00000008)
#define BIT_4 (0x00000010)
#define BIT_5 (0x00000020)
#define BIT_6 (0x00000040)
#define BIT_7 (0x00000080)
#define BIT_8 (0x00000100)
#define BIT_9 (0x00000200)
#define BIT_10 (0x00000400)
#define BIT_11 (0x00000800)
#define BIT_12 (0x00001000)
#define BIT_13 (0x00002000)
#define BIT_14 (0x00004000)
#define BIT_15 (0x00008000)
#define BIT_16 (0x00010000)
#define BIT_17 (0x00020000)
#define BIT_18 (0x00040000)
#define BIT_19 (0x00080000)
#define BIT_20 (0x00100000)
#define BIT_21 (0x00200000)
#define BIT_22 (0x00400000)
#define BIT_23 (0x00800000)
#define BIT_24 (0x01000000)
#define BIT_25 (0x02000000)
#define BIT_26 (0x04000000)
#define BIT_27 (0x08000000)
#define BIT_28 (0x10000000)
#define BIT_29 (0x20000000)
#define BIT_30 (0x40000000)
#define BIT_31 (0x80000000)

enum eSTICK_DIS_SECTION{
	SECTION0 = 0,
	SECTION1 = 1,
	SECTION2 = 2,
	SECTION3 = 3,
	SECTION4 = 4,
	SECTION5 = 5,
	SECTION6 = 6,
	SECTION7 = 7,
	INIT_SECTION 
};


enum CONTROL_STATE_MACHINE{
	IR_NULL_DIS_AND_NO_FORCE = 0,
	IR_NULL_DIS_AND_FORWARD_FORCE = 0,
	OOR_NULL_DIS_AND_NO_FORCE = 0,
	THREE = 3,
	FOUR = 4,
	FIVE = 5,
	SIX = 6,
	SEVEN = 7,
	EGIGHT = 8,
	NINE = 9,
	TEN = 10,
	ELVENEN = 11,
	TWELVE = 12,
	THRITEEN = 13,
	FOURTEEN = 14,
	FIFTEEN = 15,
	SIXTEEN = 16,
	SEVENTEEN = 17,
	EIGHTTEEN = 18,
	NINETEEN = 19,
	TWENTY = 20,
	TWENTY_ONE = 21,
	TEWENTY_TWO = 22,
	END
};



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
	int16 dutyAddInterval;
	int16 ddtmax;
	int16 targetDuty;
	int controlFuncIndex;
	int currentStickDisSection;
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



/************************joystick displacement state **********************/

#define BASE_ZERO_DIS						(27836)

#define OOR_FORWARD_NULL_DIS_VAL 			(18000)
#define IR_FORWARD_NULL_DIS_VAL				(20000)

#define OOR_BACKWARD_NULL_DIS_VAL			(35000)
#define IR_BACKWARD_NULL_DIS_VAL			(33000)


#define OOR_FORWARD_START_FORCE_DIS_VAL_MAX	(OOR_FORWARD_NULL_DIS_VAL + 100)
#define OOR_FORWARD_START_FORCE_DIS_VAL_MIN	(OOR_FORWARD_NULL_DIS_VAL + 100)
#define OOR_BACKWARD_START_FORCE_DIS_VAL_MAX	(OOR_FORWARD_NULL_DIS_VAL + 100)
#define OOR_BACKWARD_START_FORCE_DIS_VAL_MIN	(OOR_FORWARD_NULL_DIS_VAL + 100)
#define IR_FORKWARD_START_FORCE_DIS_VAL		(0)
#define OOR_BACKWARD_START_FORCE_DIS_VAL	(0)
#define IR_BACKWARD_START_FORCE_DIS_VAL		(0)

#define OOR_FORWARD_THRESHOLD_DIS_VAL		(13000)
#define IR_FORWARD_THRESHOLD_DIS_VAL		(15000)

#define OOR_BACKWARD_THRESHOLD_DIS_VAL		(42000)
#define IR_BACKWARD_THRESHOLD_DIS_VAL		(40000)

typedef void (*UPDATESTATE)(int value);

typedef struct _STICKSTATE{
	int NullDistanceForwardState;
	int NullDistanceBackwardState;
	int StartForceForwardState;
	int StartForceBackwardState;
	int ThresholdForwaredState;
	int ThresholdBackwardState;
	UPDATESTATE updateNullDisForwardState;
	UPDATESTATE updateNullDisBackwardState;

	UPDATESTATE updateStartForceDisBackwardState;
	UPDATESTATE updateStartForceBackwardState;

	UPDATESTATE updateThresholdDisForwardState;
	UPDATESTATE updateThresholdDisBackwardState;
	//Uint16 value;
	double value;
}STICKSTATE;

enum eNullDistancedState{
	IR_NULL_DIS = 0, 
	OOR_NULL_DIS = 1,
	INIT_NULL_DIS
};
enum eStartForceDistancedState{
	IR_START_FORCE_DIS = 0, 
	OOR_START_FORCE_DIS = 1,
	INIT_START_FORCE_DIS
	
};
enum eThreasholdDistancedState{
	IR_THRESHOLD_DIS = 0,
	OOR_THRESHOLD_DIS = 1,
	INIT_THRESHOLD_DIS
};

/***********************Force state*******************************/

#define FORWARD_FORCE_VALUE (-0.5)
#define BACKWARD_FORCE_VALUE (0.5)

#define FOWARD_START_FORCE (-50)
#define BACKWARD_START_FORCE (50)

typedef struct _EXTFORCESTATE
{
	int ForceState;
	UPDATESTATE updateForceState;
	//Uint16 value;
	double value;
}EXTFORCESTATE;

enum eForceState{
	NO_FORCE = 0,
	BACKWARD_FORCE = 1,
	FORWARD_FORCE = 2,
	INIT_FORCE
};


/*****************************************************************/
extern STICKSTATE gStickState;
extern EXTFORCESTATE gExternalForceState;

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

extern Uint32 gSysStateMachineNumber;




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
void ControleStateMachineSwitch(int value);
void InitGlobalVarAndFunc(void);
int LocateStickDisSection(void);


#endif
