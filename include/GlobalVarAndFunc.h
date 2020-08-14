#include "DSP2833x_Device.h"
#ifndef _GLOBAL_VAR_AND_FUNC_H
#define _GLOBAL_VAR_AND_FUNC_H


#define INCLUDE_FEATURE 1
#define EXCLUDE_FEATURE 0

#define MACHINE_FRICTION 				INCLUDE_FEATURE
#define ONLY_SPRING 	 				INCLUDE_FEATURE
#define LINEAR_SPEED_METHOD 			EXCLUDE_FEATURE
#define SPEED_CLOSED_LOOP 				INCLUDE_FEATURE
#define TEN_AVERAGE 					EXCLUDE_FEATURE
#define DUTY_GRADUAL_CHANGE 			INCLUDE_FEATURE
#define TARGET_DUTY_GRADUAL_CHANGE 		EXCLUDE_FEATURE
#define COPY_FLASH_CODE_TO_RAM 			EXCLUDE_FEATURE
#define IMPLEMENT_LSM                   EXCLUDE_FEATURE


#define KALMAN_Q  (1.1)
#define KALMAN_R  (157.1)
//#define  DEBOUNCE (0)
//#define START_FORCE_OFFSET (0)
//#define START_FORCE_DUTY (50)
#define ROLL_OFFSET (10)

//#define PI (3.14149265)

#define ROLL 0
#define PITCH 1

#define DUTY_LIMIT_P (275)
#define DUTY_LIMIT_N (-275)

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
	SECTION8 = 8,
	SECTION9 = 9,
	SECTION10 = 10,
	SECTION11 = 11,
	SECTION12 = 12,
	SECTION13 = 13,
	SECTION14 = 14,
	SECTION15 = 15,
	SECTION16 = 16,
	SECTION17 = 17,
	SECTION18 = 18,
	SECTION19 = 19,
	SECTION20 = 20,
	SECTION21 = 21,
	SECTION22 = 22,
	SECTION23 = 23,
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
    double DimL_K; //dimension of length K
    double DimL_B; //dimension of length B
	Uint16 currentHallPosition;
	Uint16 lastTimeHalllPosition;
	int16 duty;
	int16 currentDuty;
	int16 dutyAddInterval;
	int16 ddtmax;
	int16 targetDuty;
	int16 targetDuty_F;
	int16 targetDuty_V;
	double coe_Force;
	double coe_Velocity;
	double coe_Force_Max_ODE;
	double coe_Force_Min_ODE;
	double coe_Velocity_Max_ODE;
	double coe_Velocity_Min_ODE;
	int controlFuncIndex;
	int currentStickDisSection;
	int16 Ki_Threshold_f;
	double Ki_Threshold_d;
	int16 sek_f;
	double sek_d;
	double Ki_Threshold_v;
	double sek_v;
	double zeroForce;
	double velocity_last;
	double Force_Init2Pos_Thr;
	double Force_Init2Neg_Thr;
    double Accel_Init2Pos_Thr;
    double Accel_Init2Neg_Thr;
    double Velocity_Init2Pos_Thr;
    double Velocity_Init2Neg_Thr;
    double Force_Pos_Thr;
    double Force_Neg_Thr;
    double Force_Hysteresis;
    double Accel_Pos_Thr;
    double Accel_Neg_Thr;
    double Accel_Zero2Pos_Thr;
    double Accel_Zero2Neg_Thr;
    double Accel_Hysteresis;
    double Accel_Debounce_Cnt_1;
    double Accel_Debounce_Cnt_2;
    double Velocity_Pos_Thr;
    double Velocity_Neg_Thr;
    double Velocity_Zero2Pos_Thr;
    double Velocity_Zero2Neg_Thr;
    double Velocity_Hysteresis;
    double Velocity_Debounce_Cnt_1;
    double Velocity_Debounce_Cnt_2;
    int board_type;
    int lastStickDisSection;
    double Force_K;
    double Force_B;
    double friction;
//    double ob_Friction;
    double ob_velocityOpenLoop;
    double ob_velocityOpenLoop2;
    int soft_break_flag;
    double springForceK;
    double springForceB;
    double openLoop_Force_front_B;
    double openLoop_Force_rear_B;
    int isEcapRefresh;
    double JoyStickSpeed;
    int rotateDirection;
    int sixButtons;
    int RS422_Rx_Data;
    int software_version;

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
	double LF_MaxForce;
	double LF_Force0;
	double LF_Force1;
	double LF_Force2;
	double LF_Force3;
	double LF_Force4;
	double LF_Force5;
	double LF_Force6;
	double LF_Force7;
	double LF_Force8;
	double LF_Force9;

	double RB_Force0;
	double RB_Force1;
	double RB_Force2;
	double RB_Force3;
	double RB_Force4;
	double RB_Force5;
	double RB_Force6;
	double RB_Force7;
	double RB_Force8;
	double RB_Force9;
	double RB_MaxForce;

	double LF_MaxDistance;
	double LF_Distance0;
	double LF_Distance1;
	double LF_Distance2;
	double LF_Distance3;
	double LF_Distance4;
	double LF_Distance5;
	double LF_Distance6;
	double LF_Distance7;
	double LF_Distance8;
	double LF_Distance9;

	double RB_Distance0;
	double RB_Distance1;
	double RB_Distance2;
	double RB_Distance3;
	double RB_Distance4;
	double RB_Distance5;
	double RB_Distance6;
	double RB_Distance7;
	double RB_Distance8;
	double RB_Distance9;
	double RB_MaxDistance;

	double LF_StartForce;
	double RB_StartForce;

	double LF_FrontFriction;
	double LF_RearFriction;
	int RB_FrontFriction;
	int RB_RearFriction;

	double LF_EmptyDistance;
	double RB_EmptyDistance;

	double Force_Displace_K;

	double dampingFactor;
	double naturalVibrationFreq;

	int equivalentMass;
	int LF_TrimRange;
	int RB_TrimRange;

	int Trim_StepSize;
	int Trim_Speed;

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
	double springForceP[12];
	double springForceN[12];
	double displacementP[12];
	double displacementN[12];

	double K_spring_forceP[12];
	double b_P[12];
	double K_spring_forceN[12];
	double b_N[12];

	int maxPoints;
}FORCE_DISPLACE_CURVE;

typedef struct{
    Uint16 displace;
    Uint16 force;
}ANOLOG16BIT;

typedef void (*UPDATESTATE)(int value);
/************************joystick displacement state **********************/
typedef struct _STICKSTATE{
	double value;
}STICKSTATE;
/***********************Force state*******************************/
//#define FORWARD_FORCE_VALUE (3)
//#define BACKWARD_FORCE_VALUE (-3)

//#define FOWARD_START_FORCE (5)
//#define BACKWARD_START_FORCE (-5)

typedef struct _EXTFORCESTATE
{
	int ForceState; //eForceState
	UPDATESTATE updateForceState;
	double value;
}EXTFORCESTATE;

enum eForceState{
	NO_FORCE = 0,
	BACKWARD_FORCE = 1,
	FORWARD_FORCE = 2,
	INIT_FORCE
};
/*****************************************************************/

enum TRIGGER{
    TK9_TRIGGER = 0,
    AK29_BUTTON,
    FWRD_SWITCH,
    RGHT_SWITCH,
    REAR_SWITCH,
    LEFT_SWITCH
};

enum BTN_STAT{
    BTN_RELEASE = 0,
    BTN_PRESSED = 1,
    BTN_INIT
};

typedef struct _ROTATEDIRECTION{
	int rotateDirection;
	UPDATESTATE updateRotateDirection;
	int debounceCount_1;
	int debounceCount_2;
}ROTATEDIRECTION;

typedef struct _ACCELDIRECTION{
    int accelDirection;
    UPDATESTATE updateAccelDirection;
    int debounceCount_1;
    int debounceCount_2;
}ACCELDIRECTION;

enum eRotateDirection{
	STOP_DIRECTION = 0,
	BACKWARD_DIRECTION  =1,
	FORWARD_DIRECTION = 2,
	INIT_DIRECTION
};

#define ARRAYSIZE (10)
typedef struct _TENAVE{
	double displaceArray[ARRAYSIZE];
	double forceArray[ARRAYSIZE];
	double displaceArrayBak[ARRAYSIZE];
	double forceArrayBak[ARRAYSIZE];
}TENAVE;

extern STICKSTATE gStickState;
extern EXTFORCESTATE gExternalForceState;
extern ROTATEDIRECTION gRotateDirection;
extern ACCELDIRECTION gAccelDirection;

extern Uint32 gECapCount;
//extern int16 gMotorSpeedEcap;
extern RS422STATUS gRS422Status;
extern KeyValue gKeyValue;
extern SYSINFO gSysInfo;
extern SYSSTATE gSysState;
extern SYSPARA gSysPara;
extern SYSCURRENTSTATE gSysCurrentState;
extern CONFIGPARA gConfigPara;
extern FORCE_DISPLACE_CURVE gForceAndDisplaceCurve;
extern double gDebug[3];
extern int gPISO_165[8];
extern int gButtonCmd[6];
extern int gButtonStatus[6];


extern ANOLOG16BIT gAnalog16bit;
extern TENAVE gTenAverageArray;


extern int gCheckStartForceForwardMargin;
extern int gCheckStartForceBackwardMargin;

void checkPitchOrRoll(void);
void InitSysState(void);
void InitConfigParameter(void);
double KalmanFilterRodSpeed(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R);
double KalmanFilter(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R);
double KalmanFilterSpeed(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R);
double KalmanFilterForce(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R);
double KalmanFilterAccel(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R);
void UpdateForceDisplaceCurve(void);
void EnablePwmOutput(void);
void DisablePwmOutput(void);

void StateMachine(void);
void ClearFault(void);
void Enable_PWMD_BK(void);
void Disable_PWMD_BK(void);
void ControleStateMachineSwitch(int value);
void InitGlobalVarAndFunc(void);
double TenDisplaceElemntAverage(void);
void DigitalSignalPISO(void);
void Button_Debounce1(void);
void Button_Debounce2(void);
void Button_Debounce3(void);
void Button_Debounce4(void);
void Button_Debounce5(void);
void Button_Debounce6(void);
void Null_Displacement_Trim(void);
int CheckStickSetion(double val);

#endif
