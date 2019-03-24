#ifndef _ADPROCESSOR_H
#define _ADPROCESSOR_H



#define WAIT_TIMES 		(40)
#define MAX_CHANNEL		(16)

/********calculate the ADC result of ADCINB7**************/
#define GET_ADCINB7   	((AdcRegs.ADCRESULT15) >> 4)
#define GET_ADCINB1	    ((AdcRegs.ADCRESULT9) >> 4)

/*****************模拟量多通道BIT检测地址位设置************/
#define SET_AD1K		(GpioDataRegs.GPADAT.bit.GPIO30)
#define SET_AD2K		(GpioDataRegs.GPADAT.bit.GPIO29)
#define SET_AD3K		(GpioDataRegs.GPCDAT.bit.GPIO85)
#define SET_AD4K		(GpioDataRegs.GPBDAT.bit.GPIO39)
/*********************************************************/

/*****************数字量多通道BIT检测设置******************/
#define GET_DIGIT_SERIAL_N   (GpioDataRegs.GPBDAT.bit.GPIO60)
#define GET_DIGIT_SERIAL_P   (GpioDataRegs.GPBDAT.bit.GPIO59)

#define SET_DIGIT_SER_CLK_HIGH (GpioDataRegs.GPBSET.bit.GPIO52 = 1)
#define SET_DIGIT_SER_CLK_LOW  (GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1)

#define SET_DIGIT_SER_LOAD_HIGH (GpioDataRegs.GPBSET.bit.GPIO53 = 1)
#define SET_DIGIT_SER_LOAD_LOW  (GpioDataRegs.GPBCLEAR.bit.GPIO53 = 1)

/***********************紧急功率板输出模拟量****************/
#define GET_FORCE_SGN				((AdcRegs.ADCRESULT0) >> 4)
#define GET_BUS_CURRENT_P			((AdcRegs.ADCRESULT2) >> 4)
#define GET_28V_M					((AdcRegs.ADCRESULT4) >> 4)
#define GET_B_BRIDGE_CURRENT		((AdcRegs.ADCRESULT6) >> 4)
#define GET_B_BUS_CURRENT			((AdcRegs.ADCRESULT8) >> 4)
#define GET_28V						((AdcRegs.ADCRESULT10) >> 4)
#define GET_A_BRIDGE_CURRENT		((AdcRegs.ADCRESULT12) >> 4)
#define GET_A_BUS_CURRENT			((AdcRegs.ADCRESULT14) >> 4)
#define GET_DISPLACEMENT_SGN		((AdcRegs.ADCRESULT1) >> 4)
#define GET_C_BRIDGE_CURRENT		((AdcRegs.ADCRESULT7) >> 4)
#define GET_C_BUS_CURRENT			((AdcRegs.ADCRESULT9) >> 4)
/************************************************************/
#define MAX_FORCE							(2048)
#define MIN_FORCE							(2048)

#define MAX_BUS_CURRENT_P					(2048)
#define MIN_BUS_CURRENT_P					(2048)

#define MAX_28V_M							(2048)
#define MIN_28V_M							(2048)

#define MAX_B_BRIDGE_CURRENT				(2048)
#define MIN_B_BRIDGE_CURRENT				(2048)

#define MAX_B_BUS_CURRENT					(2048)
#define MIN_B_BUS_CURRENT					(2048)

#define MAX_28V								(2048)
#define MIN_28V								(2048)

#define MAX_A_BRIDGE_CURRENT				(2048)
#define MIN_A_BRIDGE_CURRENT				(2048)

#define MAX_A_BUS_CURRENT					(2048)
#define MIN_A_BUS_CURRENT					(2048)

#define MAX_DISPLACEMENT					(2048)
#define MIN_DISPLACEMENT					(2048)

#define MAX_C_BRIDGE_CURRENT				(2048)
#define MIN_C_BRIDGE_CURRENT				(2048)

#define MAX_C_BUS_CURRENT					(2048)
#define MIN_C_BUS_CURRENT					(2048)

#define CURRENT_ABNORMAL_COUNT				(10)
#define VOLTAGE_ABNORMAL_COUNT				(10)
#define TEMP_ABNORMAL_COUNT				(10)


enum Status{
	/*
	 * U38时序切换状态机
	 */
	REFRESH = 1,
	LOCK,
	TRIGGER,
	GETDATA
};
/**************************功率板模拟量数字量定义******************************************/
enum MULTCH_CTRLBRD_ANAL_IDX
{
	/* U36:CD47HC4067M  控制板模拟量多通道切换芯片
	 * X0:	AGND
	 * X1:	2V5
	 * X2:	REF_2v5
	 * X3:	BIT_PN_S
	 * X4:	BIT_5V+2V5
	 * X5:	HAND_3V3_2
	 * X6:	HAND_3V3_3
	 * X7:	VCC_1.8V
	 * X8:	2.5V_IMON_AN
	 * X9:	2.5V_TEMP_AN
	 * X10:	3V3_IMON_AN
	 * X11:	3V3_TEMP_AN
	 * X12:	12V_IMON_AN
	 * X13:	12V_TEMP_AN
	 * X14:	1V9_TEMP_AN
	 * X15:	1V9_IMON_AN
	 */
	AGND1 = 0,				//0		code value: 0
	SEN_5V,					//1		code value: x
	REF_5V_SEN,				//2		code value: x
	BIT_PN_S,				//3		code value: x
	BIT_5V_2V5,				//4		code value: x
	HAND_3V3_2,				//5		code value: x
	HAND_3V3_3,				//6		code value: x
	VCC_1V8,				//7		code value: x
	IMON_AN_2V5,			//8		code value: x
	TEMP_AN_2V5,			//9		code value: x
	IMON_AN_3V3,			//10	code value: x
	TEMP_AN_3V3,			//11	code value: x
	IMON_AN_12V,			//12	code value: x
	TEMP_AN_12V,			//13	code value: x
	TEMP_AN_1V9,			//14	code value: x
	IMON_AN_1V9,			//15	code value: x

	TOTAL_CTRLBRD_MULTI_ANAL//16
};

enum MULTCH_CTRLBRD_DIGI_IDX
{
	/* U38: MC74HC165ADG  控制板数字量并转串多通道切换芯片
	 * SER: GND
	 * A:	AVDD5V_PGOOD
	 * B:	HAND_3V3_6
	 * C:	HAND_3V3_5
	 * D:	HAND_3V3_4
	 * E:	HAND_3V3_3
	 * F:	HAND_3V3_2
	 * G:	HAND_3V3_1
	 * H:	12V_PGOOD
	 */
	PGOOD_12V = 0,	//0
	HAND_3V3_1D,		//1
	HAND_3V3_2D,		//2
	HAND_3V3_3D,		//3
	HAND_3V3_4D,		//4
	HAND_3V3_5D,		//5
	HAND_3V3_6D,		//6
	AVDD5V_PGOOD,		//7
	GND,				//8

	TOTAL_CTRLBRD_MULTI_DIGIT//9
};
/**************************功率板模拟量数字量定义******************************************/
enum MULTCH_PWERBRD_ANAL_IDX
{
	/* UG1: CD47HC4067M  功率板模拟量多通道切换芯片
	 *
	 * X0:	28V_SEN_BAK
	 * X1:	10V_BIT
	 * X2:	GY_12V_BIT
	 * X3:	BREAKU_AN
	 * X4:	ID_AN
	 * X5:	T1SEN_AN
	 * X6:	RB_OUT_AN
	 * X7:	AGND
	 * X8:	RC_OUT_AN
	 * X9:	2.5V_TEMP_AN
	 * X10:	RA_OUT_AN
	 * X11:	QREF_IMAX
	 * X12:	QREF_IMIN
	 * X13:	A5VSEN_AN
	 * X14:	XREF_IMAX
	 * X15:	XREF_IMIN
	 */
	SEN_28V_BAK = 0,	//0
	BIT_10V,			//1
	GY_12V_BIT,			//2
	BREAKU_AN,			//3
	T1SEN_AN,			//4
	ID_AN,				//5
	REF2V5,				//6
	AGND,				//7
	RC_OUT,				//8
	RB_OUT,				//9
	RA_OUT,				//10
	QREF_IMAX,			//11
	QREF_IMIN,			//12
	A5VSEN,				//13
	XREF_IMAX,			//14
	XREF_IMIN,			//15
	TOTAL_PWERBRD_MULTI_ANAL
};



enum SNGL_PWERBRD_ANAL_IDX
{
	ForceValue = 0,		//0
	BusCurrentPos,		//1
	Power28V_M,			//2
	BridgeCurrentB,		//3
	BusCurrentB,		//4
	Power28V,			//5
	BridgeCurrentA,		//6
	BusCurrentA,		//7
	DisplacementValue,	//8
	BridgeCurrentC,		//9
	BusCurrentC,		//10
	TotalChannel		//11
};


typedef int (*UV)(void);

struct SingleAnalogVar
{
	int value;
	int max;
	int min;
	// int fault_level
	// int zero_position
	// int zero_max
	// int zero_min
	// int fault_zero_p
	// int k
	// int zero_source   如果=-1，初始化时读取，如果>=0，则zero_source就是零位
	UV  updateValue;
};




/********************系统模拟量数据结构**************************/
typedef struct _AnalogVar{
	int value;
	int max;
	int min;
	int count_max;
	int count_min;
	UV updateValue;
}AnalogVar;

typedef struct _SingleChannelA{
	AnalogVar var[TotalChannel];
}SingleChannelA;

typedef struct _SingleChannelB{
	AnalogVar var[TotalChannel];
}SingleChannelB;

typedef struct _MultiChannelA{
	AnalogVar var[TOTAL_CTRLBRD_MULTI_ANAL];
}MultiChannelA;


typedef struct _SysAnalogVar{
	SingleChannelA single;
	MultiChannelA multi[2];
	SingleChannelB singleB;
}SysAnalogVar;

/**********************系统数字量数据结构****************************/

typedef struct _DigitVar{
	int valueP;
	int valueN;
	int min;
	UV updateValue;
}DigitVar;

typedef struct _SingleChannelD{
	DigitVar var[TotalChannel];
}SingleChannelD;

typedef struct _MultiChannelD{
	DigitVar var[TOTAL_CTRLBRD_MULTI_DIGIT];
}MultiChannelD;


typedef struct _SysDigitVar{
	MultiChannelD multi;
	SingleChannelD single;
}SysDigitVar;

/***********************系统状态量数据结构**********************/

typedef struct _SysMonitorVar{
	SysAnalogVar anolog;
	SysDigitVar digit;

}SysMonitorVar;



int IsSingleAnalogValueAbnormal(void);
void UpdateSingleAnalogInput(void);
void AnalogValueInspect(void);
void DigitalValueInspect(void);
void ReadAnalogValue(void);
void ReadDigitalValue(void);
int IsCommonAnalogValueAbnormal(void);
extern SysMonitorVar gSysMonitorVar;

extern const UV funcptr[];
extern const int anologMaxMinInit[][2];

#endif
