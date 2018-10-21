#ifndef _ADPROCESSOR_H
#define _ADPROCESSOR_H



#define WAIT_TIMES 		(20)
#define MAX_CHANNEL		(16)

/********calculate the ADC result of ADCINB7**************/
#define GET_ADCINB7   	((AdcRegs.ADCRESULT15) >> 4)
#define GET_ADCINB1	    ((AdcRegs.ADCRESULT9) >> 4)

/*****************模拟量多通道BIT检测地址位设置************/
#define SET_AD1K		(GpioDataRegs.GPASET.bit.GPIO30)
#define SET_AD2K		(GpioDataRegs.GPASET.bit.GPIO29)
#define SET_AD3K		(GpioDataRegs.GPCSET.bit.GPIO85)
#define SET_AD4K		(GpioDataRegs.GPBSET.bit.GPIO39)
/*********************************************************/

/*****************数字量多通道BIT检测设置******************/
#define GET_DIGIT_SERIAL_N   (GpioDataRegs.GPBDAT.bit.GPIO60)
#define GET_DIGIT_SERIAL_P   (GpioDataRegs.GPBDAT.bit.GPIO59)

#define SET_DIGIT_SER_CLK_HIGH (GpioDataRegs.GPBSET.bit.GPIO52)
#define SET_DIGIT_SER_CLK_LOW  (GpioDataRegs.GPBCLEAR.bit.GPIO52)

#define SET_DIGIT_SER_LOAD_HIGH (GpioDataRegs.GPBSET.bit.GPIO53)
#define SET_DIGIT_SER_LOAD_LOW  (GpioDataRegs.GPBCLEAR.bit.GPIO53)



/***********************紧急功率板输出模拟量****************/
#define GET_FORCE_SGN				((AdcRegs.ADCRESULT0) >> 4)
#define GET_BUS_CURRENT_P			((AdcRegs.ADCRESULT1) >> 4)
#define GET_28V_M					((AdcRegs.ADCRESULT2) >> 4)
#define GET_B_BRIDGE_CURRENT		((AdcRegs.ADCRESULT3) >> 4)
#define GET_B_BUS_CURRENT			((AdcRegs.ADCRESULT4) >> 4)
#define GET_28V						((AdcRegs.ADCRESULT5) >> 4)
#define GET_A_BRIDGE_CURRENT		((AdcRegs.ADCRESULT6) >> 4)
#define GET_A_BUS_CURRENT			((AdcRegs.ADCRESULT7) >> 4)
#define GET_DISPLACEMENT_SGN		((AdcRegs.ADCRESULT8) >> 4)
#define GET_C_BRIDGE_CURRENT		((AdcRegs.ADCRESULT11) >> 4)
#define GET_C_BUS_CURRENT			((AdcRegs.ADCRESULT12) >> 4)
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

#define MULTCH_CTRLBRD_ANAL_ARRAY_LEN				(15)
#define MULTCH_CTRLBRD_DIGI_ARRAY_LEN_N				(9)
#define MULTCH_CTRLBRD_DIGI_ARRAY_LEN_P				(9)
#define MULTCH_PWERBRD_ANAL_ARRAY_LEN				(15)
#define MULTCH_PWERBRD_DIGI_ARRAY_LEN				(0)

#define SNGLCH_CTRLBRD_ANAL_ARRAY_LEN				(0)
#define SNGLCH_CTRLBRD_DIGI_ARRAY_LEN				(0)
#define SNGLCH_PWERBRD_ANAL_ARRAY_LEN				(11)
#define SNGLCH_PWERBRD_DIGI_ARRAY_LEN_I				(15)
#define SNGLCH_PWERBRD_DIGI_ARRAY_LEN_O				(16)

enum Status{
	REFRESH = 1,
	LOCK,
	TRIGGER,
	GETDATA
};

enum MULTCH_CTRLBRD_ANAL_IDX
{
	AGND1 = 0,
	SEN_5V,
	REF_5V_SEN,
	WY_TEMP,
	WY_IMON,
	A5V_TEMP,
	A5V_IMON,
	AGND2,
	IMON_12V,
	TEMP_12V,
	LI_IMON,
	IMON_1V9,
	IMON_3V3,
	TEMP_3V3,
	LI_TEMP,
	TEMP_1V9

};

enum MULTCH_CTRLBRD_DIGI_IDX
{
	A5V_PGOOD = 0,
	WY_FAULT2,
	WY_PGOOD,
	WY_FAULT1,
	A2V5_PGOOD,
	PGOOD_12V,
	LI_FAULT1,
	LI_FAULT2,
	LI_PGOOD,
};

struct MULTCH_CTRLBRD
{
	Uint16 	index;
	int		MULTCH_CTRLBRD_ANAL_ARRAY[MULTCH_CTRLBRD_ANAL_ARRAY_LEN];
	int     MULTCH_CTRLBRD_DIGI_ARRAY_N[MULTCH_CTRLBRD_DIGI_ARRAY_LEN_N];
	int		MULTCH_CTRLBRD_DIGI_ARRAY_P[MULTCH_CTRLBRD_DIGI_ARRAY_LEN_P];

};

enum MULTCH_PWERBRD_ANAL_IDX
{
	SEN_28V_BAK = 0,
	BIT_10V,
	GY_12V_BIT,
	BREAKU_AN,
	ID_AN,
	REF2V5,
	AGND,
	RC_OUT,
	RB_OUT,
	RA_OUT,
	QREF_IMAX,
	QREF_IMIN,
	A5VSEN,
	XREF_IMAX,
	XREF_IMIN
};

struct MULTCH_PWERBRD
{
	Uint16 	index;
	int		MULTCH_PWERBRD_ANAL_ARRAY[MULTCH_PWERBRD_ANAL_ARRAY_LEN];
	int     MULTCH_PWERBRD_DIGI_ARRAY[MULTCH_PWERBRD_DIGI_ARRAY_LEN];

};

struct MULTCH
{
	struct MULTCH_CTRLBRD Mult_CtrlBrd;
	struct MULTCH_PWERBRD Mult_PwerBrd;
};

enum SNGL_PWERBRD_ANAL_IDX
{
	ForceValue = 0,
	BusCurrentPos,
	Power28V_M,
	BridgeCurrentB,
	BusCurrentB,
	Power28V,
	BridgeCurrentA,
	BusCurrentA,
	DisplacementValue,
	BridgeCurrentC,
	BusCurrentC,
	TotalChannel
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
	UV updateValue;
}AnalogVar;

typedef struct _SingleChannelA{
	AnalogVar var[TotalChannel];
}SingleChannelA;

typedef struct _MultiChannelA{
	AnalogVar var[TotalChannel];
}MultiChannelA;


typedef struct _SysAnalogVar{
	SingleChannelA single;
	MultiChannelA multi[2];
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
	DigitVar var[TotalChannel];
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
extern SysMonitorVar gSysMonitorVar;
//extern struct MULTCH gAnalogAndDigitalInspect;

extern const UV funcptr[];



#endif
