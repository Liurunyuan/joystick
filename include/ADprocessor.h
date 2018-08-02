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
#define GET_BUS_CURRENT_P					((AdcRegs.ADCRESULT1) >> 4)
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

enum Status{
	REFRESH = 1,
	LOCK,
	TRIGGER,
	GETDATA
};

enum SingleAnalogInput
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


struct MultiAnalogValue
{
	Uint16 	index;
	int		controlBoardBIT[15];
	int     powerBoardBIT[15];

};

struct AnalogAndDigitalInspect
{
	struct MultiAnalogValue MultiAnalogValue;
	int DigitalParaToSerial_N[9];
	int DigitalParaToSerial_P[9];
};

typedef int (*UV)(void);

struct SingleAnalogVar
{
	int value;
	int max;
	int min;
	UV  updateValue;
};

int IsSingleAnalogValueAbnormal(void);
void UpdateSingleAnalogInput(void);
void AnalogValueInspect(void);
void DigitalValueInspect(void);
extern struct SingleAnalogVar gSingleAnalogVar[TotalChannel];
extern struct AnalogAndDigitalInspect gAnalogAndDigitalInspect;

#endif
