#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "PWM_ISR.h"
#include "ADprocessor.h"
#include "Filter_Alg.h"
#include "SPIprocess.h"
#include "GlobalVarAndFunc.h"


FeedbackVarBuf feedbackVarBuf;
void ForceAndDisplaceProcess(int count);

/**************************************************************
 *Name:						CalForceSpeedAccel
 *Function:
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.28
 **************************************************************/
void CalForceSpeedAccel(void) {
	static int count = 0;

	//CalFuncPara(feedbackVarBuf.displacementbuf[count], feedbackVarBuf.forcebuf[count], count);
	CalFuncPara(gSysMonitorVar.anolog.single.var[DisplacementValue].value, gSysMonitorVar.anolog.single.var[ForceValue].value, count);

	count++;

	if(count >= 10){
		gKeyValue.displacement = funcParaDisplacement.a * 121 + funcParaDisplacement.b * 11 + funcParaDisplacement.c;
		gKeyValue.motorSpeed = 2*funcParaDisplacement.a*11 + funcParaDisplacement.b;
		gKeyValue.motorAccel = 2*funcParaDisplacement.a;

		gKeyValue.force = funcParaForce.a * 121 + funcParaForce.b * 11 + funcParaForce.c;

		count = 0;
	}
}
/**************************************************************
 *Name:						GetCurrentHallValue
 *Function:
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.31
 **************************************************************/
Uint16 GetCurrentHallValue(void){

	Uint16 temp;
	Uint16 a;
	Uint16 b;
	Uint16 c;

	c = GpioDataRegs.GPADAT.bit.GPIO27;
	b = GpioDataRegs.GPBDAT.bit.GPIO48;
	a = GpioDataRegs.GPBDAT.bit.GPIO49;

	temp = ((c << 2) + (b << 1) + a)^0x07;

	if(temp < 1 || temp >6){
		//TODO if temp < 1 or >6 means program abnormal, need to do something
	}
	return temp;
}
/**************************************************************
 *Name:						SwitchDirection
 *Function:
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.31
 **************************************************************/
void SwitchDirection(void){
	gSysInfo.lastTimeHalllPosition = gSysInfo.currentHallPosition;
	//gSysInfo.currentHallPosition = GetCurrentHallValue();
//3:A 2:B 1:C
	switch (gSysInfo.currentHallPosition) {
		case 4://C+ ---------------> B-
			if((4 == gSysInfo.lastTimeHalllPosition ) || (5 == gSysInfo.lastTimeHalllPosition)){
				EPwm3Regs.AQCSFRC.bit.CSFA=0x01;//shutdown A phase
				EPwm3Regs.AQCSFRC.bit.CSFB=0x01;//shutdown A phase


				EPwm1Regs.CMPA.half.CMPA = EPWM1_TIMER_HALF_TBPRD + gSysInfo.duty;//C+
				EPwm2Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD - gSysInfo.duty;//B-

				EPwm1Regs.AQCSFRC.bit.CSFA=0x00;
				EPwm1Regs.AQCSFRC.bit.CSFB=0x00;

				EPwm2Regs.AQCSFRC.bit.CSFA=0x00;
				EPwm2Regs.AQCSFRC.bit.CSFB=0x00;
			}
			break;
		case 6://C+ ---------------> A-
			if((6 == gSysInfo.lastTimeHalllPosition ) || (4 == gSysInfo.lastTimeHalllPosition)){
				EPwm2Regs.AQCSFRC.bit.CSFA=0x01;//shutdown B phase
				EPwm2Regs.AQCSFRC.bit.CSFB=0x01;//shutdown B phase

				EPwm1Regs.CMPA.half.CMPA = EPWM1_TIMER_HALF_TBPRD + gSysInfo.duty;//C+
				EPwm3Regs.CMPA.half.CMPA = EPWM1_TIMER_HALF_TBPRD - gSysInfo.duty;//A-

				EPwm1Regs.AQCSFRC.bit.CSFA=0x00;
				EPwm1Regs.AQCSFRC.bit.CSFB=0x00;

				EPwm3Regs.AQCSFRC.bit.CSFA=0x00;
				EPwm3Regs.AQCSFRC.bit.CSFB=0x00;
			}
			break;
		case 2://B+ ---------------> A-
			if((2 == gSysInfo.lastTimeHalllPosition ) || (6 == gSysInfo.lastTimeHalllPosition)){
				EPwm1Regs.AQCSFRC.bit.CSFA=0x01;//shutdown C phase
				EPwm1Regs.AQCSFRC.bit.CSFB=0x01;//shutdown C phase

				EPwm2Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD + gSysInfo.duty;//B+
				EPwm3Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD - gSysInfo.duty;//A-

				EPwm2Regs.AQCSFRC.bit.CSFA=0x00;
				EPwm2Regs.AQCSFRC.bit.CSFB=0x00;

				EPwm3Regs.AQCSFRC.bit.CSFA=0x00;
				EPwm3Regs.AQCSFRC.bit.CSFB=0x00;
			}
			break;
		case 3://B+ ---------------> C-
			if((3 == gSysInfo.lastTimeHalllPosition ) || (2 == gSysInfo.lastTimeHalllPosition)){
				EPwm3Regs.AQCSFRC.bit.CSFA=0x01;//shutdown A phase
				EPwm3Regs.AQCSFRC.bit.CSFB=0x01;//shutdown A phase

				EPwm2Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD + gSysInfo.duty;//B+
				EPwm1Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD - gSysInfo.duty;//C-

				EPwm2Regs.AQCSFRC.bit.CSFA=0x00;
				EPwm2Regs.AQCSFRC.bit.CSFB=0x00;

				EPwm1Regs.AQCSFRC.bit.CSFA=0x00;
				EPwm1Regs.AQCSFRC.bit.CSFB=0x00;
			}
			break;
		case 1://A+ ---------------> C-
			if((1 == gSysInfo.lastTimeHalllPosition ) || (3 == gSysInfo.lastTimeHalllPosition)){
				EPwm2Regs.AQCSFRC.bit.CSFA=0x01;//shutdown B phase
				EPwm2Regs.AQCSFRC.bit.CSFB=0x01;//shutdown B phase

				EPwm3Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD + gSysInfo.duty;//A+
				EPwm1Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD - gSysInfo.duty;//C-

				EPwm1Regs.AQCSFRC.bit.CSFA=0x00;
				EPwm1Regs.AQCSFRC.bit.CSFB=0x00;

				EPwm3Regs.AQCSFRC.bit.CSFA=0x00;
				EPwm3Regs.AQCSFRC.bit.CSFB=0x00;
			}
			break;
		case 5://A+ ---------------> B-
			if((5 == gSysInfo.lastTimeHalllPosition ) || (1 == gSysInfo.lastTimeHalllPosition)){
				EPwm1Regs.AQCSFRC.bit.CSFA=0x01;//shutdown C phase
				EPwm1Regs.AQCSFRC.bit.CSFB=0x01;//shutdown C phase

				EPwm3Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD + gSysInfo.duty;//A+
				EPwm2Regs.CMPA.half.CMPA = EPWM2_TIMER_HALF_TBPRD - gSysInfo.duty;//B-

				EPwm2Regs.AQCSFRC.bit.CSFA=0x00;
				EPwm2Regs.AQCSFRC.bit.CSFB=0x00;

				EPwm3Regs.AQCSFRC.bit.CSFA=0x00;
				EPwm3Regs.AQCSFRC.bit.CSFB=0x00;
			}
			break;
		default:
			//TODO need to generate alram
			break;
	}
}

/**************************************************************
 *Name:						Pwm_ISR_Thread
 *Function:					PWM interrupt function
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.6.10
 **************************************************************/
void Pwm_ISR_Thread(void)
{
	//TODO
	//start spi
	//read 12bit AD value
	//read DI
	//specific channel check
	//������ռ�ձ�����
	//prepare output
	//read spi value
	/*struct FanKui_OUT {int32 LI, WEIYI , SPEED, ACCEL, LOCK }  LW_BUFF;
	LOCK=0��PWM�߳��ѽ�����������0.25ms�ж���δ����
	LOCK=1��0.25ms�����ɸ���
	��PWM�ж��У��жϵ�10��׼��ˢ��LW_BUFFʱ������LOCK����LOCK=1�����������ݣ�����LOCK=0����LOCK=0��������������LOCK 001��
	��0.25ms�ж��У������ж��ж�LOCK����LOCK=0�����������ݣ���ȫ�ֱ������βε���ʽ���ú��������ڽ�ȫ�ֱ���ѹ����ջ�������õĺ�����һ�仰��ȫ�ֱ�����LOCK=1������LOCK=1���򱨾���LOCK 002��
	 */

	StartGetADBySpi();
	//ReadAnalogValue();
	ReadDigitalValue();

	if(IsSingleAnalogValueAbnormal() == True){
		//TODO
	}

	//TODO prepare output
	SwitchDirection();
	ReadADBySpi();

	CalForceSpeedAccel();//TODO this function has been modified, need to do more test to verify
}
/**************************************************************
 *Name:						forcebufProcess
 *Function:
 *Input:					none
 *Output:					force value
 *Author:					Simon
 *Date:						2018.10.28
 **************************************************************/
int32 forcebufProcess(void)
{
	return ((feedbackVarBuf.sumForce - feedbackVarBuf.maxForce - feedbackVarBuf.minForce) >> 3);
}
/**************************************************************
 *Name:						displacebufProcess
 *Function:
 *Input:					none
 *Output:					displacement value
 *Author:					Simon
 *Date:						2018.10.28
 **************************************************************/
int32 displacebufProcess(void)
{
	return ((feedbackVarBuf.sumDisplacement - feedbackVarBuf.maxDisplacement - feedbackVarBuf.minDisplacement) >> 3);
}
/**************************************************************
 *Name:						UpdateMaxAndMin
 *Function:
 *Input:					feedbackVarBuf
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.28
 **************************************************************/
void UpdateMaxAndMin(FeedbackVarBuf* feedbackVarBuf) {
	if (gSysMonitorVar.anolog.single.var[ForceValue].value
			>= feedbackVarBuf->maxForce) {
		feedbackVarBuf->maxForce =
				gSysMonitorVar.anolog.single.var[ForceValue].value;
	}
	if (gSysMonitorVar.anolog.single.var[ForceValue].value
			<= feedbackVarBuf->minForce) {
		feedbackVarBuf->minForce =
				gSysMonitorVar.anolog.single.var[ForceValue].value;
	}
	if (gSysMonitorVar.anolog.single.var[DisplacementValue].value
			>= feedbackVarBuf->maxDisplacement) {
		feedbackVarBuf->maxDisplacement =
				gSysMonitorVar.anolog.single.var[DisplacementValue].value;
	}
	if (gSysMonitorVar.anolog.single.var[DisplacementValue].value
			<= feedbackVarBuf->minDisplacement) {
		feedbackVarBuf->minDisplacement =
				gSysMonitorVar.anolog.single.var[DisplacementValue].value;
	}
}
/**************************************************************
 *Name:						ForceAndDisplaceProcess
 *Function:					PWM interrupt function
 *Input:					int count
 *Output:					none
 *Author:					Simon
 *Date:						2018.6.10
 **************************************************************/
void ForceAndDisplaceProcess(int count){
	/*
	feedbackVarBuf.forcebuf[count] = gSysMonitorVar.anolog.single.var[ForceValue].value;
	feedbackVarBuf.displacementbuf[count] = gSysMonitorVar.anolog.single.var[DisplacementValue].value;
	*/

	feedbackVarBuf.sumForce = feedbackVarBuf.sumForce + feedbackVarBuf.forcebuf[count];
	feedbackVarBuf.sumDisplacement = feedbackVarBuf.sumDisplacement + feedbackVarBuf.displacementbuf[count];

	//UpdateMaxAndMin(&feedbackVarBuf);
	if(count >= 9){
		gKeyValue.lock = 1;//need to remove after debug
		if(gKeyValue.lock == 0)
		{
			//TODO generate alarm;
			return;
		}
		gKeyValue.displacement = displacebufProcess();
		gKeyValue.force = forcebufProcess();
		gKeyValue.lock = 0;
		feedbackVarBuf.sumForce = 0;
		feedbackVarBuf.sumDisplacement =0;
	}
}
