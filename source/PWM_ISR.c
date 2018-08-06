#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "PWM_ISR.h"
#include "ADprocessor.h"

KeyValue gKeyValue;
FeedbackVarBuf feedbackVarBuf;
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
	//换向，占空比输出
	//prepare output
	//read spi value
	/*struct FanKui_OUT {int32 LI, WEIYI , SPEED, ACCEL, LOCK }  LW_BUFF;
	LOCK=0：PWM线程已将数据输出，0.25ms中断尚未复制
	LOCK=1：0.25ms已完成复制
	在PWM中断中，中断第10次准备刷新LW_BUFF时，检查LOCK，若LOCK=1，则复制数据，并将LOCK=0；若LOCK=0，则软件报警（LOCK 001）
	在0.25ms中断中，进入中断判断LOCK，若LOCK=0，则复制数据（将全局变量以形参的形式调用函数，等于将全局变量压入堆栈，被调用的函数第一句话将全局变量的LOCK=1）；若LOCK=1，则报警（LOCK 002）
	 */

	ReadAnalogValue();
	ReadDigitalValue();
	if(IsSingleAnalogValueAbnormal() == True)
	{
		//TODO
	}

}

int32 forcebufProcess(int* force)
{

	return 1;
}

int32 displacebufProcess(int* displace)
{
	return 1;
}

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
 *Name:						Pwm_ISR_Thread
 *Function:					PWM interrupt function
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.6.10
 **************************************************************/
void VarProcess(void){
	static int count = 0;
	static int forcebuf[10] = {0};
	static  int displacement[10] = {0};


	feedbackVarBuf.forcebuf[count] = gSysMonitorVar.anolog.single.var[ForceValue].value;
	feedbackVarBuf.displacementbuf[count] = gSysMonitorVar.anolog.single.var[DisplacementValue].value;

	UpdateMaxAndMin(&feedbackVarBuf);
	if(count > 10){
		count = 0;
		if(gKeyValue.lock == 0)
		{
			//TODO generate alarm;
			return;
		}
		gKeyValue.displacement = displacebufProcess(forcebuf);
		gKeyValue.force = forcebufProcess(displacement);
		//update motor speed;
		//update motor accel;
		gKeyValue.lock = 0;
	}
	++count;
}












