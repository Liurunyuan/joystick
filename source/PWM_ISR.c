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
	//����ռ�ձ����
	//prepare output
	//read spi value
	/*struct FanKui_OUT {int32 LI, WEIYI , SPEED, ACCEL, LOCK }  LW_BUFF;
	LOCK=0��PWM�߳��ѽ����������0.25ms�ж���δ����
	LOCK=1��0.25ms����ɸ���
	��PWM�ж��У��жϵ�10��׼��ˢ��LW_BUFFʱ�����LOCK����LOCK=1���������ݣ�����LOCK=0����LOCK=0�������������LOCK 001��
	��0.25ms�ж��У������ж��ж�LOCK����LOCK=0���������ݣ���ȫ�ֱ������βε���ʽ���ú��������ڽ�ȫ�ֱ���ѹ���ջ�������õĺ�����һ�仰��ȫ�ֱ�����LOCK=1������LOCK=1���򱨾���LOCK 002��
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












