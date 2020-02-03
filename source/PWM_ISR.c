#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "PWM_ISR.h"
#include "ADprocessor.h"
#include "Filter_Alg.h"
#include "SPIprocess.h"
#include "GlobalVarAndFunc.h"
#include "PID.h"

Uint16 real3 = 0;
#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(UpdateKeyValue, "ramfuncs")
#endif
void UpdateKeyValue(void) {
	static int calSpeedCnt = 0;

	funcParaDisplacement = calFuncPara(sumParaDisplacement);
	gKeyValue.displacement = funcParaDisplacement.a * 0.0625 + funcParaDisplacement.b * 0.25 + funcParaDisplacement.c;

#if(LINEAR_SPEED_METHOD == INCLUDE_FEATURE)
	gKeyValue.motorSpeed = KalmanFilterSpeed((funcParaDisplacement.a * 0.050625 + funcParaDisplacement.b * 0.225)/0.225, KALMAN_Q, KALMAN_R); 
#elif(TEN_AVERAGE == INCLUDE_FEATURE)
	gKeyValue.motorSpeed = TenDisplaceElemntAverage();

#else
//	gKeyValue.motorSpeed = KalmanFilterSpeed(funcParaDisplacement.b, KALMAN_Q, KALMAN_R);
#endif
	CalFuncParaSpeed(gSysInfo.JoyStickSpeed, calSpeedCnt);
	++calSpeedCnt;
	if(calSpeedCnt >= 10){
		funcParaSpeed = calFuncParaSpeed(sumParaSpeed);
		gKeyValue.motorAccel = KalmanFilterAccel(funcParaSpeed.b, 1, 150);
		calSpeedCnt = 0;
		clearSumSpeed();
		gAccelDirection.updateAccelDirection(0);
	}
}
#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(TargetDutyGradualChange, "ramfuncs")
#endif
void TargetDutyGradualChange(int targetduty){

#if(DUTY_GRADUAL_CHANGE == INCLUDE_FEATURE)
   	static int count = 0;

   	++count;
   	if(count < gSysInfo.dutyAddInterval){
       return;
   	}
   	count = 0;

	if(gSysInfo.currentDuty > targetduty){
       	gSysInfo.currentDuty = (gSysInfo.currentDuty - gSysInfo.ddtmax) < targetduty ? targetduty : (gSysInfo.currentDuty - gSysInfo.ddtmax);
    }
    else if(gSysInfo.currentDuty < targetduty){
    	gSysInfo.currentDuty = (gSysInfo.currentDuty + gSysInfo.ddtmax) > targetduty ? targetduty : (gSysInfo.currentDuty + gSysInfo.ddtmax);
    }
    else{
           //nothing need change
    }

   	if(gSysInfo.currentDuty > DUTY_LIMIT_P){
    	gSysInfo.currentDuty = DUTY_LIMIT_P;
   	}
   	else if(gSysInfo.currentDuty < DUTY_LIMIT_N){
       	gSysInfo.currentDuty = DUTY_LIMIT_N;
   	}

	gSysInfo.duty = gSysInfo.currentDuty;
#endif

#if(DUTY_GRADUAL_CHANGE == EXCLUDE_FEATURE)
    gSysInfo.duty = targetduty;
#endif
}
/**************************************************************
 *Name:						CalForceSpeedAccel
 *Function:
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.28
 **************************************************************/
#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(CalForceSpeedAccel, "ramfuncs")
#endif
void CalForceSpeedAccel(void) {

	static int count = 0;

	if(gKeyValue.lock == 1){
		return;
	}
	CalFuncPara(gSysMonitorVar.anolog.AD_16bit.var[ForceValue_16bit].value, (gSysMonitorVar.anolog.AD_16bit.var[DisplacementValue_16bit].value*gSysInfo.DimL_K+gSysInfo.DimL_B), count);

	++count;

	if(count >= DATA_AMOUNT){
		gKeyValue.lock = 1;
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
#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(GetCurrentHallValue, "ramfuncs")
#endif
Uint16 GetCurrentHallValue(void){

	Uint16 temp;
	Uint16 a;
	Uint16 b;
	Uint16 c;

	a = GpioDataRegs.GPADAT.bit.GPIO27;
	b = GpioDataRegs.GPBDAT.bit.GPIO48;
	c = GpioDataRegs.GPBDAT.bit.GPIO49;

	temp = ((a << 2) + (b << 1) + c);

	if(temp < 1 || temp >6){
		gSysState.erro.bit.software = 1;
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
#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(SwitchDirection, "ramfuncs")
#endif
void SwitchDirection(void){
	int t_duty_temp;
	Uint16 t_duty_p;
	Uint16 t_duty_n;
	gSysInfo.lastTimeHalllPosition = gSysInfo.currentHallPosition;
	gSysInfo.currentHallPosition = GetCurrentHallValue();

	t_duty_temp = gSysInfo.duty;

	//t_duty_temp = 100;

	if(t_duty_temp > EPWM2_TIMER_HALF_TBPRD) t_duty_temp = EPWM2_TIMER_HALF_TBPRD;
	else if(t_duty_temp < -( EPWM2_TIMER_HALF_TBPRD )) t_duty_temp = -( EPWM2_TIMER_HALF_TBPRD );
	t_duty_p = (Uint16)(EPWM2_TIMER_HALF_TBPRD + t_duty_temp);
	t_duty_n = (Uint16)(EPWM2_TIMER_HALF_TBPRD - t_duty_temp);

	//3:A 2:B 1:C
	switch (gSysInfo.currentHallPosition) {
		case 3://A+ ---------------> C-

			if(3 == gSysInfo.lastTimeHalllPosition){

				//APositiveToCNegtive();
				EPwm2Regs.AQCSFRC.all = 0x0009; //DisablePwm2();
				EPwm1Regs.CMPA.half.CMPA = t_duty_p;
				EPwm3Regs.CMPA.half.CMPA = t_duty_n;
				EPwm1Regs.AQCSFRC.all = 0x000f; //EnablePwm1();
				EPwm3Regs.AQCSFRC.all = 0x000f; //EnablePwm3();
			}
			else if(1 == gSysInfo.lastTimeHalllPosition){
                EPwm2Regs.AQCSFRC.all = 0x0009; //DisablePwm2();
                EPwm1Regs.CMPA.half.CMPA = t_duty_p;
                EPwm3Regs.CMPA.half.CMPA = t_duty_n;
                EPwm1Regs.AQCSFRC.all = 0x000f; //EnablePwm1();
                EPwm3Regs.AQCSFRC.all = 0x000f; //EnablePwm3();
			    gSysInfo.rotateDirection = 0; //0 means backward, 1 means forward
			}
			else if(2 == gSysInfo.lastTimeHalllPosition){
                EPwm2Regs.AQCSFRC.all = 0x0009; //DisablePwm2();
                EPwm1Regs.CMPA.half.CMPA = t_duty_p;
                EPwm3Regs.CMPA.half.CMPA = t_duty_n;
                EPwm1Regs.AQCSFRC.all = 0x000f; //EnablePwm1();
                EPwm3Regs.AQCSFRC.all = 0x000f; //EnablePwm3();
			    gSysInfo.rotateDirection = 1; //0 means backward, 1 means forward
			}
			else{
			    //TODO report error
			}
			break;
		case 1://B+ ---------------> C-
			if(1 == gSysInfo.lastTimeHalllPosition){

				//BPositiveToCNegtive();
				EPwm1Regs.AQCSFRC.all = 0x0009; //DisablePwm1();
				EPwm2Regs.CMPA.half.CMPA = t_duty_p;
				EPwm3Regs.CMPA.half.CMPA = t_duty_n;
				EPwm2Regs.AQCSFRC.all = 0x000f; //EnablePwm2();
				EPwm3Regs.AQCSFRC.all = 0x000f; //EnablePwm3();
			}
            else if(5 == gSysInfo.lastTimeHalllPosition){
                EPwm1Regs.AQCSFRC.all = 0x0009; //DisablePwm1();
                EPwm2Regs.CMPA.half.CMPA = t_duty_p;
                EPwm3Regs.CMPA.half.CMPA = t_duty_n;
                EPwm2Regs.AQCSFRC.all = 0x000f; //EnablePwm2();
                EPwm3Regs.AQCSFRC.all = 0x000f; //EnablePwm3();
                gSysInfo.rotateDirection = 0;
            }
            else if(3 == gSysInfo.lastTimeHalllPosition){
                EPwm1Regs.AQCSFRC.all = 0x0009; //DisablePwm1();
                EPwm2Regs.CMPA.half.CMPA = t_duty_p;
                EPwm3Regs.CMPA.half.CMPA = t_duty_n;
                EPwm2Regs.AQCSFRC.all = 0x000f; //EnablePwm2();
                EPwm3Regs.AQCSFRC.all = 0x000f; //EnablePwm3();
                gSysInfo.rotateDirection = 1;
            }
            else{
                //TODO report error
            }
			break;
		case 5://B+ ---------------> A-
			if(5 == gSysInfo.lastTimeHalllPosition){

				//BPositiveToANegtive();
				EPwm3Regs.AQCSFRC.all = 0x0009; //DisablePwm3();
				EPwm2Regs.CMPA.half.CMPA = t_duty_p;
				EPwm1Regs.CMPA.half.CMPA = t_duty_n;
				EPwm2Regs.AQCSFRC.all = 0x000f; //EnablePwm2();
				EPwm1Regs.AQCSFRC.all = 0x000f; //EnablePwm1();
			}
            else if(4 == gSysInfo.lastTimeHalllPosition){
                EPwm3Regs.AQCSFRC.all = 0x0009; //DisablePwm3();
                EPwm2Regs.CMPA.half.CMPA = t_duty_p;
                EPwm1Regs.CMPA.half.CMPA = t_duty_n;
                EPwm2Regs.AQCSFRC.all = 0x000f; //EnablePwm2();
                EPwm1Regs.AQCSFRC.all = 0x000f; //EnablePwm1();
                gSysInfo.rotateDirection = 0;
            }
            else if(1 == gSysInfo.lastTimeHalllPosition){
                EPwm3Regs.AQCSFRC.all = 0x0009; //DisablePwm3();
                EPwm2Regs.CMPA.half.CMPA = t_duty_p;
                EPwm1Regs.CMPA.half.CMPA = t_duty_n;
                EPwm2Regs.AQCSFRC.all = 0x000f; //EnablePwm2();
                EPwm1Regs.AQCSFRC.all = 0x000f; //EnablePwm1();
                gSysInfo.rotateDirection = 1;
            }
            else{
                //TODO report error
            }
			break;
		case 4://C+ ---------------> A-
			if(4 == gSysInfo.lastTimeHalllPosition){

				//CPositiveToANegtive();
				EPwm2Regs.AQCSFRC.all = 0x0009; //DisablePwm2();
				EPwm3Regs.CMPA.half.CMPA = t_duty_p;
				EPwm1Regs.CMPA.half.CMPA = t_duty_n;
				EPwm1Regs.AQCSFRC.all = 0x000f; //EnablePwm1();
				EPwm3Regs.AQCSFRC.all = 0x000f; //EnablePwm3();
			}
            else if(6 == gSysInfo.lastTimeHalllPosition){
                EPwm2Regs.AQCSFRC.all = 0x0009; //DisablePwm2();
                EPwm3Regs.CMPA.half.CMPA = t_duty_p;
                EPwm1Regs.CMPA.half.CMPA = t_duty_n;
                EPwm1Regs.AQCSFRC.all = 0x000f; //EnablePwm1();
                EPwm3Regs.AQCSFRC.all = 0x000f; //EnablePwm3();
                gSysInfo.rotateDirection = 0;
            }
            else if(5 == gSysInfo.lastTimeHalllPosition){
                EPwm2Regs.AQCSFRC.all = 0x0009; //DisablePwm2();
                EPwm3Regs.CMPA.half.CMPA = t_duty_p;
                EPwm1Regs.CMPA.half.CMPA = t_duty_n;
                EPwm1Regs.AQCSFRC.all = 0x000f; //EnablePwm1();
                EPwm3Regs.AQCSFRC.all = 0x000f; //EnablePwm3();
                gSysInfo.rotateDirection = 1;
            }
            else{
                //TODO report error
            }
			break;
		case 6://C+ ---------------> B-
			if(6 == gSysInfo.lastTimeHalllPosition){

				//CPositiveToBNegtive();
				EPwm1Regs.AQCSFRC.all = 0x0009; //DisablePwm1();
				EPwm3Regs.CMPA.half.CMPA = t_duty_p;
				EPwm2Regs.CMPA.half.CMPA = t_duty_n;
				EPwm2Regs.AQCSFRC.all = 0x000f; //EnablePwm2();
				EPwm3Regs.AQCSFRC.all = 0x000f; //EnablePwm3();
			}
            else if(2 == gSysInfo.lastTimeHalllPosition){
                EPwm1Regs.AQCSFRC.all = 0x0009; //DisablePwm1();
                EPwm3Regs.CMPA.half.CMPA = t_duty_p;
                EPwm2Regs.CMPA.half.CMPA = t_duty_n;
                EPwm2Regs.AQCSFRC.all = 0x000f; //EnablePwm2();
                EPwm3Regs.AQCSFRC.all = 0x000f; //EnablePwm3();
                gSysInfo.rotateDirection = 0;
            }
            else if(4 == gSysInfo.lastTimeHalllPosition){
                EPwm1Regs.AQCSFRC.all = 0x0009; //DisablePwm1();
                EPwm3Regs.CMPA.half.CMPA = t_duty_p;
                EPwm2Regs.CMPA.half.CMPA = t_duty_n;
                EPwm2Regs.AQCSFRC.all = 0x000f; //EnablePwm2();
                EPwm3Regs.AQCSFRC.all = 0x000f; //EnablePwm3();
                gSysInfo.rotateDirection = 1;
            }
            else{
                //TODO report error
            }
			break;
		case 2://A+ ---------------> B-
			if(2 == gSysInfo.lastTimeHalllPosition){

				//APositiveToBNegtive();
				EPwm3Regs.AQCSFRC.all = 0x0009; //DisablePwm3();
				EPwm1Regs.CMPA.half.CMPA = t_duty_p;
				EPwm2Regs.CMPA.half.CMPA = t_duty_n;
				EPwm1Regs.AQCSFRC.all = 0x000f; //EnablePwm1();
				EPwm2Regs.AQCSFRC.all = 0x000f; //EnablePwm2();
			}
            else if(3 == gSysInfo.lastTimeHalllPosition){
                EPwm3Regs.AQCSFRC.all = 0x0009; //DisablePwm3();
                EPwm1Regs.CMPA.half.CMPA = t_duty_p;
                EPwm2Regs.CMPA.half.CMPA = t_duty_n;
                EPwm1Regs.AQCSFRC.all = 0x000f; //EnablePwm1();
                EPwm2Regs.AQCSFRC.all = 0x000f; //EnablePwm2();
                gSysInfo.rotateDirection = 0;
            }
            else if(6 == gSysInfo.lastTimeHalllPosition){
                EPwm3Regs.AQCSFRC.all = 0x0009; //DisablePwm3();
                EPwm1Regs.CMPA.half.CMPA = t_duty_p;
                EPwm2Regs.CMPA.half.CMPA = t_duty_n;
                EPwm1Regs.AQCSFRC.all = 0x000f; //EnablePwm1();
                EPwm2Regs.AQCSFRC.all = 0x000f; //EnablePwm2();
                gSysInfo.rotateDirection = 1;
            }
            else{
                //TODO report error
            }
			break;
		default:
			gSysState.erro.bit.software = TRUE;
			DisablePwmOutput();
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
#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(Pwm_ISR_Thread, "ramfuncs")
#endif
void Pwm_ISR_Thread(void)
{

	//ReadDigitalValue();

	//ReadAnalogValue();

    ReadADBySpi();

    gSysMonitorVar.anolog.AD_16bit.var[ForceValue_16bit].value = (Uint16)(KalmanFilterForce(gAnalog16bit.force,50,50));
    gSysMonitorVar.anolog.AD_16bit.var[DisplacementValue_16bit].value = (Uint16)(KalmanFilter(gAnalog16bit.displace, KALMAN_Q, KALMAN_R));

	if((gConfigPara.stateCommand == 1) && (gSysState.warning.all == 0) && (gSysState.alarm.all == 0)){
	    if(gSysInfo.targetDuty > DUTY_LIMIT_P){
	        gSysInfo.targetDuty = DUTY_LIMIT_P;
	    }
	    else if(gSysInfo.targetDuty < DUTY_LIMIT_N){
	        gSysInfo.targetDuty = DUTY_LIMIT_N;
	    }
		TargetDutyGradualChange(gSysInfo.targetDuty);
		SwitchDirection();
	}
	else{
		DisablePwmOutput();
	}

	CalForceSpeedAccel();
	StartGetADBySpi();
}
