// TI File $Revision: /main/5 $
// Checkin $Date: October 23, 2007   13:34:09 $
//###########################################################################
//
// FILE:	DSP2833x_Adc.c
//
// TITLE:	DSP2833x ADC Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x Header Files V1.20 $
// $Release Date: August 1, 2008 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

#define ADC_usDELAY  5000L

//---------------------------------------------------------------------------
// InitAdc:
//---------------------------------------------------------------------------
// This function initializes ADC to a known state.
//
void InitAdc(void)
{
    extern void DSP28x_usDelay(Uint32 Count);


    // *IMPORTANT*
	// The ADC_cal function, which  copies the ADC calibration values from TI reserved
	// OTP into the ADCREFSEL and ADCOFFTRIM registers, occurs automatically in the
	// Boot ROM. If the boot ROM code is bypassed during the debug process, the
	// following function MUST be called for the ADC to function according
	// to specification. The clocks to the ADC MUST be enabled before calling this
	// function.
	// See the device data manual and/or the ADC Reference
	// Manual for more information.

	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
	ADC_cal();
	EDIS;




    // To powerup the ADC the ADCENCLK bit should be set first to enable
    // clocks, followed by powering up the bandgap, reference circuitry, and ADC core.
    // Before the first conversion is performed a 5ms delay must be observed
	// after power up to give all analog circuits time to power up and settle

    // Please note that for the delay function below to operate correctly the
	// CPU_RATE define statement in the DSP2833x_Examples.h file must
	// contain the correct CPU clock period in nanoseconds.

    //AdcRegs.ADCTRL3.all = 0x00E0;  // Power up bandgap/reference/ADC circuits
	AdcRegs.ADCTRL3.bit.ADCBGRFDN = 3;//  模数转换内部参考电压源电路上电
	DELAY_US(ADC_usDELAY);
  	AdcRegs.ADCTRL3.bit.ADCPWDN = 1;//  模数转换核模拟电路加电
//    DELAY_US(20); // Delay at least 20us before converting ADC channels 	//  至少20us延时
  	DELAY_US(5000);
    //DELAY_US(ADC_usDELAY);         // Delay before converting ADC channels
}

void ADC_Config(void)
{

//    AdcRegs.ADCTRL1.bit.ACQ_PS = 0x5;
////    AdcRegs.ADCTRL3.bit.ADCCLKPS = 0x5;
//    AdcRegs.ADCTRL3.bit.SMODE_SEL=0;
////    AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;
//////    AdcRegs.ADCTRL1.bit.CONT_RUN=1;


//    AdcRegs.ADCTRL1.bit.CPS = 0;
	AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 0; //启动SEQ转换（SOC）触发，0：清除不确定的SOC触发；1：软件触发SOC
	//AdcRegs.ADCTRL1.bit.CPS = 0; // 1:对HSPCLK进行2分频，0：不分频
	AdcRegs.ADCTRL3.bit.ADCCLKPS = 0x0005; //0: ADCCLK=HSPCLK/(CPS+1); 1<=ADCCLK<=15: ADCCLK=HSPCLK/(2*ADCCLKPS*(CPS+1))

	/*顺序采样模式下：一个AD转换周期（及完整的S/H采样保持周期）=SOC+ADCCLK;
	 *同步采样模式下：一个AD转换周期（及完整的S/H采样保持周期）=SOC+2*ADCCLK*/
	AdcRegs.ADCTRL1.bit.ACQ_PS = 0x0002;

	AdcRegs.ADCTRL3.bit.SMODE_SEL = 1;//0:顺序采样模式；1：同步采样模式
	AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;//0：双序列模式；1：级联模式

	/*0: 启动/停止转换方式，到达EOC时，停止采样，需要手动将SEQ_CNTR置1，
	 *1: 自动重新开始，即在SEQ_CNTR=0时，自动重新将MAXCONVn的值装入。 */
	AdcRegs.ADCTRL1.bit.CONT_RUN = 0;

	//同步模式下最大通道的配置
	AdcRegs.ADCMAXCONV.all = 0x0007;

    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0;
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1;
    AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2;
    AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x3;
    AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 0x4;
    AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 0x5;
    AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0x6;
    AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 0x7;

	AdcRegs.ADCTRL1.bit.SEQ_OVRD = 0;//装换完max通道后，排序器指针复位到初始状态
	AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;

/*
    AdcRegs.ADCTRL1.bit.SUSMOD = 2;
    AdcRegs.ADCTRL2.bit.SOC_SEQ1 	= 1;
    AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ = 0;
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;
    AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1 = 0;
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 0;
    AdcRegs.ADCTRL2.bit.SOC_SEQ2= 0;
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ2 = 0;
    AdcRegs.ADCTRL2.bit.INT_MOD_SEQ2 = 0;
    AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ2 = 0;
   // AdcRegs.ADCMAXCONV.bit.MAX_CONV1 =0xF;   // max conversion channel

    //AdcRegs.ADCMAXCONV.bit.MAX_CONV1= 0x5;
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
    AdcRegs.ADCST.bit.INT_SEQ2_CLR = 1;
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;
    DELAY_US(ADC_usDELAY);


    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0;
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1;
    AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2;
    AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x3;
    AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 0x4;
    AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 0x5;
    AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0x6;
    AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 0x7;
    AdcRegs.ADCCHSELSEQ3.bit.CONV08 = 0x8;
    AdcRegs.ADCCHSELSEQ3.bit.CONV09 = 0x9;
    AdcRegs.ADCCHSELSEQ3.bit.CONV10 = 0xa;
    AdcRegs.ADCCHSELSEQ3.bit.CONV11 = 0xb;
    AdcRegs.ADCCHSELSEQ4.bit.CONV12 = 0xc;
    AdcRegs.ADCCHSELSEQ4.bit.CONV13 = 0xd;
    AdcRegs.ADCCHSELSEQ4.bit.CONV14 = 0xe;
    AdcRegs.ADCCHSELSEQ4.bit.CONV15 = 0xf;

    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;

    //new adc init
*/
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;//允许ePWM的触发信号启动SEQ1
   // AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ2 = 1;//允许ePWM的触发信号启动SEQ1

    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 0x1;			//复位排序器SEQ1到CONV00状态
  //  AdcRegs.ADCTRL2.bit.RST_SEQ2 = 0x1;         //复位排序器SEQ1到CONV00状态
   // AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1=0;			//每个SEQ1序列结束时，INT_SEQ1置位
   // AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 0x0;		//禁止SEQ1中断

   // AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;
    //AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;

}
/*
 * Initialize ADC, including GPIO and configuration
 */
void Init_ADC(void)
{
	InitAdc();
	ADC_Config();
}
//===========================================================================
// End of file.
//===========================================================================
