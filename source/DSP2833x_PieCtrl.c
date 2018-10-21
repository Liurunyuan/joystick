// TI File $Revision: /main/1 $
// Checkin $Date: August 18, 2006   13:46:35 $
//###########################################################################
//
// FILE:	DSP2833x_PieCtrl.c
//
// TITLE:	DSP2833x Device PIE Control Register Initialization Functions.
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x Header Files V1.20 $
// $Release Date: August 1, 2008 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

//---------------------------------------------------------------------------
// InitPieCtrl: 
//---------------------------------------------------------------------------
// This function initializes the PIE control registers to a known state.
//
void InitPieCtrl(void)
{
    // Disable Interrupts at the CPU level:
    DINT;

    // Disable the PIE
    PieCtrlRegs.PIECTRL.bit.ENPIE = 0;

	// Clear all PIEIER registers:
	PieCtrlRegs.PIEIER1.all = 0;
	PieCtrlRegs.PIEIER2.all = 0;
	PieCtrlRegs.PIEIER3.all = 0;	
	PieCtrlRegs.PIEIER4.all = 0;
	PieCtrlRegs.PIEIER5.all = 0;
	PieCtrlRegs.PIEIER6.all = 0;
	PieCtrlRegs.PIEIER7.all = 0;
	PieCtrlRegs.PIEIER8.all = 0;
	PieCtrlRegs.PIEIER9.all = 0;
	PieCtrlRegs.PIEIER10.all = 0;
	PieCtrlRegs.PIEIER11.all = 0;
	PieCtrlRegs.PIEIER12.all = 0;

	// Clear all PIEIFR registers:
	PieCtrlRegs.PIEIFR1.all = 0;
	PieCtrlRegs.PIEIFR2.all = 0;
	PieCtrlRegs.PIEIFR3.all = 0;	
	PieCtrlRegs.PIEIFR4.all = 0;
	PieCtrlRegs.PIEIFR5.all = 0;
	PieCtrlRegs.PIEIFR6.all = 0;
	PieCtrlRegs.PIEIFR7.all = 0;
	PieCtrlRegs.PIEIFR8.all = 0;
	PieCtrlRegs.PIEIFR9.all = 0;
	PieCtrlRegs.PIEIFR10.all = 0;
	PieCtrlRegs.PIEIFR11.all = 0;
	PieCtrlRegs.PIEIFR12.all = 0;


}	

//---------------------------------------------------------------------------
// EnableInterrupts: 
//---------------------------------------------------------------------------
// This function enables the PIE module and CPU interrupts
//
void EnableInterrupts()
{

    // Enable the PIE
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    		
	// Enables PIE to drive a pulse into the CPU 
	PieCtrlRegs.PIEACK.all = 0xFFFF;  
	//配置TZ中断相关管脚

	// Enable Interrupts at the CPU level 
	PieCtrlRegs.PIEIER1.bit.INTx1 = 1;//ADC中断,16通道转换完成后来中断
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;//定时器0中断。
	PieCtrlRegs.PIEIER2.bit.INTx1= 1;//TZ_FAULTB触发
	//PieCtrlRegs.PIEIER2.bit.INTx2= 1;//TZ_FAULTA触发//
	//PieCtrlRegs.PIEIER2.bit.INTx3= 1;//IKA_BJ触发//
	//PieCtrlRegs.PIEIER2.bit.INTx4= 1;//IKB_BJ触发//
	PieCtrlRegs.PIEIER2.bit.INTx6 = 1;//应急开关触发
	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;//ePWM1中断
	PieCtrlRegs.PIEIER9.bit.INTx3 = 1;//SCIB接收中断
	PieCtrlRegs.PIEIER9.bit.INTx4 = 1;//SCIB发送中断
	PieCtrlRegs.PIEIER8.bit.INTx5 = 1;//SCIC RX Interrupt
	PieCtrlRegs.PIEIER8.bit.INTx6 = 1;//SCIC TX Interrupt

   // EINT;

}

/*
 * Initialize all the Interrupt
 */
void Init_Interrupt(void)
{
	//初始化CPU_T0
		InitCpuTimers();
		ConfigCpuTimer(&CpuTimer0, 120, 10000);
	    CpuTimer0Regs.TCR.bit.TIE= 1;
	    CpuTimer0Regs.TCR.bit.TSS = 0;
	    //中断配置
	    DINT;
	    InitPieCtrl();
	    IER = 0x0000;
	 	IFR = 0x0000;
	 	InitPieVectTable();

	    //IER |= M_INT1;
	    //IER |= M_INT2;
	    IER |= M_INT3;
	 	IER |= M_INT8;//SCIc
	    IER |= M_INT9;//SCIa//ECAN//scib

	    EnableInterrupts();
	    EINT;   // Enable Global interrupt INTM
	    ERTM;
	    AdcRegs.ADCST.bit.INT_SEQ1_CLR=1;//此句要有，否则进步了中断，应为在该行代码执行前，seq1中断标识已经被立起，此处需要清除
	    ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;//此句做用同上
	    ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;
	    EALLOW;
	    EPwm1Regs.TZCLR.bit.CBC=1;//清除CBC时间标志位
	    EPwm1Regs.TZCLR.bit.INT=1;//清除中断标识位
	    EDIS;
	    EPwm1Regs.ETCLR.bit.INT = 1;

}
//===========================================================================
// End of file.
//===========================================================================
