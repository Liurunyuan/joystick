// TI File $Revision: /main/1 $
// Checkin $Date: August 18, 2006   13:46:25 $
//###########################################################################
//
// FILE:	DSP2833x_Gpio.c
//
// TITLE:	DSP2833x General Purpose I/O Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x Header Files V1.20 $
// $Release Date: August 1, 2008 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"

//---------------------------------------------------------------------------
// InitGpio: 
//---------------------------------------------------------------------------
// This function initializes the Gpio to a known (default) state.
//
// For more details on configuring GPIO's as peripheral functions,
// refer to the individual peripheral examples and/or GPIO setup example. 
void InitGpio(void)
{
   EALLOW;
   
   // Each GPIO pin can be: 
   // a) a GPIO input/output
   // b) peripheral function 1
   // c) peripheral function 2
   // d) peripheral function 3
   // By default, all are GPIO Inputs 
   GpioCtrlRegs.GPAMUX1.all = 0x0000;     // GPIO functionality GPIO0-GPIO15
   GpioCtrlRegs.GPAMUX2.all = 0x0000;     // GPIO functionality GPIO16-GPIO31
   GpioCtrlRegs.GPBMUX1.all = 0x0000;     // GPIO functionality GPIO32-GPIO39
   GpioCtrlRegs.GPBMUX2.all = 0x0000;     // GPIO functionality GPIO48-GPIO63
   GpioCtrlRegs.GPCMUX1.all = 0x0000;     // GPIO functionality GPIO64-GPIO79
   GpioCtrlRegs.GPCMUX2.all = 0x0000;     // GPIO functionality GPIO80-GPIO95

   GpioCtrlRegs.GPADIR.all = 0x0000;      // GPIO0-GPIO31 are inputs
   GpioCtrlRegs.GPBDIR.all = 0x0000;      // GPIO32-GPIO63 are inputs   
   GpioCtrlRegs.GPCDIR.all = 0x0000;      // GPI064-GPIO95 are inputs

   // Each input can have different qualification
   // a) input synchronized to SYSCLKOUT
   // b) input qualified by a sampling window
   // c) input sent asynchronously (valid for peripheral inputs only)
   GpioCtrlRegs.GPAQSEL1.all = 0x0000;    // GPIO0-GPIO15 Synch to SYSCLKOUT 
   GpioCtrlRegs.GPAQSEL2.all = 0x0000;    // GPIO16-GPIO31 Synch to SYSCLKOUT
   GpioCtrlRegs.GPBQSEL1.all = 0x0000;    // GPIO32-GPIO39 Synch to SYSCLKOUT 
   GpioCtrlRegs.GPBQSEL2.all = 0x0000;    // GPIO48-GPIO63 Synch to SYSCLKOUT 

   // Pull-ups can be enabled or disabled. 
   GpioCtrlRegs.GPAPUD.all = 0x0000;      // Pullup's enabled GPIO0-GPIO31
   GpioCtrlRegs.GPBPUD.all = 0x0000;      // Pullup's enabled GPIO32-GPIO63
   GpioCtrlRegs.GPCPUD.all = 0x0000;      // Pullup's enabled GPIO64-GPIO79

   //GpioCtrlRegs.GPAPUD.all = 0xFFFF;    // Pullup's disabled GPIO0-GPIO31
   //GpioCtrlRegs.GPBPUD.all = 0xFFFF;    // Pullup's disabled GPIO32-GPIO34
   //GpioCtrlRegs.GPCPUD.all = 0xFFFF     // Pullup's disabled GPIO64-GPIO79

   EDIS;

}	

void InitOutputPin(void)
{
	/*Read the circuit diagram and find which Pin need to init as Output GPIO, finsish the code here*/
	/*
	 * GPIO30
	 * Pin index:			1
	 * Name in circuit:		AD1K
	 */
	EALLOW;
	GpioCtrlRegs.GPAMUX2.bit.GPIO30	= GPIO;
	GpioCtrlRegs.GPADIR.bit.GPIO30	= OUTPUT;
	/*
	 * GPIO29
	 * Pin index:			2
	 * Name in circuit:		AD2K
	 */
	GpioCtrlRegs.GPAMUX2.bit.GPIO29	= GPIO;
	GpioCtrlRegs.GPADIR.bit.GPIO29	= OUTPUT;
	/*
	 * GPIO7
	 * Pin index:			16
	 * Name in circuit:		KZ_WDI_DSP
	 */
	GpioCtrlRegs.GPAMUX1.bit.GPIO7	= GPIO;
	GpioCtrlRegs.GPADIR.bit.GPIO7	= OUTPUT;

	/*
	 * GPIO9
	 * Pin index:			18
	 * Name in circuit 		CLR_OC_DSP
	 */
	GpioCtrlRegs.GPAMUX1.bit.GPIO9	= GPIO;
	GpioCtrlRegs.GPADIR.bit.GPIO9	= OUTPUT;
	/*GPIO51(pin index:91) is used to feed watch dog,so GPIO51 is output*/
	/*
	 * GPIO51
	 * Pin index:			91
	 * Name in circuit:		WDI
	 */
	GpioCtrlRegs.GPBMUX2.bit.GPIO51	= GPIO;
	GpioCtrlRegs.GPBDIR.bit.GPIO51	= OUTPUT;
	/*
	 *GPIO52
	 *Pin index:			94
	 *Name in circuit:		BIT_SER_CLK
	 */
	GpioCtrlRegs.GPBMUX2.bit.GPIO52	= GPIO;
	GpioCtrlRegs.GPBDIR.bit.GPIO52	= OUTPUT;
	/*
	 *GPIO53
	 *Pin index:			95
	 *Name in circuit:		BIT_SER_LOAD
	 */
	GpioCtrlRegs.GPBMUX2.bit.GPIO53	= GPIO;
	GpioCtrlRegs.GPBDIR.bit.GPIO53	= OUTPUT;
	/*
	 *GPIO85
	 *Pin index:			172
	 *Name in circuit:		AD4K
	 */
	GpioCtrlRegs.GPCMUX2.bit.GPIO85	= GPIO;
	GpioCtrlRegs.GPCDIR.bit.GPIO85	= OUTPUT;
	/*
	 *GPIO86
	 *Pin index:			173
	 *Name in circuit:		KZ_N_DSP
	 */
	GpioCtrlRegs.GPCMUX2.bit.GPIO86	= GPIO;
	GpioCtrlRegs.GPCDIR.bit.GPIO86	= OUTPUT;
	/*
	 *GPIO87
	 *Pin index:			174
	 *Name in circuit:		EN_OUT_BUF
	 */
	GpioCtrlRegs.GPCMUX2.bit.GPIO87	= GPIO;
	GpioCtrlRegs.GPCDIR.bit.GPIO87	= OUTPUT;
	/*
	 *GPIO39
	 *Pin index:			175
	 *Name in circuit:		AD3K
	 */
	GpioCtrlRegs.GPBMUX1.bit.GPIO39	= GPIO;
	GpioCtrlRegs.GPBDIR.bit.GPIO39	= OUTPUT;
	/*
	 *GPIO31
	 *Pin index:			176
	 *Name in circuit:		KZ_P_DSP
	 */
	GpioCtrlRegs.GPAMUX2.bit.GPIO31	= GPIO;
	GpioCtrlRegs.GPADIR.bit.GPIO31	= OUTPUT;


	/*
	 *GPIO82
	 *Pin index:			165
	 *Name in circuit:		GPIO82, just for test to calculate the function running time
	 */
	GpioCtrlRegs.GPCMUX2.bit.GPIO82	= GPIO;
	GpioCtrlRegs.GPCDIR.bit.GPIO82	= OUTPUT;

	/*
	 *GPIO84
	 *Pin index:			169
	 *Name in circuit:		GPIO84, just for test£¬GPIO84-------->CNV_AD;
	 */
	GpioCtrlRegs.GPCMUX2.bit.GPIO84	= GPIO;
	GpioCtrlRegs.GPCDIR.bit.GPIO84	= OUTPUT;


	/*
	 *GPIO6
	 *Pin index:			13
	 *Name in circuit:		GPIO6;
	 */
	GpioCtrlRegs.GPAMUX1.bit.GPIO6	= GPIO;
	GpioDataRegs.GPASET.bit.GPIO6 = 1;
	GpioCtrlRegs.GPADIR.bit.GPIO6	= OUTPUT;

	//GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;

	EDIS;
}

void InitInputPin(void)
{
	/*Read the circuit diagram and find which Pin need to init as Output GPIO, finsish the code here*/
	/*
	 * GPIO10
	 * Pin index:			19
	 * Name in circuit:		DIS_DSP
	 */
	GpioCtrlRegs.GPAMUX1.bit.GPIO10	= GPIO;
	GpioCtrlRegs.GPADIR.bit.GPIO10	= INPUT;
	/*
	 * GPIO11
	 * Pin index:			20
	 * Name in circuit:		EN_PWM_DSP
	 */
	GpioCtrlRegs.GPAMUX1.bit.GPIO11	= GPIO;
	GpioCtrlRegs.GPADIR.bit.GPIO11	= INPUT;
	/*
	 * GPIO12
	 * Pin index:			21
	 * Name in circuit:		P_SD_DSP
	 */
	GpioCtrlRegs.GPAMUX1.bit.GPIO12	= GPIO;
	GpioCtrlRegs.GPADIR.bit.GPIO12	= INPUT;
	/*
	 * GPIO13
	 * Pin index:			24
	 * Name in circuit:		PG_DSP
	 */
	GpioCtrlRegs.GPAMUX1.bit.GPIO13	= GPIO;
	GpioCtrlRegs.GPADIR.bit.GPIO13	= INPUT;
	/*
	 * GPIO14
	 * Pin index:			25
	 * Name in circuit:		DC_FLT_DSP
	 */
	GpioCtrlRegs.GPAMUX1.bit.GPIO14	= GPIO;
	GpioCtrlRegs.GPADIR.bit.GPIO14	= INPUT;
	/*
	 * GPIO15
	 * Pin index:			26
	 * Name in circuit:		DC_EN_DSP
	 */
	GpioCtrlRegs.GPAMUX1.bit.GPIO15	= GPIO;
	GpioCtrlRegs.GPADIR.bit.GPIO15	= INPUT;
	/*
	 * GPIO16
	 * Pin index:			27
	 * Name in circuit:		X_OC_F_DSP
	 */
	GpioCtrlRegs.GPAMUX2.bit.GPIO16	= GPIO;
	GpioCtrlRegs.GPADIR.bit.GPIO16	= INPUT;
	/*
	 * GPIO17
	 * Pin index:			28
	 * Name in circuit:		Q_OC_F_DSP
	 */
	GpioCtrlRegs.GPAMUX2.bit.GPIO17	= GPIO;
	GpioCtrlRegs.GPADIR.bit.GPIO17	= INPUT;
	/*
	 * GPIO59
	 * Pin index:			110
	 * Name in circuit:		BIT_SER_DIN_P
	 */
	GpioCtrlRegs.GPBMUX2.bit.GPIO59	= GPIO;
	GpioCtrlRegs.GPBDIR.bit.GPIO59	= INPUT;
	/*
	 * GPIO60
	 * Pin index:			111
	 * Name in circuit:		BIT_SER_DIN_N
	 */
	GpioCtrlRegs.GPBMUX2.bit.GPIO60	= GPIO;
	GpioCtrlRegs.GPBDIR.bit.GPIO60	= INPUT;
}
/*
 * Initialize the GPIO
 */
void Init_GPIO()
{
	InitOutputPin();
	InitInputPin();
}
//===========================================================================
// End of file.
//===========================================================================
