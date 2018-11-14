// TI File $Revision: /main/2 $
// Checkin $Date: March 1, 2007   16:06:07 $
//###########################################################################
//
// FILE:	DSP2833x_Sci.c
//
// TITLE:	DSP2833x SCI Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x Header Files V1.20 $
// $Release Date: August 1, 2008 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
//---------------------------------------------------------------------------
// InitSci:
//---------------------------------------------------------------------------
// This function initializes the SCI(s) to a known state.
//
void InitSci(void)
{
	// Initialize SCI-A:
	// Initialize SCI-B:
  // Initialize SCI-C:

}

//---------------------------------------------------------------------------
// Example: InitSciGpio:
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as SCI pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.
//
// Caution:
// Only one GPIO pin should be enabled for SCITXDA/B operation.
// Only one GPIO pin shoudl be enabled for SCIRXDA/B operation.
// Comment out other unwanted lines.

void InitSciGpio(void)
{
	//InitSciaGpio();
	InitScibGpio();
	InitScicGpio();

}

void InitSciaGpio()
{
   EALLOW;


// Pull-ups can be enabled or disabled disabled by the user.
// This will enable the pullups for the specified pins.

	GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up for GPIO28 (SCIRXDA)
	GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;	   // Enable pull-up for GPIO29 (SCITXDA)


// Inputs are synchronized to SYSCLKOUT by default.
// This will select asynch (no qualification) for the selected pins.

	GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (SCIRXDA)


// This specifies which of the possible GPIO pins will be SCI functional pins.

	GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;   // Configure GPIO28 for SCIRXDA operation
	GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;   // Configure GPIO29 for SCITXDA operation

    EDIS;
}


void InitScibGpio()
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

//  GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;     // Enable pull-up for GPIO9  (SCITXDB)
//  GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;    // Enable pull-up for GPIO14 (SCITXDB)
//	GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;	   // Enable pull-up for GPIO18 (SCITXDB)
  GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;    // Enable pull-up for GPIO22 (SCITXDB)


//  GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;    // Enable pull-up for GPIO11 (SCIRXDB)
//  GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;    // Enable pull-up for GPIO15 (SCIRXDB)
//    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;	   // Enable pull-up for GPIO19 (SCIRXDB)
  GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;    // Enable pull-up for GPIO23 (SCIRXDB)

/* Set qualification for selected pins to asynch only */
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.

//  GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 3;  // Asynch input GPIO11 (SCIRXDB)
//  GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3;  // Asynch input GPIO15 (SCIRXDB)
//	GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3;  // Asynch input GPIO19 (SCIRXDB)
  GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3;  // Asynch input GPIO23 (SCIRXDB)

/* Configure SCI-B pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SCI functional pins.
// Comment out other unwanted lines.

// GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 2;    // Configure GPIO9 for SCITXDB operation
//  GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 2;   // Configure GPIO14 for SCITXDB operation
//	GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 2;   // Configure GPIO18 for SCITXDB operation
  GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 3;   // Configure GPIO22 for SCITXDB operation

//  GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 2;   // Configure GPIO11 for SCIRXDB operation
//  GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 2;   // Configure GPIO15 for SCIRXDB operation
//    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 2;   // Configure GPIO19 for SCIRXDB operation
  GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3;   // Configure GPIO23 for SCIRXDB operation

    EDIS;
}



void InitScicGpio()
{
   EALLOW;


// Pull-ups can be enabled or disabled disabled by the user.
// This will enable the pullups for the specified pins.

	GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;    // Enable pull-up for GPIO62 (SCIRXDC)
	GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;	   // Enable pull-up for GPIO63 (SCITXDC)


// Inputs are synchronized to SYSCLKOUT by default.
// This will select asynch (no qualification) for the selected pins.

	GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;  // Asynch input GPIO62 (SCIRXDC)


// This specifies which of the possible GPIO pins will be SCI functional pins.

	GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;   // Configure GPIO62 for SCIRXDC operation
	GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;   // Configure GPIO63 for SCITXDC operation

    EDIS;
}

void scib_loopback_init(void)
{
    // Note: Clocks were turned on to the SCIB peripheral
    // in the InitSysCtrl() function

 	ScibRegs.SCICCR.all 			= 0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
	ScibRegs.SCICTL1.all 			= 0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
	ScibRegs.SCICTL2.all 			= 0x0003;
	ScibRegs.SCICTL2.bit.TXINTENA 	= 1;
	ScibRegs.SCICTL2.bit.RXBKINTENA = 1;
    ScibRegs.SCIHBAUD    			= 0x00;//0x00;-------------------------0x01
    ScibRegs.SCILBAUD    			= 0x1f;//0x1f:115200-------------------0x86:9600
	ScibRegs.SCICCR.bit.LOOPBKENA 	= 0; // enable loop back
	ScibRegs.SCICTL1.all 			= 0x0023;     // Relinquish SCI from Reset
}
// Initalize the SCI FIFO
void scib_fifo_init(void)
{
    ScibRegs.SCIFFTX.bit.TXFIFOXRESET	= 0;
    ScibRegs.SCIFFRX.bit.RXFIFORESET	= 0;
    ScibRegs.SCIFFTX.all				= 0xE028;
    ScibRegs.SCIFFTX.bit.TXFFIENA		= 0;

    ScibRegs.SCIFFTX.bit.TXFFINTCLR		= 1;
    ScibRegs.SCIFFRX.all				= 0x2028;

    ScibRegs.SCIFFRX.bit.RXFFIENA		= 1;
    ScibRegs.SCIFFCT.all				= 0x0;
}

void scic_loopback_init(void)
{
    // Note: Clocks were turned on to the SCIB peripheral
    // in the InitSysCtrl() function

 	ScicRegs.SCICCR.all 				= 0x0007;	// 1 stop bit,  No loopback
                                   	   	   	   	 	// No parity,8 char bits,
                                   	   	   	   	 	// async mode, idle-line protocol
	ScicRegs.SCICTL1.all 				= 0x0003;  	// enable TX, RX, internal SCICLK,
                                   	   	   	       	// Disable RX ERR, SLEEP, TXWAKE
	ScicRegs.SCICTL2.all 				= 0x0003;
	ScicRegs.SCICTL2.bit.TXINTENA 		= 1;
	ScicRegs.SCICTL2.bit.RXBKINTENA 	= 1;
    ScicRegs.SCIHBAUD    				= 0x00;		//0x00;-------------------------0x01
    ScicRegs.SCILBAUD    				= 0x1f;		//0x1f:115200-------------------0x86:9600
	ScicRegs.SCICCR.bit.LOOPBKENA		= 0; 		// enable loop back
	ScicRegs.SCICTL1.all				= 0x0023;   // Relinquish SCI from Reset
}
// Initalize the SCI FIFO
void scic_fifo_init(void)
{
    ScicRegs.SCIFFTX.bit.TXFIFOXRESET	= 0;
    ScicRegs.SCIFFRX.bit.RXFIFORESET 	= 0;
    ScicRegs.SCIFFTX.all 				= 0xE028;
    ScicRegs.SCIFFTX.bit.TXFFIENA 		= 0;

    ScicRegs.SCIFFTX.bit.TXFFINTCLR 	= 1;
    ScicRegs.SCIFFRX.all 				= 0x2028;

    ScicRegs.SCIFFRX.bit.RXFFIENA	 	= 1;
    ScicRegs.SCIFFCT.all 				= 0x0;
}

/*
 *Initialize SCI, including GPIO and configuration
 */
void Init_SCI(void)
{
	InitSciGpio();
	scib_loopback_init();
	scib_fifo_init();
	scic_loopback_init();
	scic_fifo_init();
}
//===========================================================================
// End of file.
//===========================================================================
