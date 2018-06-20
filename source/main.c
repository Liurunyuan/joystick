/*
 * main.c
 *
 *
 * Author:					Simon
 * Date:					2018.6.7
 * Corporation:				RunZhang
 *
 * */
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "main.h"

/*git test*/

/************************************************************
 *Name:						Init_Peripheral
 *Function:					Initialize all the peripherals,
							including ADC,SCI,PWM,GPIO initialization and configuration
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.6.10
 ************************************************************/
void Init_Peripheral(void)
{
	/*Init IO pin */
	Init_GPIO();
	/*Init and config ADC*/
	Init_ADC();
	/*Init and cnofig XINTF*/
	Init_XINTF();
	/*Init and config SCI RS422*/
	Init_SCI();
	/*Init and config SPI*/
	Init_SPI();
	/*Init and config A-CAN and B-CAN */
	Init_CAN();
	/*Init and config I2C*/
	Init_I2C();
	/*Init and config CAP4,CAP5,CAP6*/
	Init_CAP();
	/*Init and config QEP2*/
	Init_QEP();
	/*PWM IO init and config*/
	Init_PWM();
}

/*************************************************************
 *Name:						FeedWatchDog
 *Function:					Feed the watch dog
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.6.10
 *************************************************************/
void FeedWatchDog(void)
{
	TOOGLE_WATCHDOG = TRUE;
}
/**************************************************************
 *Name:						Start_main_loop
 *Function:					Business logic
 *Input:					none
 *Output:					none
 *Author:					Simon
 *Date:						2018.6.10
 **************************************************************/
void Start_main_loop(void)
{
	/*tbd-----------------*/
	FeedWatchDog();
}

/***************************************************************
 *Name:						main
 *Function:
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.6.10
 ****************************************************************/
void main(void) {

	/*system init*/
	InitSysCtrl_M();
	/*peripheral init*/
	Init_Peripheral();
	/*interrupt init*/
	Init_Interrupt();

	while(1)
    {
		Start_main_loop();
	}
}












