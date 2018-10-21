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
#include "SCI_ISR.h"
#include <string.h>
#include <stdio.h>

#define UART_PRINTF

#ifdef UART_PRINTF

int fputc(int _c, register FILE *_fp);
int fputs(const char *_ptr, register FILE *_fp);

#endif

#ifdef UART_PRINTF

int fputc(int _c, register FILE *_fp){
	while(ScicRegs.SCIFFTX.bit.TXFFST != 0){
	}

	ScicRegs.SCITXBUF = (unsigned char) _c;

	return ((unsigned char) _c);
}

int fputs(const char *_ptr, register FILE *_fp){
	unsigned int i, len;

	len = strlen(_ptr);

	for(i = 0; i < len; ++i){
		while(ScicRegs.SCIFFTX.bit.TXFFST != 0){
		}
		ScicRegs.SCITXBUF =(unsigned char) _ptr[i];
	}

	return len;
}

#endif

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

void delayfunction(int sec){
	int count;
	for(count = 0; count < sec; count++){
		++count;
	}
}
int PowerOnBIT(void)
{
	//TODO   implement here, figure out what need to check, what to do if BIT fail.
	return 0;
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
	//TODO need to implement
}
void test_sci_tx(void){
	ScicRegs.SCITXBUF = 0x85;
}
void test_spi_tx(void){
	int retry = 0;
	while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG == 1){
		retry ++;
		if(retry > 200){
				//return 0;
		}
	}
	SpiaRegs.SPITXBUF = 0x0001;
}
/***************************************************************
 *Name:						GlobleVarInit
 *Function:
 *Input:				    none
 *Output:					none
 *Author:					Simon
 *Date:						2018.10.20
 ****************************************************************/
void GlobleVarInit(void){
	gRS422RxQue.front = 0;
	gRS422RxQue.rear = 0;
	memset(gRS422RxQue.rxBuff, 0, sizeof(gRS422RxQue.rxBuff));
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
	GlobleVarInit();

	PowerOnBIT();
	while(1)
	{
		Start_main_loop();
		int i;
		for(i = 0; i < 1000; ++i){
			delayfunction(32000);
		}

		test_spi_tx();
		//test_sci_tx();
		UnpackRS422A();
	}
	//test
}
