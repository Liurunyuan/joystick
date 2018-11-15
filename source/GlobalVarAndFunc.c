#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "public.h"
#include "GlobalVarAndFunc.h"


Uint32 gECapCount;

Uint16 rs422ShakeHandMsg[] = {
	0x5a,//head1
	0x5a,//head2
	0x01,//length
	0x00,//serial number
	0x00,//serial number
	0xff,//shake hand
	0xff,//shake hand
	0xff,//shake hand
	0x00,//crc1
	0x00,//crc2
	0xa5,//tail1
	0xa5 //tail2
};

bool ShakeHandWithUpperComputer(void){
	int len;
	int index;

	len = sizeof(rs422ShakeHandMsg);

	for(index = 0; index < len; ++index){
		while(ScicRegs.SCIFFTX.bit.TXFFST != 0){

		}
		ScicRegs.SCITXBUF = rs422ShakeHandMsg[index];
	}

	return 0;
}
