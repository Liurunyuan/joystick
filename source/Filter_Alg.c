#include <math.h>
#include "Filter_Alg.h"


#define SUMX 45
#define SUMXPOW2 285
#define SUMXPOW3 2025
#define SUMXPOW4 15333
#define SUMXPOW5 120825
#define SUMXPOW6 978405

SumPara sumPara = {
	SUMX,
	0,
	SUMXPOW2,
	SUMXPOW3,
	SUMXPOW4,
	0,
	0
};


FuncPara funcPara = {0,0,0};

void clearSum(void) {
	sumPara.sum_XY = 0;
	sumPara.sum_Xpow2Y = 0;
	sumPara.sum_Y = 0;
}

void calFuncPara(void){
	float temp,temp0,temp1,temp2;

	temp = 10 * (sumPara.sum_Xpow2 * sumPara.sum_Xpow4 -sumPara.sum_Xpow3*sumPara.sum_Xpow3)
			-  sumPara.sum_X* (sumPara.sum_X * sumPara.sum_Xpow4 - sumPara.sum_Xpow2 * sumPara.sum_Xpow3)
			+ sumPara.sum_Xpow2 * (sumPara.sum_X * sumPara.sum_Xpow3 - sumPara.sum_Xpow2 * sumPara.sum_Xpow2);

	temp0 = sumPara.sum_Y * (sumPara.sum_Xpow2 * sumPara.sum_Xpow4 - sumPara.sum_Xpow3*sumPara.sum_Xpow3)
			- sumPara.sum_XY*(sumPara.sum_X*sumPara.sum_Xpow4 - sumPara.sum_Xpow2*sumPara.sum_Xpow3)
			+ sumPara.sum_Xpow2Y*(sumPara.sum_X*sumPara.sum_Xpow3 - sumPara.sum_Xpow2*sumPara.sum_Xpow2);

	temp1 = 10 * (sumPara.sum_XY*SUMXPOW4 - sumPara.sum_Xpow2Y*SUMXPOW3)
			- SUMX*(sumPara.sum_Y*SUMXPOW4 - SUMXPOW2*sumPara.sum_Xpow2Y)
			+ SUMXPOW2*(sumPara.sum_Y*SUMXPOW3 - sumPara.sum_XY*SUMXPOW2);

	temp2 = 10 * (SUMXPOW2*sumPara.sum_Xpow2Y - sumPara.sum_XY*SUMXPOW3)
			- sumPara.sum_X*(sumPara.sum_X * sumPara.sum_Xpow2Y - sumPara.sum_Y*SUMXPOW3)
			+ SUMXPOW2*(SUMX*sumPara.sum_XY - sumPara.sum_Y*SUMXPOW2);

	funcPara.c = temp0/temp;
	funcPara.b = temp1/temp;
	funcPara.a = temp2/temp;

}
void CalFuncPara(int y,int count){
	/*
	if(count == 0){
		clearSum();
	}
	*/

	sumPara.sum_XY += count*y;
	sumPara.sum_Xpow2Y += count*count*y;
	sumPara.sum_Y += y;

	if(count >=9){
		calFuncPara();
		clearSum();
	}
}
