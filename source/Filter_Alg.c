#include <math.h>
#include "Filter_Alg.h"
#include "GlobalVarAndFunc.h"

#define FIRST_ORDER   (1)
#define SECOND_ORDER  (0)

#ifdef TEN_POINTS
//#define SUMX 45L
//#define SUMXPOW2 285L
//#define SUMXPOW3 2025L
//#define SUMXPOW4 15333L
//#define SUMXPOW5 120825L
//#define SUMXPOW6 978405L

#define SUMX (1.125)
#define SUMXPOW2 (0.178125)
#define SUMXPOW3 (0.031641)
#define SUMXPOW4 (0.005989)
#define SUMXPOW5 (0.001180)
#define SUMXPOW6 (0.000239)

//#define SUMXSPEED (11.25)
//#define SUMXPOW2SPEED (17.8125)
//#define SUMXPOW3SPEED (31.641)
//#define SUMXPOW4SPEED (59.89)
//#define SUMXPOW5SPEED (118)
//#define SUMXPOW6SPEED (238.868)
#define SUMXSPEED (1.125)
#define SUMXPOW2SPEED (0.178125)
#define SUMXPOW3SPEED (0.031641)
#define SUMXPOW4SPEED (0.005989)
#define SUMXPOW5SPEED (0.001180)
#define SUMXPOW6SPEED (0.000239)
#endif

#ifdef TWENTY_POINTS
#define SUMX 190L
#define SUMXPOW2 2470L
#define SUMXPOW3 36100L
#define SUMXPOW4 562666L
#define SUMXPOW5 9133300L
#define SUMXPOW6 152455810L
#endif
/***************************Add the speed sum parameters begin******************************** */
SumPara sumParaSpeed = {
	SUMXSPEED,
	0,
	SUMXPOW2SPEED,
	SUMXPOW3SPEED,
	SUMXPOW4SPEED,
	0,
	0
};
/***************************Add the speed sum parameters end******************************** */


SumPara sumParaDisplacement = {
	SUMX,
	0,
	SUMXPOW2,
	SUMXPOW3,
	SUMXPOW4,
	0,
	0
};
SumPara sumParaDisplacementB = {
	SUMX,
	0,
	SUMXPOW2,
	SUMXPOW3,
	SUMXPOW4,
	0,
	0
};

SumPara sumParaForce = {
	SUMX,
	0,
	SUMXPOW2,
	SUMXPOW3,
	SUMXPOW4,
	0,
	0
};



FuncPara funcParaDisplacement = {0,0,0};
FuncPara funcParaDisplacementb = {0,0,0};
FuncPara funcParaForce = {0,0,0};
FuncPara funcParaSpeed = {0,0,0};
#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(clearSum, "ramfuncs")
#endif
void clearSum(void) {
	sumParaDisplacement.sum_XY = 0;
	sumParaDisplacement.sum_Xpow2Y = 0;
	sumParaDisplacement.sum_Y = 0;

	// sumParaForce.sum_XY = 0;
	// sumParaForce.sum_Xpow2Y = 0;
	// sumParaForce.sum_Y = 0;
}
#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(clearSumSpeed, "ramfuncs")
#endif
void clearSumSpeed(void) {

	sumParaSpeed.sum_XY = 0;
	sumParaSpeed.sum_Xpow2Y = 0;
	sumParaSpeed.sum_Y = 0;
}
#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(Calc_LSM_Coef_Speed, "ramfuncs")
#endif
FuncPara Calc_LSM_Coef_Speed(SumPara sumPara){
	double temp,temp0,temp1;
	FuncPara funcPara;

	temp = (DATA_AMOUNT * sumParaSpeed.sum_Xpow2) - (sumParaSpeed.sum_X * sumParaSpeed.sum_X); 
	temp0 = (sumParaSpeed.sum_Y * sumParaSpeed.sum_Xpow2) - (sumParaSpeed.sum_X * sumParaSpeed.sum_XY);
	temp1 = (DATA_AMOUNT * sumParaSpeed.sum_XY) - (sumParaSpeed.sum_Y * sumParaSpeed.sum_X);
	funcPara.c = temp0 / temp;
	funcPara.b = temp1 / temp;
	funcPara.a = 0;
	return funcPara;
}

#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(Calc_10p_Error_Sum_Squares_Speed, "ramfuncs")
#endif
void Calc_10p_Error_Sum_Squares_Speed(double speed, int count){
    double tmpCount = count * 0.025;
	sumParaSpeed.sum_XY += tmpCount * speed;
	sumParaSpeed.sum_Xpow2Y += tmpCount * tmpCount * speed;
	sumParaSpeed.sum_Y += speed;
}

#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(Calc_LSM_Coef_Displace, "ramfuncs")
#endif
FuncPara Calc_LSM_Coef_Displace(SumPara sumPara){
#if(SECOND_ORDER)
	double temp,temp0,temp1,temp2;
	FuncPara funcPara;

	temp = DATA_AMOUNT * (sumPara.sum_Xpow2 * sumPara.sum_Xpow4 -sumPara.sum_Xpow3*sumPara.sum_Xpow3)
			-  sumPara.sum_X* (sumPara.sum_X * sumPara.sum_Xpow4 - sumPara.sum_Xpow2 * sumPara.sum_Xpow3)
			+ sumPara.sum_Xpow2 * (sumPara.sum_X * sumPara.sum_Xpow3 - sumPara.sum_Xpow2 * sumPara.sum_Xpow2);

	temp0 = sumPara.sum_Y * (sumPara.sum_Xpow2 * sumPara.sum_Xpow4 - sumPara.sum_Xpow3*sumPara.sum_Xpow3)
			- sumPara.sum_XY*(sumPara.sum_X*sumPara.sum_Xpow4 - sumPara.sum_Xpow2*sumPara.sum_Xpow3)
			+ sumPara.sum_Xpow2Y*(sumPara.sum_X*sumPara.sum_Xpow3 - sumPara.sum_Xpow2*sumPara.sum_Xpow2);

	temp1 = DATA_AMOUNT * (sumPara.sum_XY*SUMXPOW4 - sumPara.sum_Xpow2Y*SUMXPOW3)
			- SUMX*(sumPara.sum_Y*SUMXPOW4 - SUMXPOW2*sumPara.sum_Xpow2Y)
			+ SUMXPOW2*(sumPara.sum_Y*SUMXPOW3 - sumPara.sum_XY*SUMXPOW2);

	temp2 = DATA_AMOUNT * (SUMXPOW2*sumPara.sum_Xpow2Y - sumPara.sum_XY*SUMXPOW3)
			- sumPara.sum_X*(sumPara.sum_X * sumPara.sum_Xpow2Y - sumPara.sum_Y*SUMXPOW3)
			+ SUMXPOW2*(SUMX*sumPara.sum_XY - sumPara.sum_Y*SUMXPOW2);

	funcPara.c = temp0/temp;
	funcPara.b = temp1/temp;
	funcPara.a = temp2/temp;
	return funcPara;
#endif

#if(FIRST_ORDER)
	double temp,temp0,temp1;
	FuncPara funcPara;

	temp = (DATA_AMOUNT * sumPara.sum_Xpow2) - (sumPara.sum_X * sumPara.sum_X); 
	temp0 = (sumPara.sum_Y * sumPara.sum_Xpow2) - (sumPara.sum_X * sumPara.sum_XY);
	temp1 = (DATA_AMOUNT * sumPara.sum_XY) - (sumPara.sum_Y * sumPara.sum_X);
	funcPara.c = temp0 / temp;
	funcPara.b = temp1 / temp;
	funcPara.a = 0;
	return funcPara;
#endif
}

#if(COPY_FLASH_CODE_TO_RAM == INCLUDE_FEATURE)
#pragma CODE_SECTION(Cal_10p_Error_Sum_Squares_Displace, "ramfuncs")
#endif
void Calc_10p_Error_Sum_Squares_Displace(double displace, int count){

    double tmpCount = count * 0.025;
	sumParaDisplacement.sum_XY += tmpCount * displace;
	sumParaDisplacement.sum_Xpow2Y += tmpCount * tmpCount * displace;
	sumParaDisplacement.sum_Y += displace;
}

