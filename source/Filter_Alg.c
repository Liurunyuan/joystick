#include <math.h>
#include "Filter_Alg.h"


#define SUMX 45
#define SUMXPOW2 285
#define SUMXPOW3 2025
#define SUMXPOW4 15333
#define SUMXPOW5 120825
#define SUMXPOW6 978405

SumPara sumParaDisplacement = {
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
FuncPara funcParaForce = {0,0,0};

void clearSum(void) {
	sumParaDisplacement.sum_XY = 0;
	sumParaDisplacement.sum_Xpow2Y = 0;
	sumParaDisplacement.sum_Y = 0;

	sumParaForce.sum_XY = 0;
	sumParaForce.sum_Xpow2Y = 0;
	sumParaForce.sum_Y = 0;
}

FuncPara calFuncPara(SumPara sumPara){
	float temp,temp0,temp1,temp2;
	FuncPara funcPara;

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
	return funcPara;

}
void CalFuncPara(int force, int displace, int count){
	/*
	if(count == 0){
		clearSum();
	}
	*/

	sumParaDisplacement.sum_XY += count * displace;
	sumParaDisplacement.sum_Xpow2Y += count * count * displace;
	sumParaDisplacement.sum_Y += displace;

	sumParaForce.sum_XY += count * force;
	sumParaForce.sum_Xpow2Y += count*count * force;
	sumParaForce.sum_Y += force;

	if(count >=9){
		funcParaDisplacement = calFuncPara(sumParaDisplacement);
		funcParaForce = calFuncPara(sumParaForce);
		clearSum();
	}
}
