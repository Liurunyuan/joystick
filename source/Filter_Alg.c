#include <math.h>
#include "Filter_Alg.h"


#define SUMX 45L
#define SUMXPOW2 285L
#define SUMXPOW3 2025L
#define SUMXPOW4 15333L
#define SUMXPOW5 120825L
#define SUMXPOW6 978405L


//#define SUMX 190L
//#define SUMXPOW2 2470L
//#define SUMXPOW3 36100L
//#define SUMXPOW4 562666L
//#define SUMXPOW5 9133300L
//#define SUMXPOW6 152455810L


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

void clearSum(void) {
	sumParaDisplacement.sum_XY = 0;
	sumParaDisplacement.sum_Xpow2Y = 0;
	sumParaDisplacement.sum_Y = 0;

	sumParaForce.sum_XY = 0;
	sumParaForce.sum_Xpow2Y = 0;
	sumParaForce.sum_Y = 0;
}

FuncPara calFuncPara(SumPara sumPara){
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

}
void CalFuncPara(double force, double displace, int count){
	//force = force / 13107.0;
	//displace = displace / 13107.0;

	force = force / 1.0;
	displace = displace / 1.0;
	sumParaDisplacement.sum_XY += count * displace;
	sumParaDisplacement.sum_Xpow2Y += count * count * displace;
	sumParaDisplacement.sum_Y += displace;

	sumParaForce.sum_XY += count * force;
	sumParaForce.sum_Xpow2Y += count*count * force;
	sumParaForce.sum_Y += force;

	if(count >= (DATA_AMOUNT - 1)){
		funcParaDisplacement = calFuncPara(sumParaDisplacement);
		funcParaForce = calFuncPara(sumParaForce);
		clearSum();
	}
}
void clearSumB(void) {
	sumParaDisplacementB.sum_XY = 0;
	sumParaDisplacementB.sum_Xpow2Y = 0;
	sumParaDisplacementB.sum_Y = 0;

}
void CalFuncParaB(double displace, int count){
	//force = force / 13107.0;
	//displace = displace / 13107.0;
	int count_tmp = count;
	count = count - 10;


	displace = displace / 1000.0;
	sumParaDisplacementB.sum_XY += count * displace;
	sumParaDisplacementB.sum_Xpow2Y += count * count * displace;
	sumParaDisplacementB.sum_Y += displace;

	if(count_tmp >= 29){
		funcParaDisplacementb = calFuncPara(sumParaDisplacementB);
		clearSumB();
	}
}
