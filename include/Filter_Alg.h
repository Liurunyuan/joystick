#ifndef _FILTERALG_H
#define _FILTERALG_H

typedef struct _SumPara{
	float sum_X;
	float sum_Y;
	float sum_Xpow2;
	float sum_Xpow3;
	float sum_Xpow4;
	float sum_XY;
	float sum_Xpow2Y;
}SumPara;


typedef struct _FuncPara{
	float a;
	float b;
	float c;
}FuncPara;


void CalFuncPara(int force, int displace, int count);

extern FuncPara funcParaDisplacement;
extern FuncPara funcParaForce;
#endif
