#ifndef _FILTERALG_H
#define _FILTERALG_H

typedef struct _SumPara{
	double sum_X;
	double sum_Y;
	double sum_Xpow2;
	double sum_Xpow3;
	double sum_Xpow4;
	double sum_XY;
	double sum_Xpow2Y;
}SumPara;


typedef struct _FuncPara{
	double a;
	double b;
	double c;
}FuncPara;


void CalFuncPara(double force, double displace, int count);

extern FuncPara funcParaDisplacement;
extern FuncPara funcParaForce;
#endif
