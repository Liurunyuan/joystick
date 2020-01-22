#ifndef _FILTERALG_H
#define _FILTERALG_H


#define TEN_POINTS
//#define TWENTY_POINTS

#ifdef TEN_POINTS
#define DATA_AMOUNT (7) //for 7 points. change this value.
#endif

#ifdef TWENTY_POINTS
#define DATA_AMOUNT (20)
#endif

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
void CalFuncParaSpeed(double speed, int count);
FuncPara calFuncPara(SumPara sumPara);
FuncPara calFuncParaSpeed(SumPara sumPara);
void clearSumSpeed(void);
void clearSum(void);
void InitFilterAlg(int pointsSpeed, int pointsAcc);

extern FuncPara funcParaDisplacement;
extern FuncPara funcParaDisplacementb;
extern FuncPara funcParaSpeed;
extern FuncPara funcParaForce;
extern SumPara sumParaDisplacementB;
extern SumPara sumParaDisplacement;
extern SumPara sumParaSpeed;
#endif
