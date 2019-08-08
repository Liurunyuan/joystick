#ifndef PID_H_
#define PID_H_

typedef struct _PIDPARA{
    int32 kp;
    int32 ki;
    int32 kd;
    int32 targetPid;
}PIDPARA;


extern volatile int gTargetSpeed;
extern volatile PIDPARA gPidPara;



void InitPidVar(void);

#endif
