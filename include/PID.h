#ifndef PID_H_
#define PID_H_

typedef struct _PIDPARA{
    int32 kp_displace;
    int32 ki_displace;
    int32 kd_displace;
    //int32 targetPid_displace;
    int32 kp_force;
    int32 ki_force;
    int32 kd_force;
    //int32 targetPid_force;
}PIDPARA;


extern volatile int gTargetSpeed;
extern volatile PIDPARA gPidPara;



void InitPidVar(void);
int32 displace_PidOutput(double targetVal, double controlVar);

#endif
