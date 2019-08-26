#ifndef PID_H_
#define PID_H_

typedef struct _PIDPARA{
    int16 kp_displace;
    int16 ki_displace;
    int16 kd_displace;
    //int32 targetPid_displace;
    int16 kp_force;
    double ki_force;
    int16 kd_force;
    //int32 targetPid_force;
}PIDPARA;


extern volatile int gTargetSpeed;
extern volatile PIDPARA gPidPara;



void InitPidVar(void);
//int32 displace_PidOutput(double targetVal, double controlVar);
int16 force_PidOutput(double targetVal, double controlVar);

#endif
