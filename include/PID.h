#ifndef PID_H_
#define PID_H_

typedef struct _PIDPARA{
    int16 kp_velocity;
    double ki_velocity;

    int16 kp_force;
    double ki_force;
}PIDPARA;


extern volatile int gTargetSpeed;
extern volatile PIDPARA gPidPara;



void InitPidVar(void);
//int32 displace_PidOutput(double targetVal, double controlVar);
int16 force_PidOutput(double targetVal, double controlVar);
int16 velocity_PidOutput(double targetVal, double controlVar);

#endif
