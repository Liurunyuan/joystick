#ifndef PID_H_
#define PID_H_

typedef struct _PIDPARA{
    int16 kp_velocity_ODE;
    double ki_velocity_ODE;

    int16 kp_force_ODE;
    double ki_force_ODE;

    int16 kp_velocity_NULL;
    double ki_velocity_NULL;

    int16 kp_force_NULL;
    double ki_force_NULL;

    double K_F_ODE;
    double B_F_ODE;
    double K_F_NULL;
    double B_F_NULL;
    double K_V_ODE;
    double B_V_ODE;
    double K_V_NULL;
    double B_V_NULL;
}PIDPARA;


extern volatile int gTargetSpeed;
extern volatile PIDPARA gPidPara;



void InitPidVar(void);
//int32 displace_PidOutput(double targetVal, double controlVar);
int16 force_PidOutput(double targetVal, double controlVar);
int16 velocity_PidOutput(double targetVal, double controlVar);

#endif
