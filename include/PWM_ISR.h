#ifndef _PWM_ISR_H
#define _PWM_ISR_H


/*Switch the analog channel, plus 1 every time*/
void Pwm_ISR_Thread(void);


typedef struct _KeyValue{
	int32 force;
	int32 displacement;
	int32 motorSpeed;
	int32 motorAccel;
	int32 lock;
}KeyValue;

typedef struct _FeedbackVarBuf{

	int32 forcebuf[10];
	int32 displacementbuf[10];
	int32 maxForce;
	int32 minForce;
	int32 maxDisplacement;
	int32 minDisplacement;
}FeedbackVarBuf;












#endif
