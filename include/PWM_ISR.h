#ifndef _PWM_ISR_H
#define _PWM_ISR_H


/*Switch the analog channel, plus 1 every time*/
void Pwm_ISR_Thread(void);

enum COMPARE
{
    LESS_THAN = 0,
    LARGE_THAN
};


typedef struct _FeedbackVarBuf{

	int32 forcebuf[10];
	int32 displacementbuf[10];
	int32 sumForce;
	int32 sumDisplacement;
	int32 maxForce;
	int32 minForce;
	int32 maxDisplacement;
	int32 minDisplacement;
	void (*updateMaxMin)(void);
}FeedbackVarBuf;

extern FeedbackVarBuf feedbackVarBuf;
extern Uint16 real3;
extern Uint16 GetCurrentHallValue(void);

void UpdateKeyValue(void);
#endif
