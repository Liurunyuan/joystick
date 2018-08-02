/***************************************************/
//#define FLASH

//以下是上电科项目中用到的宏定义，开始于2016.6.12，yjs
#define EPWM1_TIMER_TBPRD 4286     //连续增减计数模式：6000--10k，3000--20k，4286--14K,120M/2/14K=4286
#define EPWM2_TIMER_TBPRD 4286     //6000--10k，3000--20k，4286--14K
#define EPWM1_TIMER_HALF_TBPRD  2143            //
#define EPWM2_TIMER_HALF_TBPRD  2143
//三种运行模式：上位机控制运行模式，独立运行模式，上位机控制调试模式
#define PC_Control_Mode 0
#define Independent_Run_Mode 1
#define PC_Debug_mode 2
//五种运行状态：停机状态、报警状态、初始化状态、正转运行状态、反转运行状态
#define Stop_State 0
#define Alarm_State 1
#define Initial_State 2
#define Forward_Run_State 3
#define Reverse_Run_State 4
#define RS422_CMD_Stop 0
#define RS422_CMD_Frun 3
#define RS422_CMD_Rrun 4

#define Tx_buf_len 10 /*每次最大发送数据包数量*/
#define Rx_buf_len 128 /*接收环形缓冲区长度*/

#define OUT_LIM_max 2078  /*半周期2143*0.97*/
#define OUT_LIM_min  64  /*半周期2143*0.03*/
#define ADD_step 1  /*参数缓变步长*/


#define TRUE 1
#define FALSE 0

#define Duty_change_max 5

//以上是上电科项目中用到的宏定义，开始于2016.6.12，yjs

#define PWM_MAX 5400 //   双极性100%
#define PWM_MIN 0		//   双极性

 
#define DEAD_TIME 360	//60---0.5uS,90--0.75uS,180--1.5uS,=DBFED*TTBCLK


#define PWM_TO_SPWM_SPEED 50
#define SPWM_TO_PWM_SPEED 30

#define PWM_MODE 0
#define SPWM_MODE 1

#define MOTOR_POLE 8
#define ANTICLOCKWISE 1
#define DEASIL 0

#define PWM_INTERVAL_TIME  200   //308--14k,  800--5k，571--7k，400--10k，308--13k，267--15k，235--17k，200--20k，PWM周期(us),100us,10kHz, 为了提高精度,将其周期*4.

#define SPEED_P_ERROR_MAX 30000
#define SPEED_P_ERROR_MIN -30000
#define SPEED_I_ERROR_MAX 30000
#define SPEED_I_ERROR_MIN -30000
#define SPEED_D_ERROR_MAX 30000
#define SPEED_D_ERROR_MIN -30000

#define SPEED_INTEGRAL_ERROR_LIMIT 50


#define  PID_P  400
#define  PID_I  400
#define  PID_D  0

/***************************************************/

#define FED_DOG   (GpioDataRegs.GPBDAT.bit.GPIO48=!GpioDataRegs.GPBDAT.bit.GPIO48)

//#define  VDC_GOAL_LOW      2790        //370V_30kW_1#
//#define  VDC_GOAL_HIGH      2790       //370V_30kW_1#

#define  VDC_GOAL_LOW      2770        //370V_75kW_3#
#define  VDC_GOAL_HIGH      2770       //370V_75kW_3#

//#define  VDC_GOAL_LOW      2780        //370V_30kW_2#
//#define  VDC_GOAL_HIGH      2780       //370V_30kW_2#

#define BUS_VDC_MAX 3200     // 410V----3200  (390,    2014.05.13)
#define BUS_VDC_MAX2 2580     //350V---2580   (330,     2014.05.13)
#define BUS_VDC_MIN 10//2238     //(电池312V，低于270V为欠压)

#define NBQ_START_VDC 1500     //   250V
#define NBQ_STOP_VDC 150     // 120V


/***************************************************/

#define  PID_P  400
#define  PID_I  400
#define  PID_D  0
/*Add by Simon on 2018.6.10*/
#define OUTPUT 	1
#define INPUT 	0
#define GPIO	0

struct ProtectPara
{
	Uint16 aChanel_aPhaseCurrent_Max;
	Uint16 aChanel_aPhaseCurrent_Min;
	Uint16 aChanel_bPhaseCurrent_Max;
	Uint16 aChanel_bPhaseCurrent_Min;
	Uint16 aChanel_busCurrent_Max;
	Uint16 aChanel_busCurrent_Min;
	Uint16 bChanel_aPhaseCurrent_Max;
	Uint16 bChanel_aPhaseCurrent_Min;
	Uint16 bChanel_bPhaseCurrent_Max;
	Uint16 bChanel_bPhaseCurrent_Min;
	Uint16 bChanel_busCurrent_Max;
	Uint16 bChanel_busCurrent_Min;
	Uint16 aTemp_Max;
	Uint16 aTemp_Min;
	Uint16 bTemp_Max;
	Uint16 bTemp_Min;
};

struct ControlPara
{
	Uint16 Target_Speed;
	Uint16 Target_PositionX;
	Uint16 Target_PositionY;
	Uint16 Kp_Speed;
	Uint16 Ki_Speed;
	Uint16 Kd_Speed;
};

struct SystemStatusPara
{
	Uint16 System_Temp;
	Uint16 Motor_Speed;
	Uint16 Motor_PositionX;
	Uint16 Motro_PositionY;
};


enum Bool{
	False = 0,
	True
};




