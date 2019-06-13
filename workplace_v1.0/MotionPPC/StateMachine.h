#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H
#include "EnumDefines.h"
#include "controller.h"
#include "TrajGen_4OrderPoly.h"

#define INT_FINE_NUM         49
#define INT_COARSE_NUM       50
#define DOUBLE_FINE_NUM         80//50  YHT
#define DOUBLE_COARSE_NUM       100//50


typedef struct tagDAStruct
{
	int iCoilDA;//电机DA
	int iOffset;//线圈偏置
	int iLimit;//线圈限幅
}DAStruct;

typedef struct tagPosInfor
{
	double dActPos;
	double dSetPoint;
	double dError;
}PosInfor;

typedef struct tagAxis	//X轴，Y轴等等
{
	double dActPos;
	double dSetPoint;
	double dError;

	PosInfor posInfor;

	double dSolveMethodOne;
	double dSolveMethodTwo;
	double dActPosNotInUse;

	double dAffData;
	double dConCalc;
	double dFeedForwardCoef;

	double SingleFeedForward_x;
	double SingleFeedForward_x_Switch;
	double SingleFeedForward_y;
	double SingleFeedForward_y_Switch;

	int bIdent;	
	double dIdentGain;


	int bHome;
	int bIntr;
	int bIntrEnable;
	double dHomePos;
	int iHomeCounter;
	int bSwitch;
	int iAxisErrCheck;

	double dStepDis;//目前未区分闭环方式
	double dJogSpd;//目前未区分闭环方式

//////////////////////////////////滤波参数////////////////////////////////////
	//double LaserFilter_b[3];
	//double LaserFilter_a[3];
	double LaserFilterOutput[3];
	double LaserFilterInput[3];
//////////////////////////////////notch参数////////////////////////////////////
	double LaserNotch_b[3];
	double LaserNotch_a[3];
	double LaserNotchOutput[3];
	double LaserNotchInput[3];

	PID sPID; 
	PID sPID_Switch;

	LLC sLLC;
	LLC sLLC_Switch;

	LLC sFilter;
	
	FIR sFIRY;
	FIR sFIRY_Switch;
	FIR sFIRX;
	FIR sFIRX_Switch;

	AFC sAFC[4];
	AFC sAFC_Switch[4];

	AxisCMD axisCMD;//自由度指令
	AxisStatus axisStatus;//自由度状态
	CloseCMD closeCMD;//自由度切换指令
	CloseStatus closeStatus;//自由度切换状态指令

	ControllerType controllerType;//控制器使用选择
	ControllerType LaserControllerType;//激光解算控制器使用选择
	double (*OutCalc)();//控制器函数指针

	TrajMode trajMode;//轨迹独立跟随模式枚举
	TrajType trajType;//轨迹类型枚举
	TrajStatus trajStatus;//自由度走轨迹状态
	int (*TrajSine_Poly)(s_TrajParam_Sine_Poly*,double*,double*,double*);//轨迹函数指针

	//int TrajGen_4_Order_Poly(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c)

	int (*TrajGen4Order)(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c);

	////////////////////轨迹参数//////////////////////////////

	s_TrajParam_4_Order_Poly sTP_Run;
	s_TrajParam_Sine_Poly sine_Run;

	double snap;	//表示加加加速度
	double jerk;	//表示加加速度
	double acc;		//表示加速度
	double vel;		//表示速度
	double shift;	//表示位移
	int cTrig;		//轨迹起始标志位

	int iExpoType;//曝光轨迹类型，估计有两种曝光类型进行选择
	
	int iTrajStartPoint;//用于记录轨迹起始点位置
	int iTrajRealTimeFlag;//用于进入记录轨迹起始点位置的flag
	
	double bHomeDone;//yht
	double stepHome;//yht
	double HomeFlagCorse;
	int iDir;
	int iLaserDir;

	double simulateError;
	double simulateActPOs;
	int closeFlag;

//	int laserDCPowerLevel;
//	int laserACPowerLevel;
}Axis;//自由度

typedef struct tagStage
{
	Axis arrAxis[6];//YHT
	
	DAStruct DA_Struct[4];

	double dDALimit;
	
	double dConDist[8];//yht
	
	int iADC[8];
	int iDAC[8];//yht
	double dInputVol[8];
	int DAOffset[8];

///////////////////////台子滤波器，这里第一个index代表可用滤波器数目//////////////////////////////
	double Filter2order_b[12][3];
	double Filter2order_a[12][3];
	double Filter2orderOutput[12][3];
	double Filter2orderInput[12][3];
	int Filter2orderFlag[12];

	int iLaser[13];
	int iCounter[6];
	int iSenor[8];
	int laserDCPowerLevel[11];
	int laserACPowerLevel[11];

	int iLaserAfterFilter[8];

	StageCMD stageCMD;//台子指令
	StageStatus stageStatus;//台子状态
	StageType stageType;
 
	s_TrajParam_4_Order_Poly sTP_Expo[10];//曝光轨迹
	int FieldTransEnable;
	void (*Traj)();

	char TrajQueue[10];	
	double TrajParam[10];

	
	int iStageErrCheck;
	char DDX_Flag;
	char DDY_Flag;
	
}Stage;//台子(掩模台与硅片台)

typedef struct tagMachine
{
	Stage coarseStage;
	Stage fineStage;
	int iExecTime;
	int iTime1;
	int iTime2;
	int iFreq;
	int iServoTime;
	int iServoCnt;
	
	int stepFlagdown;
	int stepFlagup;

	int stepFlagdownP;
	int stepFlagupP;
	
	int iTimeHome;

	int iTimeHomeUpP;
	int iTimeHomeDownP;
	int iTimeHomeDownQ;
	double dIdentData;
	
	/*int stepHome;//yht
	int HomeFlagCorse;*/

	int fineStopFlag;
	int coarseStopFlag;

	int simulationFlag;
}MACHINE;//整机

typedef struct tagTransConParam
{
	int iCmdID;
	StageType stageType;
	int iDof;
	ParamType  paramType;
	int iOrder;
	double dParamter[4][24];
}TransConParam;

typedef struct tagComData
{
	int iCMD;
	int iFineData[INT_FINE_NUM];
	int iCoarseData[INT_COARSE_NUM];
	double dFineData[DOUBLE_FINE_NUM];
	double dCoarseData[DOUBLE_COARSE_NUM];
	TransConParam transConParam;
	
}ComData;

typedef struct tagRecvData
{
	int iCMD;
	StageType stageType;
	int iDof;
	ParamType  paramType;
	int iOrder;
	int iReserved[5];
	double dParamData[24];	
	TransConParam transConParam;
}RecvData;

extern RecvData recvData;
extern ComData comData;
extern MACHINE Machine;


extern void AxisStateMachine(Axis* pAxis,int iAxisNum);
extern void StageStateMachine(Stage * pStage);
extern void SwitchStateMachine(Axis* pAxis,int iAxisNum);

#endif
