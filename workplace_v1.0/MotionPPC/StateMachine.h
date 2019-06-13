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
	int iCoilDA;//���DA
	int iOffset;//��Ȧƫ��
	int iLimit;//��Ȧ�޷�
}DAStruct;

typedef struct tagPosInfor
{
	double dActPos;
	double dSetPoint;
	double dError;
}PosInfor;

typedef struct tagAxis	//X�ᣬY��ȵ�
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

	double dStepDis;//Ŀǰδ���ֱջ���ʽ
	double dJogSpd;//Ŀǰδ���ֱջ���ʽ

//////////////////////////////////�˲�����////////////////////////////////////
	//double LaserFilter_b[3];
	//double LaserFilter_a[3];
	double LaserFilterOutput[3];
	double LaserFilterInput[3];
//////////////////////////////////notch����////////////////////////////////////
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

	AxisCMD axisCMD;//���ɶ�ָ��
	AxisStatus axisStatus;//���ɶ�״̬
	CloseCMD closeCMD;//���ɶ��л�ָ��
	CloseStatus closeStatus;//���ɶ��л�״ָ̬��

	ControllerType controllerType;//������ʹ��ѡ��
	ControllerType LaserControllerType;//������������ʹ��ѡ��
	double (*OutCalc)();//����������ָ��

	TrajMode trajMode;//�켣��������ģʽö��
	TrajType trajType;//�켣����ö��
	TrajStatus trajStatus;//���ɶ��߹켣״̬
	int (*TrajSine_Poly)(s_TrajParam_Sine_Poly*,double*,double*,double*);//�켣����ָ��

	//int TrajGen_4_Order_Poly(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c)

	int (*TrajGen4Order)(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c);

	////////////////////�켣����//////////////////////////////

	s_TrajParam_4_Order_Poly sTP_Run;
	s_TrajParam_Sine_Poly sine_Run;

	double snap;	//��ʾ�ӼӼ��ٶ�
	double jerk;	//��ʾ�Ӽ��ٶ�
	double acc;		//��ʾ���ٶ�
	double vel;		//��ʾ�ٶ�
	double shift;	//��ʾλ��
	int cTrig;		//�켣��ʼ��־λ

	int iExpoType;//�ع�켣���ͣ������������ع����ͽ���ѡ��
	
	int iTrajStartPoint;//���ڼ�¼�켣��ʼ��λ��
	int iTrajRealTimeFlag;//���ڽ����¼�켣��ʼ��λ�õ�flag
	
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
}Axis;//���ɶ�

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

///////////////////////̨���˲����������һ��index��������˲�����Ŀ//////////////////////////////
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

	StageCMD stageCMD;//̨��ָ��
	StageStatus stageStatus;//̨��״̬
	StageType stageType;
 
	s_TrajParam_4_Order_Poly sTP_Expo[10];//�ع�켣
	int FieldTransEnable;
	void (*Traj)();

	char TrajQueue[10];	
	double TrajParam[10];

	
	int iStageErrCheck;
	char DDX_Flag;
	char DDY_Flag;
	
}Stage;//̨��(��ģ̨���Ƭ̨)

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
}MACHINE;//����

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
