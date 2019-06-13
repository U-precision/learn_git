#ifndef ENUM_DEFINES_H
#define ENUM_DEFINES_H
//���ɶ�ö��
typedef enum tagStageType
{
	FineStage = 1,
	CoarseStage = 2	
	
}StageType;
typedef enum tagAxisType
{
	FineAxis = 1,
	CoarseAxis = 2	
}AxisType;

typedef enum tagAxisIndex
{
	DX = 0, 
	DY = 1, 
	TZ = 2
}AxisIndex;
typedef enum tagAxisFineIndex
{
	FDX = 0, 
	FDY = 1, 
	FTZ = 2,
	FDZ = 3, 
	FTX = 4, 
    FTY = 5, 
	ALLREFRESH_FINE = 98	
}AxisFineIndex;

typedef enum tagAxisCoarseIndex
{
	CDX = 0, 
	CDY = 1, 
	XTZ = 2, 
	YTZ = 3, 
	ALLREFRESH_COARSE = 98
}AxisCoarseIndex;

//�켣״̬
typedef enum tagTrajStatus
{
	Stop = 0, 
	Positive_ACC = 1, 
	Speed = 2,
	Negative_ACC = 3, 
	OT = 4
}TrajStatus;

//΢��̨��ֶ�̨��������ģʽ
typedef enum tagTrajMode
{
	TrajNothing = 0,
	IsTrajIndependent = 1,
	IsTrajFollow = 2
	
}TrajMode;

//���߹켣ģʽ��3�׹켣���Ľ׹켣�����ҹ켣��
typedef enum tagTrajOrder
{	
	isTrajSineAccT= 0,
	isTraj3OrderPoly = 1,
	isTraj4OrderPoly= 2
}TrajType;

//̨������ö��
typedef enum tagStageCMD
{
	C_All_Nothing = 0,
	C_Stage_Traj_Expo = 6051, 
	C_Stage_ALL_Close = 6052, 
	C_Stage_ALL_Open = 6053,
	C_Stage_ALL_Step_For = 6054, 
	C_Stage_ALL_Step_Rev = 6055,
	C_Stage_ALL_Jogfor_Start = 6056, 
	C_Stage_ALL_Jogrev_Start = 6057,
	C_Stage_ALL_Jog_Stop = 6058, 
	C_Stage_P2P_Traj = 6059,
	C_Stage_Circle_Traj = 6060
}StageCMD;

//���ɶ�����ö��
typedef enum tagAxisCMD
{
	C_Nothing = 0,
	C_Aixs_Close = 6001, 
	C_Aixs_Open = 6002, 
	C_Aixs_Step_For = 6003, 
	C_Aixs_Step_Rev = 6004,
	C_Aixs_Jogfor_Start = 6005, 
	C_Aixs_Jog_Stop = 6006, 
	C_Aixs_Ident = 6007,
	C_Aixs_Jogrev_Start = 6008, 
	C_Aixs_RunFor = 6009, 
	C_Aixs_RunRev = 6010,
	C_Aixs_Home = 6011, 
	C_Aixs_Center_Switch = 6012, 
	C_Aixs_Switch_Ruler = 6049
}AxisCMD;

//̨��״̬ö��
typedef enum tagStageStatus
{
	S_Stage_Traj_Expo = 6551, 
	S_Stage_ALL_Close = 6552, 
	S_Stage_ALL_Open = 6553,
	S_Stage_ALL_Jogfor_Start = 6554, 
	S_Stage_ALL_Jogrev_Start = 6555, 
	S_Stage_P2P_Traj = 6556,
	S_Stage_Load_Pos = 6557,
	S_Stage_Circle_Traj = 6558
}StageStatus;

//���ɶ�״̬ö��
typedef enum tagAxisStatus
{
	S_Aixs_Close = 6501, 
	S_Aixs_Open = 6502, 
	S_Aixs_Jogfor_Start = 6503, 
	S_Aixs_Ident = 6504,
	S_Aixs_Jogrev_Start = 6505, 
	S_Aixs_RunFor = 6506, 
	S_Aixs_RunRev = 6507, 
	S_Aixs_Home = 6508,
	S_Aixs_Switch_Ruler = 6549
}AxisStatus;

//�ջ�����ö��
typedef enum tagCloseCMD
{
	C_Close_Nothing = 0, 
	C_Counter_Close = 6012, 
	C_Laser_Close = 6013
}CloseCMD;

//�ջ�״̬ö��
typedef enum tagCloseStatus
{
	S_Counter_Close = 6512, 
	S_Laser_Close = 6513
}CloseStatus;

typedef enum tagParamType
{
	PID_INDEPENDENT_TYPE = 11,               //����ģʽPID����
	PID_FOLLOW_TYPE = 12,                   //����ģʽPID����
	LLC_INDEPENDENT_TYPE = 21,               //����ģʽLLC����
	LLC_FOLLOW_TYPE = 22,                    //����ģʽLLC����
	LIMIT_TYPE = 31,                        //�޷�
	FIR_INDEPENDENT_DXMOVE_TYPE = 41,         //dx�켣ʱ������ģʽFIR����
	FIR_FOLLOW_DXMOVE_TYPE = 42,             //dx�켣ʱ������ģʽFIR����
	FIR_INDEPENDENT_DYMOVE_TYPE = 51,         //dy�켣ʱ������ģʽFIR����
	FIR_FOLLOW_DYMOVE_TYPE = 52,             //dy�켣ʱ������ģʽFIR����
	ACC_COEFFICIENT_INDEPENDENT_DXMOVE = 61,   //dx�켣ʱ������ģʽ���ٶ�ǰ��ϵ��
	ACC_COEFFICIENT_FOLLOW_DXMOVE = 62,       //dx�켣ʱ������ģʽ���ٶ�ǰ��ϵ��
	ACC_COEFFICIENT_INDEPENDENT_DYMOVE = 71,   //dy�켣ʱ������ģʽ���ٶ�ǰ��ϵ��
	ACC_COEFFICIENT_FOLLOW_DYMOVE = 72,        //dy�켣ʱ������ģʽ���ٶ�ǰ��ϵ��
	
}ParamType;

//������ʹ��λ
typedef enum tagControllerType
{
	PID_flag = 0, 
	LLC_flag = 1
	
}ControllerType;


typedef enum tagTrajLast
{
	FirstIn = 0, 
	RunLast = 1
	
}TrajLast;

typedef enum tagLimitStatus
{
	Underlimit = 0, 
	Overlimit = 1
	
}LimitStatus;
#endif
