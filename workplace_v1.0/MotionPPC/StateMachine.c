#include "StateMachine.h"
#include "rtcontrol.h"
#include "TrajHandler.h"
#include "FileReadWrite.h"
#include "CMD.h"
#include "SingleTPG.h"

MACHINE Machine;

void AxisStateMachine(Axis* pAxis,int iAxisNum)
{
	int iCurStatus = pAxis->axisStatus;
	if(pAxis->closeStatus == S_Laser_Close)
	{	
		pAxis->dActPos=pAxis->dSolveMethodTwo;
		pAxis->dActPosNotInUse=pAxis->dSolveMethodOne;
	}
	else
	{
		pAxis->dActPos=pAxis->dSolveMethodOne;
		pAxis->dActPosNotInUse=pAxis->dSolveMethodTwo;
	}
	if(pAxis->axisStatus!=S_Aixs_Open)
	{
		if(pAxis->vel ==0 )
		{
			pAxis->closeFlag=1;
		}
		else if(pAxis->vel<=TP.v)
		{
			pAxis->closeFlag=2;
		}
		else if(pAxis->vel>=TP.v)
		{
			pAxis->closeFlag=2;
		}
	}
	switch(iCurStatus)
	{
		case S_Aixs_Open:
			Open(pAxis,iAxisNum);
			if(pAxis->axisCMD == C_Aixs_Close)
			{
				pAxis->axisStatus = S_Aixs_Close;
			}
			pAxis->axisCMD = C_Nothing;
		break;
		case S_Aixs_Close:
			Close(pAxis,iAxisNum);

			//����ı�״̬
			if(pAxis->axisCMD == C_Aixs_Open)
			{
				pAxis->axisStatus = S_Aixs_Open;
			}
			else if(pAxis->axisCMD == C_Aixs_Step_For)
			{
				StepFor(pAxis,iAxisNum);
				pAxis->axisStatus = S_Aixs_Close;
			}
			else if(pAxis->axisCMD == C_Aixs_Step_Rev)
			{
				StepRev(pAxis,iAxisNum);
				pAxis->axisStatus = S_Aixs_Close;
			}
			else if(pAxis->axisCMD == C_Aixs_Jogfor_Start)
			{
				pAxis->axisStatus = S_Aixs_Jogfor_Start;
			}
			else if(pAxis->axisCMD == C_Aixs_Jogrev_Start)
			{
				pAxis->axisStatus = S_Aixs_Jogrev_Start;
			}
			else if(pAxis->axisCMD == C_Aixs_Home)
			{
				pAxis->axisStatus = S_Aixs_Home;
			}
			else if(pAxis->axisCMD == C_Aixs_RunFor)
			{
				pAxis->axisStatus = S_Aixs_RunFor;	
			}
			else if(pAxis->axisCMD == C_Aixs_RunRev)
			{
				pAxis->axisStatus = S_Aixs_RunRev;
			}
			else if(pAxis->axisCMD == C_Aixs_Ident)
			{
				pAxis->axisStatus = S_Aixs_Ident;
			}		
			pAxis->axisCMD = C_Nothing;
		break;		
		case S_Aixs_Jogfor_Start:
			JogFor(pAxis,iAxisNum);
			if(pAxis->axisCMD == C_Aixs_Jog_Stop)
			{
				pAxis->axisStatus = S_Aixs_Close;
			}
			if(pAxis->axisCMD == C_Aixs_Open)
			{
				pAxis->axisStatus = S_Aixs_Open;
			}
		break;
		case S_Aixs_Jogrev_Start:
			JogRev(pAxis,iAxisNum);
			if(pAxis->axisCMD == C_Aixs_Jog_Stop)
			{
				pAxis->axisStatus = S_Aixs_Close;
			}
			if(pAxis->axisCMD == C_Aixs_Open)
			{
				pAxis->axisStatus = S_Aixs_Open;
			}
		break;
		case S_Aixs_Ident:
//			Ident(pAxis,iAxisNum);
//			Close(pAxis,iAxisNum);
		break;
		case S_Aixs_RunFor:
			//�����ܳ��˿�������κ����Ŀǰû�м�ͣ
			XYRun_Forward(pAxis,iAxisNum);
			if(pAxis->axisCMD == C_Aixs_Open)
			{
				pAxis->axisStatus = S_Aixs_Open;
				pAxis->iTrajRealTimeFlag = 0;
				pAxis->dAffData = 0.0;				
			}
		break;
		case S_Aixs_RunRev:	
			//�����ܳ��˿�������κ����Ŀǰû�м�ͣ
			XYRun_Reverse(pAxis,iAxisNum);
//			printf("haha\n");
			if(pAxis->axisCMD == C_Aixs_Open)
			{
				pAxis->axisStatus = S_Aixs_Open;
				pAxis->iTrajRealTimeFlag = 0;
				pAxis->dAffData = 0.0;
			}
		break;
		case S_Aixs_Home:
			//�����ܳ��˿�������κ�����
			Home(pAxis,iAxisNum);
			if(pAxis->axisCMD == C_Aixs_Open)
			{
				pAxis->axisStatus = S_Aixs_Open;
			}	
		break;
	}
}

void StageStateMachine(Stage *pStage)
{
	int iCurStatus = pStage->stageStatus;
	if(pStage->stageType == CoarseStage)
	{
		switch(iCurStatus)
		{
			case S_Stage_ALL_Open:
				if(pStage->arrAxis[CDX].axisStatus == S_Aixs_Close && pStage->arrAxis[CDY].axisStatus == S_Aixs_Close && pStage->arrAxis[XTZ].axisStatus == S_Aixs_Close )//&& pStage->arrAxis[YTZ].axisStatus == S_Aixs_Close)
				{
					pStage->stageStatus= S_Stage_ALL_Close;
				}
				
				pStage->stageCMD = C_All_Nothing;
			break;
			case S_Stage_ALL_Close:
				if(pStage->arrAxis[CDX].axisStatus == S_Aixs_Open && pStage->arrAxis[CDY].axisStatus == S_Aixs_Open && pStage->arrAxis[XTZ].axisStatus == S_Aixs_Open && pStage->arrAxis[YTZ].axisStatus == S_Aixs_Open)
				{
					pStage->stageStatus = S_Stage_ALL_Open;
				}
				else if(pStage->stageCMD == C_Stage_Traj_Expo)
				{
					pStage->stageStatus = S_Stage_Traj_Expo;
				}
				else if(pStage->stageCMD == C_Stage_P2P_Traj)
				{
					pStage->stageStatus = S_Stage_P2P_Traj;
				}
				else if(pStage->stageCMD == C_Stage_Circle_Traj)
				{
					pStage->stageStatus = S_Stage_Circle_Traj;
				}
				pStage->stageCMD = C_Nothing;
			break;
			case  S_Stage_Traj_Expo:    
				if(pStage->DDX_Flag==1 && pStage->DDY_Flag==1)
				{
					Traj_Coarse();
				}
				else
					pStage->Traj();

				if(pStage->arrAxis[CDX].axisStatus == S_Aixs_Open && pStage->arrAxis[CDY].axisStatus == S_Aixs_Open && pStage->arrAxis[XTZ].axisStatus == S_Aixs_Open && pStage->arrAxis[YTZ].axisStatus == S_Aixs_Open)
				{	 			
					pStage->stageStatus = S_Stage_ALL_Open;
					Coarse_Count = 0;
					pStage->stageCMD = C_All_Nothing;
					DXPosLast_Coarse = FirstIn;
					DXNegLast_Coarse = FirstIn;
					DYPosLast_Coarse = FirstIn;
					DYNegLast_Coarse = FirstIn;
					logMsg("The Traj is stop!!\n",0,0,0,0,0,0);
				}
			break;
			case S_Stage_P2P_Traj:

			break;
			case S_Stage_Circle_Traj:
				Circle(pStage);
				break;
		}
	}
	else if(pStage->stageType == FineStage)
	{
		switch(iCurStatus)
		{
			case S_Stage_ALL_Open:
				if(pStage->arrAxis[FDX].axisStatus == S_Aixs_Close && pStage->arrAxis[FDY].axisStatus == S_Aixs_Close && pStage->arrAxis[FTZ].axisStatus == S_Aixs_Close)
				{
					pStage->stageStatus = S_Stage_ALL_Close;
				}
				pStage->stageCMD = C_All_Nothing;
			break;
			case S_Stage_ALL_Close:
				if(pStage->arrAxis[FDX].axisStatus == S_Aixs_Open && pStage->arrAxis[FDY].axisStatus == S_Aixs_Open && pStage->arrAxis[FTZ].axisStatus == S_Aixs_Open)
				{
					pStage->stageStatus = S_Stage_ALL_Open;
				}
				if(pStage->stageCMD == C_Stage_Traj_Expo)
				{
					pStage->stageStatus = S_Stage_Traj_Expo;
				}
				if(pStage->stageCMD == C_Stage_P2P_Traj)
				{
					pStage->stageStatus = S_Stage_P2P_Traj;
				}
			break;
			case S_Stage_Traj_Expo:
				pStage->Traj();
				if(pStage->arrAxis[FDX].axisStatus == S_Aixs_Open && pStage->arrAxis[FDY].axisStatus == S_Aixs_Open && pStage->arrAxis[FTZ].axisStatus == S_Aixs_Open)
				{
					pStage->stageStatus = S_Stage_ALL_Open;
					Fine_Count = 0;
					pStage->stageCMD = C_All_Nothing;
					DXPosLast_Fine = FirstIn;
					DXNegLast_Fine = FirstIn;
					DYPosLast_Fine = FirstIn;
					DYNegLast_Fine = FirstIn;
					logMsg("The Traj is stop!!\n",0,0,0,0,0,0);
				}
			break;
			case S_Stage_P2P_Traj:

			break;
			case S_Stage_Load_Pos:	
			break;	
		}
	}	
}

void SwitchStateMachine(Axis* pAxis,int iAxisNum)
{
	switch(pAxis->closeStatus)
	{
		case S_Counter_Close:
			CounterClose(pAxis,iAxisNum);
			if(pAxis->closeCMD == C_Laser_Close)
			{
				pAxis->closeStatus = S_Laser_Close;
			}
		break;
		case S_Laser_Close:
			LaserSwitch(pAxis,iAxisNum);
			if(pAxis->closeCMD == C_Counter_Close)
			{
				pAxis->closeStatus = S_Counter_Close;
			}
		break;
	}
}

