#include "ConCalc.h"
#include "StateMachine.h"

/*ȫ�ֺ�������*/

/*
**�������ƣ�Concalc_Coarse_DX
**�����������
**����ֵ��  ��
**�����������ֶ�̨DX������������ǰ�������������
*/
double Concalc_Coarse_DX()
{
	Stage* pStage = &Machine.coarseStage;
	double dConCalc = 0.0;
	
	pStage->arrAxis[CDX].dError = pStage->arrAxis[CDX].dSetPoint - pStage->arrAxis[CDX].dActPos;

	if(pStage->arrAxis[CDX].axisStatus != S_Aixs_Open)
	{
		if(pStage->arrAxis[CDX].closeStatus == S_Laser_Close)
		{
			if(pStage->arrAxis[CDX].LaserControllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[CDX].sPID_Switch,pStage->arrAxis[CDX].dError);
			}
			else if(pStage->arrAxis[CDX].LaserControllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[CDX].sLLC_Switch,pStage->arrAxis[CDX].dError);
			}

			//ǰ��
			if(pStage->arrAxis[CDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDX].dFeedForwardCoef*pStage->arrAxis[CDX].dAffData;
			}

			/*if(pStage->arrAxis[CDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDX].SingleFeedForward_x_Switch*pStage->arrAxis[CDX].dAffData;
			}
			else if(pStage->arrAxis[CDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDX].SingleFeedForward_y_Switch*pStage->arrAxis[CDY].dAffData;
			}*/
		}
		else if(pStage->arrAxis[CDX].closeStatus == S_Counter_Close)
		{
			if(pStage->arrAxis[CDX].controllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[CDX].sPID,pStage->arrAxis[CDX].dError);
			}
			else if(pStage->arrAxis[CDX].controllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[CDX].sLLC,pStage->arrAxis[CDX].dError);
			}

			//ǰ��
			if(pStage->arrAxis[CDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDX].dFeedForwardCoef*pStage->arrAxis[CDX].dAffData;
			}
		
			/*if(pStage->arrAxis[CDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDX].SingleFeedForward_x*pStage->arrAxis[CDX].dAffData;
			}
			else if(pStage->arrAxis[CDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDX].SingleFeedForward_y*pStage->arrAxis[CDY].dAffData;
			}*/
		}
	}
	else
	{
		dConCalc = 0.0;
		PIDClear(&pStage->arrAxis[CDX].sPID);
		LLCClear(&pStage->arrAxis[CDX].sLLC);
		PIDClear(&pStage->arrAxis[CDX].sPID_Switch);
		LLCClear(&pStage->arrAxis[CDX].sLLC_Switch);
		LLCClear(&pStage->arrAxis[CDX].sLLC_Switch);
		ClearLPF(&pStage->arrAxis[CDX]);
		//logMsg("Controler Coarse DX is Open!!\n",0,0,0,0,0,0);
	}
	return(dConCalc);
}
/*
**�������ƣ�Concalc_Coarse_DY
**�����������
**����ֵ��  ��
**�����������ֶ�̨DX������������ǰ�������������
*/
double Concalc_Coarse_DY()
{
	Stage* pStage = &Machine.coarseStage;
	double dConCalc = 0.0;
	
	pStage->arrAxis[CDY].dError = pStage->arrAxis[CDY].dSetPoint - pStage->arrAxis[CDY].dActPos;

	if(pStage->arrAxis[CDY].axisStatus != S_Aixs_Open)
	{
		if(pStage->arrAxis[CDY].closeStatus == S_Laser_Close)
		{
			if(pStage->arrAxis[CDY].LaserControllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[CDY].sPID_Switch,pStage->arrAxis[CDY].dError);
			}
			else if(pStage->arrAxis[CDY].LaserControllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[CDY].sLLC_Switch,pStage->arrAxis[CDY].dError);
			}

			//ǰ��
			if(pStage->arrAxis[CDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDY].dFeedForwardCoef*pStage->arrAxis[CDY].dAffData;
			}
			
			/*if(pStage->arrAxis[CDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDY].SingleFeedForward_x_Switch*pStage->arrAxis[CDX].dAffData;
			}
			else if(pStage->arrAxis[CDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDY].SingleFeedForward_y_Switch*pStage->arrAxis[CDY].dAffData;
			}*/
		}
		else if(pStage->arrAxis[CDY].closeStatus == S_Counter_Close)
		{
			if(pStage->arrAxis[CDY].controllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[CDY].sPID,pStage->arrAxis[CDY].dError);
			}
			else if(pStage->arrAxis[CDY].controllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[CDY].sLLC,pStage->arrAxis[CDY].dError);
			}
			//ǰ��
			if(pStage->arrAxis[CDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDY].dFeedForwardCoef*pStage->arrAxis[CDY].dAffData;
			}
			/*if(pStage->arrAxis[CDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDY].SingleFeedForward_x*pStage->arrAxis[CDX].dAffData;
			}
			else if(pStage->arrAxis[CDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDY].SingleFeedForward_y*pStage->arrAxis[CDY].dAffData;
			}*/
		}
	}
	else
	{	
		dConCalc = 0.0;
		PIDClear(&pStage->arrAxis[CDY].sPID);
		LLCClear(&pStage->arrAxis[CDY].sLLC);
		PIDClear(&pStage->arrAxis[CDY].sPID_Switch);
		LLCClear(&pStage->arrAxis[CDY].sLLC_Switch);
		LLCClear(&pStage->arrAxis[CDY].sLLC_Switch);
		ClearLPF(&pStage->arrAxis[CDY]);
		//logMsg("Controler Coarse DY is Open!!\n",0,0,0,0,0,0);
	}
	return(dConCalc);
}
/*
**�������ƣ�Concalc_Coarse_XTZ
**�����������
**����ֵ��  ��
**�����������ֶ�̨XTZ������������ǰ�������������
*/
double Concalc_Coarse_XTZ()
{
	Stage* pStage = &Machine.coarseStage;
	double dConCalc = 0.0;
	
	pStage->arrAxis[XTZ].dError = pStage->arrAxis[XTZ].dSetPoint - pStage->arrAxis[XTZ].dActPos;

	if(pStage->arrAxis[XTZ].axisStatus != S_Aixs_Open)
	{
		if(pStage->arrAxis[XTZ].closeStatus == S_Laser_Close)
		{
			if(pStage->arrAxis[XTZ].LaserControllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[XTZ].sPID_Switch,pStage->arrAxis[XTZ].dError);
			}
			else if(pStage->arrAxis[XTZ].LaserControllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[XTZ].sLLC_Switch,pStage->arrAxis[XTZ].dError);
			}
			//ǰ��
			if(pStage->arrAxis[XTZ].axisStatus == S_Aixs_RunFor || pStage->arrAxis[XTZ].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[XTZ].dFeedForwardCoef*pStage->arrAxis[XTZ].dAffData;
			}
			/*if(pStage->arrAxis[CDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[XTZ].SingleFeedForward_x_Switch*pStage->arrAxis[CDX].dAffData;
			}
			else if(pStage->arrAxis[CDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[XTZ].SingleFeedForward_y_Switch*pStage->arrAxis[CDY].dAffData;
			}*/
		}
		else if(pStage->arrAxis[XTZ].closeStatus == S_Counter_Close)
		{
			if(pStage->arrAxis[XTZ].controllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[XTZ].sPID,pStage->arrAxis[XTZ].dError);
			}
			else if(pStage->arrAxis[XTZ].controllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[XTZ].sLLC,pStage->arrAxis[XTZ].dError);
			}
			//ǰ��
			if(pStage->arrAxis[XTZ].axisStatus == S_Aixs_RunFor || pStage->arrAxis[XTZ].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[XTZ].dFeedForwardCoef*pStage->arrAxis[XTZ].dAffData;
			}
			/*if(pStage->arrAxis[CDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[XTZ].SingleFeedForward_x*pStage->arrAxis[CDX].dAffData;
			}
			else if(pStage->arrAxis[CDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[XTZ].SingleFeedForward_y*pStage->arrAxis[CDY].dAffData;
			}*/
		}
	}
	else
	{
		dConCalc = 0.0;
		PIDClear(&pStage->arrAxis[XTZ].sPID);
		LLCClear(&pStage->arrAxis[XTZ].sLLC);
		PIDClear(&pStage->arrAxis[XTZ].sPID_Switch);
		LLCClear(&pStage->arrAxis[XTZ].sLLC_Switch);
		LLCClear(&pStage->arrAxis[XTZ].sFilter);
		ClearLPF(&pStage->arrAxis[XTZ]);
	}
	return(dConCalc);
}
/*
**�������ƣ�Concalc_Coarse_YTZ
**�����������
**����ֵ��  ��
**�����������ֶ�̨XTZ������������ǰ�������������
*/
double Concalc_Coarse_YTZ()
{
	Stage * pStage = &Machine.coarseStage;
	double dConCalc = 0.0;
	
	pStage->arrAxis[YTZ].dError = pStage->arrAxis[YTZ].dSetPoint - pStage->arrAxis[YTZ].dActPos;

	if(pStage->arrAxis[YTZ].axisStatus != S_Aixs_Open)
	{
		if(pStage->arrAxis[YTZ].closeStatus == S_Laser_Close)
		{
			if(pStage->arrAxis[YTZ].LaserControllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[YTZ].sPID_Switch,pStage->arrAxis[YTZ].dError);
			}
			else if(pStage->arrAxis[YTZ].LaserControllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[YTZ].sLLC_Switch,pStage->arrAxis[YTZ].dError);
			}
			//ǰ��
			if(pStage->arrAxis[YTZ].axisStatus == S_Aixs_RunFor || pStage->arrAxis[YTZ].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[YTZ].dFeedForwardCoef*pStage->arrAxis[YTZ].dAffData;
			}

			/*if(pStage->arrAxis[CDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[YTZ].SingleFeedForward_x_Switch*pStage->arrAxis[CDX].dAffData;
			}
			else if(pStage->arrAxis[CDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[YTZ].SingleFeedForward_y_Switch*pStage->arrAxis[CDY].dAffData;
			}*/
		}
		else if(pStage->arrAxis[YTZ].closeStatus == S_Counter_Close)
		{
			if(pStage->arrAxis[YTZ].controllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[YTZ].sPID,pStage->arrAxis[YTZ].dError);
			}
			else if(pStage->arrAxis[YTZ].controllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[YTZ].sLLC,pStage->arrAxis[YTZ].dError);
			}
			if(pStage->arrAxis[YTZ].axisStatus == S_Aixs_RunFor || pStage->arrAxis[YTZ].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[YTZ].dFeedForwardCoef*pStage->arrAxis[YTZ].dAffData;
			}
			/*if(pStage->arrAxis[CDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[YTZ].SingleFeedForward_x*pStage->arrAxis[CDX].dAffData;
			}
			else if(pStage->arrAxis[CDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[YTZ].SingleFeedForward_y*pStage->arrAxis[CDY].dAffData;
			}*/
		}
	}
	else
	{
		dConCalc = 0.0;
		PIDClear(&pStage->arrAxis[YTZ].sPID);
		LLCClear(&pStage->arrAxis[YTZ].sLLC);
		PIDClear(&pStage->arrAxis[YTZ].sPID_Switch);
		LLCClear(&pStage->arrAxis[YTZ].sLLC_Switch);
		LLCClear(&pStage->arrAxis[YTZ].sFilter);
		ClearLPF(&pStage->arrAxis[YTZ]);
	}
	return(dConCalc);
}
/*
**�������ƣ�Concalc_Fine_DX
**�����������
**����ֵ��  ��
**����������΢��̨DX������������ǰ�������������
*/
double Concalc_Fine_DX()
{
	Stage* pStage = &Machine.fineStage;
	double dConCalc = 0.0;
	
	pStage->arrAxis[FDX].dError = pStage->arrAxis[FDX].dSetPoint - pStage->arrAxis[FDX].dActPos;

	if(pStage->arrAxis[FDX].axisStatus != S_Aixs_Open)
	{
		if(pStage->arrAxis[FDX].closeStatus == S_Laser_Close)
		{
			if(pStage->arrAxis[FDX].LaserControllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FDX].sPID_Switch,pStage->arrAxis[FDX].dError);
			}
			else if(pStage->arrAxis[FDX].LaserControllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FDX].sLLC_Switch,pStage->arrAxis[FDX].dError);
			}

			//ǰ��
			if(pStage->arrAxis[FDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDX].dFeedForwardCoef*pStage->arrAxis[FDX].dAffData;
			}
			/*if(pStage->arrAxis[FDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDX].SingleFeedForward_x_Switch*pStage->arrAxis[FDX].dAffData;
			}
			else if(pStage->arrAxis[FDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDX].SingleFeedForward_y_Switch*pStage->arrAxis[FDY].dAffData;
			}*/
		}
		else if(pStage->arrAxis[FDX].closeStatus == S_Counter_Close)
		{
			if(pStage->arrAxis[FDX].controllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FDX].sPID,pStage->arrAxis[FDX].dError);
			}
			else if(pStage->arrAxis[FDX].controllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FDX].sLLC,pStage->arrAxis[FDX].dError);
			}
				//dConCalc = LLCFilter(&pStage->arrAxis[FDX].sFilter,dConCalc);
				dConCalc = LLCCalculate(&pStage->arrAxis[FDX].sFilter,dConCalc);
			//ǰ��
			if(pStage->arrAxis[FDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDX].dFeedForwardCoef*pStage->arrAxis[FDX].dAffData;
			}
			/*if(pStage->arrAxis[FDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDX].SingleFeedForward_x*pStage->arrAxis[FDX].dAffData;
			}
			else if(pStage->arrAxis[FDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDX].SingleFeedForward_y*pStage->arrAxis[FDY].dAffData;
			}*/
		}
	}
	else
	{
		dConCalc = 0.0;
		PIDClear(&pStage->arrAxis[FDX].sPID);
		LLCClear(&pStage->arrAxis[FDX].sLLC);
		PIDClear(&pStage->arrAxis[FDX].sPID_Switch);
		LLCClear(&pStage->arrAxis[FDX].sLLC_Switch);
		LLCClear(&pStage->arrAxis[FDX].sFilter);
		ClearLPF(&pStage->arrAxis[FDX]);
	}
	return(dConCalc);
}
/*
**�������ƣ�Concalc_Fine_DY
**�����������
**����ֵ��  ��
**����������΢��̨DY������������ǰ�������������
*/
double Concalc_Fine_DY()
{
	Stage * pStage = &Machine.fineStage;
	double dConCalc = 0.0;
	
	pStage->arrAxis[FDY].dError = pStage->arrAxis[FDY].dSetPoint - pStage->arrAxis[FDY].dActPos;

	if(pStage->arrAxis[FDY].axisStatus != S_Aixs_Open)
	{
		if(pStage->arrAxis[FDY].closeStatus == S_Laser_Close)
		{
			if(pStage->arrAxis[FDY].LaserControllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FDY].sPID_Switch,pStage->arrAxis[FDY].dError);
			}
			else if(pStage->arrAxis[FDY].LaserControllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FDY].sLLC_Switch,pStage->arrAxis[FDY].dError);
			}

			//ǰ��
			if(pStage->arrAxis[FDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDY].dFeedForwardCoef*pStage->arrAxis[FDY].dAffData;
			}
			/*if(pStage->arrAxis[FDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDY].SingleFeedForward_x_Switch*pStage->arrAxis[FDX].dAffData;
			}
			else if(pStage->arrAxis[FDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDY].SingleFeedForward_y_Switch*pStage->arrAxis[FDY].dAffData;
			}*/
		}
		else if(pStage->arrAxis[FDY].closeStatus == S_Counter_Close)
		{
			if(pStage->arrAxis[FDY].controllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FDY].sPID,pStage->arrAxis[FDY].dError);
			}
			else if(pStage->arrAxis[FDY].controllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FDY].sLLC,pStage->arrAxis[FDY].dError);
			}
//			dConCalc = LLCFilter(&pStage->arrAxis[FDY].sFilter,dConCalc);
			//ǰ��
			dConCalc = LLCCalculate(&pStage->arrAxis[FDY].sFilter,dConCalc);
			if(pStage->arrAxis[FDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDY].dFeedForwardCoef*pStage->arrAxis[FDY].dAffData;
			}
		}
	}
	else
	{
		dConCalc = 0.0;
		PIDClear(&pStage->arrAxis[FDY].sPID);
		LLCClear(&pStage->arrAxis[FDY].sLLC);
		PIDClear(&pStage->arrAxis[FDY].sPID_Switch);
		LLCClear(&pStage->arrAxis[FDY].sLLC_Switch);
		LLCClear(&pStage->arrAxis[FDY].sFilter);
		ClearLPF(&pStage->arrAxis[FDY]);
	}
	return(dConCalc);
}


/*YHT���*/
/*
**�������ƣ�Concalc_Fine_DZ
**�����������
**����ֵ��  ��
**����������΢��̨DZ������������ǰ�������������
*/
double Concalc_Fine_DZ()
{
	Stage * pStage = &Machine.fineStage;
	double dConCalc = 0.0;
	
	pStage->arrAxis[FDZ].dError = pStage->arrAxis[FDZ].dSetPoint - pStage->arrAxis[FDZ].dActPos;

	if(pStage->arrAxis[FDZ].axisStatus != S_Aixs_Open)
	{
		if(pStage->arrAxis[FDZ].closeStatus == S_Laser_Close)
		{
			if(pStage->arrAxis[FDZ].LaserControllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FDZ].sPID_Switch,pStage->arrAxis[FDZ].dError);
			}
			else if(pStage->arrAxis[FDZ].LaserControllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FDZ].sLLC_Switch,pStage->arrAxis[FDZ].dError);
			}

			//ǰ��
			if(pStage->arrAxis[FDZ].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDZ].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDZ].dFeedForwardCoef*pStage->arrAxis[FDZ].dAffData;
			}
		}
		else if(pStage->arrAxis[FDZ].closeStatus == S_Counter_Close)
		{
			if(pStage->arrAxis[FDZ].controllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FDZ].sPID,pStage->arrAxis[FDZ].dError);
			}
			else if(pStage->arrAxis[FDZ].controllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FDZ].sLLC,pStage->arrAxis[FDZ].dError);
			}

//			dConCalc = LLCFilter(&pStage->arrAxis[FDZ].sFilter,dConCalc);
			dConCalc = LLCCalculate(&pStage->arrAxis[FDZ].sFilter,dConCalc);
			//ǰ��
			if(pStage->arrAxis[FDZ].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDZ].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDZ].dFeedForwardCoef*pStage->arrAxis[FDZ].dAffData;
			}
		}
	}
	else
	{
		dConCalc = 0.0;
		PIDClear(&pStage->arrAxis[FDZ].sPID);
		LLCClear(&pStage->arrAxis[FDZ].sLLC);
		PIDClear(&pStage->arrAxis[FDZ].sPID_Switch);
		LLCClear(&pStage->arrAxis[FDZ].sLLC_Switch);
		LLCClear(&pStage->arrAxis[FDZ].sFilter);
		ClearLPF(&pStage->arrAxis[FDZ]);
	}
	return(dConCalc);
}

/*
**�������ƣ�Concalc_Fine_TX
**�����������
**����ֵ��  ��
**����������΢��̨TX������������ǰ�������������
*/
double Concalc_Fine_TX()
{
	Stage * pStage = &Machine.fineStage;
	double dConCalc = 0.0;
	
	pStage->arrAxis[FTX].dError = pStage->arrAxis[FTX].dSetPoint - pStage->arrAxis[FTX].dActPos;

	if(pStage->arrAxis[FTX].axisStatus != S_Aixs_Open)
	{
		if(pStage->arrAxis[FTX].closeStatus == S_Laser_Close)
		{
			if(pStage->arrAxis[FTX].LaserControllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FTX].sPID_Switch,pStage->arrAxis[FTX].dError);
			}
			else if(pStage->arrAxis[FTX].LaserControllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FTX].sLLC_Switch,pStage->arrAxis[FTX].dError);
			}

			//ǰ��
			if(pStage->arrAxis[FTX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FTX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FTX].dFeedForwardCoef*pStage->arrAxis[FTX].dAffData;
			}
		}
		else if(pStage->arrAxis[FTX].closeStatus == S_Counter_Close)
		{
			if(pStage->arrAxis[FTX].controllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FTX].sPID,pStage->arrAxis[FTX].dError);
			}
			else if(pStage->arrAxis[FTX].controllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FTX].sLLC,pStage->arrAxis[FTX].dError);
			}

//			dConCalc = LLCFilter(&pStage->arrAxis[FTX].sFilter,dConCalc);
			dConCalc = LLCCalculate(&pStage->arrAxis[FTX].sFilter,dConCalc);
			//ǰ��
			if(pStage->arrAxis[FTX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FTX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FTX].dFeedForwardCoef*pStage->arrAxis[FTX].dAffData;
			}

		}
	}
	else
	{
		dConCalc = 0.0;
		PIDClear(&pStage->arrAxis[FTX].sPID);
		LLCClear(&pStage->arrAxis[FTX].sLLC);
		PIDClear(&pStage->arrAxis[FTX].sPID_Switch);
		LLCClear(&pStage->arrAxis[FTX].sLLC_Switch);
		LLCClear(&pStage->arrAxis[FTX].sFilter);
		ClearLPF(&pStage->arrAxis[FTX]);
	}
	return(dConCalc);
}

/*
**�������ƣ�Concalc_Fine_TY
**�����������
**����ֵ��  ��
**����������΢��̨TY������������ǰ�������������
*/
double Concalc_Fine_TY()
{
	Stage * pStage = &Machine.fineStage;
	double dConCalc = 0.0;
	
	pStage->arrAxis[FTY].dError = pStage->arrAxis[FTY].dSetPoint - pStage->arrAxis[FTY].dActPos;

	if(pStage->arrAxis[FTY].axisStatus != S_Aixs_Open)
	{
		if(pStage->arrAxis[FTY].closeStatus == S_Laser_Close)
		{
			if(pStage->arrAxis[FTY].LaserControllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FTY].sPID_Switch,pStage->arrAxis[FTY].dError);
			}
			else if(pStage->arrAxis[FTY].LaserControllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FTY].sLLC_Switch,pStage->arrAxis[FTY].dError);
			}

			//ǰ��
			if(pStage->arrAxis[FTY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FTY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FTY].dFeedForwardCoef*pStage->arrAxis[FTY].dAffData;
			}


			
			/*if(pStage->arrAxis[FDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDY].SingleFeedForward_x_Switch*pStage->arrAxis[FDX].dAffData;
			}
			else if(pStage->arrAxis[FDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDY].SingleFeedForward_y_Switch*pStage->arrAxis[FDY].dAffData;
			}*/
		}
		else if(pStage->arrAxis[FTY].closeStatus == S_Counter_Close)
		{
			if(pStage->arrAxis[FTY].controllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FTY].sPID,pStage->arrAxis[FTY].dError);
			}
			else if(pStage->arrAxis[FTY].controllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FTY].sLLC,pStage->arrAxis[FTY].dError);
			}

//			dConCalc = LLCFilter(&pStage->arrAxis[FTY].sFilter,dConCalc);
			dConCalc = LLCCalculate(&pStage->arrAxis[FTY].sFilter,dConCalc);
			//ǰ��
			if(pStage->arrAxis[FTY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FTY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FTY].dFeedForwardCoef*pStage->arrAxis[FTY].dAffData;
			}


			/*if(pStage->arrAxis[FDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDY].SingleFeedForward_x*pStage->arrAxis[FDX].dAffData;
			}
			else if(pStage->arrAxis[FDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDY].SingleFeedForward_y*pStage->arrAxis[FDY].dAffData;
			}*/
		}
	}
	else
	{
		dConCalc = 0.0;
		PIDClear(&pStage->arrAxis[FTY].sPID);
		LLCClear(&pStage->arrAxis[FTY].sLLC);
		PIDClear(&pStage->arrAxis[FTY].sPID_Switch);
		LLCClear(&pStage->arrAxis[FTY].sLLC_Switch);
		LLCClear(&(pStage->arrAxis[FTY].sFilter));
		ClearLPF(&pStage->arrAxis[FTY]);
	}
	return(dConCalc);
}


/*
**�������ƣ�Concalc_Fine_TZ
**�����������
**����ֵ��  ��
**����������΢��̨DY������������ǰ�������������
*/
double Concalc_Fine_TZ()
{
	Stage * pStage = &Machine.fineStage;
	double dConCalc = 0.0;
	
	pStage->arrAxis[FTZ].dError = pStage->arrAxis[FTZ].dSetPoint - pStage->arrAxis[FTZ].dActPos;

	if(pStage->arrAxis[FTZ].axisStatus != S_Aixs_Open)
	{
		if(pStage->arrAxis[FTZ].closeStatus == S_Laser_Close)
		{
			if(pStage->arrAxis[FTZ].LaserControllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FTZ].sPID_Switch,pStage->arrAxis[FTZ].dError);
			}
			else if(pStage->arrAxis[FTZ].LaserControllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FTZ].sLLC_Switch,pStage->arrAxis[FTZ].dError);
			}


			//ǰ��
			if(pStage->arrAxis[FTZ].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FTZ].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FTZ].dFeedForwardCoef*pStage->arrAxis[FTZ].dAffData;
			}

			
			/*if(pStage->arrAxis[FDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FTZ].SingleFeedForward_x_Switch*pStage->arrAxis[FDX].dAffData;
			}
			else if(pStage->arrAxis[FDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FTZ].SingleFeedForward_y_Switch*pStage->arrAxis[FDY].dAffData;
			}*/
		}
		else if(pStage->arrAxis[FTZ].closeStatus == S_Counter_Close)
		{
			if(pStage->arrAxis[FTZ].controllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FTZ].sPID,pStage->arrAxis[FTZ].dError);
			}
			else if(pStage->arrAxis[FTZ].controllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FTZ].sLLC,pStage->arrAxis[FTZ].dError);
			}

//			dConCalc = LLCFilter(&pStage->arrAxis[FTZ].sFilter,dConCalc);
			dConCalc = LLCCalculate(&pStage->arrAxis[FTZ].sFilter,dConCalc);
			//ǰ��
			if(pStage->arrAxis[FTZ].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FTZ].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FTZ].dFeedForwardCoef*pStage->arrAxis[FTZ].dAffData;
			}

			
			/*if(pStage->arrAxis[FDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FTZ].SingleFeedForward_x*pStage->arrAxis[FDX].dAffData;
			}
			else if(pStage->arrAxis[FDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FTZ].SingleFeedForward_y*pStage->arrAxis[FDY].dAffData;
			}*/
		}
	}
	else
	{
		dConCalc = 0.0;
		PIDClear(&pStage->arrAxis[FTZ].sPID);
		LLCClear(&pStage->arrAxis[FTZ].sLLC);
		PIDClear(&pStage->arrAxis[FTZ].sPID_Switch);
		LLCClear(&pStage->arrAxis[FTZ].sLLC_Switch);
		LLCClear(&(pStage->arrAxis[FTZ].sFilter));
		ClearLPF(&pStage->arrAxis[FTZ]);
	}
	return(dConCalc);
}

