#include "controller.h"
#include "StateMachine.h"

void PIDInit (PID *pp)
{
	memset (pp,0,sizeof(PID));
}

/*double PID_Calc_Balance(PID *pp,long NextPoint)
{
  long dError,error;
  double vOut;
    
  error = pp->SetPoint-NextPoint;                   //PError
  pp->sumError = pp->sumError + error;
  pp->dError = error - pp->error;
  pp->error = error;  
  vOut =	pp->Proportion * pp->error + pp->Integral * pp->sumError*0.0002 + pp->Derivative * pp->dError;
  return vOut;
}
*/
/*ȫ�ֺ�������*/
double PIDCalculate(PID* pp, double error)
{
  double vOut;    
  pp->sumError = pp->sumError + error;
  pp->dError = error - pp->error;
  pp->error = error;  
  vOut =	pp->Proportion * pp->error + pp->Integral * pp->sumError*0.0002 + pp->Derivative * pp->dError;
  return vOut;	
}

void PIDClear(PID* pp)
{
	pp->sumError = 0.0;
	pp->dError = 0.0;
	pp->error = 0.0;
}

double LLCCalculate(LLC* pp, double error)
{
	int i = 0;
	pp->pIn[0] = error;
	pp->pUout[0] = 0.0;
	//����������
	for(i=0;i< pp->order;i++)
	{
		pp->pUout[0] += pp->pB[i]*pp->pIn[i]; 
	}		  
	for(i=1;i<pp->order;i++)
	{
		pp->pUout[0] -= pp->pA[i]*pp->pUout[i]; 
	}
	//��������
	for(i=pp->order-1; i>0; i--)
	{
		pp->pUout[i] = pp->pUout[i-1];
		pp->pIn[i] = pp->pIn[i-1];
	}
	return pp->pUout[0];
}


double LLCFilter(LLC* pp, double pIn)
{
	int i = 0;
	pp->pIn[0] = pIn;
	pp->pUout[0] = 0.0;
	//����������
	for(i=0;i< pp->order;i++)
	{
		pp->pUout[0] += pp->pB[i]*pp->pIn[i]; 
	}
	for(i=1;i<pp->order;i++)
	{
		pp->pUout[0] -= pp->pA[i]*pp->pUout[i]; 
	}
	//��������
	for(i=pp->order-1; i>0; i--)
	{
		pp->pUout[i] = pp->pUout[i-1];
		pp->pIn[i] = pp->pIn[i-1];
	}
	return pp->pUout[0];
}

void LLCClear(LLC* pp)
{
	int i = 0;
	for(i=0; i<pp->order; i++)
	{
		pp->pIn[i] = 0.0;
		pp->pUout[i] = 0.0;
	}
}
/******************************************
��������Filter2order
����ֵ��double	�����˲���������DA
������double *Input	�����ɶȵ�����
		double flp_Laser	�˲�Ƶ�ʣ�HZ��
		double clp_Laser	����
		double* blp		�˲������ӵĲ���
		double* alp		�˲�����ĸ�Ĳ���
		double* Output	�˲������
		int* Flag		�����˲��������ı�־λ
����˵�����˲��������������˲����Ĳ��������
******************************************/
double Filter2order(double *Input,double flp_Laser,double clp_Laser,double* blp,double* alp , double* Output,int* Flag)
{
  double temp1=0.0;
  double temp2=0.0;
  int i=0;
  double wlp_Laser=0.0;
  //��ɢ��
  wlp_Laser=2.0*3.1415926 * flp_Laser;
  if (*Flag==1)
  {
	  temp1=4.0+4.0*clp_Laser*wlp_Laser*0.0002+wlp_Laser*wlp_Laser*0.0002*0.0002;
	  temp2=wlp_Laser*wlp_Laser*0.0002*0.0002;
	  *blp = temp2/temp1;
	  *(blp+1) = 2.0*temp2/temp1;
	  *(blp+2) = temp2/temp1;

	  *alp = 1.0;
	  *(alp+1) = (2.0*temp2-8.0)/temp1;
	  *(alp+2) = (temp1-8.0*clp_Laser*wlp_Laser*0.0002)/temp1;
  
  	  *Flag = 0;
  }
  //�˲�
  Output[0]=0.0;
  for(i=0;i<3;i++)
  {
	  Output[0] += -alp[i]*Output[i] + blp[i]*Input[i];
  }
  for(i=2;i>0;i--)
  {
	  Input[i]=Input[i-1];
	  Output[i]=Output[i-1];
  }
  return Output[0];
}

/******************************************
��������Notch_Laser
����ֵ��double	���ؿ�����������DA
������double *Input	�����ɶȵ�����
		double fnp_Laser	�ݲ�Ƶ�ʣ�HZ��
		double cnp_Laser	�ݲ�Ƶ�ʣ�HZ��
		double fnz_Laser	���
		double cnz_Laser	����
		double* blp		�ݲ����������Ӳ���
		double* alp		�ݲ���������ĸ����
		double* Output	�ݲ����������
		int* Flag		�����ݲ��������Ĳ���
����˵�����ݲ������������������ݲ��������Ĳ��������
******************************************/
double Notch_Laser(double *Input,double fnp_Laser,double cnp_Laser,double fnz_Laser,double cnz_Laser,double* blp,double* alp , double* Output,int* Flag)
{
  double temp1=0.0;
  double temp2=0.0;
  double temp3=0.0;
  int i=0;
  double wnp_Laser=0.0;
  double wnz_Laser=0.0;
  
  wnp_Laser=2.0*3.1415926*fnp_Laser;
  wnz_Laser=2.0*3.1415926*fnz_Laser;
  //��ɢ��
  if (*Flag==1)
  {
	  temp1=4.0+4.0*cnp_Laser*wnp_Laser*0.0002+wnp_Laser*wnp_Laser*0.0002*0.0002;
	  temp2=4.0+4.0*cnz_Laser*wnz_Laser*0.0002+wnz_Laser*wnz_Laser*0.0002*0.0002;
	  temp3=wnp_Laser*wnp_Laser/wnz_Laser/wnz_Laser;
	  	
	  *blp = temp2*temp3/temp1;
	  *(blp+1) = (2.0*wnz_Laser*wnz_Laser*0.0002*0.0002-8.0)*temp3/temp1;
	  *(blp+2) = (temp2-8.0*cnz_Laser*wnz_Laser*0.0002)*temp3/temp1;

	  *alp = 1.0;
	  *(alp+1) = (2.0*wnp_Laser*wnp_Laser*0.0002*0.0002-8.0)/temp1;
	  *(alp+2) = (temp1-8.0*cnp_Laser*wnp_Laser*0.0002)/temp1;
  
  	  *Flag = 0;
  }
  //�������
  Output[0]=0.0;  
  for(i=0;i<3;i++)
  {
	  Output[0] += -alp[i]*Output[i] + blp[i]*Input[i];
	  
  }
  //����
  for(i=2;i>0;i--)
  {
	  Input[i]=Input[i-1];
	  Output[i]=Output[i-1];
  }

  return Output[0];
}
/*****************************************
��������FIR_Controller
����ֵ��double	���ؿ�����������DA
������FIR *pp	�洢FIRǰ���������Ĳ���
		int lCurAcc	���������루���ٶȣ�
����˵����FIRǰ������������������FIRǰ����
******************************************/
double FIR_Controller(FIR *pp,double lCurAcc)
{
	int i=0;
	pp->FIR_In[0] = lCurAcc;
	pp->FIR_Out =0.0;
	for(i=0;i<pp->FIR_iOrder;i++)
	{
		  pp->FIR_Out +=  pp->FIR_Coeff[i]*pp->FIR_In[i];
	}
	for(i=pp->FIR_iOrder-1;i>0;i--)
	{
		pp->FIR_In[i]=pp->FIR_In[i-1];
	}

	return pp->FIR_Out;

}
/*****************************************
��������AFCController
����ֵ��double	���ؿ�����������DA
������AFC *PP		AFC�Ľṹ��
			 double error	�������
		int lCurAcc	���������루���ٶȣ�
����˵����FIRǰ������������������FIRǰ����
******************************************/
double AFCController(AFC *PP,double error)
{
	PP->pin[0] = error;
	PP->pout[0] = PP->pB[0] * PP->pin[0] + PP->pB[1]*PP->pin[1] + PP->pB[2]*PP->pin[2] - PP->pA[1]*PP->pout[1] - PP->pA[2] * PP->pout[2];
	PP->pin[2] = PP->pin[1];
	PP->pin[1] = PP->pin[0];
	PP->pout[2] = PP->pout[1];
	PP->pout[1] = PP->pout[0];
	return PP->pout[0];
}

void ClearLPF(Axis *F_iAxis)
{
	memset(F_iAxis->LaserFilterInput,0,3*sizeof(double));
	memset(F_iAxis->LaserFilterOutput,0,3*sizeof(double));

	memset(F_iAxis->LaserNotchInput,0,3*sizeof(double));
	memset(F_iAxis->LaserNotchOutput,0,3*sizeof(double));
}

