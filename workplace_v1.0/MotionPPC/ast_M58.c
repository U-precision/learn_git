/*********************  ast_M58_Init  *********************************
 * initialization function to determinate M58 and get local VME address 
 * 
 * input :
 * 		boardNO:	M72 number	0..3
 * 					one of M-Module Carrier Board slots
 *		address:	M-Module Carrier Board address
 *		slot   :	M-Module Carrier Board slot	
 * Output:
 * 		return  : 	ERROR    --  init error
 * 					OK       --  success
 **********************************************************************/  

#include "StateMachine.h"
#include "math.h"
#include "ast_M58.h"

#include "ast_Header.h"

void ast_M58_Init()
{
	 //M72_Channel_INTStatusLow = 0x001F;
	 //M72_Channel_INTStatusHigh= 0x001F;
	M58_Channel_ConReg0 = 0x0007;
	M58_Channel_ConReg2 = 0x0000;
	//Machine.stepHome =  0;
	 Machine.iTimeHome  =  0;
	 Machine.stepFlagup = 0;		//
	 Machine.stepFlagdown = 0;
	 //Machine.HomeFlagCorse = 0;

	Machine.stepFlagdownP = 0;		//��������λ
	Machine.stepFlagupP = 0;		//��������λ
	Machine.iTimeHomeUpP =0;
	Machine.iTimeHomeDownP = 0;
	Machine.iTimeHomeDownQ = 0;

	 Machine.coarseStage.arrAxis[2].stepHome =  0.0;
	 Machine.coarseStage.arrAxis[0].stepHome =  0.0;
	 Machine.coarseStage.arrAxis[2].HomeFlagCorse =  0.0;
	 Machine.coarseStage.arrAxis[0].HomeFlagCorse =  0.0;
	 Machine.coarseStage.arrAxis[2].bHomeDone=0.0;
	 Machine.coarseStage.arrAxis[0].bHomeDone=0.0;
	 Machine.fineStopFlag = 0;
	 Machine.coarseStopFlag = 0;
}


void ast_M58_DIO()
{
	//���Ḻ��λ
	if((Port_B_A&0x02) == 0x02)
	{
		Machine.iTimeHomeDownQ++;
	}	  
	else
	{
		Machine.iTimeHomeDownQ = 0;
		Machine.stepFlagdown = 0;
	}
	if( Machine.iTimeHomeDownQ == 1000)
	{
//		comData.iFineData[14] =1;
		Machine.stepFlagdown = 1;
	}
	if((Port_B_A&0x02) == 0x00)
	{
//		comData.iFineData[14] =3;
		Machine.stepFlagdown = 0;
				  
	}
	//����1 ����λ����
	if((Port_B_A&0x08) == 0x08)
	{
		Machine.iTimeHomeDownP++;
	}
	else
	{
		Machine.iTimeHomeDownP = 0;
		Machine.stepFlagdownP = 0;
	}
	if( Machine.iTimeHomeDownP == 1000)
	{
	//	comData.iFineData[17] =1;
		Machine.stepFlagdownP = 1;
	}
	if((Port_B_A&0x08) == 0x00)
	{
//		comData.iFineData[17] =3;
		Machine.stepFlagdownP = 0;
	}

	
	//���� ����λ����
	if((Port_B_A&0x01) == 0x00)
	{
//		comData.iFineData[11] =2;
		Machine.stepFlagup = 0;	
	}
	if((Port_B_A&0x01) == 0x01)		//�Ѿ���ȡ����λ��
	{
		Machine.iTimeHome++;		//�Ѿ���ȡ����λ��
	}
	else
	{
         Machine.iTimeHome=0;
		 Machine.stepFlagup = 0;
	}
    if( Machine.iTimeHome == 1000)
    {
//         comData.iFineData[11] =1;
		 Machine.stepFlagup = 1;
	}

	 //��������λ
	if((Port_B_A&0x04) == 0x04)
	{
		Machine.iTimeHomeUpP++;
	}
	else
	{
		Machine.iTimeHomeUpP=0;
		Machine.stepFlagupP = 0;
	}
	if( Machine.iTimeHomeUpP == 1000)
	 {
//		comData.iFineData[18] =1;
		Machine.stepFlagupP = 1;
	 }
	if((Port_B_A&0x04) == 0x00)
	{
//		comData.iFineData[18] =2;
	}
	
}



void ast_M58_Protect()
{

  if(( Machine.stepFlagdown == 1 )&&(Machine.coarseStage.arrAxis[0].axisStatus != S_Aixs_Home))//���Ḻ��λ
  	{
	  if(Machine.coarseStage.arrAxis[0].dSetPoint  <    Machine.coarseStage.arrAxis[0].dActPos)
	  	{

		Machine.coarseStage.arrAxis[0].dSetPoint  =    Machine.coarseStage.arrAxis[0].dActPos;

		Machine.coarseStage.arrAxis[0].axisStatus = S_Aixs_Close;

	    }


	   if(Machine.coarseStage.arrAxis[1].dSetPoint  <	  Machine.coarseStage.arrAxis[1].dActPos)
		{
			Machine.coarseStage.arrAxis[1].dSetPoint  =    Machine.coarseStage.arrAxis[1].dActPos;
	
			Machine.coarseStage.arrAxis[1].axisStatus = S_Aixs_Close;
	   	}
    }


   if( (Machine.stepFlagdownP == 1 )&&(Machine.coarseStage.arrAxis[0].axisStatus != S_Aixs_Home))//��������λ
	   {
		 if(Machine.coarseStage.arrAxis[0].dSetPoint  >    Machine.coarseStage.arrAxis[0].dActPos)
		   {
   
		   Machine.coarseStage.arrAxis[0].dSetPoint  =	  Machine.coarseStage.arrAxis[0].dActPos;
   
		   Machine.coarseStage.arrAxis[0].axisStatus = S_Aixs_Close;
   
		   }
   
   
		  if(Machine.coarseStage.arrAxis[1].dSetPoint  >	 Machine.coarseStage.arrAxis[1].dActPos)
		   {
	   
			   Machine.coarseStage.arrAxis[1].dSetPoint  =	  Machine.coarseStage.arrAxis[1].dActPos;
	   
			   Machine.coarseStage.arrAxis[1].axisStatus = S_Aixs_Close;
		   }
	   }




  
  

   if( (Machine.stepFlagup == 1 ) && (Machine.coarseStage.arrAxis[2].axisStatus != S_Aixs_Home)   )//���Ḻ��
  	{
	  if(Machine.coarseStage.arrAxis[2].dSetPoint  <    Machine.coarseStage.arrAxis[2].dActPos)
	  	{

		Machine.coarseStage.arrAxis[2].dSetPoint  =    Machine.coarseStage.arrAxis[2].dActPos;

		Machine.coarseStage.arrAxis[2].axisStatus = S_Aixs_Close;

	    }

    }



   if((Machine.stepFlagupP == 1)&& (Machine.coarseStage.arrAxis[2].axisStatus != S_Aixs_Home))//��������
  	{
	  if(Machine.coarseStage.arrAxis[2].dSetPoint  >    Machine.coarseStage.arrAxis[2].dActPos)
	  	{

		Machine.coarseStage.arrAxis[2].dSetPoint  =    Machine.coarseStage.arrAxis[2].dActPos;

		Machine.coarseStage.arrAxis[2].axisStatus = S_Aixs_Close;

	    }

    }

}



void ast_M58_SoftProtect()
{
  //��������λ

   if( Machine.coarseStage.arrAxis[2].bHomeDone==2)

   {
    
	if(( Machine.coarseStage.arrAxis[2].dActPos > 101000000)&& (Machine.coarseStage.arrAxis[2].axisStatus != S_Aixs_Home))//��������
		{
		  if(Machine.coarseStage.arrAxis[2].dSetPoint  >	Machine.coarseStage.arrAxis[2].dActPos)
			{
	
			Machine.coarseStage.arrAxis[2].dSetPoint  =    Machine.coarseStage.arrAxis[2].dActPos;
	
			Machine.coarseStage.arrAxis[2].axisStatus = S_Aixs_Close;
	
			}
	
		}

    if(( Machine.coarseStage.arrAxis[2].dActPos < -38000000)&& (Machine.coarseStage.arrAxis[2].axisStatus != S_Aixs_Home))//��������
		{
		  if(Machine.coarseStage.arrAxis[2].dSetPoint  <	Machine.coarseStage.arrAxis[2].dActPos)
			{
	
			Machine.coarseStage.arrAxis[2].dSetPoint  =    Machine.coarseStage.arrAxis[2].dActPos;
	
			Machine.coarseStage.arrAxis[2].axisStatus = S_Aixs_Close;
	
			}
	
		}
     

   }


//��������λ


   if( Machine.coarseStage.arrAxis[0].bHomeDone==2)

   {
    
	if(( Machine.coarseStage.arrAxis[0].dActPos > 73000000)&& (Machine.coarseStage.arrAxis[0].axisStatus != S_Aixs_Home))//��������
		{
		  if(Machine.coarseStage.arrAxis[0].dSetPoint  >	Machine.coarseStage.arrAxis[0].dActPos)
			{
	
			Machine.coarseStage.arrAxis[0].dSetPoint  =    Machine.coarseStage.arrAxis[0].dActPos;
	
			Machine.coarseStage.arrAxis[0].axisStatus = S_Aixs_Close;
	
			}
	
		}

    if(( Machine.coarseStage.arrAxis[0].dActPos < -63000000)&& (Machine.coarseStage.arrAxis[0].axisStatus != S_Aixs_Home))//��������
		{
		  if(Machine.coarseStage.arrAxis[0].dSetPoint  <	Machine.coarseStage.arrAxis[0].dActPos)
			{
	
			Machine.coarseStage.arrAxis[0].dSetPoint  =    Machine.coarseStage.arrAxis[0].dActPos;
	
			Machine.coarseStage.arrAxis[0].axisStatus = S_Aixs_Close;
	
			}
	
		}
     

   }



   

}





void HomeAll()

{

   if(( Machine.coarseStage.arrAxis[2].bHomeDone==2.0)&&(Machine.coarseStage.arrAxis[0].HomeFlagCorse ==  1.0 ) )
   	{
    Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_Home;

   }

   if( Machine.coarseStage.arrAxis[0].bHomeDone==2.0)
   	{
	   
	  // Machine.coarseStage.arrAxis[1].axisStatus = S_Aixs_Close;

   }


}



void AxisStop(Axis* pAxis,int iAxisNum)
{

      pAxis->dSetPoint  =    pAxis->dActPos;
	
	  pAxis->axisStatus = S_Aixs_Close;


}


void P2PTrajParamGen(Axis* pAxis,int iAxisNum)
{
	  double Freq = 5000.0;//����Ƶ��
		double Amax = 0.1 * 9.8;//�����ٶ�����ֵ
		double Vmax = 0.1;//����ٶ�����ֵ
		double Smax = 0.1;//�г�
		double TotalTime = 0.5;//�켣��ʱ��Լ��
		double WaitTime = 0.0;//�켣��ɺ�ȴ�ʱ��Լ��
		const char* FileName = "TrajParam.txt";
	/**********�������л�ȡ�켣����**********/
	/*	int ch;
		opterr = 0;
		while((ch = getopt(argc, argv, "F:A:V:S:T:W:O:")) != -1)
		{
			switch(ch)
			{
				case 'F':
					Freq = atof(optarg);
					break;
				case 'A':
					Amax = 9.8 * atof(optarg);
					break;
				case 'V':
					Vmax = atof(optarg);
					break;
				case 'S':
					Smax = atof(optarg);
					break;
				case 'T':
					TotalTime = atof(optarg);
					break;
				case 'W':
					WaitTime = atof(optarg);
					break;
				case 'O':
					FileName = optarg;
					break;
			}
		}*/
    
    double ta;//���ٶ�ʱ��
    double tv;//���ٶ�ʱ��
    double t1, t2, t3, t4;//�ֱ�Ϊ���ٶν�ֹʱ�䣬���ٶν�ֹʱ�䣬���ٶν�ֹʱ��͹켣��ʱ��

    ta = 2.0 * Vmax / Amax;
    tv = Smax / Vmax - ta;
   /* if((ta < 0) || (tv < 0))
    {
        cout << "Trajectory does NOT exist!" << endl;
        return -1;
    }*/
    ta = ceil(ta * Freq) / Freq;
    tv = ceil(tv * Freq) / Freq;
    Vmax = Smax / (ta + tv);
    Amax = 2.0 * Vmax / ta;
    t1 = ta;
    t2 = ta + tv;
    t3 = 2.0 * ta + tv;
    t4 = 2.0 * ta + tv + WaitTime;

 //�켣��������

 comData.dCoarseData[38] = 1.0/ Freq;//����1 Q��λ����home
 comData.dCoarseData[39] = Amax;//����1 P��λ����
 
 comData.dCoarseData[40] =	Vmax ;//����1 Q��λ����home
 comData.dCoarseData[41] = Smax;//����1 P��λ����
 
 comData.dCoarseData[42] = ta;//����1 Q��λ����home
 comData.dCoarseData[43] = tv;//����1 P��λ����
 
 comData.dCoarseData[44] =	t1 ;//����1 Q��λ����home
 comData.dCoarseData[45] = t2;

 comData.dCoarseData[46] =	 t3 ;//����1 Q��λ����home
 comData.dCoarseData[47] = t4;

 /*TrajParam << setiosflags(ios::fixed) << setprecision(16) << Freq << endl;
 TrajParam << setiosflags(ios::fixed) << setprecision(16) << Amax << endl;
 TrajParam << setiosflags(ios::fixed) << setprecision(16) << Vmax << endl;
 TrajParam << setiosflags(ios::fixed) << setprecision(16) << Smax << endl;
 TrajParam << setiosflags(ios::fixed) << setprecision(16) << ta << endl;
 TrajParam << setiosflags(ios::fixed) << setprecision(16) << tv << endl;
 TrajParam << setiosflags(ios::fixed) << setprecision(16) << t1 << endl;
 TrajParam << setiosflags(ios::fixed) << setprecision(16) << t2 << endl;
 TrajParam << setiosflags(ios::fixed) << setprecision(16) << t3 << endl;
 TrajParam << setiosflags(ios::fixed) << setprecision(16) << t4 << endl;*/


}




