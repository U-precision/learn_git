#include "FileReadWrite.h"
#include "stdio.h"
#include "StateMachine.h"
#include "TrajHandler.h"
#include "GetData.h"

double DirData[12]={0};

double P2PData[12]={0};
double ExpoData[24]={0};
double dIdentBuff[BUFLENGTH]={0.0};
double ParamData[20]={0};
double PIDParam[20]={0};





int XTrajBuff[BUFLENGTH]={0};
int YTrajBuff[BUFLENGTH]={0};
int TzTrajBuff[BUFLENGTH]={0};

void Test(MACHINE *pMachine)
{

	static int s_iRandCnt=0;
	static int s_iCnt[3]={0,0,0};

	pMachine->fineStage.arrAxis[0].posInfor.dActPos=pMachine->fineStage.arrAxis[0].dActPos;
	pMachine->fineStage.arrAxis[1].posInfor.dActPos=pMachine->fineStage.arrAxis[1].dActPos;
	pMachine->fineStage.arrAxis[2].posInfor.dActPos=pMachine->fineStage.arrAxis[2].dActPos;
	
	pMachine->fineStage.arrAxis[0].posInfor.dSetPoint=pMachine->fineStage.arrAxis[0].dSetPoint;
	pMachine->fineStage.arrAxis[1].posInfor.dSetPoint=pMachine->fineStage.arrAxis[1].dSetPoint;
	pMachine->fineStage.arrAxis[2].posInfor.dSetPoint=pMachine->fineStage.arrAxis[2].dSetPoint;
	
	pMachine->fineStage.arrAxis[0].posInfor.dError=pMachine->fineStage.arrAxis[0].dError;
	pMachine->fineStage.arrAxis[1].posInfor.dError=pMachine->fineStage.arrAxis[1].dError;
	pMachine->fineStage.arrAxis[2].posInfor.dError=pMachine->fineStage.arrAxis[2].dError;


	if(s_iRandCnt==0)
	{
		srand((unsigned)time(NULL));
		s_iRandCnt=1;
	}
	

	if(Machine.fineStage.arrAxis[0].closeStatus==S_Laser_Close)
	{
		if(Machine.fineStage.iStageErrCheck==0)
		{

				if(s_iCnt[0]<100000)
				{
					Machine.fineStage.arrAxis[0].dError=XTrajBuff[s_iCnt[0]]+rand()%4-2+Machine.fineStage.arrAxis[0].dAffData*1e-4;	
				
					Machine.fineStage.arrAxis[0].dActPos=Machine.fineStage.arrAxis[0].dSetPoint-Machine.fineStage.arrAxis[0].dError;	
					
					s_iCnt[0]++;

					if(s_iCnt[0]==100000)
					{
						s_iCnt[0]=0;
					}
				}
				else
				{		
					s_iCnt[0]=0;			
				}
		}



		
	}


	if(Machine.fineStage.arrAxis[1].closeStatus==S_Laser_Close)
	{
		if(Machine.fineStage.iStageErrCheck==0)
		{

			if(s_iCnt[1]<100000)
			{
				Machine.fineStage.arrAxis[1].dError=YTrajBuff[s_iCnt[1]]+rand()%4-2+Machine.fineStage.arrAxis[1].dAffData*1e-4;
				Machine.fineStage.arrAxis[1].dActPos=
					Machine.fineStage.arrAxis[1].dSetPoint-
					Machine.fineStage.arrAxis[1].dError;
				
				s_iCnt[1]++;


				if(s_iCnt[1]==100000)
				{
					s_iCnt[1]=0;
				
				}

				
			}
			else
			{
				s_iCnt[1]=0;
			
			}
		}
	}

	if(Machine.fineStage.arrAxis[2].closeStatus==S_Laser_Close)
	{

		if(Machine.fineStage.iStageErrCheck==0)
		{

			if(s_iCnt[2]<100000)
			{
				Machine.fineStage.arrAxis[2].dError=TzTrajBuff[s_iCnt[2]]+rand()%6-3+Machine.fineStage.arrAxis[2].dAffData*1e-4;
				Machine.fineStage.arrAxis[2].dActPos=
					Machine.fineStage.arrAxis[2].dSetPoint-
					Machine.fineStage.arrAxis[2].dError;
				
				s_iCnt[2]++;

				if(s_iCnt[2]==100000)
				{
					s_iCnt[2]=0;
				
				}


				
			}
			else
			{
				s_iCnt[2]=0;

			}
		}
	}


	



}




void ReadFile()
{
	
	FILE * pFile;
	int iLineLen[10]={0};


	pFile=fopen("XTrajData.txt","r");
	while(!feof(pFile))
	{
		fscanf(pFile,"%d\n",(XTrajBuff+iLineLen[0]));
		iLineLen[0]++;	
	}
	fclose(pFile);	


	pFile=fopen("YTrajData.txt","r");
	while(!feof(pFile))
	{

		fscanf(pFile,"%d\n",(YTrajBuff+iLineLen[1]));
		iLineLen[1]++;

	}
	fclose(pFile);
       
    pFile=fopen("TzTrajData.txt","r");
	while(!feof(pFile))
	{
		fscanf(pFile,"%d\n",(TzTrajBuff+iLineLen[2]));
		iLineLen[2]++;

	}
	fclose(pFile);  
	g_iDataLen[0]=iLineLen[0];
	g_iDataLen[1]=iLineLen[1];
	g_iDataLen[2]=iLineLen[2];


}

int ReadExpoTrajFile()
{

	FILE * pFile;
	int iLineLen[2]={0};

	pFile=fopen("./P2PTraj/ExpoTrajType.txt","r");
	if (pFile == NULL)
	{
		logMsg("Open ExpoTrajType file failed \n",0,0,0,0,0,0);
		return 1;

	}
	while(!feof(pFile))
	{
		fscanf(pFile,"%lf\n",&(ExpoData[iLineLen[1]]));
		iLineLen[1]++; 
	}
	fclose(pFile);



	pFile=fopen("./P2PTraj/ExpoField.txt","r");
	if (pFile == NULL)
	{
		logMsg("Open ExpoField file failed \n",0,0,0,0,0,0);
		return 1;

	}
	fscanf(pFile,"%lf\n",&(ExpoData[0]));

	
	logMsg("Expo data %lf	%lf	%lf\n",ExpoData[0],ExpoData[1],ExpoData[2],0,0,0);
	fclose(pFile);

	return 2;


}




int ReadP2PTrajFile()
{
	FILE * pFile;
	int iLineLen[2]={0};


	pFile=fopen("./P2PTraj/P2PTrajType.txt","r");
	if (pFile == NULL)
	{
		logMsg("Open P2PTrajType file failed \n",0,0,0,0,0,0);
		return 1;

	}
	while(!feof(pFile))
	{
		fscanf(pFile,"%lf\n",&(P2PData[iLineLen[1]]));
		iLineLen[1]++; 
	}
	fclose(pFile);

	
	return 2;

		
}


int ReadDir()
{
	FILE * pFile;
	int iLineLen[2]={0};

	pFile=fopen("./P2PTraj/Dir.txt","r");
	if (pFile == NULL)
	{
		logMsg("Open P2PTrajType file failed \n",0,0,0,0,0,0);
		return 1;
	}
	while(!feof(pFile))
	{
		fscanf(pFile,"%lf\n",&(DirData[iLineLen[1]]));
		iLineLen[1]++; 
	}
	fclose(pFile);
	return 2;
}

void ReadIdentFile()
{
	
	FILE * pFile;
	int iLineLen[10]={0};
	pFile=fopen("IdentU_Dx5.txt","r");
	while(!feof(pFile))
	{
		fscanf(pFile,"%lf\n",(dIdentBuff+iLineLen[0]));		
		iLineLen[0]++;	
	}
	fclose(pFile);	
	logMsg("value1=%x,value2=%x\n",13,9,0,0,0,0);
}
/*
void ReadParam(char axis,StageType stage,double *Data)
{
	FILE* CoarseParamFile,FineParamFile;
	int i=0;
	switch(stage)
	{
		case CoarseStage:
			CoarseParamFile = fopen("ParamData.txt","r");
			if(CoarseParamFile==NULL)
			{
				printf("ParamFile open error\n");
				return;
			}
			switch(axis)
			{
				case 0:
					while(!feof(ParamFile))
					{
						fscanf(ParamFile,"%lf\n",(ParamData+i));
						i++;
					}
					fclose(ParamFile);
	//				printf("value=%lf\n,value1=%lf\n,value2=%lf\n,value3=%lf\n,value4=%lf\n,value5=%lf\n,value6=%lf\n,value7=%lf\n",

		//			ParamData[0],ParamData[1],ParamData[2],ParamData[3],ParamData[4],ParamData[5],ParamData[6],ParamData[7]);
				case 1:
					break;
				default :
					break;
			}
		case FineStage:
			FineParamFile = fopen("ParamData.txt","r");
			if(FineParamFile==NULL)
			{
				printf("FineParamFile open error\n");
				return;
			}
			switch(axis)
			{
				case 0:
					break;
				default:
					break;
			}
		default :
			break;
	}
}
*/
void ReadParam()
{
	FILE* CoarseParamFile,FineParamFile;
	int i=0;
	CoarseParamFile = fopen("CoarseParamData.txt","r");
	if(CoarseParamFile==NULL)
	{
		printf("ParamFile open error\n");
		return;
	}
	while(!feof(CoarseParamFile))
	{
		fscanf(CoarseParamFile,"%lf\n",(ParamData+i));
		i++;
	}
	fclose(CoarseParamFile);
//	printf("value=%lf\n,value1=%lf\n,value2=%lf\n,value3=%lf\n,value4=%lf\n,value5=%lf\n,value6=%lf\n,value7=%lf\n,
//		value8=%lf\n,value9=%lf\n,value10=%lf\n,value11=%lf\n,value12=%lf\n,value13=%lf\n,value14=%lf\n,value15=%lf\n",
//		ParamData[0],ParamData[1],ParamData[2],ParamData[3],ParamData[4],ParamData[5],ParamData[6],ParamData[7],
//		ParamData[8],ParamData[9],ParamData[10],ParamData[11],ParamData[12],ParamData[13],ParamData[14],ParamData[15]);
			
}

void ReadPID()
{
	FILE* PIDFile;
	int i=0;
	PIDFile = fopen("PIDParam.txt","r");
	if(PIDFile==NULL)
	{
		printf("PIDFile open error\n");
		return;
	}
	while(!feof(PIDFile))
	{
		fscanf(PIDFile,"%lf\n",(PIDParam+i));
		i++;
	}
	fclose(PIDFile);
//	printf("value=%lf\nvalue1=%lf\nvalue2=%lf\nvalue3=%lf\nvalue4=%lf\nvalue5=%lf\nvalue6=%lf\nvalue7=%lf\n\
//		value8=%lf\nvalue9=%lf\nvalue10=%lf\nvalue11=%lf\nvalue12=%lf\nvalue13=%lf\nvalue14=%lf\nvalue15=%lf\n",
//		PIDParam[0],PIDParam[1],PIDParam[2],PIDParam[3],PIDParam[4],PIDParam[5],PIDParam[6],PIDParam[7],
//		PIDParam[8],PIDParam[9],PIDParam[10],PIDParam[11],PIDParam[12],PIDParam[13],PIDParam[14],PIDParam[15]);
			
}

void ReadDAOffset()
{
	FILE* DAOffsetFile;
	int i=0;
	DAOffsetFile = fopen("DAOffset.txt","r");
	if(DAOffsetFile==NULL)
	{
		printf("DAOffsetFile open error\n");
		return;
	}
	while(!feof(DAOffsetFile))
	{
		if(i < 4)
		{
			fscanf(DAOffsetFile,"%d\n",(Machine.coarseStage.DAOffset+i));
		}
		else
		{
			fscanf(DAOffsetFile,"%d\n",(Machine.fineStage.DAOffset+i-4));
		}
		
		i++;
	}
	fclose(DAOffsetFile);
//	printf("value=%lf\n,value1=%lf\n,value2=%lf\n,value3=%lf\n,value4=%lf\n,value5=%lf\n,value6=%lf\n,value7=%lf\n,
//		value8=%lf\n,value9=%lf\n,value10=%lf\n,value11=%lf\n,value12=%lf\n,value13=%lf\n,value14=%lf\n,value15=%lf\n",
//		ParamData[0],ParamData[1],ParamData[2],ParamData[3],ParamData[4],ParamData[5],ParamData[6],ParamData[7],
//		ParamData[8],ParamData[9],ParamData[10],ParamData[11],ParamData[12],ParamData[13],ParamData[14],ParamData[15]);
			
}
//extern CIRCLE circleParam;
void ReadCircleParam()
{
	FILE* CircleParamFile;
	int i=0;
	CircleParamFile = fopen("CircleParamData.txt","r");
	if(CircleParamFile==NULL)
	{
		printf("ParamFile open error\n");
		return;
	}
	while(!feof(CircleParamFile))
	{
		fscanf(CircleParamFile,"%d\n",(&circleParam.R + i));
		i++;
	}
	fclose(CircleParamFile);
//	printf("value=%lf\n,value1=%lf\n,value2=%lf\n,value3=%lf\n,value4=%lf\n,value5=%lf\n,value6=%lf\n,value7=%lf\n,
//		value8=%lf\n,value9=%lf\n,value10=%lf\n,value11=%lf\n,value12=%lf\n,value13=%lf\n,value14=%lf\n,value15=%lf\n",
//		ParamData[0],ParamData[1],ParamData[2],ParamData[3],ParamData[4],ParamData[5],ParamData[6],ParamData[7],
//		ParamData[8],ParamData[9],ParamData[10],ParamData[11],ParamData[12],ParamData[13],ParamData[14],ParamData[15]);
//		printf("%d\n%d\n",circleParam.R,circleParam.cirNum);	
}

void ReadDirParam()
{
	FILE * DirFile;
	int i=0;
	DirFile=fopen("Dir.txt","r");
	if (DirFile == NULL)
	{
		logMsg("Open Dir file failed \n",0,0,0,0,0,0);
		return ;
	}
	while(!feof(DirFile))
	{
		if(i < 6)
		{
			fscanf(DirFile,"%d\n",&(Machine.fineStage.arrAxis[i].iDir));
	//		printf("%d\n",Machine.fineStage.arrAxis[i].Dir);
		}
		else
		{
			fscanf(DirFile,"%d\n",&(Machine.fineStage.arrAxis[i].iLaserDir));
		}
		i++; 
	}
	fclose(DirFile);
}

void ReadRecordIndex()
{
	FILE * RecordIndexFile;
	int i=0;
	double value;
	RecordIndexFile=fopen("RecordIndex.txt","r");
	if (RecordIndexFile == NULL)
	{
		logMsg("Open RecordIndex file failed \n",0,0,0,0,0,0);
		return ;
	}
	while(!feof(RecordIndexFile))
	{
		if(i < 3)
		{
			fscanf(RecordIndexFile,"%d\n",&(recvData.iReserved[i]));
			printf("%d\n",recvData.iReserved[i]);
		}
		i++; 
	}
	fclose(RecordIndexFile);
	if(recvData.iReserved[1] == 50)
	{
		memoryStype=0;
		recordMap[recvData.iReserved[1]].arg2 = recvData.iReserved[2];
	//	value = &Machine.coarseStage.arrAxis[0].dActPos;
	//	printf("%d\n",&Machine.coarseStage.arrAxis[0].dActPos);
	//	printf("%d\n",recordMap[recvData.iReserved[1]].arg1);
		recordMap[recvData.iReserved[1]].func(recordMap[recvData.iReserved[1]].arg1,recordMap[recvData.iReserved[1]].arg2);
	}
	else
	{
		memoryStype=1;
	}
}


