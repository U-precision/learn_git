#include <vxworks.h>
#include <vme.h>
#include <taskLib.h>
#include <sockLib.h>
#include <ioLib.h>
#include <inetLib.h>
#include <logLib.h>
#include <string.h>
#include <fioLib.h>
#include <stdio.h>
#include <memLib.h>
#include <stdLib.h>
#include <semLib.h>
#include <netinet\\tcp.h>
//#include <N1225A.h>


#include "posixtimer.h"
#include "ConParamInit.h"
#include "rtcontrol.h"


#include "ComWithWin.h"
#include "ZMI4104.h"
#include "ast_M58.h"
#include "N1225A.h"

//任务变量

SEM_ID	DataCollect_SemId;	//收集数据信号量
SEM_ID  TransDataToWin_SemId;//上传参数信号量



int ttimer;
int	tDataCollect;
int tServoCycle;
int tRecvFromWin;
int tSendToWin;

#define STACK_SIZE 	12000//8000  //The size of task stack






void InitCounter()
{
	ast_M72_Init(0,0x1C0000,2);	
	ast_M72_Counter_Start(0,0,4);
	ast_M72_Counter_Start(0,1,4);
	ast_M72_Counter_Start(0,2,4);
	ast_M72_Counter_Start(0,3,4);

}
void InitAIO()
{
	setupaio();	
	ast_M58_Init();
	/* Machine.stepHome =  0;
	 Machine.iTimeHome  =  0;
	 Machine.stepFlagup = 0;
	 Machine.stepFlagdown = 0;
	 Machine.HomeFlagCorse = 0;*/
}
 
void comtest()
{
	long long *mac;
	unsigned int value = 0xff,value_3 = 0xff,value_2 = 0xff;
	unsigned int comparatoraddr = 0,comparatoraddr_1 = 0,comparatoraddr_2 = 0;
	readMAC(1,mac);
	comparatoraddr = CalBoardAxis_Addr(1,1)+ StandardSamplingPosReg;
	comparatoraddr_1 = CalBoardAxis_Addr(1,2)+ StandardSamplingPosReg;
	comparatoraddr_2 = CalBoardAxis_Addr(2,1)+ AutoSamplingPosReg;
	value = ReadReg(comparatoraddr);
	value_2 = ReadReg(comparatoraddr_1);
	value_3 = ReadReg(comparatoraddr_2);
	printf("value = %x,value_2 = %x,value_3 = %x\r\n",value,value_2,value_3);
}


void N1225ATest()
{
	static int *Data;
	static char number = 0;
	unsigned char i = 0;unsigned int j=0;
	FILE *File;
//	Data = (unsigned int *)malloc(Axis_SUM * sizeof(int));
	File = fopen("C:\\DataFile.txt", "w+");
	if (File == NULL)
	{
		printf("创建文件失败\r\n");
		return;
	}
	fprintf(File, "Test rate[Hz]:%d\r\n", 5000);
	N1225A_Init();
	while (1)
	{
		ReadAllPositions(Data, 2);
		fprintf(File, "time[ms]:%.1f\t", (0.2*(number+1)));
		for (i = 0; i < Axis_SUM; i++)
		{
			fprintf(File, "Pos[%d]:%x\t\t", i, *(Data + i));
		}
		fprintf(File, "\r\n");
		number++;
		if (number == 20000)
		{
			fclose(File);
//			free(Data);
			return;
		}
		for(j=0;j<200000;j++);
	}

}
void TaskInit(void)
{
//	printf("Net init error%d   %d\n",sizeof(long),sizeof(long long));

	InitMachine();
	
	DataCollect_SemId = semBCreate(SEM_Q_FIFO,SEM_EMPTY);
	TransDataToWin_SemId = semBCreate(SEM_Q_FIFO,SEM_EMPTY);
	
	ttimer = taskSpawn("TimerInit",80,0,STACK_SIZE,
	(FUNCPTR) TimerInit,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

	tDataCollect = taskSpawn("DataCollect",191,0,STACK_SIZE,
	(FUNCPTR) DataCollect,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	
	tServoCycle = taskSpawn("ServoCycle",90,0,STACK_SIZE*2,
		(FUNCPTR) ServoCycle,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	
	tRecvFromWin = taskSpawn("RecvFromWin",95,0,STACK_SIZE,
	(FUNCPTR) RecvFromWin,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

	tSendToWin = taskSpawn("SendToWin",100,0,STACK_SIZE,
	(FUNCPTR) SendToWin,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	
}


void TaskEnd(void)
{

	ExitNet();

	TimerDelete();
	taskDelete(ttimer);
	taskDelete(tDataCollect);
	taskDelete(tServoCycle);
	taskDelete(tRecvFromWin);
	taskDelete(tSendToWin);

	semDelete(DataCollect_SemId);
	semDelete(TransDataToWin_SemId);
	semDelete(servotimeSemId);	

	logMsg("Task Stop\n",0,0,0,0,0,0);
}


