#ifndef CMD_H
#define CMD_H

#define	TRUE		1
#define FALSE		0

#define SET_PARAMETER				5000
#define GET_PARAMETER				5001
#define STAGE_EXPO					5002

#define STAGE_P2P_TRAJ				5003
#define JOG_SPEED					5004
#define ABSOLUTE_MOVE				5007
#define ABSOLUTE_XMOVE				5008
#define ABSOLUTE_YMOVE				5009

#define EXPOSet                     29
#define STOPCOLLECT					30
#define P2PSet                     30029

#define STAGE_LOAD_POS				99990
#define Coarse_Both_MOVE			99992
#define ALL_CLOSE            		99994 

#define READ_FILE					26
#define RECORD_DATA                 27
#define SAVE_FILE					28
#define READ_EXPO_FILE				29

#define RESET_LASER					32

#define RECORD_SELECT				50

/////////////  Stage Control Command  /////////////////////////////////////
#define ALL_OPEN                        10000  // �ֶ�̨΢��̨ȫ����

#define COARSE_ALL_OPEN             	20001  // �ֶ�̨ȫ����
#define COARSE_ALL_CLOSE            	20002  // �ֶ�̨ȫ�ջ�
#define COARSE_ALL_HOME					20008  // �ֶ�̨ȫ����


#define FINE_ALL_OPEN             		10001  // ΢��̨ȫ����
#define FINE_ALL_CLOSE            		10002  // ΢��̨ȫ�ջ�
#define FINE_ALL_HOME					10008  // ΢��̨ȫ����
#define FINE_LASER_SWITCH				10009  // ΢��̨ȫ�л�

// Coarse DX
#define COARSE_DX_OPEN				20101  // ��DX����
#define COARSE_DX_CLOSE				20102  // ��DX�ջ�
#define COARSE_DX_STEP_FORWARD		20103  // ��DXstep+
#define COARSE_DX_STEP_REVERSE      20104  // ��DXstep-		
#define COARSE_DX_JOG_FORWARD    	20105  // ��DXJog+
#define COARSE_DX_JOG_REVERSE    	20106  // ��DXJog-
#define COARSE_DX_JOG_STOP     		20107  // ��DXJog+ͣ
#define COARSE_DX_HOME				20108  // ��DX����
#define COARSE_DX_LASERSWITCH		20109  // �ֶ�̨DX���ڲ�
#define COARSE_DX_EDDYSWITCH		20110  // �ֶ�̨DX�лص�����
#define COARSE_DX_IDENTIFY		    20111  // ��DX��ʶ
#define COARSE_DX_RUN_FORWARD		20112	// ��DX�߹켣+
#define COARSE_DX_RUN_REVERSE		20113  // ��DX�߹켣-
// Coarse DY
#define COARSE_DY_OPEN				20201  // ��DY����
#define COARSE_DY_CLOSE				20202  // ��DY�ջ�
#define COARSE_DY_STEP_FORWARD		20203  // ��DYstep+
#define COARSE_DY_STEP_REVERSE      20204  // ��DYstep-		
#define COARSE_DY_JOG_FORWARD    	20205  // ��DYJog+
#define COARSE_DY_JOG_REVERSE    	20206  // ��DYJog-
#define COARSE_DY_JOG_STOP			20207  // ��DYJog+ͣ
#define COARSE_DY_HOME				20208  // ��DY����
#define COARSE_DY_LASERSWITCH		20209  // �ֶ�̨DY�м���
#define COARSE_DY_EDDYSWITCH		20210  // �ֶ�̨DY�лص�����
#define COARSE_DY_IDENTIFY		    20211  // ��DY��ʶ
#define COARSE_DY_RUN_FORWARD		20212	// ��DY�߹켣+
#define COARSE_DY_RUN_REVERSE		20213  // ��DY�߹켣-
// Coarse XTZ
#define COARSE_XTZ_OPEN				20301  // ��XTZ����
#define COARSE_XTZ_CLOSE			20302  // ��XTZ�ջ�
#define COARSE_XTZ_STEP_FORWARD		20303  // ��XTZstep+
#define COARSE_XTZ_STEP_REVERSE     20304  // ��XTZstep-		
#define COARSE_XTZ_JOG_FORWARD    	20305  // ��XTZJog+
#define COARSE_XTZ_JOG_REVERSE    	20306  // ��XTZJog-
#define COARSE_XTZ_JOG_STOP	     	20307  // ��XTZJog+ͣ
#define COARSE_XTZ_HOME				20308  // ��XTZ����
#define COARSE_XTZ_LASERSWITCH		20309  // �ֶ�̨XTZ�м���
#define COARSE_XTZ_EDDYSWITCH		20310  // �ֶ�̨XTZ�лص�����
#define COARSE_XTZ_IDENTIFY		    20311  // ��XTZ��ʶ
#define COARSE_XTZ_RUN_FORWARD		20312	// ��XTZ�߹켣+
#define COARSE_XTZ_RUN_REVERSE		20313  // ��XTZ�߹켣-
// Coarse YTZ
#define COARSE_YTZ_OPEN				20401  // ��YTZ����
#define COARSE_YTZ_CLOSE			20402  // ��YTZ�ջ�
#define COARSE_YTZ_STEP_FORWARD		20403  // ��YTZstep+
#define COARSE_YTZ_STEP_REVERSE     20404  // ��YTZstep-		
#define COARSE_YTZ_JOG_FORWARD    	20405  // ��YTZJog+
#define COARSE_YTZ_JOG_REVERSE    	20406  // ��YTZJog-
#define COARSE_YTZ_JOG_STOP     	20407  // ��YTZJog+ͣ
#define COARSE_YTZ_HOME				20408  // ��YTZ����
#define COARSE_YTZ_LASERSWITCH		20409  // �ֶ�̨YTZ�м���
#define COARSE_YTZ_EDDYSWITCH		20410  // �ֶ�̨YTZ�лص�����
#define COARSE_YTZ_IDENTIFY		    20411  // ��YTZ��ʶ
#define COARSE_YTZ_RUN_FORWARD		20412	// ��YTZ�߹켣+
#define COARSE_YTZ_RUN_REVERSE		20413  // ��YTZ�߹켣-
//
#define COARSE_DDX_OPEN				20501  // ��YTZ����
#define COARSE_DDX_CLOSE			20502  // ��YTZ�ջ�
#define COARSE_DDX_STEP_FORWARD		20503  // ��YTZstep+
#define COARSE_DDX_STEP_REVERSE     20504  // ��YTZstep-		
#define COARSE_DDX_JOG_FORWARD    	20505  // ��YTZJog+
#define COARSE_DDX_JOG_REVERSE    	20506  // ��YTZJog-
#define COARSE_DDX_JOG_STOP     	20507  // ��YTZJog+ͣ
#define COARSE_DDX_HOME				20508  // ��YTZ����
#define COARSE_DDX_LASERSWITCH		20509  // �ֶ�̨YTZ�м���
#define COARSE_DDX_EDDYSWITCH		20510  // �ֶ�̨YTZ�лص�����
#define COARSE_DDX_IDENTIFY		    20511  // ��YTZ��ʶ
#define COARSE_DDX_RUN_FORWARD		20512	// ��YTZ�߹켣+
#define COARSE_DDX_RUN_REVERSE		20513  // ��YTZ�߹켣-

#define COARSE_DDY_OPEN				20601  // ��YTZ����
#define COARSE_DDY_CLOSE			20602  // ��YTZ�ջ�
#define COARSE_DDY_STEP_FORWARD		20603  // ��YTZstep+
#define COARSE_DDY_STEP_REVERSE     20604  // ��YTZstep-		
#define COARSE_DDY_JOG_FORWARD    	20605  // ��YTZJog+
#define COARSE_DDY_JOG_REVERSE    	20606  // ��YTZJog-
#define COARSE_DDY_JOG_STOP     	20607  // ��YTZJog+ͣ
#define COARSE_DDY_HOME				20608  // ��YTZ����
#define COARSE_DDY_LASERSWITCH		20609  // �ֶ�̨YTZ�м���
#define COARSE_DDY_EDDYSWITCH		20610  // �ֶ�̨YTZ�лص�����
#define COARSE_DDY_IDENTIFY		    20611  // ��YTZ��ʶ
#define COARSE_DDY_RUN_FORWARD		20612	// ��YTZ�߹켣+
#define COARSE_DDY_RUN_REVERSE		20613  // ��YTZ�߹켣-

// Fine DX
#define FINE_DX_OPEN				10101  // ΢DX����
#define FINE_DX_CLOSE				10102  // ΢DX�ջ�
#define FINE_DX_STEP_FORWARD		10103  // ΢DXstep+
#define FINE_DX_STEP_REVERSE      	10104  // ΢DXstep-		
#define FINE_DX_JOG_FORWARD    		10105  // ΢DXJog+
#define FINE_DX_JOG_REVERSE    		10106  // ΢DXJog-
#define FINE_DX_JOG_STOP	     	10107  // ΢DXJog+ͣ
#define FINE_DX_HOME				10108  // ΢DX����
#define FINE_DX_LASERSWITCH			10109  // ΢��̨DX�м���
#define FINE_DX_EDDYSWITCH			10110  // ΢��̨DX�лص�����
#define FINE_DX_IDENTIFY		    10111  // ΢DX��ʶ
#define FINE_DX_RUN_FORWARD			10112	// ΢DX�߹켣+
#define FINE_DX_RUN_REVERSE			10113  // ΢DX�߹켣-
// Fine DY
#define FINE_DY_OPEN				10201  // ΢DY����
#define FINE_DY_CLOSE				10202  // ΢DY�ջ�
#define FINE_DY_STEP_FORWARD		10203  // ΢DYstep+
#define FINE_DY_STEP_REVERSE      	10204  // ΢DYstep-		
#define FINE_DY_JOG_FORWARD    		10205  // ΢DYJog+
#define FINE_DY_JOG_REVERSE    		10206  // ΢DYJog-
#define FINE_DY_JOG_STOP			10207  // ΢DYJog+ͣ
#define FINE_DY_HOME				10208  // ΢DY����
#define FINE_DY_LASERSWITCH			10209  // ΢��̨DY�м���
#define FINE_DY_EDDYSWITCH			10210  // ΢��̨DY�лص�����
#define FINE_DY_IDENTIFY		    10211  // ΢DY��ʶ
#define FINE_DY_RUN_FORWARD			10212	// ΢DY�߹켣+
#define FINE_DY_RUN_REVERSE			10213  // ΢DY�߹켣-

//Fine TZ
#define FINE_TZ_OPEN				10301  // ΢TZ����
#define FINE_TZ_CLOSE				10302  // ΢TZ�ջ�
#define FINE_TZ_STEP_FORWARD		10303  // ΢TZstep+
#define FINE_TZ_STEP_REVERSE      	10304  // ΢TZstep-		
#define FINE_TZ_JOG_FORWARD    		10305  // ΢TZJog+
#define FINE_TZ_JOG_REVERSE    		10306  // ΢TZJog-
#define FINE_TZ_JOG_STOP			10307  // ΢TZJog+ͣ
#define FINE_TZ_HOME				10308  // ΢TZ����
#define FINE_TZ_LASERSWITCH			10309  // ΢��̨TZ�м���
#define FINE_TZ_EDDYSWITCH			10310  // ΢��̨TZ�лص�����
#define FINE_TZ_IDENTIFY		    10311  // ΢TZ��ʶ
#define FINE_TZ_RUN_FORWARD			10312	// ΢TZ�߹켣+
#define FINE_TZ_RUN_REVERSE			10313  // ΢TZ�߹켣-

// BOTH DX
#define BOTH_DX_OPEN				10401  // ΢DX����
#define BOTH_DX_CLOSE				10402  // ΢DX�ջ�
#define BOTH_DX_STEP_FORWARD		10403  // ΢DXstep+
#define BOTH_DX_STEP_REVERSE      	10404  // ΢DXstep-		
#define BOTH_DX_JOG_FORWARD    		10405  // ΢DXJog+
#define BOTH_DX_JOG_REVERSE    		10406  // ΢DXJog-
#define BOTH_DX_JOG_STOP	     	10407  // ΢DXJog+ͣ
#define BOTH_DX_HOME				10408  // ΢DX����
#define BOTH_DX_LASERSWITCH			10409  // ΢��̨DX�м���
#define BOTH_DX_EDDYSWITCH			10410  // ΢��̨DX�лص�����
#define BOTH_DX_IDENTIFY		    10411  // ΢DX��ʶ
#define BOTH_DX_RUN_FORWARD			10412	// ΢DX�߹켣+
#define BOTH_DX_RUN_REVERSE			10413  // ΢DX�߹켣-
// BOTH DY
#define BOTH_DY_OPEN				10501  // ΢DY����
#define BOTH_DY_CLOSE				10502  // ΢DY�ջ�
#define BOTH_DY_STEP_FORWARD		10503  // ΢DYstep+
#define BOTH_DY_STEP_REVERSE      	10504  // ΢DYstep-		
#define BOTH_DY_JOG_FORWARD    		10505  // ΢DYJog+
#define BOTH_DY_JOG_REVERSE    		10506  // ΢DYJog-
#define BOTH_DY_JOG_STOP			10507  // ΢DYJog+ͣ
#define BOTH_DY_HOME				10508  // ΢DY����
#define BOTH_DY_LASERSWITCH			10509  // ΢��̨DY�м���
#define BOTH_DY_EDDYSWITCH			10510  // ΢��̨DY�лص�����
#define BOTH_DY_IDENTIFY		    10511  // ΢DY��ʶ
#define BOTH_DY_RUN_FORWARD			10512	// ΢DY�߹켣+
#define BOTH_DY_RUN_REVERSE			10513  // ΢DY�߹켣-

//BOTH TZ
#define BOTH_TZ_OPEN				10601  // ΢TZ����
#define BOTH_TZ_CLOSE				10602  // ΢TZ�ջ�
#define BOTH_TZ_STEP_FORWARD		10603  // ΢TZstep+
#define BOTH_TZ_STEP_REVERSE      	10604  // ΢TZstep-		
#define BOTH_TZ_JOG_FORWARD    		10605  // ΢TZJog+
#define BOTH_TZ_JOG_REVERSE    		10606  // ΢TZJog-
#define BOTH_TZ_JOG_STOP			10607  // ΢TZJog+ͣ
#define BOTH_TZ_HOME				10608  // ΢TZ����
#define BOTH_TZ_LASERSWITCH			10609  // ΢��̨TZ�м���
#define BOTH_TZ_EDDYSWITCH			10610  // ΢��̨TZ�лص�����
#define BOTH_TZ_IDENTIFY		    10611  // ΢TZ��ʶ
#define BOTH_TZ_RUN_FORWARD			10612	// ΢TZ�߹켣+
#define BOTH_TZ_RUN_REVERSE			10613  // ΢TZ�߹켣-

//yht
// Fine DZ
#define FINE_DZ_OPEN				10701  // ΢DZ����
#define FINE_DZ_CLOSE				10702  // ΢DZ�ջ�
#define FINE_DZ_STEP_FORWARD		10703  // ΢DZstep+
#define FINE_DZ_STEP_REVERSE      	10704  // ΢DZstep-		
#define FINE_DZ_JOG_FORWARD    		10705  // ΢DZJog+
#define FINE_DZ_JOG_REVERSE    		10706  // ΢DZJog-
#define FINE_DZ_JOG_STOP	     	10707  // ΢DZJog+ͣ
#define FINE_DZ_HOME				10708  // ΢DZ����
#define FINE_DZ_LASERSWITCH			10709  // ΢��̨DX�м���
#define FINE_DZ_EDDYSWITCH			10710  // ΢��̨DX�лص�����
#define FINE_DZ_IDENTIFY		    10711  // ΢DX��ʶ
#define FINE_DZ_RUN_FORWARD			10712	// ΢DX�߹켣+
#define FINE_DZ_RUN_REVERSE			10713  // ΢DX�߹켣-
// Fine TX
#define FINE_TX_OPEN				10801  // ΢DY����
#define FINE_TX_CLOSE				10802  // ΢DY�ջ�
#define FINE_TX_STEP_FORWARD		10803  // ΢DYstep+
#define FINE_TX_STEP_REVERSE      	10804  // ΢DYstep-		
#define FINE_TX_JOG_FORWARD    		10805  // ΢DYJog+
#define FINE_TX_JOG_REVERSE    		10806  // ΢DYJog-
#define FINE_TX_JOG_STOP			10807  // ΢DYJog+ͣ
#define FINE_TX_HOME				10808  // ΢DY����
#define FINE_TX_LASERSWITCH			10809  // ΢��̨DY�м���
#define FINE_TX_EDDYSWITCH			10810  // ΢��̨DY�лص�����
#define FINE_TX_IDENTIFY		    10811  // ΢DY��ʶ
#define FINE_TX_RUN_FORWARD			10812	// ΢DY�߹켣+
#define FINE_TX_RUN_REVERSE			10813  // ΢DY�߹켣-

//Fine TY
#define FINE_TY_OPEN				10901  // ΢TZ����
#define FINE_TY_CLOSE				10902  // ΢TZ�ջ�
#define FINE_TY_STEP_FORWARD		10903  // ΢TZstep+
#define FINE_TY_STEP_REVERSE      	10904  // ΢TZstep-		
#define FINE_TY_JOG_FORWARD    		10905  // ΢TZJog+
#define FINE_TY_JOG_REVERSE    		10906  // ΢TZJog-
#define FINE_TY_JOG_STOP			10907  // ΢TZJog+ͣ
#define FINE_TY_HOME				10908  // ΢TZ����
#define FINE_TY_LASERSWITCH			10909  // ΢��̨TZ�м���
#define FINE_TY_EDDYSWITCH			10910  // ΢��̨TZ�лص�����
#define FINE_TY_IDENTIFY		    10911  // ΢TZ��ʶ
#define FINE_TY_RUN_FORWARD			10912	// ΢TZ�߹켣+
#define FINE_TY_RUN_REVERSE			10913  // ΢TZ�߹켣-

extern void GetCMD(int *pCMD);
#endif
