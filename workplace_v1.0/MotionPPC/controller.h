#ifndef CONTROLLER_H
#define CONTROLLER_H
/*类型定义*/
typedef struct PID
{
	double Proportion;
	double Integral;
	double Derivative;
	double error;
	double dError;
	double sumError;
}PID;
typedef struct LLC
{
	double *pIn;         
	double *pUout;
	double *pA;
	double *pB;
	int order;
}LLC;
typedef struct FIR
{
	double	*FIR_Coeff; 
	double	*FIR_In;
	double	FIR_Out;
	int  FIR_iOrder;
} FIR;

typedef struct AFC
{
	double *pA;
	double *pB;
	double *pin;
	double *pout;
} AFC;
/*全局函数声明*/
extern PID sCogPID;

extern double PIDCalculate(PID* pp, double error);
extern void PIDClear(PID* pp);
extern double LLCCalculate(LLC* pp, double error);
extern void LLCClear(LLC* pp);
extern double Filter2order(double *Input,double flp_Laser,double clp_Laser,double* blp,double* alp , double* Output,int* Flag);
extern double FIR_Controller(FIR *pp,double lCurAcc);
extern double Notch_Laser(double *Input,double fnp_Laser,double cnp_Laser,double fnz_Laser,double cnz_Laser,double* blp,double* alp , double* Output,int* Flag);
extern double PID_Calc_Balance(PID *pp,long NextPoint);

extern void PIDInit (PID *pp);


#endif
