/*********************************************
文件名：ConParam.c
文件功能：设置所有反馈控制器与前馈控制器的参数
函数表：void SetPID_Counter(int iAxisNum)
		void SetLLCCounter(int iAxisNum)
		void SetPID_Switch(int iAxisNum)
		void SetLLC_Switch(int iAxisNum)
		void SetLP(int iAxisNum)
*********************************************/
#include "ConParamInit.h"
#include "controller.h"
#include "rtcontrol.h"
#include "StateMachine.h"
#include "TrajHandler.h"
#include "ConCalc.h"
#include "GetData.h"

////////////////////////////////////////////////////////x////////////////////////////////////////////////
//偏置电压
int CoarseOffset[4] = {0.0,0.0,0.0,0.0};
int FineOffset[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
int CoarseLimit[4] = {3.0*3276.8/2,3.0*3276.8/2,3.0*3276.8/2,3.0*3276.8/2};
int FineLimit[6] = {3.0*3276.8/2,3.0*3276.8/2,3.0*3276.8/2,3.0*3276.8/2,3.0*3276.8/2,3.0*3276.8/2};

int Coarse_CounterPIDType[COARSE_AXIS_NUM] = {0,0,0,0};
int Coarse_SwitchPIDType[COARSE_AXIS_NUM] = {0,0,0,0};
int Fine_CounterPIDType[FINE_AXIS_NUM] = {0,0,0,0,0,0};
int Fine_SwitchPIDType[FINE_AXIS_NUM] = {0,0,0,0,0,0};

/////////////////////////////////////粗动台////////////////////////////////////////////
//DX
double Coarse_bx[ORDER_COARSE_X]={1.000000000000,0.0,0.0,0.0,0.0}; 
double Coarse_ax[ORDER_COARSE_X]={1.000000000000,0.0,0.0,0.0,0.0};
double Coarse_UXout[ORDER_COARSE_X]={0.0};
double Coarse_Xin[ORDER_COARSE_X]={0.0};
int Coarse_order_x=ORDER_COARSE_X;

//DY
double Coarse_by[ORDER_COARSE_Y]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Coarse_ay[ORDER_COARSE_Y]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Coarse_UYout[ORDER_COARSE_Y]={0.0};
double Coarse_Yin[ORDER_COARSE_Y]={0.0};
int Coarse_order_y=ORDER_COARSE_Y;

//XTZ
double Coarse_bxtz[ORDER_COARSE_XTZ]={1.000000000000, 0.0, 0.0};
double Coarse_axtz[ORDER_COARSE_XTZ]={1.000000000000, 0.0, 0.0};
double Coarse_UXTZout[ORDER_COARSE_XTZ]={0.0};
double Coarse_XTZin[ORDER_COARSE_XTZ]={0.0};
int Coarse_order_xtz=ORDER_COARSE_XTZ;

//YTZ
double Coarse_bytz[ORDER_COARSE_YTZ]={1.000000000000, 0.0, 0.0};
double Coarse_aytz[ORDER_COARSE_YTZ]={1.000000000000, 0.0, 0.0};
double Coarse_UYTZout[ORDER_COARSE_YTZ]={0.0};
double Coarse_YTZin[ORDER_COARSE_YTZ]={0.0};
int Coarse_order_ytz=ORDER_COARSE_YTZ;

/////////////////////////////////////粗动台切换////////////////////////////////////////////
//DX
double Switch_Coarse_bx[SWITCH_ORDER_COARSE_X]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Coarse_ax[SWITCH_ORDER_COARSE_X]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Coarse_UXout[SWITCH_ORDER_COARSE_X]={0.0};
double Switch_Coarse_Xin[SWITCH_ORDER_COARSE_X]={0.0};
int Switch_Coarse_order_x=SWITCH_ORDER_COARSE_X;

//DY
double Switch_Coarse_by[SWITCH_ORDER_COARSE_Y]={1.000000000000, 0.0, 0.0};
double Switch_Coarse_ay[SWITCH_ORDER_COARSE_Y]={1.000000000000, 0.0, 0.0};
double Switch_Coarse_UYout[SWITCH_ORDER_COARSE_Y]={0.0};
double Switch_Coarse_Yin[SWITCH_ORDER_COARSE_Y]={0.0};
int Switch_Coarse_order_y=SWITCH_ORDER_COARSE_Y;

//XTZ
double Switch_Coarse_bxtz[ORDER_COARSE_XTZ]={1.000000000000, 0.0, 0.0};
double Switch_Coarse_axtz[ORDER_COARSE_XTZ]={1.000000000000, 0.0, 0.0};
double Switch_Coarse_UXTZout[ORDER_COARSE_XTZ]={0.0};
double Switch_Coarse_XTZin[ORDER_COARSE_XTZ]={0.0};
int Switch_Coarse_order_xtz=ORDER_COARSE_XTZ;

//YTZ
double Switch_Coarse_bytz[SWITCH_ORDER_COARSE_YTZ]={1.000000000000, 0.0, 0.0};
double Switch_Coarse_aytz[SWITCH_ORDER_COARSE_YTZ]={1.000000000000, 0.0, 0.0};
double Switch_Coarse_UYTZout[SWITCH_ORDER_COARSE_YTZ]={0.0};
double Switch_Coarse_YTZin[SWITCH_ORDER_COARSE_YTZ]={0.0};
int Switch_Coarse_order_ytz=SWITCH_ORDER_COARSE_YTZ;

////////////////////////////////////////////////////////滤波器参数////////////////////////////////////////////////
double  ILP_b[ORDER_LP]={1.0,-1.4814478,0.5905994};
double  ILP_a[ORDER_LP]={0.0272879,0.0545758,0.0272879};

/////////////////////////////////////微动台////////////////////////////////////////////
//DX
double Fine_bx[ORDER_FINE_X]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_ax[ORDER_FINE_X]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_UXout[ORDER_FINE_X]={0.0};
double Fine_Xin[ORDER_FINE_X]={0.0};
int Fine_order_x=ORDER_FINE_X;
//DY
double Fine_by[ORDER_FINE_Y]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_ay[ORDER_FINE_Y]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_UYout[ORDER_FINE_Y]={0.0};
double Fine_Yin[ORDER_FINE_Y]={0.0};
int Fine_order_y=ORDER_FINE_Y;
//TZ
double Fine_btz[ORDER_FINE_TZ]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_atz[ORDER_FINE_TZ]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_UTZout[ORDER_FINE_TZ]={0.0};
double Fine_TZin[ORDER_FINE_TZ]={0.0};
int Fine_order_tz=ORDER_FINE_TZ;

//DZ
double Fine_bz[ORDER_FINE_Z]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_az[ORDER_FINE_Z]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_UZout[ORDER_FINE_Z]={0.0};
double Fine_Zin[ORDER_FINE_Z]={0.0};
int Fine_order_z=ORDER_FINE_Z;

//TX
double Fine_btx[ORDER_FINE_TX]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_atx[ORDER_FINE_TX]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_UTXout[ORDER_FINE_TX]={0.0};
double Fine_TXin[ORDER_FINE_TX]={0.0};
int Fine_order_tx=ORDER_FINE_TX;
//TY
double Fine_bty[ORDER_FINE_TY]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_aty[ORDER_FINE_TY]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_UTYout[ORDER_FINE_TY]={0.0};
double Fine_TYin[ORDER_FINE_TY]={0.0};
int Fine_order_ty=ORDER_FINE_TY;

///////////////微动台滤波
//DX
//double FineFilter_bx[ORDER_FINE_FILTER_X]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
//double FineFilter_ax[ORDER_FINE_FILTER_X]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
// 300 Hz lowpass filter
//double FineFilter_bx[ORDER_FINE_FILTER_X]={ 0.02728790, 0.05457580, 0.02728790, 0.00000000, 0.00000000 };
//double FineFilter_ax[ORDER_FINE_FILTER_X]={ 1.00000000, -1.48144783, 0.59059944, 0.00000000, 0.00000000 };

// 200 Hz lowpass filter
double FineFilter_bx[ORDER_FINE_FILTER_X]={0.01323136, 0.02646273, 0.01323136, 0.00000000, 0.00000000};
double FineFilter_ax[ORDER_FINE_FILTER_X]={1.00000000, -1.64930918, 0.70223464, 0.00000000, 0.00000000};

// 120 Hz lowpass filter
//double FineFilter_bx[ORDER_FINE_FILTER_X]={0.00511094, 0.01022189, 0.00511094, 0.00000000, 0.00000000};
//double FineFilter_ax[ORDER_FINE_FILTER_X]={1.00000000, -1.78785744, 0.80830121, 0.00000000, 0.00000000};



double FineFilter_UXout[ORDER_FINE_FILTER_X]={0.0};
double FineFilter_Xin[ORDER_FINE_FILTER_X]={0.0};
int FineFilter_order_x=ORDER_FINE_FILTER_X;
//DY
//double FineFilter_by[ORDER_FINE_FILTER_Y]={ 0.02728790, 0.05457580, 0.02728790, 0.00000000, 0.00000000 };
//double FineFilter_ay[ORDER_FINE_FILTER_Y]={ 1.00000000, -1.48144783, 0.59059944, 0.00000000, 0.00000000 };
// 120 Hz lowpass filter
double FineFilter_by[ORDER_FINE_FILTER_X]={0.01323136, 0.02646273, 0.01323136, 0.00000000, 0.00000000};
double FineFilter_ay[ORDER_FINE_FILTER_X]={1.00000000, -1.64930918, 0.70223464, 0.00000000, 0.00000000};

double FineFilter_UYout[ORDER_FINE_FILTER_Y]={0.0};
double FineFilter_Yin[ORDER_FINE_FILTER_Y]={0.0};
int FineFilter_order_y=ORDER_FINE_FILTER_Y;
//TZ
//double FineFilter_btz[ORDER_FINE_FILTER_TZ]={ 0.02728790, 0.05457580, 0.02728790, 0.00000000, 0.00000000 };
//double FineFilter_atz[ORDER_FINE_FILTER_TZ]={ 1.00000000, -1.48144783, 0.59059944, 0.00000000, 0.00000000 };
// 120 Hz lowpass filter
double FineFilter_btz[ORDER_FINE_FILTER_X]={0.01323136, 0.02646273, 0.01323136, 0.00000000, 0.00000000};
double FineFilter_atz[ORDER_FINE_FILTER_X]={1.00000000, -1.64930918, 0.70223464, 0.00000000, 0.00000000};

// double FineFilter_btz[ORDER_FINE_FILTER_TZ]={0.00511094, 0.01022189, 0.00511094, 0.00000000, 0.00000000};
// double FineFilter_atz[ORDER_FINE_FILTER_TZ]={1.00000000, -1.78785744, 0.80830121, 0.00000000, 0.00000000};
double FineFilter_UTZout[ORDER_FINE_FILTER_TZ]={0.0};
double FineFilter_TZin[ORDER_FINE_FILTER_TZ]={0.0};
int FineFilter_order_tz=ORDER_FINE_FILTER_TZ;

//DZ
//double FineFilter_bz[ORDER_FINE_FILTER_Z]={ 0.02728790, 0.05457580, 0.02728790, 0.00000000, 0.00000000 };
//double FineFilter_az[ORDER_FINE_FILTER_Z]={ 1.00000000, -1.48144783, 0.59059944, 0.00000000, 0.00000000 };
// 120 Hz lowpass filter
double FineFilter_bz[ORDER_FINE_FILTER_X]={0.01323136, 0.02646273, 0.01323136, 0.00000000, 0.00000000};
double FineFilter_az[ORDER_FINE_FILTER_X]={1.00000000, -1.64930918, 0.70223464, 0.00000000, 0.00000000};

double FineFilter_UZout[ORDER_FINE_FILTER_Z]={0.0};
double FineFilter_Zin[ORDER_FINE_FILTER_Z]={0.0};
int FineFilter_order_z=ORDER_FINE_FILTER_Z;
//TX
//double FineFilter_btx[ORDER_FINE_FILTER_TX]={ 0.02728790, 0.05457580, 0.02728790, 0.00000000, 0.00000000 };
//double FineFilter_atx[ORDER_FINE_FILTER_TX]={ 1.00000000, -1.48144783, 0.59059944, 0.00000000, 0.00000000 };
// 120 Hz lowpass filter
double FineFilter_btx[ORDER_FINE_FILTER_X]={0.01323136, 0.02646273, 0.01323136, 0.00000000, 0.00000000};
double FineFilter_atx[ORDER_FINE_FILTER_X]={1.00000000, -1.64930918, 0.70223464, 0.00000000, 0.00000000};

double FineFilter_UTXout[ORDER_FINE_FILTER_TX]={0.0};
double FineFilter_TXin[ORDER_FINE_FILTER_TX]={0.0};
int FineFilter_order_tx=ORDER_FINE_FILTER_TX;
//TY
//double FineFilter_bty[ORDER_FINE_FILTER_TY]={ 0.02728790, 0.05457580, 0.02728790, 0.00000000, 0.00000000 };
//double FineFilter_aty[ORDER_FINE_FILTER_TY]={ 1.00000000, -1.48144783, 0.59059944, 0.00000000, 0.00000000 };
// 120 Hz lowpass filter
double FineFilter_bty[ORDER_FINE_FILTER_X]={0.01323136, 0.02646273, 0.01323136, 0.00000000, 0.00000000};
double FineFilter_aty[ORDER_FINE_FILTER_X]={1.00000000, -1.64930918, 0.70223464, 0.00000000, 0.00000000};

double FineFilter_UTYout[ORDER_FINE_FILTER_TY]={0.0};
double FineFilter_TYin[ORDER_FINE_FILTER_TY]={0.0};
int FineFilter_order_ty=ORDER_FINE_FILTER_TY;



/////////////////////////////////////微动台切换////////////////////////////////////////////
//DX
double Switch_Fine_bx[SWITCH_ORDER_FINE_X]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_ax[SWITCH_ORDER_FINE_X]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_UXout[SWITCH_ORDER_FINE_X]={0.0};
double Switch_Fine_Xin[SWITCH_ORDER_FINE_X]={0.0};
int Switch_Fine_order_x=SWITCH_ORDER_FINE_X;
//DY
double Switch_Fine_by[SWITCH_ORDER_FINE_Y]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_ay[SWITCH_ORDER_FINE_Y]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_UYout[SWITCH_ORDER_FINE_Y]={0.0};
double Switch_Fine_Yin[SWITCH_ORDER_FINE_Y]={0.0};
int Switch_Fine_order_y=SWITCH_ORDER_FINE_Y;
//TZ
double Switch_Fine_btz[SWITCH_ORDER_FINE_TZ]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_atz[SWITCH_ORDER_FINE_TZ]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_UTZout[SWITCH_ORDER_FINE_TZ]={0.0};
double Switch_Fine_TZin[SWITCH_ORDER_FINE_TZ]={0.0};
int Switch_Fine_order_tz=SWITCH_ORDER_FINE_TZ;

//DZ
double Switch_Fine_bz[SWITCH_ORDER_FINE_Z]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_az[SWITCH_ORDER_FINE_Z]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_UZout[SWITCH_ORDER_FINE_Z]={0.0};
double Switch_Fine_Zin[SWITCH_ORDER_FINE_Z]={0.0};
int Switch_Fine_order_z=SWITCH_ORDER_FINE_Z;
//TX
double Switch_Fine_btx[SWITCH_ORDER_FINE_TX]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_atx[SWITCH_ORDER_FINE_TX]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_UTXout[SWITCH_ORDER_FINE_TX]={0.0};
double Switch_Fine_TXin[SWITCH_ORDER_FINE_TX]={0.0};
int Switch_Fine_order_tx=SWITCH_ORDER_FINE_TX;
//TY
double Switch_Fine_bty[SWITCH_ORDER_FINE_TY]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_aty[SWITCH_ORDER_FINE_TY]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_UTYout[SWITCH_ORDER_FINE_TY]={0.0};
double Switch_Fine_TYin[SWITCH_ORDER_FINE_TY]={0.0};
int Switch_Fine_order_ty=SWITCH_ORDER_FINE_TY;






/////////////////////////////// 粗动台Y轨迹FIR////////////////////////////////////////////////////
double	DY_FIR_Coarse_Coeff[4][DY_ORDER_COARSE_FIR]={
//-------------------------------------------------------------------------------------------------------------------------------
//    0           1               2               3               4               5               6								
//-------------------------------------------------------------------------------------------------------------------------------  
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//DX
	{ 0.000000000000, 0.000000000000,   0.000000000000, 0.000000000000,   0.000000000000, 0.000000000000,  0.000000000000},//DY
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//XTZ
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//YTZ
//-------------------------------------------------------------------------------------------------------------------------------
};
//---IN
double	DY_FIR_Coarse_In_DX[DY_ORDER_COARSE_FIR]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DY_FIR_Coarse_In_DY[DY_ORDER_COARSE_FIR]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DY_FIR_Coarse_In_XTZ[DY_ORDER_COARSE_FIR]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DY_FIR_Coarse_In_YTZ[DY_ORDER_COARSE_FIR]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
//---OUT
double	DY_FIR_Coarse_Out[4]={0.0, 0.0, 0.0, 0.0};
//ORDER
int FIR_DY_Coarse_iOrder = DY_ORDER_COARSE_FIR;
/////////////////////////////// 粗动台X轨迹FIR////////////////////////////////////////////////////
double	DX_FIR_Coarse_Coeff[4][DX_ORDER_COARSE_FIR]={
//-------------------------------------------------------------------------------------------------------------------------------
//    0           1               2               3               4               5               6								
//-------------------------------------------------------------------------------------------------------------------------------  
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//DX
	{ 0.000000000000, 0.000000000000,   0.000000000000, 0.000000000000,   0.000000000000, 0.000000000000,  0.000000000000},//DY
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//XTZ
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//YTZ
//-------------------------------------------------------------------------------------------------------------------------------
};
//---IN
double	DX_FIR_Coarse_In_DX[DX_ORDER_COARSE_FIR]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DX_FIR_Coarse_In_DY[DX_ORDER_COARSE_FIR]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DX_FIR_Coarse_In_XTZ[DX_ORDER_COARSE_FIR]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DX_FIR_Coarse_In_YTZ[DX_ORDER_COARSE_FIR]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
//---OUT
double	DX_FIR_Coarse_Out[4]={0.0, 0.0, 0.0, 0.0};
//ORDER
int FIR_DX_Coarse_iOrder = DX_ORDER_COARSE_FIR;
/////////////////////////////// 粗动台切换Y轨迹FIR////////////////////////////////////////////////////
double	DY_FIR_Coarse_Switch_Coeff[4][DY_ORDER_COARSE_FIR_SWITCH]={
//-------------------------------------------------------------------------------------------------------------------------------
//    0           1               2               3               4               5               6								
//-------------------------------------------------------------------------------------------------------------------------------  
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//DX
	{ 0.000000000000, 0.000000000000,   0.000000000000, 0.000000000000,   0.000000000000, 0.000000000000,  0.000000000000},//DY
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//XTZ
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//YTZ
//-------------------------------------------------------------------------------------------------------------------------------
};
//---IN
double	DY_FIR_Coarse_Switch_In_DX[DY_ORDER_COARSE_FIR_SWITCH]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DY_FIR_Coarse_Switch_In_DY[DY_ORDER_COARSE_FIR_SWITCH]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DY_FIR_Coarse_Switch_In_XTZ[DY_ORDER_COARSE_FIR_SWITCH]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DY_FIR_Coarse_Switch_In_YTZ[DY_ORDER_COARSE_FIR_SWITCH]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
//---OUT
double	DY_FIR_Coarse_Switch_Out[4]={0.0, 0.0, 0.0, 0.0};
//ORDER
int FIR_DY_Coarse_iOrder_Switch = DY_ORDER_COARSE_FIR_SWITCH;
/////////////////////////////// 粗动台切换X轨迹FIR////////////////////////////////////////////////////
double	DX_FIR_Coarse_Switch_Coeff[4][DX_ORDER_COARSE_FIR_SWITCH]={
//-------------------------------------------------------------------------------------------------------------------------------
//    0           1               2               3               4               5               6								
//-------------------------------------------------------------------------------------------------------------------------------  
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//DX
	{ 0.000000000000, 0.000000000000,   0.000000000000, 0.000000000000,   0.000000000000, 0.000000000000,  0.000000000000},//DY
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//XTZ
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//YTZ
//-------------------------------------------------------------------------------------------------------------------------------
};
//---IN
double	DX_FIR_Coarse_Switch_In_DX[DX_ORDER_COARSE_FIR_SWITCH]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DX_FIR_Coarse_Switch_In_DY[DX_ORDER_COARSE_FIR_SWITCH]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DX_FIR_Coarse_Switch_In_XTZ[DX_ORDER_COARSE_FIR_SWITCH]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DX_FIR_Coarse_Switch_In_YTZ[DX_ORDER_COARSE_FIR_SWITCH]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
//---OUT
double	DX_FIR_Coarse_Switch_Out[4]={0.0, 0.0, 0.0, 0.0};
//ORDER
int FIR_DX_Coarse_iOrder_Switch = DX_ORDER_COARSE_FIR_SWITCH;
/////////////////////////////// 微动台Y轨迹FIR////////////////////////////////////////////////////
double	DY_FIR_Fine_Coeff[6][DY_ORDER_FINE_FIR]={
//-------------------------------------------------------------------------------------------------------------------------------
//    0           1               2               3               4               5               6								
//-------------------------------------------------------------------------------------------------------------------------------  
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//DX
	{ 0.000000000000, 0.000000000000,   0.000000000000, 0.000000000000,   0.000000000000, 0.000000000000,  0.000000000000},//DY
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//TZ
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//DZ
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//TX
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//TY
//-------------------------------------------------------------------------------------------------------------------------------
};
//---IN
double	DY_FIR_Fine_In_DX[DY_ORDER_FINE_FIR]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DY_FIR_Fine_In_DY[DY_ORDER_FINE_FIR]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DY_FIR_Fine_In_TZ[DY_ORDER_FINE_FIR]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DY_FIR_Fine_In_DZ[DY_ORDER_FINE_FIR]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DY_FIR_Fine_In_TX[DY_ORDER_FINE_FIR]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DY_FIR_Fine_In_TY[DY_ORDER_FINE_FIR]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};


//---OUT
double	DY_FIR_Fine_Out[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//ORDER
int FIR_DY_Fine_iOrder = DY_ORDER_FINE_FIR;
/////////////////////////////// 微动台X轨迹FIR////////////////////////////////////////////////////
double	DX_FIR_Fine_Coeff[6][DX_ORDER_FINE_FIR]={
//-------------------------------------------------------------------------------------------------------------------------------
//    0           1               2               3               4               5               6								
//-------------------------------------------------------------------------------------------------------------------------------  
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//DX
	{ 0.000000000000, 0.000000000000,   0.000000000000, 0.000000000000,   0.000000000000, 0.000000000000,  0.000000000000},//DY
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//TZ
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//DZ
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//TX
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//TY
//-------------------------------------------------------------------------------------------------------------------------------
};
//---IN
double	DX_FIR_Fine_In_DX[DX_ORDER_FINE_FIR]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DX_FIR_Fine_In_DY[DX_ORDER_FINE_FIR]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DX_FIR_Fine_In_TZ[DX_ORDER_FINE_FIR]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DX_FIR_Fine_In_DZ[DX_ORDER_FINE_FIR]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DX_FIR_Fine_In_TX[DX_ORDER_FINE_FIR]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DX_FIR_Fine_In_TY[DX_ORDER_FINE_FIR]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};

//---OUT
double	DX_FIR_Fine_Out[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//ORDER
int FIR_DX_Fine_iOrder = DX_ORDER_FINE_FIR;
/////////////////////////////// 微动台切换Y轨迹FIR////////////////////////////////////////////////////
double	DY_FIR_Fine_Switch_Coeff[6][DY_ORDER_FINE_FIR_SWITCH]={
//-------------------------------------------------------------------------------------------------------------------------------
//    0           1               2               3               4               5               6								
//-------------------------------------------------------------------------------------------------------------------------------  
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//DX
	{ 0.000000000000, 0.000000000000,   0.000000000000, 0.000000000000,   0.000000000000, 0.000000000000,  0.000000000000},//DY
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//TZ
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//DZ
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//TX
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//TY
//-------------------------------------------------------------------------------------------------------------------------------
};
//---IN
double	DY_FIR_Fine_Switch_In_DX[DY_ORDER_FINE_FIR_SWITCH]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DY_FIR_Fine_Switch_In_DY[DY_ORDER_FINE_FIR_SWITCH]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DY_FIR_Fine_Switch_In_TZ[DY_ORDER_FINE_FIR_SWITCH]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DY_FIR_Fine_Switch_In_DZ[DY_ORDER_FINE_FIR_SWITCH]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DY_FIR_Fine_Switch_In_TX[DY_ORDER_FINE_FIR_SWITCH]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DY_FIR_Fine_Switch_In_TY[DY_ORDER_FINE_FIR_SWITCH]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};

//---OUT
double	DY_FIR_Fine_Switch_Out[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//ORDER
int FIR_DY_Fine_iOrder_Switch = DY_ORDER_FINE_FIR_SWITCH;
/////////////////////////////// 微动台切换X轨迹FIR////////////////////////////////////////////////////
double	DX_FIR_Fine_Switch_Coeff[6][DX_ORDER_FINE_FIR_SWITCH]={
//-------------------------------------------------------------------------------------------------------------------------------
//    0           1               2               3               4               5               6								
//-------------------------------------------------------------------------------------------------------------------------------  
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//DX
	{ 0.000000000000, 0.000000000000,   0.000000000000, 0.000000000000,   0.000000000000, 0.000000000000,  0.000000000000},//DY
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//TZ
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//DZ
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//TX
	{ 0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000,  0.000000000000, -0.000000000000, 0.000000000000},//TY
//-------------------------------------------------------------------------------------------------------------------------------
};
//---IN
double	DX_FIR_Fine_Switch_In_DX[DX_ORDER_FINE_FIR_SWITCH]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DX_FIR_Fine_Switch_In_DY[DX_ORDER_FINE_FIR_SWITCH]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DX_FIR_Fine_Switch_In_TZ[DX_ORDER_FINE_FIR_SWITCH]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DX_FIR_Fine_Switch_In_DZ[DX_ORDER_FINE_FIR_SWITCH]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DX_FIR_Fine_Switch_In_TX[DX_ORDER_FINE_FIR_SWITCH]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double	DX_FIR_Fine_Switch_In_TY[DX_ORDER_FINE_FIR_SWITCH]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};

//---OUT
double	DX_FIR_Fine_Switch_Out[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//ORDER
int FIR_DX_Fine_iOrder_Switch = DX_ORDER_FINE_FIR_SWITCH;
/*********************************微动台AFC控制器******************************/
//DX方向系数
double DX_Fine_x_in[4][6] = {
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	};
double DX_Fine_y_out[4][6] = {
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 };
double DX_Fine_b[4][6] = {
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 };
double DX_Fine_a[4][6] = {
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 };
//DY方向系数
double DY_Fine_x_in[4][6] = {
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	};
double DY_Fine_y_out[4][6] = {
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 };
double DY_Fine_b[4][6] = {
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
						 };
double DY_Fine_a[4][6] = {
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 };
//TZ方向系数
double TZ_Fine_x_in[4][6] = {
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	};
double TZ_Fine_y_out[4][6] = {
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 };
double TZ_Fine_b[4][6] = {
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 };
double TZ_Fine_a[4][6] = {
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 };


//DZ方向系数
double DZ_Fine_x_in[4][6] = {
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	};
double DZ_Fine_y_out[4][6] = {
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 };
double DZ_Fine_b[4][6] = {
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 };
double DZ_Fine_a[4][6] = {
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 };

//TX方向系数
double TX_Fine_x_in[4][6] = {
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	};
double TX_Fine_y_out[4][6] = {
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 };
double TX_Fine_b[4][6] = {
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 };
double TX_Fine_a[4][6] = {
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 };
//TY方向系数
double TY_Fine_x_in[4][6] = {
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	};
double TY_Fine_y_out[4][6] = {
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 };
double TY_Fine_b[4][6] = {
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 };
double TY_Fine_a[4][6] = {
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 };

/*********************************切换后的AFC控制器******************************/
//DX方向系数
double DX_Fine_Switch_x_in[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						           };
double DX_Fine_Switch_y_out[4][6] = {
						            {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	            {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	            {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	            {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						            };
double DX_Fine_Switch_b[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						        };
double DX_Fine_Switch_a[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						        };
//DY方向系数
double DY_Fine_Switch_x_in[4][6] = {		           
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						           };
						           
double DY_Fine_Switch_y_out[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						           };
double DY_Fine_Switch_b[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						        };
double DY_Fine_Switch_a[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						        };
//TZ方向系数
double TZ_Fine_Switch_x_in[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						           };
double TZ_Fine_Switch_y_out[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						           };
double TZ_Fine_Switch_b[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						         };
double TZ_Fine_Switch_a[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						        };

//DZ方向系数
double DZ_Fine_Switch_x_in[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						           };
double DZ_Fine_Switch_y_out[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						           };
double DZ_Fine_Switch_b[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						         };
double DZ_Fine_Switch_a[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						        };
//TX方向系数
double TX_Fine_Switch_x_in[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						           };
double TX_Fine_Switch_y_out[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						           };
double TX_Fine_Switch_b[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						         };
double TX_Fine_Switch_a[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						        };
//TY方向系数
double TY_Fine_Switch_x_in[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						           };
double TY_Fine_Switch_y_out[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						           };
double TY_Fine_Switch_b[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						         };
double TY_Fine_Switch_a[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						        };

/******************************************
函数名：SetPID_Coarse_Counter
返回值：无
参数表：int iAxisNum	粗动台四自由度轴号
功能说明：设置电涡流闭环下的粗动台四自由度的PID参数
******************************************/
void SetPID_Coarse_Counter(int iAxis)
{
	switch(iAxis)
	{
		case CDX:
			Machine.coarseStage.arrAxis[CDX].sPID.Proportion = 0.000025;
			Machine.coarseStage.arrAxis[CDX].sPID.Integral = 0.0;
			Machine.coarseStage.arrAxis[CDX].sPID.Derivative = 0.00025;
		break;
		case CDY:
			Machine.coarseStage.arrAxis[CDY].sPID.Proportion = 0.000025;
			Machine.coarseStage.arrAxis[CDY].sPID.Integral = 0.0;
			Machine.coarseStage.arrAxis[CDY].sPID.Derivative = 0.00025;
		break;
		case XTZ:
			Machine.coarseStage.arrAxis[XTZ].sPID.Proportion = 0.000025;
			Machine.coarseStage.arrAxis[XTZ].sPID.Integral = 0.0;
			Machine.coarseStage.arrAxis[XTZ].sPID.Derivative = 0.00025;
		break;
		case YTZ:
			Machine.coarseStage.arrAxis[YTZ].sPID.Proportion = 0.000025;
			Machine.coarseStage.arrAxis[YTZ].sPID.Integral = 0.0;
			Machine.coarseStage.arrAxis[YTZ].sPID.Derivative = 0.00025;
		break;
		default:	break;
	}
}

/******************************************
函数名：SetLLC_Coarse_Counter
返回值：无
参数表：int iAxisNum	粗动台四自由度轴号
功能说明：设置电涡流闭环下的粗动台四自由度的超前滞后控制器参数
******************************************/
void SetLLC_Coarse_Counter(int iAxis)
{
	switch(iAxis)
	{
		case CDX:
			Machine.coarseStage.arrAxis[CDX].sLLC.pUout = Coarse_UXout;
			Machine.coarseStage.arrAxis[CDX].sLLC.pA = Coarse_ax;
			Machine.coarseStage.arrAxis[CDX].sLLC.pB = Coarse_bx;
			Machine.coarseStage.arrAxis[CDX].sLLC.pIn = Coarse_Xin;
			Machine.coarseStage.arrAxis[CDX].sLLC.order = Coarse_order_x;
		break;
		case CDY:
			Machine.coarseStage.arrAxis[CDY].sLLC.pUout = Coarse_UYout;
			Machine.coarseStage.arrAxis[CDY].sLLC.pA = Coarse_ay;
			Machine.coarseStage.arrAxis[CDY].sLLC.pB = Coarse_by;
			Machine.coarseStage.arrAxis[CDY].sLLC.pIn = Coarse_Yin;
			Machine.coarseStage.arrAxis[CDY].sLLC.order = Coarse_order_y;
		break;
		case XTZ:
			Machine.coarseStage.arrAxis[XTZ].sLLC.pUout = Coarse_UXTZout;
			Machine.coarseStage.arrAxis[XTZ].sLLC.pA = Coarse_axtz;
			Machine.coarseStage.arrAxis[XTZ].sLLC.pB = Coarse_bxtz;
			Machine.coarseStage.arrAxis[XTZ].sLLC.pIn = Coarse_XTZin;
			Machine.coarseStage.arrAxis[XTZ].sLLC.order = Coarse_order_xtz;
		break;
		case YTZ:
			Machine.coarseStage.arrAxis[YTZ].sLLC.pUout = Coarse_UYTZout;
			Machine.coarseStage.arrAxis[YTZ].sLLC.pA = Coarse_aytz;
			Machine.coarseStage.arrAxis[YTZ].sLLC.pB = Coarse_bytz;
			Machine.coarseStage.arrAxis[YTZ].sLLC.pIn = Coarse_YTZin;
			Machine.coarseStage.arrAxis[YTZ].sLLC.order = Coarse_order_ytz;
		break;
		default:	break;
	}	
}

/******************************************
函数名：SetPID_Coarse_Switch
返回值：无
参数表：int iAxisNum	粗动台四自由度轴号
功能说明：设置切换后的粗动台四自由度PID参数
******************************************/
void SetPID_Coarse_Switch(int iAxis)
{
	switch(iAxis)
	{
		case CDX:
			Machine.coarseStage.arrAxis[CDX].sPID_Switch.Proportion = 1.0;
			Machine.coarseStage.arrAxis[CDX].sPID_Switch.Integral = 0.0;
			Machine.coarseStage.arrAxis[CDX].sPID_Switch.Derivative = 0.0;
		break;
		case CDY:
			Machine.coarseStage.arrAxis[CDY].sPID_Switch.Proportion = 1.0;
			Machine.coarseStage.arrAxis[CDY].sPID_Switch.Integral = 0.0;
			Machine.coarseStage.arrAxis[CDY].sPID_Switch.Derivative = 0.0;
		break;
		case XTZ:
			Machine.coarseStage.arrAxis[XTZ].sPID_Switch.Proportion = 1.0;
			Machine.coarseStage.arrAxis[XTZ].sPID_Switch.Integral = 0.0;
			Machine.coarseStage.arrAxis[XTZ].sPID_Switch.Derivative = 0.0;
		break;
		case YTZ:
			Machine.coarseStage.arrAxis[YTZ].sPID_Switch.Proportion = 1.0;
			Machine.coarseStage.arrAxis[YTZ].sPID_Switch.Integral = 0.0;
			Machine.coarseStage.arrAxis[YTZ].sPID_Switch.Derivative = 0.0;
		break;
		default:	break;
	}
}
/******************************************
函数名：SetLLC_Coarse_Switch
返回值：无
参数表：int iAxisNum	粗动台四自由度轴号
功能说明：设置切换后的粗动台四自由度超前滞后控制器参数
******************************************/
void SetLLC_Coarse_Switch(int iAxis)
{
	switch(iAxis)
	{
		case CDX:
			Machine.coarseStage.arrAxis[CDX].sLLC_Switch.pUout = Switch_Coarse_UXout;
			Machine.coarseStage.arrAxis[CDX].sLLC_Switch.pA = Switch_Coarse_ax;
			Machine.coarseStage.arrAxis[CDX].sLLC_Switch.pB = Switch_Coarse_bx;
			Machine.coarseStage.arrAxis[CDX].sLLC_Switch.pIn = Switch_Coarse_Xin;
			Machine.coarseStage.arrAxis[CDX].sLLC_Switch.order = Switch_Coarse_order_x;
		break;
		case CDY:
			Machine.coarseStage.arrAxis[CDY].sLLC_Switch.pUout = Switch_Coarse_UYout;
			Machine.coarseStage.arrAxis[CDY].sLLC_Switch.pA = Switch_Coarse_ay;
			Machine.coarseStage.arrAxis[CDY].sLLC_Switch.pB = Switch_Coarse_by;
			Machine.coarseStage.arrAxis[CDY].sLLC_Switch.pIn = Switch_Coarse_Yin;
			Machine.coarseStage.arrAxis[CDY].sLLC_Switch.order = Switch_Coarse_order_y;
		break;
		case XTZ:
			Machine.coarseStage.arrAxis[XTZ].sLLC_Switch.pUout = Switch_Coarse_UXTZout;
			Machine.coarseStage.arrAxis[XTZ].sLLC_Switch.pA = Switch_Coarse_axtz;
			Machine.coarseStage.arrAxis[XTZ].sLLC_Switch.pB = Switch_Coarse_bxtz;
			Machine.coarseStage.arrAxis[XTZ].sLLC_Switch.pIn = Switch_Coarse_XTZin;
			Machine.coarseStage.arrAxis[XTZ].sLLC_Switch.order = Switch_Coarse_order_xtz;
		break;
		case YTZ:
			Machine.coarseStage.arrAxis[YTZ].sLLC_Switch.pUout = Switch_Coarse_UYTZout;
			Machine.coarseStage.arrAxis[YTZ].sLLC_Switch.pA = Switch_Coarse_aytz;
			Machine.coarseStage.arrAxis[YTZ].sLLC_Switch.pB = Switch_Coarse_bytz;
			Machine.coarseStage.arrAxis[YTZ].sLLC_Switch.pIn = Switch_Coarse_YTZin;
			Machine.coarseStage.arrAxis[YTZ].sLLC_Switch.order = Switch_Coarse_order_ytz;
		break;
		default:	break;
	}
}

/******************************************
函数名：SetFIRY_Coarse
返回值：无
参数表：int iAxisNum	四自由度轴号
功能说明：设置走DY方向轨迹时粗动台四自由度FIR前馈的参数
******************************************/
void SetFIRY_Coarse(int iAxis)
{
	switch(iAxis)
	{
		case CDX:
			Machine.coarseStage.arrAxis[CDX].sFIRY.FIR_In = DY_FIR_Coarse_In_DX;
			Machine.coarseStage.arrAxis[CDX].sFIRY.FIR_Out = DY_FIR_Coarse_Out[CDX];
			Machine.coarseStage.arrAxis[CDX].sFIRY.FIR_Coeff = DY_FIR_Coarse_Coeff[CDX];
			Machine.coarseStage.arrAxis[CDX].sFIRY.FIR_iOrder = FIR_DY_Coarse_iOrder;
		break;
		case CDY:
			Machine.coarseStage.arrAxis[CDY].sFIRY.FIR_In = DY_FIR_Coarse_In_DY;
			Machine.coarseStage.arrAxis[CDY].sFIRY.FIR_Out = DY_FIR_Coarse_Out[CDY];
			Machine.coarseStage.arrAxis[CDY].sFIRY.FIR_Coeff = DY_FIR_Coarse_Coeff[CDY];
			Machine.coarseStage.arrAxis[CDY].sFIRY.FIR_iOrder = FIR_DY_Coarse_iOrder;
		break;
		case XTZ:
			Machine.coarseStage.arrAxis[XTZ].sFIRY.FIR_In = DY_FIR_Coarse_In_XTZ;
			Machine.coarseStage.arrAxis[XTZ].sFIRY.FIR_Out = DY_FIR_Coarse_Out[XTZ];
			Machine.coarseStage.arrAxis[XTZ].sFIRY.FIR_Coeff = DY_FIR_Coarse_Coeff[XTZ];
			Machine.coarseStage.arrAxis[XTZ].sFIRY.FIR_iOrder = FIR_DY_Coarse_iOrder;
		break;
		case YTZ:
			Machine.coarseStage.arrAxis[YTZ].sFIRY.FIR_In = DY_FIR_Coarse_In_YTZ;
			Machine.coarseStage.arrAxis[YTZ].sFIRY.FIR_Out = DY_FIR_Coarse_Out[YTZ];
			Machine.coarseStage.arrAxis[YTZ].sFIRY.FIR_Coeff = DY_FIR_Coarse_Coeff[YTZ];
			Machine.coarseStage.arrAxis[YTZ].sFIRY.FIR_iOrder = FIR_DY_Coarse_iOrder;
		break;
		default:	break;
	}
}
/******************************************
函数名：SetFIRX_Coarse
返回值：无
参数表：int iAxisNum	四自由度轴号
功能说明：设置走DX方向轨迹时粗动台四自由度FIR前馈的参数
******************************************/
void SetFIRX_Coarse(int iAxis)
{
	switch(iAxis)
	{
		case CDX:
			Machine.coarseStage.arrAxis[CDX].sFIRX.FIR_In = DX_FIR_Coarse_In_DX;
			Machine.coarseStage.arrAxis[CDX].sFIRX.FIR_Out = DX_FIR_Coarse_Out[CDX];
			Machine.coarseStage.arrAxis[CDX].sFIRX.FIR_Coeff = DX_FIR_Coarse_Coeff[CDX];
			Machine.coarseStage.arrAxis[CDX].sFIRX.FIR_iOrder = FIR_DX_Coarse_iOrder;
		break;
		case CDY:
			Machine.coarseStage.arrAxis[CDY].sFIRX.FIR_In = DX_FIR_Coarse_In_DY;
			Machine.coarseStage.arrAxis[CDY].sFIRX.FIR_Out = DX_FIR_Coarse_Out[CDY];
			Machine.coarseStage.arrAxis[CDY].sFIRX.FIR_Coeff = DX_FIR_Coarse_Coeff[CDY];
			Machine.coarseStage.arrAxis[CDY].sFIRX.FIR_iOrder = FIR_DX_Coarse_iOrder;
		break;
		case XTZ:
			Machine.coarseStage.arrAxis[XTZ].sFIRX.FIR_In = DX_FIR_Coarse_In_XTZ;
			Machine.coarseStage.arrAxis[XTZ].sFIRX.FIR_Out = DX_FIR_Coarse_Out[XTZ];
			Machine.coarseStage.arrAxis[XTZ].sFIRX.FIR_Coeff = DX_FIR_Coarse_Coeff[XTZ];
			Machine.coarseStage.arrAxis[XTZ].sFIRX.FIR_iOrder = FIR_DX_Coarse_iOrder;
		break;
		case YTZ:
			Machine.coarseStage.arrAxis[YTZ].sFIRX.FIR_In = DX_FIR_Coarse_In_YTZ;
			Machine.coarseStage.arrAxis[YTZ].sFIRX.FIR_Out = DX_FIR_Coarse_Out[YTZ];
			Machine.coarseStage.arrAxis[YTZ].sFIRX.FIR_Coeff = DX_FIR_Coarse_Coeff[YTZ];
			Machine.coarseStage.arrAxis[YTZ].sFIRX.FIR_iOrder = FIR_DX_Coarse_iOrder;
		break;
		default:	break;
	}
}
/******************************************
函数名：SetFIRY_Coarse_Switch
返回值：无
参数表：int iAxisNum	四自由度轴号
功能说明：设置走DY方向轨迹时粗动台四自由度切换FIR前馈的参数
******************************************/
void SetFIRY_Coarse_Switch(int iAxis)
{
	switch(iAxis)
	{
		case CDX:
			Machine.coarseStage.arrAxis[CDX].sFIRY_Switch.FIR_In = DY_FIR_Coarse_Switch_In_DX;
			Machine.coarseStage.arrAxis[CDX].sFIRY_Switch.FIR_Out = DY_FIR_Coarse_Switch_Out[CDX];
			Machine.coarseStage.arrAxis[CDX].sFIRY_Switch.FIR_Coeff = DY_FIR_Coarse_Switch_Coeff[CDX];
			Machine.coarseStage.arrAxis[CDX].sFIRY_Switch.FIR_iOrder = FIR_DY_Coarse_iOrder_Switch;
		break;
		case CDY:
			Machine.coarseStage.arrAxis[CDY].sFIRY_Switch.FIR_In = DY_FIR_Coarse_Switch_In_DY;
			Machine.coarseStage.arrAxis[CDY].sFIRY_Switch.FIR_Out = DY_FIR_Coarse_Switch_Out[CDY];
			Machine.coarseStage.arrAxis[CDY].sFIRY_Switch.FIR_Coeff = DY_FIR_Coarse_Switch_Coeff[CDY];
			Machine.coarseStage.arrAxis[CDY].sFIRY_Switch.FIR_iOrder = FIR_DY_Coarse_iOrder_Switch;
		break;
		case XTZ:
			Machine.coarseStage.arrAxis[XTZ].sFIRY_Switch.FIR_In = DY_FIR_Coarse_Switch_In_XTZ;
			Machine.coarseStage.arrAxis[XTZ].sFIRY_Switch.FIR_Out = DY_FIR_Coarse_Switch_Out[XTZ];
			Machine.coarseStage.arrAxis[XTZ].sFIRY_Switch.FIR_Coeff = DY_FIR_Coarse_Switch_Coeff[XTZ];
			Machine.coarseStage.arrAxis[XTZ].sFIRY_Switch.FIR_iOrder = FIR_DY_Coarse_iOrder_Switch;
		break;
		case YTZ:
			Machine.coarseStage.arrAxis[YTZ].sFIRY_Switch.FIR_In = DY_FIR_Coarse_Switch_In_YTZ;
			Machine.coarseStage.arrAxis[YTZ].sFIRY_Switch.FIR_Out = DY_FIR_Coarse_Switch_Out[YTZ];
			Machine.coarseStage.arrAxis[YTZ].sFIRY_Switch.FIR_Coeff = DY_FIR_Coarse_Switch_Coeff[YTZ];
			Machine.coarseStage.arrAxis[YTZ].sFIRY_Switch.FIR_iOrder = FIR_DY_Coarse_iOrder_Switch;
		break;
		default:	break;
	}
}
/******************************************
函数名：SetFIRX_Coarse_Switch
返回值：无
参数表：int iAxisNum	四自由度轴号
功能说明：设置走DX方向轨迹时粗动台四自由度切换FIR前馈的参数
******************************************/
void SetFIRX_Coarse_Switch(int iAxis)
{
	switch(iAxis)
	{
		case CDX:
			Machine.coarseStage.arrAxis[CDX].sFIRX_Switch.FIR_In = DX_FIR_Coarse_Switch_In_DX;
			Machine.coarseStage.arrAxis[CDX].sFIRX_Switch.FIR_Out = DX_FIR_Coarse_Switch_Out[CDX];
			Machine.coarseStage.arrAxis[CDX].sFIRX_Switch.FIR_Coeff = DX_FIR_Coarse_Switch_Coeff[CDX];
			Machine.coarseStage.arrAxis[CDX].sFIRX_Switch.FIR_iOrder = FIR_DX_Coarse_iOrder_Switch;
		break;
		case CDY:
			Machine.coarseStage.arrAxis[CDY].sFIRX_Switch.FIR_In = DX_FIR_Coarse_Switch_In_DY;
			Machine.coarseStage.arrAxis[CDY].sFIRX_Switch.FIR_Out = DX_FIR_Coarse_Switch_Out[CDY];
			Machine.coarseStage.arrAxis[CDY].sFIRX_Switch.FIR_Coeff = DX_FIR_Coarse_Switch_Coeff[CDY];
			Machine.coarseStage.arrAxis[CDY].sFIRX_Switch.FIR_iOrder = FIR_DX_Coarse_iOrder_Switch;
		break;
		case XTZ:
			Machine.coarseStage.arrAxis[XTZ].sFIRX_Switch.FIR_In = DX_FIR_Coarse_Switch_In_XTZ;
			Machine.coarseStage.arrAxis[XTZ].sFIRX_Switch.FIR_Out = DX_FIR_Coarse_Switch_Out[XTZ];
			Machine.coarseStage.arrAxis[XTZ].sFIRX_Switch.FIR_Coeff = DX_FIR_Coarse_Switch_Coeff[XTZ];
			Machine.coarseStage.arrAxis[XTZ].sFIRX_Switch.FIR_iOrder = FIR_DX_Coarse_iOrder_Switch;
		break;
		case YTZ:
			Machine.coarseStage.arrAxis[YTZ].sFIRX_Switch.FIR_In = DX_FIR_Coarse_Switch_In_YTZ;
			Machine.coarseStage.arrAxis[YTZ].sFIRX_Switch.FIR_Out = DX_FIR_Coarse_Switch_Out[YTZ];
			Machine.coarseStage.arrAxis[YTZ].sFIRX_Switch.FIR_Coeff = DX_FIR_Coarse_Switch_Coeff[YTZ];
			Machine.coarseStage.arrAxis[YTZ].sFIRX_Switch.FIR_iOrder = FIR_DX_Coarse_iOrder_Switch;
		break;
		default:	break;
	}
}
/******************************************
函数名：SetPID_Fine_Counter
返回值：无
参数表：int iAxisNum	微动台三自由度轴号
功能说明：设置电涡流闭环下的微动台三自由度的PID参数
******************************************/
void SetPID_Fine_Counter(int iAxis)
{
	switch(iAxis)
	{
		case FDX:
			Machine.fineStage.arrAxis[FDX].sPID.Proportion = 0.000017;//0.000025;
			Machine.fineStage.arrAxis[FDX].sPID.Integral = 0.000096;//0.00003;//0.0;
			Machine.fineStage.arrAxis[FDX].sPID.Derivative = 0.000212;//0.0;
		break;
		case FDY:
			Machine.fineStage.arrAxis[FDY].sPID.Proportion = 0.000017;//0.000025;
			Machine.fineStage.arrAxis[FDY].sPID.Integral = 0.000096;//0.00003;//0.0;
			Machine.fineStage.arrAxis[FDY].sPID.Derivative = 0.000412;//0.0;
		break;
		case FTZ:
			Machine.fineStage.arrAxis[FTZ].sPID.Proportion = 0.000017;//0.000025;
			Machine.fineStage.arrAxis[FTZ].sPID.Integral = 0.00007;
			Machine.fineStage.arrAxis[FTZ].sPID.Derivative = 0.0002;//0.0;
		break;
		case FDZ:
			Machine.fineStage.arrAxis[FDZ].sPID.Proportion = 0.000026;
			Machine.fineStage.arrAxis[FDZ].sPID.Integral = 0.00005;
			Machine.fineStage.arrAxis[FDZ].sPID.Derivative = 0.0003;//0.0;
		break;
		case FTX:
			Machine.fineStage.arrAxis[FTX].sPID.Proportion = 0.000025;
			Machine.fineStage.arrAxis[FTX].sPID.Integral = 0.00006;
			Machine.fineStage.arrAxis[FTX].sPID.Derivative = 0.00015;
		break;
		case FTY:
			Machine.fineStage.arrAxis[FTY].sPID.Proportion = 0.00002;
			Machine.fineStage.arrAxis[FTY].sPID.Integral = 0.000035;
			Machine.fineStage.arrAxis[FTY].sPID.Derivative = 0.0002;	
		break;
	}
}

void SetLLC_Fine_Filter(int iAxis)
{
	switch(iAxis)
	{
		case FDX:
			Machine.fineStage.arrAxis[FDX].sFilter.pUout = FineFilter_UXout;
			Machine.fineStage.arrAxis[FDX].sFilter.pA = FineFilter_ax;
			Machine.fineStage.arrAxis[FDX].sFilter.pB = FineFilter_bx;
			Machine.fineStage.arrAxis[FDX].sFilter.pIn = FineFilter_Xin;
			Machine.fineStage.arrAxis[FDX].sFilter.order = FineFilter_order_x;
		break;
		case FDY:
			Machine.fineStage.arrAxis[FDY].sFilter.pUout = FineFilter_UYout;
			Machine.fineStage.arrAxis[FDY].sFilter.pA = FineFilter_ay;
			Machine.fineStage.arrAxis[FDY].sFilter.pB = FineFilter_by;
			Machine.fineStage.arrAxis[FDY].sFilter.pIn = FineFilter_Yin;
			Machine.fineStage.arrAxis[FDY].sFilter.order = FineFilter_order_y;
		break;
		case FTZ:
			Machine.fineStage.arrAxis[FTZ].sFilter.pUout = FineFilter_UTZout;
			Machine.fineStage.arrAxis[FTZ].sFilter.pA = FineFilter_atz;
			Machine.fineStage.arrAxis[FTZ].sFilter.pB = FineFilter_btz;
			Machine.fineStage.arrAxis[FTZ].sFilter.pIn = FineFilter_TZin;
			Machine.fineStage.arrAxis[FTZ].sFilter.order = FineFilter_order_tz;
			break;
		case FDZ:
			Machine.fineStage.arrAxis[FDZ].sFilter.pUout = FineFilter_UZout;
			Machine.fineStage.arrAxis[FDZ].sFilter.pA = FineFilter_az;
			Machine.fineStage.arrAxis[FDZ].sFilter.pB = FineFilter_bz;
			Machine.fineStage.arrAxis[FDZ].sFilter.pIn = FineFilter_Zin;
			Machine.fineStage.arrAxis[FDZ].sFilter.order = FineFilter_order_z;
			break;
		case FTX:
			Machine.fineStage.arrAxis[FTX].sFilter.pUout = FineFilter_UTXout;
			Machine.fineStage.arrAxis[FTX].sFilter.pA = FineFilter_atx;
			Machine.fineStage.arrAxis[FTX].sFilter.pB = FineFilter_btx;
			Machine.fineStage.arrAxis[FTX].sFilter.pIn = FineFilter_TXin;
			Machine.fineStage.arrAxis[FTX].sFilter.order = FineFilter_order_tx;
			break;
		case FTY:
			Machine.fineStage.arrAxis[FTY].sFilter.pUout = FineFilter_UTYout;
			Machine.fineStage.arrAxis[FTY].sFilter.pA = FineFilter_aty;
			Machine.fineStage.arrAxis[FTY].sFilter.pB = FineFilter_bty;
			Machine.fineStage.arrAxis[FTY].sFilter.pIn = FineFilter_TYin;
			Machine.fineStage.arrAxis[FTY].sFilter.order = FineFilter_order_ty;
			break;
	}
}

/******************************************
函数名：SetLLC_Fine_Counter
返回值：无
参数表：int iAxisNum	微动台三自由度轴号
功能说明：设置电涡流闭环下的微动台三自由度的超前滞后控制器参数
******************************************/
void SetLLC_Fine_Counter(int iAxis)
{
	switch(iAxis)
	{
		case FDX:
			Machine.fineStage.arrAxis[FDX].sLLC.pUout = Fine_UXout;
			Machine.fineStage.arrAxis[FDX].sLLC.pA = Fine_ax;
			Machine.fineStage.arrAxis[FDX].sLLC.pB = Fine_bx;
			Machine.fineStage.arrAxis[FDX].sLLC.pIn = Fine_Xin;
			Machine.fineStage.arrAxis[FDX].sLLC.order = Fine_order_x;
		break;
		case FDY:
			Machine.fineStage.arrAxis[FDY].sLLC.pUout = Fine_UYout;
			Machine.fineStage.arrAxis[FDY].sLLC.pA = Fine_ay;
			Machine.fineStage.arrAxis[FDY].sLLC.pB = Fine_by;
			Machine.fineStage.arrAxis[FDY].sLLC.pIn = Fine_Yin;
			Machine.fineStage.arrAxis[FDY].sLLC.order = Fine_order_y;
		break;
		case FTZ:
			Machine.fineStage.arrAxis[FTZ].sLLC.pUout = Fine_UTZout;
			Machine.fineStage.arrAxis[FTZ].sLLC.pA = Fine_atz;
			Machine.fineStage.arrAxis[FTZ].sLLC.pB = Fine_btz;
			Machine.fineStage.arrAxis[FTZ].sLLC.pIn = Fine_TZin;
			Machine.fineStage.arrAxis[FTZ].sLLC.order = Fine_order_tz;
			break;
		case FDZ:
			Machine.fineStage.arrAxis[FDZ].sLLC.pUout = Fine_UZout;
			Machine.fineStage.arrAxis[FDZ].sLLC.pA = Fine_az;
			Machine.fineStage.arrAxis[FDZ].sLLC.pB = Fine_bz;
			Machine.fineStage.arrAxis[FDZ].sLLC.pIn = Fine_Zin;
			Machine.fineStage.arrAxis[FDZ].sLLC.order = Fine_order_z;
			break;
		case FTX:
			Machine.fineStage.arrAxis[FTX].sLLC.pUout = Fine_UTXout;
			Machine.fineStage.arrAxis[FTX].sLLC.pA = Fine_atx;
			Machine.fineStage.arrAxis[FTX].sLLC.pB = Fine_btx;
			Machine.fineStage.arrAxis[FTX].sLLC.pIn = Fine_TXin;
			Machine.fineStage.arrAxis[FTX].sLLC.order = Fine_order_tx;
			break;
		case FTY:
			Machine.fineStage.arrAxis[FTY].sLLC.pUout = Fine_UTYout;
			Machine.fineStage.arrAxis[FTY].sLLC.pA = Fine_aty;
			Machine.fineStage.arrAxis[FTY].sLLC.pB = Fine_bty;
			Machine.fineStage.arrAxis[FTY].sLLC.pIn = Fine_TYin;
			Machine.fineStage.arrAxis[FTY].sLLC.order = Fine_order_ty;
		break;
	}
}
/******************************************
函数名：SetPID_Fine_Switch
返回值：无
参数表：int iAxisNum	微动台三自由度轴号
功能说明：设置切换后的微动台三自由度PID参数
******************************************/
void SetPID_Fine_Switch(int iAxis)
{
	switch(iAxis)
	{
		/*case FDX:
			Machine.fineStage.arrAxis[FDX].sPID_Switch.Proportion = 0.00015;//0.0001;
			Machine.fineStage.arrAxis[FDX].sPID_Switch.Integral = 0.002;//0.0;
			Machine.fineStage.arrAxis[FDX].sPID_Switch.Derivative = 0.001;//0.0005;
		break;
		case FDY:
			Machine.fineStage.arrAxis[FDY].sPID_Switch.Proportion = 0.00015;//0.0001;
			Machine.fineStage.arrAxis[FDY].sPID_Switch.Integral = 0.0025;//0.0;
			Machine.fineStage.arrAxis[FDY].sPID_Switch.Derivative = 0.001;//0.0005;
		break;
		case FTZ:
			Machine.fineStage.arrAxis[FTZ].sPID_Switch.Proportion = 0.0002;//0.0001;
			Machine.fineStage.arrAxis[FTZ].sPID_Switch.Integral = 0.0;//0.0;
			Machine.fineStage.arrAxis[FTZ].sPID_Switch.Derivative = 0.0005;//0.0005;
		break;*/

		case FDX:
			Machine.fineStage.arrAxis[FDX].sPID_Switch.Proportion = 0.00008;//0.0001;
			Machine.fineStage.arrAxis[FDX].sPID_Switch.Integral = 0.0;//0.002;
			Machine.fineStage.arrAxis[FDX].sPID_Switch.Derivative = 0.0005;//0.0005;
		break;
		case FDY:
			Machine.fineStage.arrAxis[FDY].sPID_Switch.Proportion = 0.00008;//0.0001;
			Machine.fineStage.arrAxis[FDY].sPID_Switch.Integral = 0.0;//0.002;
			Machine.fineStage.arrAxis[FDY].sPID_Switch.Derivative = 0.0005;//0.0005;
		break;
		case FTZ:
			Machine.fineStage.arrAxis[FTZ].sPID_Switch.Proportion = 0.0001;//0.0001;
			Machine.fineStage.arrAxis[FTZ].sPID_Switch.Integral = 0.0001;//0.0;
			Machine.fineStage.arrAxis[FTZ].sPID_Switch.Derivative = 0.0001;//0.0005;
			break;
		case FDZ:
			Machine.fineStage.arrAxis[FDZ].sPID_Switch.Proportion = 0.0001;//0.0001;
			Machine.fineStage.arrAxis[FDZ].sPID_Switch.Integral = 0.0;
			Machine.fineStage.arrAxis[FDZ].sPID_Switch.Derivative = 0.0;//0.0005;
			break;
		case FTX:
			Machine.fineStage.arrAxis[FTX].sPID_Switch.Proportion = 0.0001;//0.0001;
			Machine.fineStage.arrAxis[FTX].sPID_Switch.Integral = 0.0;
			Machine.fineStage.arrAxis[FTX].sPID_Switch.Derivative = 0.0;//0.0005;
			break;
		case FTY:
			Machine.fineStage.arrAxis[FTY].sPID_Switch.Proportion = 0.0001;//0.0001;
			Machine.fineStage.arrAxis[FTY].sPID_Switch.Integral = 0.0;
			Machine.fineStage.arrAxis[FTY].sPID_Switch.Derivative = 0.0;//0.0005;
		break;

		
	}
}
/******************************************
函数名：SetLLC_Fine_Switch
返回值：无
参数表：int iAxisNum	微动台三自由度轴号
功能说明：设置切换后的微动台三自由度超前滞后控制器参数
******************************************/
void SetLLC_Fine_Switch(int iAxis)
{
	switch(iAxis)
	{
		case FDX:
			Machine.fineStage.arrAxis[FDX].sLLC_Switch.pUout = Switch_Fine_UXout;
			Machine.fineStage.arrAxis[FDX].sLLC_Switch.pA = Switch_Fine_ax;
			Machine.fineStage.arrAxis[FDX].sLLC_Switch.pB = Switch_Fine_bx;
			Machine.fineStage.arrAxis[FDX].sLLC_Switch.pIn = Switch_Fine_Xin;
			Machine.fineStage.arrAxis[FDX].sLLC_Switch.order = Switch_Fine_order_x;
		break;
		case FDY:
			Machine.fineStage.arrAxis[FDY].sLLC_Switch.pUout = Switch_Fine_UYout;
			Machine.fineStage.arrAxis[FDY].sLLC_Switch.pA = Switch_Fine_ay;
			Machine.fineStage.arrAxis[FDY].sLLC_Switch.pB = Switch_Fine_by;
			Machine.fineStage.arrAxis[FDY].sLLC_Switch.pIn = Switch_Fine_Yin;
			Machine.fineStage.arrAxis[FDY].sLLC_Switch.order = Switch_Fine_order_y;
		break;
		case FTZ:
			Machine.fineStage.arrAxis[FTZ].sLLC_Switch.pUout = Switch_Fine_UTZout;
			Machine.fineStage.arrAxis[FTZ].sLLC_Switch.pA = Switch_Fine_atz;
			Machine.fineStage.arrAxis[FTZ].sLLC_Switch.pB = Switch_Fine_btz;
			Machine.fineStage.arrAxis[FTZ].sLLC_Switch.pIn = Switch_Fine_TZin;
			Machine.fineStage.arrAxis[FTZ].sLLC_Switch.order = Switch_Fine_order_tz;
		break;
		case FDZ:
			Machine.fineStage.arrAxis[FDZ].sLLC_Switch.pUout = Switch_Fine_UZout;
			Machine.fineStage.arrAxis[FDZ].sLLC_Switch.pA = Switch_Fine_az;
			Machine.fineStage.arrAxis[FDZ].sLLC_Switch.pB = Switch_Fine_bz;
			Machine.fineStage.arrAxis[FDZ].sLLC_Switch.pIn = Switch_Fine_Zin;
			Machine.fineStage.arrAxis[FDZ].sLLC_Switch.order = Switch_Fine_order_z;
		break;
		case FTX:
			Machine.fineStage.arrAxis[FTX].sLLC_Switch.pUout = Switch_Fine_UTXout;
			Machine.fineStage.arrAxis[FTX].sLLC_Switch.pA = Switch_Fine_atx;
			Machine.fineStage.arrAxis[FTX].sLLC_Switch.pB = Switch_Fine_btx;
			Machine.fineStage.arrAxis[FTX].sLLC_Switch.pIn = Switch_Fine_TXin;
			Machine.fineStage.arrAxis[FTX].sLLC_Switch.order = Switch_Fine_order_tx;
		break;
		case FTY:
			Machine.fineStage.arrAxis[FTY].sLLC_Switch.pUout = Switch_Fine_UTYout;
			Machine.fineStage.arrAxis[FTY].sLLC_Switch.pA = Switch_Fine_atx;
			Machine.fineStage.arrAxis[FTZ].sLLC_Switch.pB = Switch_Fine_btx;
			Machine.fineStage.arrAxis[FTY].sLLC_Switch.pIn = Switch_Fine_TYin;
			Machine.fineStage.arrAxis[FTY].sLLC_Switch.order = Switch_Fine_order_tx;
		break;
	}
}
/******************************************
函数名：SetFIRY_Fine
返回值：无
参数表：int iAxisNum	三自由度轴号
功能说明：设置走DY方向轨迹时微动台三自由度FIR前馈的参数
******************************************/
void SetFIRY_Fine(int iAxis)
{
	switch(iAxis)
	{
		case FDX:
			Machine.fineStage.arrAxis[FDX].sFIRY.FIR_In = DY_FIR_Fine_In_DX;
			Machine.fineStage.arrAxis[FDX].sFIRY.FIR_Out = DY_FIR_Fine_Switch_Out[FDX];
			Machine.fineStage.arrAxis[FDX].sFIRY.FIR_Coeff = DY_FIR_Fine_Coeff[FDX];
			Machine.fineStage.arrAxis[FDX].sFIRY.FIR_iOrder = FIR_DY_Fine_iOrder;
		break;
		case FDY:
			Machine.fineStage.arrAxis[FDY].sFIRY.FIR_In = DY_FIR_Fine_In_DY;
			Machine.fineStage.arrAxis[FDY].sFIRY.FIR_Out = DY_FIR_Fine_Switch_Out[FDY];
			Machine.fineStage.arrAxis[FDY].sFIRY.FIR_Coeff = DY_FIR_Fine_Coeff[FDY];
			Machine.fineStage.arrAxis[FDY].sFIRY.FIR_iOrder = FIR_DY_Fine_iOrder;
		break;
		case FTZ:
			Machine.fineStage.arrAxis[FTZ].sFIRY.FIR_In = DY_FIR_Fine_In_TZ;
			Machine.fineStage.arrAxis[FTZ].sFIRY.FIR_Out = DY_FIR_Fine_Switch_Out[FTZ];
			Machine.fineStage.arrAxis[FTZ].sFIRY.FIR_Coeff = DY_FIR_Fine_Coeff[FTZ];
			Machine.fineStage.arrAxis[FTZ].sFIRY.FIR_iOrder = FIR_DY_Fine_iOrder;
		break;
		case FDZ:
			Machine.fineStage.arrAxis[FDZ].sFIRY.FIR_In = DY_FIR_Fine_In_DZ;
			Machine.fineStage.arrAxis[FDZ].sFIRY.FIR_Out = DY_FIR_Fine_Switch_Out[FDZ];
			Machine.fineStage.arrAxis[FDZ].sFIRY.FIR_Coeff = DY_FIR_Fine_Coeff[FDZ];
			Machine.fineStage.arrAxis[FDZ].sFIRY.FIR_iOrder = FIR_DY_Fine_iOrder;
		break;
		case FTX:
			Machine.fineStage.arrAxis[FTX].sFIRY.FIR_In = DY_FIR_Fine_In_TX;
			Machine.fineStage.arrAxis[FTX].sFIRY.FIR_Out = DY_FIR_Fine_Switch_Out[FTX];
			Machine.fineStage.arrAxis[FTX].sFIRY.FIR_Coeff = DY_FIR_Fine_Coeff[FTX];
			Machine.fineStage.arrAxis[FTX].sFIRY.FIR_iOrder = FIR_DY_Fine_iOrder;
		break;
		case FTY:
			Machine.fineStage.arrAxis[FTY].sFIRY.FIR_In = DY_FIR_Fine_In_TY;
			Machine.fineStage.arrAxis[FTY].sFIRY.FIR_Out = DY_FIR_Fine_Switch_Out[FTY];
			Machine.fineStage.arrAxis[FTY].sFIRY.FIR_Coeff = DY_FIR_Fine_Coeff[FTY];
			Machine.fineStage.arrAxis[FTY].sFIRY.FIR_iOrder = FIR_DY_Fine_iOrder;
		break;
	}
}
/******************************************
函数名：SetFIRX_Coarse
返回值：无
参数表：int iAxisNum	四自由度轴号
功能说明：设置走DX方向轨迹时粗动台四自由度FIR前馈的参数
******************************************/
void SetFIRX_Fine(int iAxis)
{
	switch(iAxis)
	{
		case FDX:
			Machine.fineStage.arrAxis[FDX].sFIRX.FIR_In = DX_FIR_Fine_In_DX;
			Machine.fineStage.arrAxis[FDX].sFIRX.FIR_Out = DX_FIR_Fine_Out[FDX];
			Machine.fineStage.arrAxis[FDX].sFIRX.FIR_Coeff = DX_FIR_Fine_Coeff[FDX];
			Machine.fineStage.arrAxis[FDX].sFIRX.FIR_iOrder = FIR_DX_Fine_iOrder;
		break;
		case FDY:
			Machine.fineStage.arrAxis[FDY].sFIRX.FIR_In = DX_FIR_Fine_In_DY;
			Machine.fineStage.arrAxis[FDY].sFIRX.FIR_Out = DX_FIR_Fine_Out[FDY];
			Machine.fineStage.arrAxis[FDY].sFIRX.FIR_Coeff = DX_FIR_Fine_Coeff[FDY];
			Machine.fineStage.arrAxis[FDY].sFIRX.FIR_iOrder = FIR_DX_Fine_iOrder;
		break;
		case FTZ:
			Machine.fineStage.arrAxis[FTZ].sFIRX.FIR_In = DX_FIR_Fine_In_TZ;
			Machine.fineStage.arrAxis[FTZ].sFIRX.FIR_Out = DX_FIR_Fine_Out[FTZ];
			Machine.fineStage.arrAxis[FTZ].sFIRX.FIR_Coeff = DX_FIR_Fine_Coeff[FTZ];
			Machine.fineStage.arrAxis[FTZ].sFIRX.FIR_iOrder = FIR_DX_Fine_iOrder;
		break;
		case FDZ:
			Machine.fineStage.arrAxis[FDZ].sFIRX.FIR_In = DX_FIR_Fine_In_DZ;
			Machine.fineStage.arrAxis[FDZ].sFIRX.FIR_Out = DX_FIR_Fine_Out[FDZ];
			Machine.fineStage.arrAxis[FDZ].sFIRX.FIR_Coeff = DX_FIR_Fine_Coeff[FDZ];
			Machine.fineStage.arrAxis[FDZ].sFIRX.FIR_iOrder = FIR_DX_Fine_iOrder;
		break;
		case FTX:
			Machine.fineStage.arrAxis[FTX].sFIRX.FIR_In = DX_FIR_Fine_In_TX;
			Machine.fineStage.arrAxis[FTX].sFIRX.FIR_Out = DX_FIR_Fine_Out[FTX];
			Machine.fineStage.arrAxis[FTX].sFIRX.FIR_Coeff = DX_FIR_Fine_Coeff[FTX];
			Machine.fineStage.arrAxis[FTX].sFIRX.FIR_iOrder = FIR_DX_Fine_iOrder;
		break;
		case FTY:
			Machine.fineStage.arrAxis[FTY].sFIRX.FIR_In = DX_FIR_Fine_In_TY;
			Machine.fineStage.arrAxis[FTY].sFIRX.FIR_Out = DX_FIR_Fine_Out[FTY];
			Machine.fineStage.arrAxis[FTY].sFIRX.FIR_Coeff = DX_FIR_Fine_Coeff[FTY];
			Machine.fineStage.arrAxis[FTY].sFIRX.FIR_iOrder = FIR_DX_Fine_iOrder;
		break;
		
	}
}
/******************************************
函数名：SetFIRY_Fine_Switch
返回值：无
参数表：int iAxisNum	三自由度轴号
功能说明：设置走DY方向轨迹时微动台三自由度切换FIR前馈的参数
******************************************/
void SetFIRY_Fine_Switch(int iAxis)
{
	switch(iAxis)
	{
		case FDX:
			Machine.fineStage.arrAxis[FDX].sFIRY_Switch.FIR_In = DY_FIR_Fine_Switch_In_DX;
			Machine.fineStage.arrAxis[FDX].sFIRY_Switch.FIR_Out = DY_FIR_Fine_Switch_Out[FDX];
			Machine.fineStage.arrAxis[FDX].sFIRY_Switch.FIR_Coeff = DY_FIR_Fine_Switch_Coeff[FDX];
			Machine.fineStage.arrAxis[FDX].sFIRY_Switch.FIR_iOrder = FIR_DY_Fine_iOrder_Switch;
		break;
		case FDY:
			Machine.fineStage.arrAxis[FDY].sFIRY_Switch.FIR_In = DY_FIR_Fine_Switch_In_DY;
			Machine.fineStage.arrAxis[FDY].sFIRY_Switch.FIR_Out = DY_FIR_Fine_Switch_Out[FDY];
			Machine.fineStage.arrAxis[FDY].sFIRY_Switch.FIR_Coeff = DY_FIR_Fine_Switch_Coeff[FDY];
			Machine.fineStage.arrAxis[FDY].sFIRY_Switch.FIR_iOrder = FIR_DY_Fine_iOrder_Switch;
		break;
		case FTZ:
			Machine.fineStage.arrAxis[FTZ].sFIRY_Switch.FIR_In = DY_FIR_Fine_Switch_In_TZ;
			Machine.fineStage.arrAxis[FTZ].sFIRY_Switch.FIR_Out = DY_FIR_Fine_Switch_Out[FTZ];
			Machine.fineStage.arrAxis[FTZ].sFIRY_Switch.FIR_Coeff = DY_FIR_Fine_Switch_Coeff[FTZ];
			Machine.fineStage.arrAxis[FTZ].sFIRY_Switch.FIR_iOrder = FIR_DY_Fine_iOrder_Switch;
		break;
		case FDZ:
			Machine.fineStage.arrAxis[FDZ].sFIRY_Switch.FIR_In = DY_FIR_Fine_Switch_In_DZ;
			Machine.fineStage.arrAxis[FDZ].sFIRY_Switch.FIR_Out = DY_FIR_Fine_Switch_Out[FDZ];
			Machine.fineStage.arrAxis[FDZ].sFIRY_Switch.FIR_Coeff = DY_FIR_Fine_Switch_Coeff[FDZ];
			Machine.fineStage.arrAxis[FDZ].sFIRY_Switch.FIR_iOrder = FIR_DY_Fine_iOrder_Switch;
		break;
		case FTX:
			Machine.fineStage.arrAxis[FTX].sFIRY_Switch.FIR_In = DY_FIR_Fine_Switch_In_TX;
			Machine.fineStage.arrAxis[FTX].sFIRY_Switch.FIR_Out = DY_FIR_Fine_Switch_Out[FTX];
			Machine.fineStage.arrAxis[FTX].sFIRY_Switch.FIR_Coeff = DY_FIR_Fine_Switch_Coeff[FTX];
			Machine.fineStage.arrAxis[FTX].sFIRY_Switch.FIR_iOrder = FIR_DY_Fine_iOrder_Switch;
		break;
		case FTY:
			Machine.fineStage.arrAxis[FTY].sFIRY_Switch.FIR_In = DY_FIR_Fine_Switch_In_TY;
			Machine.fineStage.arrAxis[FTY].sFIRY_Switch.FIR_Out = DY_FIR_Fine_Switch_Out[FTY];
			Machine.fineStage.arrAxis[FTY].sFIRY_Switch.FIR_Coeff = DY_FIR_Fine_Switch_Coeff[FTY];
			Machine.fineStage.arrAxis[FTY].sFIRY_Switch.FIR_iOrder = FIR_DY_Fine_iOrder_Switch;
		break;
	}
}
/******************************************
函数名：SetFIRX_Fine_Switch
返回值：无
参数表：int iAxisNum	三自由度轴号
功能说明：设置走DX方向轨迹时微动台三自由度切换FIR前馈的参数
******************************************/
void SetFIRX_Fine_Switch(int iAxis)
{
	switch(iAxis)
	{
		case FDX:
			Machine.fineStage.arrAxis[FDX].sFIRX_Switch.FIR_In = DX_FIR_Fine_Switch_In_DX;
			Machine.fineStage.arrAxis[FDX].sFIRX_Switch.FIR_Out = DX_FIR_Fine_Switch_Out[FDX];
			Machine.fineStage.arrAxis[FDX].sFIRX_Switch.FIR_Coeff = DX_FIR_Fine_Switch_Coeff[FDX];
			Machine.fineStage.arrAxis[FDX].sFIRX_Switch.FIR_iOrder = FIR_DX_Fine_iOrder_Switch;
		break;
		case FDY:
			Machine.fineStage.arrAxis[FDY].sFIRX_Switch.FIR_In = DX_FIR_Fine_Switch_In_DY;
			Machine.fineStage.arrAxis[FDY].sFIRX_Switch.FIR_Out = DX_FIR_Fine_Switch_Out[FDY];
			Machine.fineStage.arrAxis[FDY].sFIRX_Switch.FIR_Coeff = DX_FIR_Fine_Switch_Coeff[FDY];
			Machine.fineStage.arrAxis[FDY].sFIRX_Switch.FIR_iOrder = FIR_DX_Fine_iOrder_Switch;
		break;
		case FTZ:
			Machine.fineStage.arrAxis[FTZ].sFIRX_Switch.FIR_In = DX_FIR_Fine_Switch_In_TZ;
			Machine.fineStage.arrAxis[FTZ].sFIRX_Switch.FIR_Out = DX_FIR_Fine_Switch_Out[FTZ];
			Machine.fineStage.arrAxis[FTZ].sFIRX_Switch.FIR_Coeff = DX_FIR_Fine_Switch_Coeff[FTZ];
			Machine.fineStage.arrAxis[FTZ].sFIRX_Switch.FIR_iOrder = FIR_DX_Fine_iOrder_Switch;
		break;
		case FDZ:
			Machine.fineStage.arrAxis[FDZ].sFIRX_Switch.FIR_In = DX_FIR_Fine_Switch_In_DZ;
			Machine.fineStage.arrAxis[FDZ].sFIRX_Switch.FIR_Out = DX_FIR_Fine_Switch_Out[FDZ];
			Machine.fineStage.arrAxis[FDZ].sFIRX_Switch.FIR_Coeff = DX_FIR_Fine_Switch_Coeff[FDZ];
			Machine.fineStage.arrAxis[FDZ].sFIRX_Switch.FIR_iOrder = FIR_DX_Fine_iOrder_Switch;
		break;
		case FTX:
			Machine.fineStage.arrAxis[FTX].sFIRX_Switch.FIR_In = DX_FIR_Fine_Switch_In_TX;
			Machine.fineStage.arrAxis[FTX].sFIRX_Switch.FIR_Out = DX_FIR_Fine_Switch_Out[FTX];
			Machine.fineStage.arrAxis[FTX].sFIRX_Switch.FIR_Coeff = DX_FIR_Fine_Switch_Coeff[FTX];
			Machine.fineStage.arrAxis[FTX].sFIRX_Switch.FIR_iOrder = FIR_DX_Fine_iOrder_Switch;
		break;
		case FTY:
			Machine.fineStage.arrAxis[FTY].sFIRX_Switch.FIR_In = DX_FIR_Fine_Switch_In_TY;
			Machine.fineStage.arrAxis[FTY].sFIRX_Switch.FIR_Out = DX_FIR_Fine_Switch_Out[FTY];
			Machine.fineStage.arrAxis[FTY].sFIRX_Switch.FIR_Coeff = DX_FIR_Fine_Switch_Coeff[FTY];
			Machine.fineStage.arrAxis[FTY].sFIRX_Switch.FIR_iOrder = FIR_DX_Fine_iOrder_Switch;
		break;
	}
}

/******************************************
函数名：SetAFC
返回值：无
参数表：int iAxisNum	三自由度轴号
功能说明：设置激光干涉仪闭环下的微动台三自由度测量侧的AFC控制器参数
******************************************/
void SetAFC(int iAxis,int i)
{
	switch(iAxis)
	{
		case FDX:
			Machine.fineStage.arrAxis[FDX].sAFC[i].pin = DX_Fine_x_in[i];
			Machine.fineStage.arrAxis[FDX].sAFC[i].pout = DX_Fine_y_out[i];
			Machine.fineStage.arrAxis[FDX].sAFC[i].pA = DX_Fine_a[i];
			Machine.fineStage.arrAxis[FDX].sAFC[i].pB = DX_Fine_b[i];
		break;
		case FDY:
			Machine.fineStage.arrAxis[FDY].sAFC[i].pin = DY_Fine_x_in[i];
			Machine.fineStage.arrAxis[FDY].sAFC[i].pout = DY_Fine_y_out[i];
			Machine.fineStage.arrAxis[FDY].sAFC[i].pA = DY_Fine_a[i];
			Machine.fineStage.arrAxis[FDY].sAFC[i].pB = DY_Fine_b[i];
		break;
		case FTZ:
			Machine.fineStage.arrAxis[FTZ].sAFC[i].pin = TZ_Fine_x_in[i];
			Machine.fineStage.arrAxis[FTZ].sAFC[i].pout = TZ_Fine_y_out[i];
			Machine.fineStage.arrAxis[FTZ].sAFC[i].pA = TZ_Fine_a[i];
			Machine.fineStage.arrAxis[FTZ].sAFC[i].pB = TZ_Fine_b[i];
		break;
		case FDZ:
			Machine.fineStage.arrAxis[FDZ].sAFC[i].pin = DZ_Fine_x_in[i];
			Machine.fineStage.arrAxis[FDZ].sAFC[i].pout = DZ_Fine_y_out[i];
			Machine.fineStage.arrAxis[FDZ].sAFC[i].pA = DZ_Fine_a[i];
			Machine.fineStage.arrAxis[FDZ].sAFC[i].pB = DZ_Fine_b[i];
		break;
		case FTX:
			Machine.fineStage.arrAxis[FTX].sAFC[i].pin = TX_Fine_x_in[i];
			Machine.fineStage.arrAxis[FTX].sAFC[i].pout = TX_Fine_y_out[i];
			Machine.fineStage.arrAxis[FTX].sAFC[i].pA = TX_Fine_a[i];
			Machine.fineStage.arrAxis[FTX].sAFC[i].pB = TX_Fine_b[i];
		break;
		case FTY:
			Machine.fineStage.arrAxis[FTY].sAFC[i].pin = TY_Fine_x_in[i];
			Machine.fineStage.arrAxis[FTY].sAFC[i].pout = TY_Fine_y_out[i];
			Machine.fineStage.arrAxis[FTY].sAFC[i].pA = TY_Fine_a[i];
			Machine.fineStage.arrAxis[FTY].sAFC[i].pB = TY_Fine_b[i];
		break;
	}
}
/******************************************
函数名：SetAFC_Switch
返回值：无
参数表：int iAxisNum	三自由度轴号
功能说明：设置激光干涉仪闭环下的微动台三自由度测量侧的AFC控制器参数
******************************************/
void SetAFC_Switch(int iAxis,int i)
{
	switch(iAxis)
	{
		case FDX:
			Machine.fineStage.arrAxis[FDX].sAFC_Switch[i].pin = DX_Fine_Switch_x_in[i];
			Machine.fineStage.arrAxis[FDX].sAFC_Switch[i].pout = DX_Fine_Switch_y_out[i];
			Machine.fineStage.arrAxis[FDX].sAFC_Switch[i].pA = DX_Fine_Switch_a[i];
			Machine.fineStage.arrAxis[FDX].sAFC_Switch[i].pB = DX_Fine_Switch_b[i];
		break;
		case FDY:
			Machine.fineStage.arrAxis[FDY].sAFC_Switch[i].pin = DY_Fine_Switch_x_in[i];
			Machine.fineStage.arrAxis[FDY].sAFC_Switch[i].pout = DY_Fine_Switch_y_out[i];
			Machine.fineStage.arrAxis[FDY].sAFC_Switch[i].pA = DY_Fine_Switch_a[i];
			Machine.fineStage.arrAxis[FDY].sAFC_Switch[i].pB = DY_Fine_Switch_b[i];
		break;
		case FTZ:
			Machine.fineStage.arrAxis[FTZ].sAFC_Switch[i].pin = TZ_Fine_Switch_x_in[i];
			Machine.fineStage.arrAxis[FTZ].sAFC_Switch[i].pout = TZ_Fine_Switch_y_out[i];
			Machine.fineStage.arrAxis[FTZ].sAFC_Switch[i].pA = TZ_Fine_Switch_a[i];
			Machine.fineStage.arrAxis[FTZ].sAFC_Switch[i].pB = TZ_Fine_Switch_b[i];
		break;
		case FDZ:
			Machine.fineStage.arrAxis[FDZ].sAFC_Switch[i].pin = DZ_Fine_Switch_x_in[i];
			Machine.fineStage.arrAxis[FDZ].sAFC_Switch[i].pout = DZ_Fine_Switch_y_out[i];
			Machine.fineStage.arrAxis[FDZ].sAFC_Switch[i].pA = DZ_Fine_Switch_a[i];
			Machine.fineStage.arrAxis[FDZ].sAFC_Switch[i].pB = DZ_Fine_Switch_b[i];
		break;
		case FTX:
			Machine.fineStage.arrAxis[FTX].sAFC_Switch[i].pin = TX_Fine_Switch_x_in[i];
			Machine.fineStage.arrAxis[FTX].sAFC_Switch[i].pout = TX_Fine_Switch_y_out[i];
			Machine.fineStage.arrAxis[FTX].sAFC_Switch[i].pA = TX_Fine_Switch_a[i];
			Machine.fineStage.arrAxis[FTX].sAFC_Switch[i].pB = TX_Fine_Switch_b[i];
		break;
		case FTY:
			Machine.fineStage.arrAxis[FTY].sAFC_Switch[i].pin = TY_Fine_Switch_x_in[i];
			Machine.fineStage.arrAxis[FTY].sAFC_Switch[i].pout = TY_Fine_Switch_y_out[i];
			Machine.fineStage.arrAxis[FTY].sAFC_Switch[i].pA = TY_Fine_Switch_a[i];
			Machine.fineStage.arrAxis[FTY].sAFC_Switch[i].pB = TY_Fine_Switch_b[i];
		break;
	}
}

void InitCoarseConParam()
{
	int i = 0;
	for(i=0;i<4;i++)
	{
		SetPID_Coarse_Counter(i);
		SetLLC_Coarse_Counter(i);
		SetPID_Coarse_Switch(i);
		SetLLC_Coarse_Switch(i);
		SetFIRY_Coarse(i);
		SetFIRX_Coarse(i);
		SetFIRY_Coarse_Switch(i);
		SetFIRX_Coarse_Switch(i);
	}
}

void InitFineConParam()
{
	int i = 0;
	int j = 0;
	int k = 0;
	for(i=0;i<6;i++)
	{
		SetPID_Fine_Counter(i);
		SetLLC_Fine_Counter(i);
		SetLLC_Fine_Filter(i);
		SetPID_Fine_Switch(i);
		SetLLC_Fine_Switch(i);
		SetFIRY_Fine(i);
		SetFIRX_Fine(i);
		SetFIRY_Fine_Switch(i);
		SetFIRX_Fine_Switch(i);
	}
	for(j=0;j<6;j++)
	{
		for(k=0;k<4;k++)
		{
			SetAFC(j,k);
			SetAFC_Switch(j,k);
		}
	}
}



void InitMachine()
{
	InitBuff();
	InitBoard();
	InitStage();
	Machine.simulationFlag =0;
}
int InitBoard()
{
	if(InitNet())//需要网络初始化
	{
		printf("Net init error\n");
		return;
	}
	InitCounter();
	InitAIO();
	InitLaser();
	irqConnect();
	N1225A_Init();
}

void InitStage()
{
	InitCoarseStage();
	InitFineStage();
	InitTrajParam();
}

void InitCoarseStage()
{
	int i = 0;
	for(i=0;i<4;i++)
	{
		Machine.coarseStage.DA_Struct[i].iOffset = CoarseOffset[i];
		Machine.coarseStage.DA_Struct[i].iLimit = FineLimit[i];
	}
	Machine.coarseStage.Traj = Traj_Coarse;

	Machine.coarseStage.DDX_Flag=0;
	InitStageStatus(&Machine.coarseStage,CoarseStage);

	InitCoarseAxis(Machine.coarseStage.arrAxis,COARSE_AXIS_NUM);
}
void InitFineStage()
{
	int i = 0;
	for(i=0;i<6;i++)
	{
		Machine.fineStage.DA_Struct[i].iOffset = FineOffset[i];
		Machine.fineStage.DA_Struct[i].iLimit = FineLimit[i];
	}

	Machine.fineStage.Traj = Traj_Fine;

	InitStageStatus(&Machine.fineStage,FineStage);
	InitFineAxis(Machine.fineStage.arrAxis,FINE_AXIS_NUM);
}


void InitCoarseAxis(Axis *pAxis,int iAxisNum)
{
	int i = 0;
	for(i=0;i<iAxisNum;i++)
	{
		(pAxis+i)->controllerType = (ControllerType)Coarse_CounterPIDType[i];
		(pAxis+i)->LaserControllerType = (ControllerType)Coarse_SwitchPIDType[i];
	}

	
	(pAxis+0)->OutCalc = Concalc_Coarse_DX;
	(pAxis+1)->OutCalc = Concalc_Coarse_DY;
	(pAxis+2)->OutCalc = Concalc_Coarse_XTZ;
	(pAxis+3)->OutCalc = Concalc_Coarse_YTZ;

	

	(pAxis+0)->TrajSine_Poly = TrajSine_Poly_Coarse;
	(pAxis+1)->TrajSine_Poly = TrajSine_Poly_Coarse;


	(pAxis+0)->TrajGen4Order = TrajGen4OrderX1;
	(pAxis+1)->TrajGen4Order = TrajGen4OrderX2;
	(pAxis+2)->TrajGen4Order = TrajGen4OrderY1;
	(pAxis+3)->TrajGen4Order = TrajGen4OrderY2;

	(pAxis+0)->dFeedForwardCoef = 0;//0.000001;//加速度前馈系数
	(pAxis+1)->dFeedForwardCoef = 0;//0.000001;
	(pAxis+2)->dFeedForwardCoef = 0;//0.000001;
	(pAxis+3)->dFeedForwardCoef = 0;//0.000001;

	(pAxis+0)->dStepDis = 100;
	(pAxis+1)->dStepDis = 100;
	(pAxis+2)->dStepDis = 100;
	(pAxis+3)->dStepDis = 100;

	pAxis[0].dJogSpd = 1000;//100单轴不行
	pAxis[1].dJogSpd = 1000;
	pAxis[2].dJogSpd = 1000;
	pAxis[3].dJogSpd = 1000;

	pAxis[0].dIdentGain =1e-8;// 0.3;
	pAxis[1].dIdentGain =1e-8;// 0.3;
	pAxis[2].dIdentGain = 1e-8;//0.03;

	pAxis[3].dIdentGain =1e-8;// 0.3;
	pAxis[4].dIdentGain =1e-8;// 0.3;
	pAxis[5].dIdentGain = 1e-8;//0.03;

	//Axis Common Status
	InitAxisStatus(Machine.coarseStage.arrAxis,COARSE_AXIS_NUM);	
	InitCoarseConParam();
}

void InitFineAxis(Axis *pAxis,int iAxisNum)
{
	int i = 0;
	for(i=0;i<iAxisNum;i++)
	{
		(pAxis+i)->controllerType = (ControllerType)Fine_CounterPIDType[i];
		(pAxis+i)->LaserControllerType = (ControllerType)Fine_SwitchPIDType[i];
	}
	(pAxis+0)->OutCalc = Concalc_Fine_DX;
	(pAxis+1)->OutCalc = Concalc_Fine_DY;
	(pAxis+2)->OutCalc = Concalc_Fine_TZ;

	//YHT
	(pAxis+3)->OutCalc = Concalc_Fine_DZ;
	(pAxis+4)->OutCalc = Concalc_Fine_TX;
	(pAxis+5)->OutCalc = Concalc_Fine_TY;
	

	(pAxis+0)->TrajSine_Poly = TrajSine_Poly_Fine;
	(pAxis+1)->TrajSine_Poly = TrajSine_Poly_Fine;
	(pAxis+2)->TrajSine_Poly = TrajSine_Poly_Fine;
	//YHT
	(pAxis+3)->TrajSine_Poly = TrajSine_Poly_Fine;
	(pAxis+4)->TrajSine_Poly = TrajSine_Poly_Fine;
	(pAxis+5)->TrajSine_Poly = TrajSine_Poly_Fine;
	
	(pAxis+0)->TrajGen4Order = TrajGen4OrderFineX;
	(pAxis+1)->TrajGen4Order = TrajGen4OrderFineY;
	(pAxis+2)->TrajGen4Order = TrajGen4OrderFineTz;
	
	

	(pAxis+0)->dFeedForwardCoef = 0.000001;
	(pAxis+1)->dFeedForwardCoef = 0.000001;
	(pAxis+2)->dFeedForwardCoef = 0.000001;
     //YHT
	(pAxis+3)->dFeedForwardCoef = 0.000001;
	(pAxis+4)->dFeedForwardCoef = 0.000001;
	(pAxis+5)->dFeedForwardCoef = 0.000001;

	(pAxis+0)->dStepDis = 100;
	(pAxis+1)->dStepDis = 100;
	(pAxis+2)->dStepDis = 100;
	//YHT
	(pAxis+3)->dStepDis = 100;
	(pAxis+4)->dStepDis = 100;
	(pAxis+5)->dStepDis = 100;

	(pAxis+0)->dJogSpd = 50;
	(pAxis+1)->dJogSpd = 50;
	(pAxis+2)->dJogSpd = 50;
	//YHT
	(pAxis+3)->dJogSpd = 50;
	(pAxis+4)->dJogSpd = 50;
	(pAxis+5)->dJogSpd = 50;


	pAxis[0].dIdentGain =1e-9;// 0.3;
	pAxis[1].dIdentGain =1e-9;// 0.3;
	pAxis[2].dIdentGain = 1e-9;//0.03;

	pAxis[3].dIdentGain =1e-9;// 0.3;
	pAxis[4].dIdentGain =1e-9;// 0.3;
	pAxis[5].dIdentGain = 1e-9;//0.03;

	pAxis[0].iDir = 1;
	pAxis[1].iDir = 1;
	pAxis[2].iDir = 1;
	pAxis[3].iDir = 1;
	pAxis[4].iDir = 1;
	pAxis[5].iDir = 1;
	
	pAxis[0].iLaserDir = 1;
	pAxis[1].iLaserDir = 1;
	pAxis[2].iLaserDir = 1;
	pAxis[3].iLaserDir = 1;
	pAxis[4].iLaserDir = 1;
	pAxis[5].iLaserDir = 1;
	
	//Axis Common Status
	InitAxisStatus(Machine.fineStage.arrAxis,FINE_AXIS_NUM);
	InitFineConParam();	
}

void InitAxisStatus(Axis *pAxis,int iAxisNum)
{
	int i = 0;
	for(i=0;i<iAxisNum;i++)
	{
		(pAxis+i)->axisCMD = C_Aixs_Open;
		(pAxis+i)->axisStatus = S_Aixs_Open;
		(pAxis+i)->closeStatus = S_Counter_Close;
		(pAxis+i)->trajType = isTraj4OrderPoly;//isTrajSineAccT;
		(pAxis+i)->trajMode = IsTrajIndependent;
	}
}

void InitStageStatus(Stage *pStage,int iStageType)
{
	int i = 0;
	pStage->stageCMD = C_All_Nothing;
	pStage->stageStatus = S_Stage_ALL_Open;
	pStage->stageType = (StageType)iStageType;
}
