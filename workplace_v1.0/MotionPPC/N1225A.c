#include "N1225A.h"

/*
 *	void readMAC(uint8_t BoadID,long *MAC)
 *	@brief ��ȡ������MAC��ַ��һ��6���ֽڣ����з�ʽΪMAC[5]:MAC[4]:MAC[3]:
 *			MAC[2]:MAC[1]:MAC[0];
 *	@param BoadID �������ţ�1-��;
 *	@param *MAC �������ض����ĵ�ַ
 *	@retval None
 */
void readMAC(unsigned char BoadID,long long* MAC)
{
	unsigned int BoadAddr = CalBoard_Addr(BoadID);

	unsigned short MAC_H, MAC_M, MAC_L;
	MAC_H = *((unsigned int*)(BoadAddr + MACReg_H));
	MAC_M = *((unsigned int*)(BoadAddr + MACReg_M));
	MAC_L = *((unsigned int*)(BoadAddr + MACReg_L));
	printf("mac = %x,mac = %x,mac = %x,%d\r\n",MAC_H,MAC_M,MAC_L,sizeof(MAC_H));

	
/*	
	unsigned int MAC_MSW, MAC_LSW;
//	long long mac;
	MAC_MSW = *((unsigned int *)(BoadAddr + MACReg_MSW));
	MAC_LSW = *((unsigned int *)(BoadAddr + MACReg_MSW));
	MAC_MSW = Swap_32(MAC_MSW);
	MAC_LSW = Swap_32(MAC_LSW);
//	mac = MAC_MSW;  
//	logMsg("mac=%x,%x\n",MAC_MSW,MAC_LSW,0,0,0,0);
	printf("mac_h = %x,mac_l=%x,%d\r\n",MAC_MSW,MAC_LSW,sizeof(MAC_MSW));
//	*MAC = (mac << 32) | (MAC_LSW << 0);
*/
}

/*
 * void communicationTest(uint8_t BoadID)
 * @brief �������԰忨�͵��������Ƿ�ͨ������������������1״̬�ƿ�ʼ��˸��һ��ʱ���ָ���ɫ����
 * @param BoadID �������ţ�1-��;
 * @retval None
 */
void communicationTest(unsigned char BoadID)
{
	unsigned int Addr = N1225A_VME_BASE_ADDR + (BoadID - 1) * N1225A_Board_ADDR_INCE;
	unsigned int i = 0;
	*((unsigned int *)(Addr + ControlStatusReg_Axis1)) = 0x0080;
	for ( i = 0; i < 10000; i++);
	*((unsigned int *)(Addr + ControlStatusReg_Axis1)) = 0x0000;
}

/*
 * unsigned int ReadReg(unsigned int Reg, unsigned char BoadID)
 * @brief ��ȡ�Ĵ�����ֵ
 * @param BoadID �������ţ�1-��;
 * @param Reg �Ĵ�����ַ��
 * @retval val ���ص�ǰ�Ĵ�����ֵ
 */
unsigned int ReadReg(unsigned int Reg)//, unsigned char BoadID)
{
	unsigned int Val;
	Val = *((unsigned int*)Reg);
	return Val;
}

/*
 * void WriteReg(unsigned int Reg, unsigned char BoadID, unsigned short Val)
 * @brief д��Ĵ���
 * @param BoadID �������ţ�1-��;
 * @param Reg �Ĵ�����ַ��
 * @retval None
 */

void WriteReg(unsigned int Reg, unsigned int Val)
{
//	unsigned int RegAddr = CalBoard_Addr(BoardID) + Reg;
	*((unsigned int*)Reg) = Val;
}
/*
 * void SetRegBit(unsigned char BoardID, unsigned int Reg,unsigned char index)
 * @brief ���Ĵ����е�ĳһλ��λ
 * @param BoadID �������ţ�1-��;
 * @param Reg �Ĵ�����ַ��
 * @param index ��Ҫ��λ��bit
 * @retval None
 */
void SetRegBit(unsigned char BoardID, unsigned int Reg,unsigned char index)
{
	unsigned int RegAddr = CalBoard_Addr(BoardID) + Reg;
	unsigned int temp;
	temp = *((unsigned int*)RegAddr);
	temp |= (0x01 << index);
	*((unsigned int*)RegAddr) = temp;
}

/*
 * void ClearRegBit(unsigned char BoardID, unsigned int Reg, unsigned char index)
 * @brief ���Ĵ����е�ĳһλ��0
 * @param BoadID �������ţ�1-��;
 * @param Reg �Ĵ�����ַ��
 * @param index ��Ҫ��0��bit
 * @retval None
 */
void ClearRegBit(unsigned char BoardID, unsigned int Reg, unsigned char index)
{
	unsigned int RegAddr = CalBoard_Addr(BoardID) + Reg;
	unsigned int temp;
	temp = *((unsigned int*)RegAddr);
	temp &= ~(0x01 << index);
	*((unsigned int*)RegAddr) = temp;
}
/*
 * unsigned int AutoReadPos(unsigned char BoardID, unsigned char Axis,unsigned char Pos)
 * @brief �Զ���ȡ���λ�üĴ���
 * @param BoadID �������ţ�1-��;
 * @param Axis ������ 1-4��
 * @param Pos λ�üĴ��� 1-6
 * @retval Position ���ص�ǰλ�üĴ�����ֵ
 */
long long AutoReadPos(unsigned char BoardID, unsigned char Axis,unsigned char Pos)
{
	unsigned int PosAddr = CalBoardAxisPosition_Addr(BoardID, Axis, Pos) + AutoSamplingPosReg;
	unsigned int ExtPosAddr = CalBoardAxisPosition_Addr(BoardID, Axis, Pos) + AutoSamplingPosExtReg;
	long long pos_value = 0;
	pos_value = *((unsigned int*)ExtPosAddr);
	pos_value = (long long)(pos_value << 32) + *((unsigned int*)PosAddr) ;
	return pos_value;
}


/*
 * void BoardReset(unsigned char BoardID)
 * @brief ͬʱ��λһ����ϵ��������λ�û������ż�����װ�Ĵ�����ֵ
 * @param BoadID �������ţ�1-��;
 * @retval None
 */
void BoardReset(unsigned char BoardID)
{
	unsigned int RegAddr = CalBoard_Addr(BoardID) + CommandReg;	
	*((unsigned int*)RegAddr) |= (0x01 << 14);
}

/*
 * void AxisReset(unsigned int BoardID, unsigned char Axis)
 * @brief �����һ����ƺ�״̬�Ĵ����е�Ԥ������λΪ0�����ḴλΪ0��
 *        ���Ԥ�������λΪ1������λ������Ϊ��λ��ƫ�ƼĴ����е�ֵ��
 * @param BoadID �������ţ�1-��;
 * @retval None
 */
void AxisReset(unsigned int BoardID, unsigned char Axis)
{
	unsigned int RegAddr = (Axis - 1) * N1225A_Axis_ADDR_INCE + CommandReg;
//	*((unsigned int*)RegAddr) |= (0x01<<8);
	SetRegBit(BoardID, RegAddr, 8);
}

/*
 * void N1225A_Init(unsigned int nBoard)
 * @brief ��ʼ��
 * @retval None
 */
void N1225A_Init(void)
{
	unsigned char board = 0,axis = 0;
	for (board = 1; board <= Board_SUM; board++)
	{
		unsigned int BoardAddr = CalBoard_Addr(board);
		for (axis = 1; axis <= 4; axis++)
		{
			unsigned int AxisAddr = CalBoardAxis_Addr(board,axis);
			LaserSourceInit(board, axis);
			SetupLSB(board, axis, 0);
			OutputControlInit(board, axis);
			SampleDelay(board, axis, 0x00FF);
//			AxisReset(board, axis);
			WriteReg((AxisAddr + SamplModeMaskReg), 0x00000000);		//ÿ�β��������󶼸����ٶȺ�λ�üĴ���
			WriteReg((AxisAddr + GainSquelchSettingsReg), 0x00000000);		//turn on AGC, turn off squelch
		}
		BoardReset(board);
		WriteReg((BoardAddr + ErrorStatusResetReg) , 0xFFFFFFFF);			//clear any errors
		WriteReg((BoardAddr + IRQErrorMaskReg), 0x00000000);			//disable any interrupts
		WriteReg((BoardAddr + OutputHoldRateControlReg), 0x00000000);			//Do NOT drive output HOLD line
	}
	
//	with the clock divider output
}

/*
 * void LaserSourceInit(unsigned int BoardID,unsigned char Axis)
 * @brief ��N1225A���ϵ�����г�ʼ��
 * @param BoadID �������ţ�1-��;
 * @param Axis ������ 1-4��
 * @retval None
 */
void LaserSourceInit(unsigned int BoardID,unsigned char Axis)
{
	unsigned int BoardAddr = CalBoard_Addr(BoardID);
	unsigned int AxisAddr = CalBoardAxis_Addr(BoardID, Axis);
	unsigned short MeasA = (Axis - 1) << 4;
	if (BoardID == 1)
	{
		unsigned int MeasB_1 = 3;
		unsigned int value_1 = (MeasA & 0X00F0) | (MeasB_1 & 0x000f);
		*((unsigned int *)(AxisAddr + LaserSourceControlReg)) = value_1;		//���õ�һ���ĵ�����Ϊ�ο���׼
//		printf("value_1 = %x\r\n",value_1);
	}
	else
	{
		unsigned short MeasB_2 = 4;
		unsigned short value_2 = (MeasA & 0X00F0) | (MeasB_2 & 0x000F);
		*((unsigned int *)(AxisAddr + LaserSourceControlReg)) = value_2;		//������Ĳο���׼ȫ����Ϊ��һ���ĵ�����
//		printf("value_2 = %x\r\n", value_2);
	}
}

/*
 * void SetupLSB(unsigned int BoardID, unsigned char Axis,unsigned short LSB)
 * @brief ���ò�������
 * @param BoadID �������ţ�1-��;
 * @param Axis ������ 1-4��
 * @param LSB �������ȣ���ѡ���ֵ������
				0.000: 0.15 nm resolution
				1.001: 0.3 nm resolution
				2.010: 0.6 nm resolution
				3.011: 1.2 nm resolution
				4.100: 2.4 nm resolution
				5.101: 4.8 nm resolution
				6.110: 4.8 nm resolution
				7.111: 4.8 nm resolution
 * @retval None
 */
void SetupLSB(unsigned int BoardID, unsigned char Axis,unsigned short LSB)
{
	unsigned int AxisAddr = CalBoardAxis_Addr(BoardID, Axis);
	*((unsigned int *)(AxisAddr + SetupReg)) = (0X0007 & LSB);	
}

/*
 * void SampleDelay(unsigned int BoardID, unsigned char Axis,)
 * @brief ���ö�ȡλ�üĴ������ٶȼĴ������ӳ�ʱ��
 * @param BoadID �������ţ�1-��;
 * @param Axis ������ 1-4��
 * @param time �����Ҫ�Զ���������Ĵ�����ֵ����ȫΪ1���������Ĵ�����ֵΪ0xFF,
 * @retval None
 */
void SampleDelay(unsigned int BoardID, unsigned char Axis,unsigned short time)
{
	unsigned int AxisAddr = CalBoardAxis_Addr(BoardID, Axis);
	*((unsigned int *)(AxisAddr + SampleDelayReg)) = time;
}

/*
 * void OutputControlInit(unsigned int BoardID, unsigned char Axis)
 * @brief ��N1225A����������Ƴ�ʼ������ʼ��λ,���õ�һ���Ϊ�첽ģʽ1��ʱ������������İ�����Ϊ�첽ģʽ0��
 *        ����ÿ�����P2�ӿڵ�ַ��һ��
 * @param BoadID �������ţ�1-��;
 * @param Axis ������ 1-4��
 * @retval None
 */
void OutputControlInit(unsigned int BoardID, unsigned char Axis)
{
	unsigned int RegAddr = CalBoardAxis_Addr(BoardID,Axis) + OutputControlStatusReg;
	unsigned short AxisAddr_P2 = (BoardID - 1) + (Axis - 1);
	if ( Axis == 1)
	{
		if(BoardID == 1)
		{
			WriteReg(RegAddr,(0xD000| AxisAddr_P2));
		}
		else
		{
			WriteReg(RegAddr,(0x4000 | AxisAddr_P2));
		}		
	}
	else
	{
		WriteReg(RegAddr,(0x0000 | AxisAddr_P2));
	}
}

/*
 * void ReadAllPositions(unsigned int *position,unsigned char PosReg)
 * @brief �����е�����в��������ж���
 * @param *position����Ҫ���������;
 * @param PosReg ��Ҫ�����ļĴ�����1-4��
 * @retval None
 */
void ReadAllPositions(int *position,unsigned char PosReg)
{
	unsigned short Drivesample = 0x01 << (PosReg + 8);
	unsigned char board = 0, axis = 0;
	unsigned int ComRegAddr = CalBoard_Addr(1) + CommandReg;
	unsigned int Value = ReadReg(ComRegAddr);
	unsigned char i = 0;
//	long long Temp;
	Value |= Drivesample;
	WriteReg(ComRegAddr, Value);
	for (board = 1; board <= Board_SUM; board++)
	{
		for (axis = 1; axis <= 4; axis++)
		{
			if ((board == 1 && axis == 4) || (board == 3 && axis == 4))
				continue;
			else
			{
				unsigned int PosReg_Addr = CalBoardAxisPosition_Addr(board, axis, PosReg) + StandardSamplingPosReg;
				*(position+i) = ReadReg(PosReg_Addr);
				if (i == Axis_SUM)
					return;
				i++;
			}
		}
	}
}

void readDCPowerLevel(char nBoard,char nAxis,int* powerLevel)
{
	unsigned int regAddr = CalBoardAxis_Addr(nBoard, nAxis) + PowerLevelReg;
	*powerLevel = *((unsigned int *)regAddr) & 0x0000ffff;
//	printf("Data = %x\r\n",Data);
}

void readACPowerLevel(char nBoard,char nAxis,int* powerLevel)
{
	unsigned int regAddr = CalBoardAxis_Addr(nBoard, nAxis) + PowerLevelReg;
	*powerLevel = *((unsigned int *)regAddr)>>16;

}