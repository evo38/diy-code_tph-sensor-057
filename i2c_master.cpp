#include <msp430.h>
#include "i2c_master.h"


#ifdef USE_I2C_MASTER_B0
	uint8_t	i2c_master_buf_str_b0[I2C_MASTER_B0_BUF_SIZE];		// �������� ����� ������/��������
	uint8_t	i2c_master_buf_len_b0,								// �������� ����� ����� ������ (� ������)
			i2c_master_buf_pos_b0;								// �������� ����� ������� ������� � ������ (������)
			
	uint8_t	i2c_master_buf_busy_b0;								// ����� ������: BIT0=��������� ������

	void	(*i2c_master_buf_callback_f_b0)() = 0;				// ����� �������� ��������� �� Callback-�������
	
	uint8_t	i2c_master_buf_inited_b0 = 0;
#endif

#ifdef USE_I2C_MASTER_B1
	uint8_t	i2c_master_buf_str_b1[I2C_MASTER_B1_BUF_SIZE];		// �������� ����� ������/��������
	uint8_t	i2c_master_buf_len_b1,								// �������� ����� ����� ������ (� ������)
			i2c_master_buf_pos_b1;								// �������� ����� ������� ������� � ������ (������)
			
	uint8_t	i2c_master_buf_busy_b1;								// ����� ������: BIT0=��������� ������

	void	(*i2c_master_buf_callback_f_b1)() = 0;				// ����� �������� ��������� �� Callback-�������
#endif



void I2C_Master_Init(unsigned int Div, uint8_t Module)
{
	#ifdef USE_I2C_MASTER_B0
		if (Module == I2C_MASTER_B0)
		{
			I2C_MASTER_B0_PSEL |= I2C_MASTER_B0_SDA + I2C_MASTER_B0_SCL;					// ��������� �����
			
			#ifdef I2C_MASTER_B0_PSEL2
				I2C_MASTER_B0_PSEL2 |= I2C_MASTER_B0_SDA + I2C_MASTER_B0_SCL;				// ��������� �����
			#endif
			
			UCB0CTL1 = UCSWRST;                        										// ������ USCI - �������
			
			UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;      										// ������ USCI ���: I2C Master � ���������� ������
			UCB0CTL1 = I2C_MASTER_B0_SEL + UCSWRST;    										// ����� ��������� �������, USCI ���������� � ��������� ������
			
			UCB0BR0 = (uint8_t) (Div & 0xFF);                        							// ������� ������� ��������� �������, ������� ����
			UCB0BR1 = (uint8_t) ((Div >> 8) & 0xFF);											// ������� ������� ��������� �������, ������� ����
			
			UCB0CTL1 &= ~UCSWRST;                       									// ������� USCI � ������� �����
			
			UCB0I2CIE = UCNACKIE;															// ��������� ���������� �� NACK
		}
	#endif
	
	#if defined(USE_I2C_MASTER_B0) && defined(USE_I2C_MASTER_B1)
		else
	#endif

	#ifdef USE_I2C_MASTER_B1
		if (Module == I2C_MASTER_B1)
		{
			I2C_MASTER_B1_PSEL |= I2C_MASTER_B1_SDA + I2C_MASTER_B1_SCL;					// ��������� �����
			
			#ifdef I2C_MASTER_B1_PSEL2
				I2C_MASTER_B1_PSEL2 |= I2C_MASTER_B1_SDA + I2C_MASTER_B1_SCL;				// ��������� �����
			#endif
			
			UCB1CTL1 = UCSWRST;                        										// ������ USCI - �������
			
			UCB1CTL0 = UCMST + UCMODE_3 + UCSYNC;      										// ������ USCI ���: I2C Master � ���������� ������
			UCB1CTL1 = I2C_MASTER_B1_SEL + UCSWRST;    										// ����� ��������� �������, USCI ���������� � ��������� ������
			
			UCB1BR0 = (uint8_t) (Div & 0xFF);                        							// ������� ������� ��������� �������, ������� ����
			UCB1BR1 = (uint8_t) ((Div >> 8) & 0xFF);											// ������� ������� ��������� �������, ������� ����
			
			UCB1CTL1 &= ~UCSWRST;                       									// ������� USCI � ������� �����
			
			UCB1I2CIE = UCNACKIE;															// ��������� ���������� �� NACK
		}
	#endif
}

//------------------------------------------------------------------------------------------
void I2C_Master_Transmit(uint8_t Address, uint8_t* Data, uint8_t Nd, void (*Callback)(), uint8_t Module)
{
	/**
	 * ������� �������� Nd ���� �������� � ������� Address
	 * @param {uint8_t} {Address} ����� ��������
	 * @param {uint8_t*} {Data} ��������� �� ������ ������
	 * @param {uint8_t} {Nd} ���������� ���� ��� �������� � �������
	 * @param {void (*)()} {Callback} Callback-�������
	 * @param {uint8_t} {Module} ����� ������
	 * @return {void}}
	 */
	
	_BIS_SR(GIE);
	
	//volatile 
			
	#ifdef USE_I2C_MASTER_B0
		if (Module == I2C_MASTER_B0)
		{
			UCB0I2CSA = Address;															// ������������� ����� ��������
			
			if (Data)
			{
				uint8_t i = 0;
				while (i < Nd) {															// �������� ������ ��� ��������
					i2c_master_buf_str_b0[i] = Data[i];										// � �����
					i++;
				}
			}
			
			i2c_master_buf_len_b0 = Nd;														// �������� � ����� ���������� ���� ��� ��������
			i2c_master_buf_callback_f_b0 = Callback;										// �������� � ����� ��������� �� Callback-�������
			i2c_master_buf_pos_b0 = 0;														// ������� ������� � ������ ��������
			i2c_master_buf_busy_b0 = 0x01;													// ������ �����
			
			if (!i2c_master_buf_inited_b0)
				i2c_master_buf_inited_b0++;
			
			IE2 &= ~UCB0RXIE;																// ������ ���������� �� RX
			IE2 |= UCB0TXIE;                            									// ���������� ���������� �� TX
			UCB0CTL1 |= UCTR + UCTXSTT;                 									// USCI I2C - �� ��������, ��������� �����-�������
		}
	#endif
	
	#if defined(USE_I2C_MASTER_B0) && defined(USE_I2C_MASTER_B1)
		else
	#endif

	#ifdef USE_I2C_MASTER_B1
		if (Module == I2C_MASTER_B1)
		{
			UCB1I2CSA = Address;															// ������������� ����� ��������
			
			if (Data)
			{
				uint8_t i = 0;
				while (i < Nd)																// �������� ������ ��� ��������
					i2c_master_buf_str_b1[i] = Data[i++];									// � �����
			}
			
			i2c_master_buf_len_b1 = Nd;														// �������� � ����� ���������� ���� ��� ��������
			i2c_master_buf_callback_f_b1 = Callback;										// �������� � ����� ��������� �� Callback-�������
			i2c_master_buf_pos_b1 = 0;														// ������� ������� � ������ ��������
			i2c_master_buf_busy_b1 = 0x01;													// ������ �����
			
			UC1IE &= ~UCB1RXIE;																// ������ ���������� �� RX
			UC1IE |= UCB1TXIE;                            									// ���������� ���������� �� TX
			UCB1CTL1 |= UCTR + UCTXSTT;                 									// USCI I2C - �� ��������, ��������� �����-�������
		}
	#endif
}

//------------------------------------------------------------------------------------------
void I2C_Master_Receive(uint8_t Address, uint8_t Nd, void (*Callback)(), uint8_t Module)
{
	/**
	 * ������� ������ Nd ���� �� ��������
	 * @param {uint8_t} {Address} ����� ��������
	 * @param {uint8_t} {Nd} ���-�� ���� ��� ������
	 * @param {void (*)()} {Callback} Callback-�������
	 * @param {uint8_t} {Module} ����� ������
	 * @return {void}
	 */
	
	_BIS_SR(GIE);
	
	#ifdef USE_I2C_MASTER_B0
		if (Module == I2C_MASTER_B0)
		{
			i2c_master_buf_pos_b0 = 0;														// ������� ������� � ������
			i2c_master_buf_len_b0 = Nd;														// �������� � ����� ���������� ���� ��� ������
			i2c_master_buf_callback_f_b0 = Callback;										// �������� � ����� ��������� �� Callback-�������
			i2c_master_buf_busy_b0 = 0x01;													// ������ �����
			
			if (i2c_master_buf_inited_b0)
				i2c_master_buf_inited_b0 = 2;
			
			IE2 &= ~UCB0TXIE;																// ������ ���������� �� TX
			IE2 |= UCB0RXIE;                           										// ���������� ���������� �� RX
			
			UCB0CTL1 &= ~UCTR;																// USCI I2C ������ - �� �����
			UCB0CTL1 |= UCTXSTT;                      										// �������� �����-�������
		}
	#endif
	
	#if defined(USE_I2C_MASTER_B0) && defined(USE_I2C_MASTER_B1)
		else
	#endif

	#ifdef USE_I2C_MASTER_B1
		if (Module == I2C_MASTER_B1)
		{
			i2c_master_buf_pos_b1 = 0;														// ������� ������� � ������
			i2c_master_buf_len_b1 = Nd;														// �������� � ����� ���������� ���� ��� ������
			i2c_master_buf_callback_f_b1 = Callback;										// �������� � ����� ��������� �� Callback-�������
			i2c_master_buf_busy_b1 = 0x01;													// ������ �����
			
			UC1IE &= ~UCB1TXIE;																// ������ ���������� �� TX
			UC1IE |= UCB1RXIE;                           									// ���������� ���������� �� RX
			
			UCB1CTL1 &= ~UCTR;																// USCI I2C ������ - �� �����
			UCB1CTL1 |= UCTXSTT;                      										// �������� �����-�������
		}
	#endif
}

//------------------------------------------------------------------------------------------
uint8_t* I2C_Master_Answer(uint8_t Module)
{
	#ifdef USE_I2C_MASTER_B0
		if (Module == I2C_MASTER_B0)
			return i2c_master_buf_str_b0;
	#endif
	
	#if defined(USE_I2C_MASTER_B0) && defined(USE_I2C_MASTER_B1)
		else
	#endif

	#ifdef USE_I2C_MASTER_B1
		if (Module == I2C_MASTER_B1)
			return i2c_master_buf_str_b1;
	#endif
	
	return 0;
}

//------------------------------------------------------------------------------------------
uint8_t I2C_Master_AnswerLength(uint8_t Module)
{	 
	#ifdef USE_I2C_MASTER_B0
		if (Module == I2C_MASTER_B0)
			return i2c_master_buf_pos_b0;
	#endif
	
	#if defined(USE_I2C_MASTER_B0) && defined(USE_I2C_MASTER_B1)
		else
	#endif

	#ifdef USE_I2C_MASTER_B1
		if (Module == I2C_MASTER_B1)
			return i2c_master_buf_pos_b1;
	#endif
	
	return 0;
}

//------------------------------------------------------------------------------------------
void I2C_Master_Stop(uint8_t Module)
{
	#ifdef USE_I2C_MASTER_B0
		if (Module == I2C_MASTER_B0)
		{
			IE2 &= ~(UCB0RXIE + UCB0TXIE);
			
			UCB0CTL1 |= UCTXSTP;
			while (UCB0CTL1 & UCTXSTP);
			
			i2c_master_buf_busy_b0 &= 0xFE;
		}
	#endif
	
	#if defined(USE_I2C_MASTER_B0) && defined(USE_I2C_MASTER_B1)
		else
	#endif

	#ifdef USE_I2C_MASTER_B1
		if (Module == I2C_MASTER_B1)
		{
			UC1IE &= ~(UCB1RXIE + UCB1TXIE);
			
			UCB1CTL1 |= UCTXSTP;
			while (UCB1CTL1 & UCTXSTP);
			
			i2c_master_buf_busy_b1 &= 0xFE;
		}
	#endif
}

//------------------------------------------------------------------------------------------
bool I2C_Master_Busy(uint8_t Module)
{
	#ifdef USE_I2C_MASTER_B0
		if (Module == I2C_MASTER_B0)
			return (i2c_master_buf_busy_b0 & 0x01);
	#endif
	
	#if defined(USE_I2C_MASTER_B0) && defined(USE_I2C_MASTER_B1)
		else
	#endif

	#ifdef USE_I2C_MASTER_B1
		if (Module == I2C_MASTER_B1)
			return (i2c_master_buf_busy_b1 & 0x01);
	#endif
	
	return false;
}

//------------------------------------------------------------------------------------------
bool I2C_Master_LineBusy(uint8_t Module)
{
	#ifdef USE_I2C_MASTER_B0
		if (Module == I2C_MASTER_B0)
			return (UCB0STAT & UCBBUSY);
	#endif
	
	#if defined(USE_I2C_MASTER_B0) && defined(USE_I2C_MASTER_B1)
		else
	#endif

	#ifdef USE_I2C_MASTER_B1
		if (Module == I2C_MASTER_B1)
			return (UCB1STAT & UCBBUSY);
	#endif
	
	return false;
}

//------------------------------------------------------------------------------------------
void I2C_Master_ClearReceiveBuffer(uint8_t Module)
{
	#ifdef USE_I2C_MASTER_B0
		if (Module == I2C_MASTER_B0)
		{
			uint8_t buf = UCB0RXBUF;
			buf++;
		}
	#endif
	
	#if defined(USE_I2C_MASTER_B0) && defined(USE_I2C_MASTER_B1)
		else
	#endif

	#ifdef USE_I2C_MASTER_B1
		if (Module == I2C_MASTER_B1)
		{
			uint8_t buf = UCB1RXBUF;
			buf++;
		}
	#endif
}

//------------------------------------------------------------------------------------------
#ifdef USE_I2C_MASTER_B0
	void I2C_MASTER_TX_Service_B0()
	{
		if ((IE2 & UCB0TXIE) && (IFG2 & UCB0TXIFG))
		{
			if (i2c_master_buf_pos_b0 < i2c_master_buf_len_b0)
				UCB0TXBUF = i2c_master_buf_str_b0[i2c_master_buf_pos_b0++];
			else
			{
				IE2 &= ~(UCB0TXIE);															// ������ ����������
				IFG2 &= ~UCB0TXIFG;															// ����� ����� ���������� �� TX
				UCB0CTL1 |= UCTXSTP;														// ���������� ����-�������
				
				while (UCB0CTL1 & UCTXSTP);													// �������� ��������� ����-�������
				
				i2c_master_buf_busy_b0 &= 0xFE;
				
				if (i2c_master_buf_callback_f_b0 != 0)										// ���� ������ Callback-�������
					i2c_master_buf_callback_f_b0();											// �������� ��
			}
		}
		
		if ((IE2 & UCB0RXIE) && (IFG2 & UCB0RXIFG))
		{
			if (i2c_master_buf_pos_b0 < i2c_master_buf_len_b0)
			{
				//if (i2c_master_buf_inited_b0 < 2 || i2c_master_buf_inited_b0 == 3)
					i2c_master_buf_str_b0[i2c_master_buf_pos_b0++] = UCB0RXBUF;
				//else if (i2c_master_buf_inited_b0 == 2)
					//i2c_master_buf_inited_b0++;
			}
			else
			{
				IE2 &= ~(UCB0RXIE);															// ������ ���������� �� RX
				UCB0CTL1 |= UCTXSTP;														// ���������� ����-�������
				
				while (UCB0CTL1 & UCTXSTP);													// �������� ��������� ����-�������
				
				i2c_master_buf_busy_b0 &= 0xFE;
				
				if (i2c_master_buf_callback_f_b0 != 0)										// ���� ������ Callback-�������
					i2c_master_buf_callback_f_b0();											// �������� ��
			}
		}
	}
	
	//--------------------------------------------------------------------------------------
	void I2C_MASTER_RX_Service_B0()
	{
		if (UCB0STAT & UCNACKIFG)															// ��������� ����-�������, ���� ������� �������� NACK
		{
			UCB0STAT &= ~UCNACKIFG;															// ����� ����� ���������� �� NACK
			
			UCB0CTL1 |= UCTXSTP;															// ���������� ����-�������
			while (UCB0CTL1 & UCTXSTP);														// �������� ��������� ����-�������
			
			i2c_master_buf_busy_b0 &= 0xFE;

			if (i2c_master_buf_callback_f_b0 != 0)											// ���� ������ Callback-�������
				i2c_master_buf_callback_f_b0();												// �������� ��
		}
	}
#endif

#ifdef USE_I2C_MASTER_B1
	void I2C_MASTER_TX_Service_B1()
	{
		if ((UC1IE & UCB1TXIE) && (UC1IFG & UCB1TXIFG))
		{
			if (i2c_master_buf_pos_b1 < i2c_master_buf_len_b1)
				UCB1TXBUF = i2c_master_buf_str_b1[i2c_master_buf_pos_b1++];
			else
			{
				UC1IE &= ~(UCB1TXIE);														// ������ ����������
				UC1IFG &= ~UCB1TXIFG;														// ����� ����� ���������� �� TX
				UCB1CTL1 |= UCTXSTP;														// ���������� ����-�������
				
				while (UCB1CTL1 & UCTXSTP);													// �������� ��������� ����-�������
				
				i2c_master_buf_busy_b1 &= 0xFE;
				
				if (i2c_master_buf_callback_f_b1 != 0)										// ���� ������ Callback-�������
					i2c_master_buf_callback_f_b1();											// �������� ��
			}
		}
		
		if ((UC1IE & UCB1RXIE) && (UC1IFG & UCB1RXIFG))
		{
			if (i2c_master_buf_pos_b1 < i2c_master_buf_len_b1)
				i2c_master_buf_str_b1[i2c_master_buf_pos_b1++] = UCB1RXBUF;
			else
			{
				UC1IE &= ~(UCB1RXIE);														// ������ ���������� �� RX
				UCB1CTL1 |= UCTXSTP;														// ���������� ����-�������
				
				while (UCB1CTL1 & UCTXSTP);													// �������� ��������� ����-�������
				
				i2c_master_buf_busy_b1 &= 0xFE;
				
				if (i2c_master_buf_callback_f_b1 != 0)										// ���� ������ Callback-�������
					i2c_master_buf_callback_f_b1();											// �������� ��
			}
		}
	}
	
	//--------------------------------------------------------------------------------------
	void I2C_MASTER_RX_Service_B1()
	{
		if (UCB1STAT & UCNACKIFG)															// ��������� ����-�������, ���� ������� �������� NACK
		{
			UCB1STAT &= ~UCNACKIFG;
			
			UCB1CTL1 |= UCTXSTP;															// ���������� ����-�������
			while (UCB1CTL1 & UCTXSTP);														// �������� ��������� ����-�������
			
			i2c_master_buf_busy_b1 &= 0xFE;
			
			if (i2c_master_buf_callback_f_b1 != 0)											// ���� ������ Callback-�������
				i2c_master_buf_callback_f_b1();												// �������� ��
		}
	}
#endif