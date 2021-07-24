#include <msp430.h>
#include "modbus_rtu.h"

#define MODBUS_TMRF_DELAY_RX	0x01		// ������� (����) �������: �������� 3,5����. ����� �������
#define MODBUS_TMRF_DELAY_TX	0x02		// ������� (����) �������: �������� 3,5����. ����� ���������
#define MODBUS_TMRF_LISTEN		0x03		// ������� (����) �������: ���������, �������� ����������
#define MODBUS_TMRF_WRITE		0x04		// ������� (����) �������: ������

char	MODBUS_A0_PEN = 0,					// �������� ��������: ���.(1)/����.(0)
		MODBUS_A0_PAR = 0,					// ��� �������� ��������: ��������(0)/������(1)
		MODBUS_A0_MSB = 0,					// ������� �����: �������-������(0)/�������-������(1)
		MODBUS_A0_SPB = 1,					// ���������� ����-����� (1 ��� 2)
		MODBUS_A0_MCTL = 0x04,
		MODBUS_A0_BR0 = 0x68,
		MODBUS_A0_BR1 = 0x00,				// �������� ������� (������� BR)
		MODBUS_TA0_ID = ID_3;				// ������� ��������� ������� (ID_0=1, ID_1=2, ID_2=4, ID_3=8)

unsigned int	MODBUS_TMR_INT_LONG_A0 = 119,			// �����, ���������� ����� ����� (3,5 �������). ����� = 3,5����. = 0,003646 �� ��� ������� ������� 32768 ��
				MODBUS_TMR_INT_SHORT_A0 = 51;			// ������������ ���������� ����� ����� ��������� � ����� (1,5 �������). ����� = 1,5����. = 0,001563 �� ��� ������� ������� 32768 ��

char	modbus_buf_str_a0[MODBUS_BUF_SIZE_A0],	// ����� ������-�����������
		modbus_buf_addr_a0,						// ����� ���������������� � ���� MODBUS A0
		modbus_buf_len_a0,						// ����� �������� ����� ������
		modbus_buf_busy_a0 = 0,					// ���� ���������
		modbus_buf_err_a0,						// ����� �������� ������
		modbus_tmr_f_a0,						// ������� ������� (����) �������
		modbus_tmr_short_int_a0;				// ����, ������������, ��� ����� 1,5����. ������

unsigned int	modbus_buf_crc_a0,				// ����� �������� ����������� �����
				modbus_buf_pos_a0;				// ������� ������� ������� � ������ ������-�����������
		
void	(*modbus_buf_callback_f_a0)();			// ����� �������� ��������� �� Callback-�������
unsigned int (*modbus_crc_f_a0)(char*, char);	// ����� �������� ��������� �� ������� ���������� CRC16


/**
 * ������� ������������� ������ UART (Hardware)
 * @param {char} {Address} ����������� ����� ���������� (����������������) � ���� MODBUS
 * @return {void}
 */
void MODBUS_Init(char Address, unsigned int (*crc_f)(char*, char)) {
	modbus_buf_addr_a0 = Address;	// �������� � ����� �����
	modbus_crc_f_a0 = crc_f;		// �������� ��������� �� ������� ���������� CRC16
	
	MODBUS_PSEL_A0 |= MODBUS_TX_A0 + MODBUS_RX_A0;			// ������������� GPIO ��� UART
	MODBUS_PSEL2_A0 |= MODBUS_TX_A0 + MODBUS_RX_A0;			// ������������� GPIO ��� UART
	
	UCA0CTL1 |= UCSWRST;			// UART � ��������� ������
	UCA0CTL1 |= MODBUS_A0_SEL;		// �������� ������ UART
	
	if (MODBUS_A0_PEN == 1) {
		UCA0CTL0 |= UCPEN;			// �������� �������� ���.
	}
	
	if (MODBUS_A0_PAR == 1) {
		UCA0CTL0 |= UCPAR;			// ��� UCPEN �������� �� ��������
	}
	
	if (MODBUS_A0_MSB == 1) {
		UCA0CTL0 |= UCMSB;			// ������� ��� ������
	}
	
	if (MODBUS_A0_SPB == 2) {
		UCA0CTL0 |= UCSPB;			// 2 ����-����
	}

	UCA0BR0 = MODBUS_A0_BR0;
	UCA0BR1 = MODBUS_A0_BR1;
	
	UCA0MCTL = MODBUS_A0_MCTL;		// ���������� ����� ���������. ��� �������� �������������� ������
	
	IE2 &= ~(UCA0RXIE + UCA0TXIE);	// ������ ���������� �� RX � TX
	
	UCA0CTL1 &= ~UCSWRST;			// �������� UART-������
	
	TA0CTL = MODBUS_TA0_SEL + MODBUS_TA0_ID;	// ������ �0, �������.
}

//------------------------------------------------------------------------------------------
/**
 * ����������� ������� �������� MODBUS-���������
 * @param {char} {Address} ����� (� ������ Slave ��������� ����������� �����, � ������ Master - ����� ��������)
 * @param {char} {f} ��� �������
 * @param {char*} {Data} ��������� �� ������ ������ (�� ����� (������ ������-4))
 * @param {char} {DataLength} ����� ������� ������
 * @param {void*()} {Callback} Callback-�������, �������, ���������� �� ��������� ��������
 * @param {bool} {Delay} �������� ����� ���������. ����� �������� ������������ ���������� MODBUS_TMR_INT_DELAY_Ax
 * @return {void}
 */
void MODBUS_Send(char Address, char f, char* Data, char DataLength, void (*Callback)(), bool Delay) {	 
	if (modbus_buf_busy_a0) {
		return ;
	}
	
	if (DataLength > MODBUS_BUF_SIZE_A0 - 4) {
		return ;
	}
	
	modbus_tmr_f_a0 = MODBUS_TMRF_DELAY_TX;
	modbus_buf_callback_f_a0 = Callback;
	
	modbus_buf_len_a0 = DataLength + 4;
	
	modbus_buf_str_a0[0] = Address;
	modbus_buf_str_a0[1] = f;
	
	DataLength += 2;
		
	if (Data != 0)
	{
		for (char i = 2; i < DataLength; i++) {
			modbus_buf_str_a0[i] = Data[i - 2];
		}
	}
	
	unsigned int crc = modbus_crc_f_a0(modbus_buf_str_a0, DataLength);
	
	modbus_buf_str_a0[DataLength] = crc & 0xFF;
	modbus_buf_str_a0[DataLength + 1] = (crc & 0xFF00) >> 8;
	
	TA0CCTL0 = CCIE;
	TA0CCTL1 = 0;

	TA0CCR0 = Delay ? MODBUS_TMR_INT_LONG_A0 : 1;
	TA0CTL |= MC_1 + TACLR;
}

//------------------------------------------------------------------------------------------
/**
 * ������� ������ ��������� (� ������ ���������)
 * @param {void (*)()} {Callback} Callback-�������, �������, ���������� �� ��������� ������
 * @param {bool} {Delay} �������� ����� ������� ���������. ����� �������� ������������ ���������� MODBUS_TMR_INT_DELAY_Ax
 * @return {void}
 */
void MODBUS_Listen(void (*Callback)(), bool Delay) {
	if (modbus_buf_busy_a0) {
		return ;
	}
	
	modbus_tmr_f_a0 = MODBUS_TMRF_DELAY_RX;
	modbus_buf_callback_f_a0 = Callback;
	
	modbus_tmr_short_int_a0 = 0;						// ����� ����� "�����������" ��������� 1,5����.
	
	TA0CCTL0 = CCIE;
	TA0CCTL1 = 0;

	TA0CCR0 = Delay ? MODBUS_TMR_INT_LONG_A0 : 1;
	TA0CTL |= MC_1 + TACLR;
}

//------------------------------------------------------------------------------------------
/**
 * �������� ��������� �� ���������
 * @return {char*}
 */
char* MODBUS_GetAnswer() {
	return (modbus_buf_str_a0 + 2);
}

//------------------------------------------------------------------------------------------
/**
 * �������� ���������� ���� ����������
 * @return {char}
 */
char MODBUS_GetLength() {
	return modbus_buf_pos_a0 - 4;
}

//------------------------------------------------------------------------------------------
/**
 * ���������� �������� i-��� ������� � ������
 * @param {char} {S} �������� �������
 * @param {char} {Index} ������ � ������� ������
 * @return {void}
 */
void MODBUS_SetCharInBuffer(char S, char Index) {	
	Index += 2;
	
	if (Index >= MODBUS_BUF_SIZE_A0 - 2) {
		return;
	}
	
	modbus_buf_str_a0[Index] = S;
}

//------------------------------------------------------------------------------------------
/**
 * �������� �������� �����
 * @return {char}
 */
char MODBUS_GetAddress() {	
	return modbus_buf_str_a0[0];
}

//------------------------------------------------------------------------------------------
/**
 * �������� ��������� � �������
 * @return {char}
 */
char MODBUS_GetFunction() {
	return modbus_buf_str_a0[1];
}

//------------------------------------------------------------------------------------------
/**
 * �������� �������� CRC
 * @return {unsigned int}
 */
unsigned int MODBUS_GetCRC() {
	return modbus_buf_crc_a0;
}

//------------------------------------------------------------------------------------------
/**
 * �������������� ��������� ���������� (������/��������)
 * @param {char} {Module} ����� ������
 * @return {void}
 */
void MODBUS_StopTransaction() {
	TA0CTL &= ~MC_1;					// ��������� �������
	IE2 &= ~(UCA0RXIE + UCA0TXIE);		// ������ ���������� �� RX � TX
	
	modbus_buf_busy_a0 = 0;
}

//------------------------------------------------------------------------------------------
/**
 * �������� ��������� ������
 * @return {bool}
 */
bool MODBUS_IsBusy() {
	return modbus_buf_busy_a0;
}

//------------------------------------------------------------------------------------------
/**
 * ���������� ��� ������
 * @return {char}
 */
char MODBUS_Error() {
	return modbus_buf_err_a0;
}

//------------------------------------------------------------------------------------------
void MODBUS_RX_Service_A0() {
	if ((IE2 & UCA0RXIE) && (IFG2 & UCA0RXIFG)) {				// ���� ���������� �� ������ UCA0
		if (!(TA0CTL & MC_1)) {									// ���� ������ ��� �� ���������� (������ �����)
			modbus_tmr_short_int_a0 = 0;						// ����� ����� "�����������" ��������� 1,5����. 
			
			TA0CCR0 = MODBUS_TMR_INT_LONG_A0;					// �������� ��������� �����
			TA0CCR1	= MODBUS_TMR_INT_SHORT_A0;					// �������� ������ ����� (�� �������)
			
			TA0CCTL0 |= CCIE;									// ���������� ���������� �� ��������� �����
			TA0CCTL1 |= CCIE;									// ���������� ���������� �� ������
		}
		
		TA0CTL &= ~MC_1;										// ��������� �������
		
		if (modbus_tmr_short_int_a0) {							// ������ �����
			IE2 &= ~UCA0RXIE;									// ������ ���������� �� RX
			
			modbus_buf_err_a0 = MODBUS_ERR_FRAME;				// ������������� ������ �����
			modbus_buf_busy_a0 = 0;								// ������� ���� ���������
					
			if (modbus_buf_callback_f_a0 != 0) {				// ���� ������ �� Callback-������� ����
				modbus_buf_callback_f_a0();						// ����� Callback-�������
			}
		}
		
		if (modbus_buf_pos_a0 == MODBUS_BUF_SIZE_A0) {			// ���� �������� ������ ������
			IE2 &= ~UCA0RXIE;									// ������ ���������� �� RX
			
			modbus_buf_busy_a0 = 0;								// ����� ����� ���������
			modbus_buf_err_a0 = MODBUS_ERR_BUF;					// ������������� ������
			
			if (modbus_buf_callback_f_a0 != 0) {				// ���� ������ �� Callback-������� ����
				modbus_buf_callback_f_a0();						// ����� Callback-�������
			}
			
			return ;
		}
		
		modbus_tmr_short_int_a0 = 0;						// ����� ����� "�����������" ��������� 1,5����.
		
		TA0CTL |= MC_1 + TACLR;								// ������ �������
		
		modbus_buf_str_a0[modbus_buf_pos_a0++] = UCA0RXBUF;	// �������� � ����� �������� ����
	}
}

//------------------------------------------------------------------------------------------
void MODBUS_TX_Service_A0() {
	if ((IE2 & UCA0TXIE) && (IFG2 & UCA0TXIFG)) {
		if (modbus_buf_pos_a0 < modbus_buf_len_a0) {
			UCA0TXBUF = modbus_buf_str_a0[modbus_buf_pos_a0++];
		} else {
			IE2 &= ~UCA0TXIE;														// ������ ����������
			
			TA0CCTL0 = CCIE;
			TA0CCTL1 = 0;
			
			TA0CCR0 = MODBUS_TMR_INT_LONG_A0;
			TA0CTL |= MC_1 + TACLR;
		}
	}
}

//------------------------------------------------------------------------------------------
void MODBUS_CCR0_Service_A0() {
	TA0CTL &= ~MC_1;														// ��������� �������
	TA0CCTL0 = 0;
	TA0CCTL1 = 0;

	switch (modbus_tmr_f_a0) {
		case MODBUS_TMRF_LISTEN:		// ������� �������� ��������� 3,5����. ��� ������
			// ����� �����
			IE2 &= ~UCA0RXIE;																			// ������ ����������
			
			modbus_buf_busy_a0 = 0;																		// ��������� ����� �����������
			
			if (modbus_buf_pos_a0 < 5)																	// ���� ������� ������ 5 ����
				modbus_buf_err_a0 = MODBUS_ERR_FRAME_LEN;												// ������������� ������
			
			modbus_buf_crc_a0 = 0x0000;																	// ����� ������ CRC
			modbus_buf_crc_a0 |= modbus_buf_str_a0[modbus_buf_pos_a0 - 1] << 8;							// �������� ������� ���� � �����
			modbus_buf_crc_a0 |= modbus_buf_str_a0[modbus_buf_pos_a0 - 2];								// �������� ������� ���� � �����
			
			if (modbus_buf_crc_a0 != (*modbus_crc_f_a0)(modbus_buf_str_a0, modbus_buf_pos_a0 - 2)) {	// ���� CRC �������� != CRC �����������
				modbus_buf_err_a0 = MODBUS_ERR_CRC;														// ������������� ������
			}
			
			if (modbus_buf_str_a0[0] > 0 && modbus_buf_str_a0[0] != modbus_buf_addr_a0)	{				// ���� ����� ���������� � �� ���
				modbus_buf_err_a0 = MODBUS_ERR_ADDR;													// ������������� ������
			}
			
			if (modbus_buf_callback_f_a0 != 0) {														// ���� ������ �� Callback-������� ����
				(*modbus_buf_callback_f_a0)();															// ����� Callback-�������
			}
		return;
		case MODBUS_TMRF_WRITE:
			// ����� �����
			modbus_buf_busy_a0 = 0;														// ��������� ����� �����������
			
			if (modbus_buf_callback_f_a0 != 0) {										// ���� ������ �� Callback-������� ����
				modbus_buf_callback_f_a0();												// ����� Callback-�������
			}
		return;
		case MODBUS_TMRF_DELAY_RX:		// ������� ��������� ���������� ���������� ����� �������		
			modbus_buf_pos_a0 = 0;														// ������� ������� � ������ = 0
			modbus_tmr_f_a0 = MODBUS_TMRF_LISTEN;										// ������� ������� = ���������
			
			modbus_buf_busy_a0 = 1;														// ��������� ����� ���������
			modbus_buf_err_a0 = 0;														// ����� ������
			modbus_tmr_short_int_a0 = 0;												// ����� ����� "�����������" ��������� 1,5����.
			
			IFG2 &= ~UCA0RXIFG;															// ����� ����� ���������� �� RX
			IE2 |= UCA0RXIE;															// ���������� ���������� �� RX
		return;
		case MODBUS_TMRF_DELAY_TX:		// ������� ��������� ���������� ���������� ����� ���������
			modbus_buf_pos_a0 = 0;														// ������� ������� � ������ = 0
			modbus_tmr_f_a0 = MODBUS_TMRF_WRITE;										// ������� ������� = ������
			
			modbus_buf_busy_a0 = 1;														// ��������� ����� ���������
			modbus_buf_err_a0 = 0;														// ����� ������
			
			IE2 |= UCA0TXIE;															// ���������� ���������� �� TX
		return;
	}
}

//------------------------------------------------------------------------------------------
void MODBUS_CCR1_Service_A0(unsigned int IV) {
	if (IV != 0x02) {
		return ;
	}

	if (modbus_tmr_f_a0 != MODBUS_TMRF_LISTEN) {
		return ;
	}
	
	modbus_tmr_short_int_a0 = 1;
}

//------------------------------------------------------------------------------------------
