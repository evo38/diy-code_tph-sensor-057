#pragma once

#define MODBUS_ERR_CRC			0x01			// ������: ������������ CRC
#define MODBUS_ERR_ADDR			0x02			// ������: ������������ ������
#define MODBUS_ERR_FRAME		0x03			// ������: ������ �����. �������� ������� ����� ��������� �������
#define MODBUS_ERR_FRAME_LEN	0x04			// ������: ������ ����� �����. ���������� �������� ���� ������ 5
#define MODBUS_ERR_BUF			0x05			// ������: ������������ ������

// ������������
#define MODBUS_A0_SEL			UCSSEL_2		// ����� ��������� ������� ������ USCI A0
#define MODBUS_PSEL_A0			P1SEL			// ������� ������ ������� ����� Px (A0)
#define MODBUS_PSEL2_A0			P1SEL2			// ������� ������ ������� 2 ����� Px (��� ���������� ������ ����������������) (A0)	
#define MODBUS_TX_A0			BIT2			// ��� ����� Px - TX
#define MODBUS_RX_A0			BIT1			// ��� ����� Px - RX
#define MODBUS_TA0_SEL			TASSEL_2		// ����� ��������� ������� (TASSEL_0=TACLK, TASSEL_1=ACLK, TASSEL_2=SMCLK, TASSEL_3=INCLK)

#define MODBUS_BUF_SIZE_A0		256				// ������ ������ ������-�����������


void MODBUS_Init(char Address, unsigned int (*crc_f)(char*, char));									// ������� ������������� ������ UART (Hardware)
void MODBUS_Send(char Address, char f, char* Data, char DataLength,									// ������� �������� MODBUS-���������
					void (*Callback)() = 0, bool Delay = false);									// 
void MODBUS_Listen(void (*Callback)() = 0, bool Delay = false);										// ������� ������ ��������� (� ������ ���������)

char* MODBUS_GetAnswer();																			// �������� ��������� �� ���������
char MODBUS_GetLength();																			// �������� ���������� ���� ����������

void MODBUS_SetCharInBuffer(char S, char Index);													// ���������� �������� i-��� ������� � ������

char MODBUS_GetAddress();																			// �������� �������� �����
char MODBUS_GetFunction();																			// �������� ��������� � �������
unsigned int MODBUS_GetCRC();																		// �������� �������� CRC

void MODBUS_StopTransaction();																		// �������������� ��������� ���������� (������/��������)
bool MODBUS_IsBusy();																				// �������� ��������� ������
char MODBUS_Error();																				// ���������� ��� ������


/**
 * �������� ������� � ����������� ���������� UART RX:
 *
	#pragma vector=USCIAB0RX_VECTOR
	__interrupt void USCIABRX(void)
	{
		MODBUS_RX_Service_A0();
		// ... ��������� ���������� �� USCI_B (��������)
	}
 */
void MODBUS_RX_Service_A0();

/**
 * �������� ������� � ����������� ���������� UART TX:
 
	#pragma vector=USCIAB0TX_VECTOR
	__interrupt void USCIABTX(void)
	{
		MODBUS_TX_Service_A0();
		// ... ��������� ���������� �� USCI_B (��������)
	}
 */
void MODBUS_TX_Service_A0();

/**
 * �������� ������� � ����������� ���������� TimerA0 CCR0:
 *
	#pragma vector=TIMER0_A0_VECTOR
	__interrupt void TA0I0(void)
	{
		MODBUS_CCR0_Service_A0();
		// ... ������ ��� �� �������������
	}
*/
void MODBUS_CCR0_Service_A0();

/**
 * �������� ������� � ����������� ���������� TimerA0 CCR1:
 *
	#pragma vector=TIMER0_A1_VECTOR
	__interrupt void TA0I1(void)
	{
		unsigned int iv = TA0IV;
		
		MODBUS_CCR1_Service_A0(iv);
		// ... ������ ��� �� �������������
	}
 */
void MODBUS_CCR1_Service_A0(unsigned int);
