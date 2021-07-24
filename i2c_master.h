#pragma once

//--------------------------------------------------------------------------------------
#include <stdint.h>

//--------------------------------------------------------------------------------------
#define I2C_MASTER_B0					0
#define I2C_MASTER_B1					1

//--------------------------------------------------------------------------------------

#define	USE_I2C_MASTER_B0										// ������������ ������ USCI B0 ��� I2C Master
//#define USE_I2C_MASTER_B1										// ������������ ������ USCI B1 ��� I2C Master
#define I2C_MASTER_DEFAULT					I2C_MASTER_B0		// ������ �� ���������

#define I2C_MASTER_B0_BUF_SIZE				32					// ������ ������ ������/�������� ��� ������ B0 (����)
#define I2C_MASTER_B1_BUF_SIZE				32					// ������ ������ ������/�������� ��� ������ B1 (����)

#define I2C_MASTER_B0_SEL				UCSSEL_2			// �������� ������ ������
#define I2C_MASTER_B1_SEL				UCSSEL_2			// �������� ������ ������

#define I2C_MASTER_B0_PSEL				P1SEL				// ������� PxSEL
#define I2C_MASTER_B0_PSEL2				P1SEL2				// ������� PxSEL2 (��� ���������� ������ ����������������)
	
#define I2C_MASTER_B0_SDA				BIT7				// ��� SDA �� Px
#define I2C_MASTER_B0_SCL				BIT6				// ��� SCL �� Px

#define I2C_MASTER_B1_PSEL				P1SEL				// ������� PxSEL
#define I2C_MASTER_B1_PSEL2				P1SEL2				// ������� PxSEL2 (��� ���������� ������ ����������������)
	
#define I2C_MASTER_B1_SDA				BIT7				// ��� SDA �� Px
#define I2C_MASTER_B1_SCL				BIT6				// ��� SCL �� Px

void I2C_Master_Init(unsigned int Div, uint8_t Module = I2C_MASTER_DEFAULT);				// ������� ������������� ������ USCI Bx ��� I2C
void I2C_Master_Transmit(uint8_t Address, uint8_t* Data, uint8_t Nd,								// ������� �������� Nd ���� �������� � ������� Address
							void (*Callback)() = 0, uint8_t Module = I2C_MASTER_DEFAULT);
void I2C_Master_Receive(uint8_t Address, uint8_t Nd, void (*Callback)() = 0,
							uint8_t Module = I2C_MASTER_DEFAULT);							// ������� ������ Nd ���� �� ��������

uint8_t* I2C_Master_Answer(uint8_t Module = I2C_MASTER_DEFAULT);								// ���������� ��������� �� �������� ���������
uint8_t I2C_Master_AnswerLength(uint8_t Module = I2C_MASTER_DEFAULT);							// ���������� ���������� �������� ��������
void I2C_Master_ClearReceiveBuffer(uint8_t Module = I2C_MASTER_DEFAULT);

void I2C_Master_Stop(uint8_t Module = I2C_MASTER_DEFAULT);									// �������������� ��������� ������, �������� ����-�������
bool I2C_Master_Busy(uint8_t Module = I2C_MASTER_DEFAULT);									// �������� ��������� ������ (�� ������������ (UCBxSTAT & UCBBUSY) !!!)
bool I2C_Master_LineBusy(uint8_t Module = I2C_MASTER_DEFAULT);								// �������� ��������� �����

//------------------------------------------------------------------------------------------------------------
#ifdef USE_I2C_MASTER_B0
	void I2C_MASTER_TX_Service_B0();
	void I2C_MASTER_RX_Service_B0();
#endif

#ifdef USE_I2C_MASTER_B1
	void I2C_MASTER_TX_Service_B1();
	void I2C_MASTER_RX_Service_B1();
#endif

/**
 *	�������� ������� I2C_MASTER_TX_Service_Bx � ����������� ���������� USCI TX:
	
	#pragma vector=USCIAB0TX_VECTOR
	__interrupt void USCIAB0TX_ISR(void)
	{
		I2C_MASTER_TX_Service_B0();
		// ... ������ ��� (��������, ��� ��������� ���������� �� USCI A0)
	}
 */
/**
 *	�������� ������� I2C_MASTER_RX_Service_Bx � ����������� ���������� �� USCI RX:

	#pragma vector=USCIAB0RX_VECTOR
	__interrupt void USCIAB0RX_ISR(void)
	{
		I2C_MASTER_RX_Service_B0();
		// ... ������ ��� (��������, ��� ��������� ���������� �� USCI A0)
	}
 */