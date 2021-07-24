#pragma once

//--------------------------------------------------------------------------------------
#include <stdint.h>

//--------------------------------------------------------------------------------------
#define I2C_MASTER_B0					0
#define I2C_MASTER_B1					1

//--------------------------------------------------------------------------------------

#define	USE_I2C_MASTER_B0										// Использовать модуль USCI B0 как I2C Master
//#define USE_I2C_MASTER_B1										// Использовать модуль USCI B1 как I2C Master
#define I2C_MASTER_DEFAULT					I2C_MASTER_B0		// Модуль по умолчанию

#define I2C_MASTER_B0_BUF_SIZE				32					// Размер буфера приема/передачи для модуля B0 (байт)
#define I2C_MASTER_B1_BUF_SIZE				32					// Размер буфера приема/передачи для модуля B1 (байт)

#define I2C_MASTER_B0_SEL				UCSSEL_2			// Тактовый сигнал модуля
#define I2C_MASTER_B1_SEL				UCSSEL_2			// Тактовый сигнал модуля

#define I2C_MASTER_B0_PSEL				P1SEL				// Регистр PxSEL
#define I2C_MASTER_B0_PSEL2				P1SEL2				// Регистр PxSEL2 (при отсутствии просто закомментировать)
	
#define I2C_MASTER_B0_SDA				BIT7				// Пин SDA на Px
#define I2C_MASTER_B0_SCL				BIT6				// Пин SCL на Px

#define I2C_MASTER_B1_PSEL				P1SEL				// Регистр PxSEL
#define I2C_MASTER_B1_PSEL2				P1SEL2				// Регистр PxSEL2 (при отсутствии просто закомментировать)
	
#define I2C_MASTER_B1_SDA				BIT7				// Пин SDA на Px
#define I2C_MASTER_B1_SCL				BIT6				// Пин SCL на Px

void I2C_Master_Init(unsigned int Div, uint8_t Module = I2C_MASTER_DEFAULT);				// Функция инициализации модуля USCI Bx как I2C
void I2C_Master_Transmit(uint8_t Address, uint8_t* Data, uint8_t Nd,								// Функция отправки Nd байт ведомому с адресом Address
							void (*Callback)() = 0, uint8_t Module = I2C_MASTER_DEFAULT);
void I2C_Master_Receive(uint8_t Address, uint8_t Nd, void (*Callback)() = 0,
							uint8_t Module = I2C_MASTER_DEFAULT);							// Функция приема Nd байт от ведомого

uint8_t* I2C_Master_Answer(uint8_t Module = I2C_MASTER_DEFAULT);								// Возвращает указатель на принятый результат
uint8_t I2C_Master_AnswerLength(uint8_t Module = I2C_MASTER_DEFAULT);							// Возвращает количество принятых символов
void I2C_Master_ClearReceiveBuffer(uint8_t Module = I2C_MASTER_DEFAULT);

void I2C_Master_Stop(uint8_t Module = I2C_MASTER_DEFAULT);									// Принудительная остановка модуля, посылает стоп-условие
bool I2C_Master_Busy(uint8_t Module = I2C_MASTER_DEFAULT);									// Проверка занятости модуля (не эквивалентно (UCBxSTAT & UCBBUSY) !!!)
bool I2C_Master_LineBusy(uint8_t Module = I2C_MASTER_DEFAULT);								// Проверка занятости линии

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
 *	Вызывать функцию I2C_MASTER_TX_Service_Bx в обработчике прерываний USCI TX:
	
	#pragma vector=USCIAB0TX_VECTOR
	__interrupt void USCIAB0TX_ISR(void)
	{
		I2C_MASTER_TX_Service_B0();
		// ... Прочий код (например, для обработки прерываний от USCI A0)
	}
 */
/**
 *	Вызывать функцию I2C_MASTER_RX_Service_Bx в обработчике прерываний от USCI RX:

	#pragma vector=USCIAB0RX_VECTOR
	__interrupt void USCIAB0RX_ISR(void)
	{
		I2C_MASTER_RX_Service_B0();
		// ... Прочий код (например, для обработки прерываний от USCI A0)
	}
 */