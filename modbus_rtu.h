#pragma once

#define MODBUS_ERR_CRC			0x01			// Ошибка: несовпадение CRC
#define MODBUS_ERR_ADDR			0x02			// Ошибка: несовпадение адреса
#define MODBUS_ERR_FRAME		0x03			// Ошибка: ошибка кадра. Превышен таймаут между соседними байтами
#define MODBUS_ERR_FRAME_LEN	0x04			// Ошибка: ошибка длины кадра. Количество принятых байт меньше 5
#define MODBUS_ERR_BUF			0x05			// Ошибка: Переполнение буфера

// конфигурация
#define MODBUS_A0_SEL			UCSSEL_2		// Выбор тактового сигнала модуля USCI A0
#define MODBUS_PSEL_A0			P1SEL			// Регистр выбора функции порта Px (A0)
#define MODBUS_PSEL2_A0			P1SEL2			// Регистр выбора функции 2 порта Px (при отсутствии просто закомментировать) (A0)	
#define MODBUS_TX_A0			BIT2			// Бит порта Px - TX
#define MODBUS_RX_A0			BIT1			// Бит порта Px - RX
#define MODBUS_TA0_SEL			TASSEL_2		// Выбор тактового сигнала (TASSEL_0=TACLK, TASSEL_1=ACLK, TASSEL_2=SMCLK, TASSEL_3=INCLK)

#define MODBUS_BUF_SIZE_A0		256				// Размер буфера приемо-передатчика


void MODBUS_Init(char Address, unsigned int (*crc_f)(char*, char));									// Функция инициализации модуля UART (Hardware)
void MODBUS_Send(char Address, char f, char* Data, char DataLength,									// Функция отправки MODBUS-сообщения
					void (*Callback)() = 0, bool Delay = false);									// 
void MODBUS_Listen(void (*Callback)() = 0, bool Delay = false);										// Функция приема сообщения (в начале прослушка)

char* MODBUS_GetAnswer();																			// Получить указатель на результат
char MODBUS_GetLength();																			// Получить количество байт результата

void MODBUS_SetCharInBuffer(char S, char Index);													// Установить значение i-ого символа в буфере

char MODBUS_GetAddress();																			// Получить принятый адрес
char MODBUS_GetFunction();																			// Получить принятого № функции
unsigned int MODBUS_GetCRC();																		// Получить принятое CRC

void MODBUS_StopTransaction();																		// Принудительная остановка транзакции (приема/передачи)
bool MODBUS_IsBusy();																				// Проверка занятости модуля
char MODBUS_Error();																				// Возвращает код ошибки


/**
 * Вызывать функцию в обработчике прерываний UART RX:
 *
	#pragma vector=USCIAB0RX_VECTOR
	__interrupt void USCIABRX(void)
	{
		MODBUS_RX_Service_A0();
		// ... Обработка прерываний от USCI_B (например)
	}
 */
void MODBUS_RX_Service_A0();

/**
 * Вызывать функцию в обработчике прерываний UART TX:
 
	#pragma vector=USCIAB0TX_VECTOR
	__interrupt void USCIABTX(void)
	{
		MODBUS_TX_Service_A0();
		// ... Обработка прерываний от USCI_B (например)
	}
 */
void MODBUS_TX_Service_A0();

/**
 * Вызывать функцию в обработчике прерываний TimerA0 CCR0:
 *
	#pragma vector=TIMER0_A0_VECTOR
	__interrupt void TA0I0(void)
	{
		MODBUS_CCR0_Service_A0();
		// ... прочий код по необходимости
	}
*/
void MODBUS_CCR0_Service_A0();

/**
 * Вызывать функцию в обработчике прерываний TimerA0 CCR1:
 *
	#pragma vector=TIMER0_A1_VECTOR
	__interrupt void TA0I1(void)
	{
		unsigned int iv = TA0IV;
		
		MODBUS_CCR1_Service_A0(iv);
		// ... прочий код по необходимости
	}
 */
void MODBUS_CCR1_Service_A0(unsigned int);
