#include <msp430.h>
#include "modbus_rtu.h"

#define MODBUS_TMRF_DELAY_RX	0x01		// Функция (роль) таймера: задержка 3,5симв. перед приемом
#define MODBUS_TMRF_DELAY_TX	0x02		// Функция (роль) таймера: задержка 3,5симв. перед передачей
#define MODBUS_TMRF_LISTEN		0x03		// Функция (роль) таймера: прослушка, контроль интервалов
#define MODBUS_TMRF_WRITE		0x04		// Функция (роль) таймера: запись

char	MODBUS_A0_PEN = 0,					// Контроль четности: вкл.(1)/выкл.(0)
		MODBUS_A0_PAR = 0,					// Тип контроля четности: нечетный(0)/четный(1)
		MODBUS_A0_MSB = 0,					// Порядок битов: младший-первый(0)/старший-первый(1)
		MODBUS_A0_SPB = 1,					// Количество стоп-битов (1 или 2)
		MODBUS_A0_MCTL = 0x04,
		MODBUS_A0_BR0 = 0x68,
		MODBUS_A0_BR1 = 0x00,				// Делитель частоты (регистр BR)
		MODBUS_TA0_ID = ID_3;				// Деление тактового сигнала (ID_0=1, ID_1=2, ID_2=4, ID_3=8)

unsigned int	MODBUS_TMR_INT_LONG_A0 = 119,			// Время, означающее конец кадра (3,5 символа). Время = 3,5симв. = 0,003646 мс при частоте таймера 32768 Гц
				MODBUS_TMR_INT_SHORT_A0 = 51;			// Максимальное расстояние между двумя символами в кадре (1,5 символа). Время = 1,5симв. = 0,001563 мс при частоте таймера 32768 Гц

char	modbus_buf_str_a0[MODBUS_BUF_SIZE_A0],	// Буфер приемо-передатчика
		modbus_buf_addr_a0,						// Адрес микроконтроллера в сети MODBUS A0
		modbus_buf_len_a0,						// Буфер хранения длины данных
		modbus_buf_busy_a0 = 0,					// Флаг занятости
		modbus_buf_err_a0,						// Буфер хранения ошибки
		modbus_tmr_f_a0,						// Текущая функция (роль) таймера
		modbus_tmr_short_int_a0;				// Флаг, показывающий, что время 1,5симв. прошло

unsigned int	modbus_buf_crc_a0,				// Буфер хранения контрольной суммы
				modbus_buf_pos_a0;				// Текущая позиция курсора в буфере приемо-передатчика
		
void	(*modbus_buf_callback_f_a0)();			// Буфер хранения указателя на Callback-функцию
unsigned int (*modbus_crc_f_a0)(char*, char);	// Буфер хранения указателя на функцию вычисления CRC16


/**
 * Функция инициализации модуля UART (Hardware)
 * @param {char} {Address} Собственный адрес устройства (микроконтроллера) в сети MODBUS
 * @return {void}
 */
void MODBUS_Init(char Address, unsigned int (*crc_f)(char*, char)) {
	modbus_buf_addr_a0 = Address;	// Загоняем в буфер адрес
	modbus_crc_f_a0 = crc_f;		// Загоняем указатель на функцию вычисления CRC16
	
	MODBUS_PSEL_A0 |= MODBUS_TX_A0 + MODBUS_RX_A0;			// Конфигурируем GPIO для UART
	MODBUS_PSEL2_A0 |= MODBUS_TX_A0 + MODBUS_RX_A0;			// Конфигурируем GPIO для UART
	
	UCA0CTL1 |= UCSWRST;			// UART в состоянии сброса
	UCA0CTL1 |= MODBUS_A0_SEL;		// Тактовый сигнал UART
	
	if (MODBUS_A0_PEN == 1) {
		UCA0CTL0 |= UCPEN;			// Контроль четности вкл.
	}
	
	if (MODBUS_A0_PAR == 1) {
		UCA0CTL0 |= UCPAR;			// При UCPEN проверка на четность
	}
	
	if (MODBUS_A0_MSB == 1) {
		UCA0CTL0 |= UCMSB;			// Страший бит первый
	}
	
	if (MODBUS_A0_SPB == 2) {
		UCA0CTL0 |= UCSPB;			// 2 стоп-бита
	}

	UCA0BR0 = MODBUS_A0_BR0;
	UCA0BR1 = MODBUS_A0_BR1;
	
	UCA0MCTL = MODBUS_A0_MCTL;		// Определяет маску модуляции. Это помогает минимизировать ошибки
	
	IE2 &= ~(UCA0RXIE + UCA0TXIE);	// Запрет прерываний от RX и TX
	
	UCA0CTL1 &= ~UCSWRST;			// Включаем UART-модуль
	
	TA0CTL = MODBUS_TA0_SEL + MODBUS_TA0_ID;	// Таймер А0, останов.
}

//------------------------------------------------------------------------------------------
/**
 * Асинхронная функция отправки MODBUS-сообщения
 * @param {char} {Address} Адрес (в режиме Slave повторяем собственный адрес, в режиме Master - адрес ведомого)
 * @param {char} {f} Код функции
 * @param {char*} {Data} Указатель на массив данных (не более (размер буфера-4))
 * @param {char} {DataLength} Длина массива данных
 * @param {void*()} {Callback} Callback-функция, функция, вызываемая по окончанию передачи
 * @param {bool} {Delay} Задержка перед передачей. Длина задержки определяется константой MODBUS_TMR_INT_DELAY_Ax
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
 * Функция приема сообщения (в начале прослушка)
 * @param {void (*)()} {Callback} Callback-функция, функция, вызываемая по окончанию приема
 * @param {bool} {Delay} Задержка перед началом прослушки. Длина задержки определяется константой MODBUS_TMR_INT_DELAY_Ax
 * @return {void}
 */
void MODBUS_Listen(void (*Callback)(), bool Delay) {
	if (modbus_buf_busy_a0) {
		return ;
	}
	
	modbus_tmr_f_a0 = MODBUS_TMRF_DELAY_RX;
	modbus_buf_callback_f_a0 = Callback;
	
	modbus_tmr_short_int_a0 = 0;						// Сброс флага "прошедшости" интервала 1,5симв.
	
	TA0CCTL0 = CCIE;
	TA0CCTL1 = 0;

	TA0CCR0 = Delay ? MODBUS_TMR_INT_LONG_A0 : 1;
	TA0CTL |= MC_1 + TACLR;
}

//------------------------------------------------------------------------------------------
/**
 * Получить указатель на результат
 * @return {char*}
 */
char* MODBUS_GetAnswer() {
	return (modbus_buf_str_a0 + 2);
}

//------------------------------------------------------------------------------------------
/**
 * Получить количество байт результата
 * @return {char}
 */
char MODBUS_GetLength() {
	return modbus_buf_pos_a0 - 4;
}

//------------------------------------------------------------------------------------------
/**
 * Установить значение i-ого символа в буфере
 * @param {char} {S} Значение символа
 * @param {char} {Index} Индекс в массиве буфера
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
 * Получить принятый адрес
 * @return {char}
 */
char MODBUS_GetAddress() {	
	return modbus_buf_str_a0[0];
}

//------------------------------------------------------------------------------------------
/**
 * Получить принятого № функции
 * @return {char}
 */
char MODBUS_GetFunction() {
	return modbus_buf_str_a0[1];
}

//------------------------------------------------------------------------------------------
/**
 * Получить принятое CRC
 * @return {unsigned int}
 */
unsigned int MODBUS_GetCRC() {
	return modbus_buf_crc_a0;
}

//------------------------------------------------------------------------------------------
/**
 * Принудительная остановка транзакции (приема/передачи)
 * @param {char} {Module} Выбор модуля
 * @return {void}
 */
void MODBUS_StopTransaction() {
	TA0CTL &= ~MC_1;					// Остановка таймера
	IE2 &= ~(UCA0RXIE + UCA0TXIE);		// Запрет прерываний от RX и TX
	
	modbus_buf_busy_a0 = 0;
}

//------------------------------------------------------------------------------------------
/**
 * Проверка занятости модуля
 * @return {bool}
 */
bool MODBUS_IsBusy() {
	return modbus_buf_busy_a0;
}

//------------------------------------------------------------------------------------------
/**
 * Возвращает код ошибки
 * @return {char}
 */
char MODBUS_Error() {
	return modbus_buf_err_a0;
}

//------------------------------------------------------------------------------------------
void MODBUS_RX_Service_A0() {
	if ((IE2 & UCA0RXIE) && (IFG2 & UCA0RXIFG)) {				// Если прерывание от модуля UCA0
		if (!(TA0CTL & MC_1)) {									// Если таймер еще не запускался (начало кадра)
			modbus_tmr_short_int_a0 = 0;						// Сброс флага "прошедшости" интервала 1,5симв. 
			
			TA0CCR0 = MODBUS_TMR_INT_LONG_A0;					// Контроль окончания кадра
			TA0CCR1	= MODBUS_TMR_INT_SHORT_A0;					// Контроль ошибки кадра (по времени)
			
			TA0CCTL0 |= CCIE;									// Разрешение прерывания по окончанию кадра
			TA0CCTL1 |= CCIE;									// Разрешение прерывания по ошибке
		}
		
		TA0CTL &= ~MC_1;										// Остановка таймера
		
		if (modbus_tmr_short_int_a0) {							// Ошибка кадра
			IE2 &= ~UCA0RXIE;									// Запрет прерываний от RX
			
			modbus_buf_err_a0 = MODBUS_ERR_FRAME;				// Устанавливаем ошибку кадра
			modbus_buf_busy_a0 = 0;								// Снимаем флаг занятости
					
			if (modbus_buf_callback_f_a0 != 0) {				// Если ссылка на Callback-функцию есть
				modbus_buf_callback_f_a0();						// Вызов Callback-функции
			}
		}
		
		if (modbus_buf_pos_a0 == MODBUS_BUF_SIZE_A0) {			// Если превышен размер буфера
			IE2 &= ~UCA0RXIE;									// Запрет прерываний от RX
			
			modbus_buf_busy_a0 = 0;								// Сброс флага занятости
			modbus_buf_err_a0 = MODBUS_ERR_BUF;					// Устанавливаем ошибку
			
			if (modbus_buf_callback_f_a0 != 0) {				// Если ссылка на Callback-функцию есть
				modbus_buf_callback_f_a0();						// Вызов Callback-функции
			}
			
			return ;
		}
		
		modbus_tmr_short_int_a0 = 0;						// Сброс флага "прошедшости" интервала 1,5симв.
		
		TA0CTL |= MC_1 + TACLR;								// Запуск таймера
		
		modbus_buf_str_a0[modbus_buf_pos_a0++] = UCA0RXBUF;	// Загоняем в буфер принятый байт
	}
}

//------------------------------------------------------------------------------------------
void MODBUS_TX_Service_A0() {
	if ((IE2 & UCA0TXIE) && (IFG2 & UCA0TXIFG)) {
		if (modbus_buf_pos_a0 < modbus_buf_len_a0) {
			UCA0TXBUF = modbus_buf_str_a0[modbus_buf_pos_a0++];
		} else {
			IE2 &= ~UCA0TXIE;														// Запрет прерываний
			
			TA0CCTL0 = CCIE;
			TA0CCTL1 = 0;
			
			TA0CCR0 = MODBUS_TMR_INT_LONG_A0;
			TA0CTL |= MC_1 + TACLR;
		}
	}
}

//------------------------------------------------------------------------------------------
void MODBUS_CCR0_Service_A0() {
	TA0CTL &= ~MC_1;														// Остановка таймера
	TA0CCTL0 = 0;
	TA0CCTL1 = 0;

	switch (modbus_tmr_f_a0) {
		case MODBUS_TMRF_LISTEN:		// Функция контроля интервала 3,5симв. при приеме
			// Конец кадра
			IE2 &= ~UCA0RXIE;																			// Запрет прерываний
			
			modbus_buf_busy_a0 = 0;																		// Установка флага свободности
			
			if (modbus_buf_pos_a0 < 5)																	// Если принято меньше 5 байт
				modbus_buf_err_a0 = MODBUS_ERR_FRAME_LEN;												// Устанавливаем ошибку
			
			modbus_buf_crc_a0 = 0x0000;																	// Сброс буфера CRC
			modbus_buf_crc_a0 |= modbus_buf_str_a0[modbus_buf_pos_a0 - 1] << 8;							// Загоняем старшит байт в буфер
			modbus_buf_crc_a0 |= modbus_buf_str_a0[modbus_buf_pos_a0 - 2];								// Загоняем младший байт в буфер
			
			if (modbus_buf_crc_a0 != (*modbus_crc_f_a0)(modbus_buf_str_a0, modbus_buf_pos_a0 - 2)) {	// Если CRC принятое != CRC вычисленное
				modbus_buf_err_a0 = MODBUS_ERR_CRC;														// Устанавливаем ошибку
			}
			
			if (modbus_buf_str_a0[0] > 0 && modbus_buf_str_a0[0] != modbus_buf_addr_a0)	{				// Если адрес конкретный и не наш
				modbus_buf_err_a0 = MODBUS_ERR_ADDR;													// Устанавливаем ошибку
			}
			
			if (modbus_buf_callback_f_a0 != 0) {														// Если ссылка на Callback-функцию есть
				(*modbus_buf_callback_f_a0)();															// Вызов Callback-функции
			}
		return;
		case MODBUS_TMRF_WRITE:
			// Конец кадра
			modbus_buf_busy_a0 = 0;														// Установка флага свободности
			
			if (modbus_buf_callback_f_a0 != 0) {										// Если ссылка на Callback-функцию есть
				modbus_buf_callback_f_a0();												// Вызов Callback-функции
			}
		return;
		case MODBUS_TMRF_DELAY_RX:		// Функция генерации временного промежутка перед приемом		
			modbus_buf_pos_a0 = 0;														// Позиция курсора в буфере = 0
			modbus_tmr_f_a0 = MODBUS_TMRF_LISTEN;										// Функция таймера = прослушка
			
			modbus_buf_busy_a0 = 1;														// Установка флага занятости
			modbus_buf_err_a0 = 0;														// Сброс ошибки
			modbus_tmr_short_int_a0 = 0;												// Сброс флага "прошедшости" интервала 1,5симв.
			
			IFG2 &= ~UCA0RXIFG;															// Сброс флага прерывания по RX
			IE2 |= UCA0RXIE;															// Разрешение прерывания по RX
		return;
		case MODBUS_TMRF_DELAY_TX:		// Функция генерации временного промежутка перед передачей
			modbus_buf_pos_a0 = 0;														// Позиция курсора в буфере = 0
			modbus_tmr_f_a0 = MODBUS_TMRF_WRITE;										// Функция таймера = запись
			
			modbus_buf_busy_a0 = 1;														// Установка флага занятости
			modbus_buf_err_a0 = 0;														// Сброс ошибки
			
			IE2 |= UCA0TXIE;															// Разрешение прерывания по TX
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
