#include <msp430.h>
#include "i2c_master.h"


#ifdef USE_I2C_MASTER_B0
	uint8_t	i2c_master_buf_str_b0[I2C_MASTER_B0_BUF_SIZE];		// Выделяем буфер приема/передачи
	uint8_t	i2c_master_buf_len_b0,								// Выделяем буфер длины строки (в буфере)
			i2c_master_buf_pos_b0;								// Выделяем буфер текущей позиции в строке (буфере)
			
	uint8_t	i2c_master_buf_busy_b0;								// Флаги модуля: BIT0=занятость модуля

	void	(*i2c_master_buf_callback_f_b0)() = 0;				// Буфер хранения указателя на Callback-функцию
	
	uint8_t	i2c_master_buf_inited_b0 = 0;
#endif

#ifdef USE_I2C_MASTER_B1
	uint8_t	i2c_master_buf_str_b1[I2C_MASTER_B1_BUF_SIZE];		// Выделяем буфер приема/передачи
	uint8_t	i2c_master_buf_len_b1,								// Выделяем буфер длины строки (в буфере)
			i2c_master_buf_pos_b1;								// Выделяем буфер текущей позиции в строке (буфере)
			
	uint8_t	i2c_master_buf_busy_b1;								// Флаги модуля: BIT0=занятость модуля

	void	(*i2c_master_buf_callback_f_b1)() = 0;				// Буфер хранения указателя на Callback-функцию
#endif



void I2C_Master_Init(unsigned int Div, uint8_t Module)
{
	#ifdef USE_I2C_MASTER_B0
		if (Module == I2C_MASTER_B0)
		{
			I2C_MASTER_B0_PSEL |= I2C_MASTER_B0_SDA + I2C_MASTER_B0_SCL;					// Настройка порта
			
			#ifdef I2C_MASTER_B0_PSEL2
				I2C_MASTER_B0_PSEL2 |= I2C_MASTER_B0_SDA + I2C_MASTER_B0_SCL;				// Настройка порта
			#endif
			
			UCB0CTL1 = UCSWRST;                        										// Модуль USCI - сброшен
			
			UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;      										// Модуль USCI как: I2C Master в синхронном режиме
			UCB0CTL1 = I2C_MASTER_B0_SEL + UCSWRST;    										// Выбор тактового сигнала, USCI удерживаем в состоянии сброса
			
			UCB0BR0 = (uint8_t) (Div & 0xFF);                        							// Деление частоты тактового сигнала, младший байт
			UCB0BR1 = (uint8_t) ((Div >> 8) & 0xFF);											// Деление частоты тактового сигнала, старший байт
			
			UCB0CTL1 &= ~UCSWRST;                       									// Выводим USCI в рабочий режим
			
			UCB0I2CIE = UCNACKIE;															// Разрешаем прерывания по NACK
		}
	#endif
	
	#if defined(USE_I2C_MASTER_B0) && defined(USE_I2C_MASTER_B1)
		else
	#endif

	#ifdef USE_I2C_MASTER_B1
		if (Module == I2C_MASTER_B1)
		{
			I2C_MASTER_B1_PSEL |= I2C_MASTER_B1_SDA + I2C_MASTER_B1_SCL;					// Настройка порта
			
			#ifdef I2C_MASTER_B1_PSEL2
				I2C_MASTER_B1_PSEL2 |= I2C_MASTER_B1_SDA + I2C_MASTER_B1_SCL;				// Настройка порта
			#endif
			
			UCB1CTL1 = UCSWRST;                        										// Модуль USCI - сброшен
			
			UCB1CTL0 = UCMST + UCMODE_3 + UCSYNC;      										// Модуль USCI как: I2C Master в синхронном режиме
			UCB1CTL1 = I2C_MASTER_B1_SEL + UCSWRST;    										// Выбор тактового сигнала, USCI удерживаем в состоянии сброса
			
			UCB1BR0 = (uint8_t) (Div & 0xFF);                        							// Деление частоты тактового сигнала, младший байт
			UCB1BR1 = (uint8_t) ((Div >> 8) & 0xFF);											// Деление частоты тактового сигнала, старший байт
			
			UCB1CTL1 &= ~UCSWRST;                       									// Выводим USCI в рабочий режим
			
			UCB1I2CIE = UCNACKIE;															// Разрешаем прерывания по NACK
		}
	#endif
}

//------------------------------------------------------------------------------------------
void I2C_Master_Transmit(uint8_t Address, uint8_t* Data, uint8_t Nd, void (*Callback)(), uint8_t Module)
{
	/**
	 * Функция отправки Nd байт ведомому с адресом Address
	 * @param {uint8_t} {Address} Адрес ведомого
	 * @param {uint8_t*} {Data} Указатель на массив данных
	 * @param {uint8_t} {Nd} Количество байт для передачи в массиве
	 * @param {void (*)()} {Callback} Callback-функция
	 * @param {uint8_t} {Module} Выбор модуля
	 * @return {void}}
	 */
	
	_BIS_SR(GIE);
	
	//volatile 
			
	#ifdef USE_I2C_MASTER_B0
		if (Module == I2C_MASTER_B0)
		{
			UCB0I2CSA = Address;															// Устанавливаем адрес ведомого
			
			if (Data)
			{
				uint8_t i = 0;
				while (i < Nd) {															// Копируем строку для передачи
					i2c_master_buf_str_b0[i] = Data[i];										// в буфер
					i++;
				}
			}
			
			i2c_master_buf_len_b0 = Nd;														// Помещаем в буфер количество байт для передачи
			i2c_master_buf_callback_f_b0 = Callback;										// Помещаем в буфер указатель на Callback-функцию
			i2c_master_buf_pos_b0 = 0;														// Текущая позиция в буфере передачи
			i2c_master_buf_busy_b0 = 0x01;													// Модуль занят
			
			if (!i2c_master_buf_inited_b0)
				i2c_master_buf_inited_b0++;
			
			IE2 &= ~UCB0RXIE;																// Запрет прерываний по RX
			IE2 |= UCB0TXIE;                            									// Разрешение прерываний по TX
			UCB0CTL1 |= UCTR + UCTXSTT;                 									// USCI I2C - на передачу, формируем старт-условие
		}
	#endif
	
	#if defined(USE_I2C_MASTER_B0) && defined(USE_I2C_MASTER_B1)
		else
	#endif

	#ifdef USE_I2C_MASTER_B1
		if (Module == I2C_MASTER_B1)
		{
			UCB1I2CSA = Address;															// Устанавливаем адрес ведомого
			
			if (Data)
			{
				uint8_t i = 0;
				while (i < Nd)																// Копируем строку для передачи
					i2c_master_buf_str_b1[i] = Data[i++];									// в буфер
			}
			
			i2c_master_buf_len_b1 = Nd;														// Помещаем в буфер количество байт для передачи
			i2c_master_buf_callback_f_b1 = Callback;										// Помещаем в буфер указатель на Callback-функцию
			i2c_master_buf_pos_b1 = 0;														// Текущая позиция в буфере передачи
			i2c_master_buf_busy_b1 = 0x01;													// Модуль занят
			
			UC1IE &= ~UCB1RXIE;																// Запрет прерываний по RX
			UC1IE |= UCB1TXIE;                            									// Разрешение прерываний по TX
			UCB1CTL1 |= UCTR + UCTXSTT;                 									// USCI I2C - на передачу, формируем старт-условие
		}
	#endif
}

//------------------------------------------------------------------------------------------
void I2C_Master_Receive(uint8_t Address, uint8_t Nd, void (*Callback)(), uint8_t Module)
{
	/**
	 * Функция приема Nd байт от ведомого
	 * @param {uint8_t} {Address} Адрес ведомого
	 * @param {uint8_t} {Nd} Кол-во байт для приема
	 * @param {void (*)()} {Callback} Callback-функция
	 * @param {uint8_t} {Module} Выбор модуля
	 * @return {void}
	 */
	
	_BIS_SR(GIE);
	
	#ifdef USE_I2C_MASTER_B0
		if (Module == I2C_MASTER_B0)
		{
			i2c_master_buf_pos_b0 = 0;														// Текущая позиция в буфере
			i2c_master_buf_len_b0 = Nd;														// Помещаем в буфер количество байт для приема
			i2c_master_buf_callback_f_b0 = Callback;										// Помещаем в буфер указатель на Callback-функцию
			i2c_master_buf_busy_b0 = 0x01;													// Модуль занят
			
			if (i2c_master_buf_inited_b0)
				i2c_master_buf_inited_b0 = 2;
			
			IE2 &= ~UCB0TXIE;																// Запрет прерываний по TX
			IE2 |= UCB0RXIE;                           										// Разрешение прерываний по RX
			
			UCB0CTL1 &= ~UCTR;																// USCI I2C модуль - на прием
			UCB0CTL1 |= UCTXSTT;                      										// Посылаем старт-условие
		}
	#endif
	
	#if defined(USE_I2C_MASTER_B0) && defined(USE_I2C_MASTER_B1)
		else
	#endif

	#ifdef USE_I2C_MASTER_B1
		if (Module == I2C_MASTER_B1)
		{
			i2c_master_buf_pos_b1 = 0;														// Текущая позиция в буфере
			i2c_master_buf_len_b1 = Nd;														// Помещаем в буфер количество байт для приема
			i2c_master_buf_callback_f_b1 = Callback;										// Помещаем в буфер указатель на Callback-функцию
			i2c_master_buf_busy_b1 = 0x01;													// Модуль занят
			
			UC1IE &= ~UCB1TXIE;																// Запрет прерываний по TX
			UC1IE |= UCB1RXIE;                           									// Разрешение прерываний по RX
			
			UCB1CTL1 &= ~UCTR;																// USCI I2C модуль - на прием
			UCB1CTL1 |= UCTXSTT;                      										// Посылаем старт-условие
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
				IE2 &= ~(UCB0TXIE);															// Запрет прерываний
				IFG2 &= ~UCB0TXIFG;															// Сброс флага прерывания по TX
				UCB0CTL1 |= UCTXSTP;														// Отправляем стоп-условие
				
				while (UCB0CTL1 & UCTXSTP);													// Ожидание окончания стоп-условия
				
				i2c_master_buf_busy_b0 &= 0xFE;
				
				if (i2c_master_buf_callback_f_b0 != 0)										// Если задана Callback-функция
					i2c_master_buf_callback_f_b0();											// вызываем ее
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
				IE2 &= ~(UCB0RXIE);															// Запрет прерываний от RX
				UCB0CTL1 |= UCTXSTP;														// Отправляем стоп-условие
				
				while (UCB0CTL1 & UCTXSTP);													// Ожидание окончания стоп-условия
				
				i2c_master_buf_busy_b0 &= 0xFE;
				
				if (i2c_master_buf_callback_f_b0 != 0)										// Если задана Callback-функция
					i2c_master_buf_callback_f_b0();											// вызываем ее
			}
		}
	}
	
	//--------------------------------------------------------------------------------------
	void I2C_MASTER_RX_Service_B0()
	{
		if (UCB0STAT & UCNACKIFG)															// Отправить стоп-условие, если ведомый отправил NACK
		{
			UCB0STAT &= ~UCNACKIFG;															// Сброс флага прерывания по NACK
			
			UCB0CTL1 |= UCTXSTP;															// Отправляем стоп-условие
			while (UCB0CTL1 & UCTXSTP);														// Ожидание окончания стоп-условия
			
			i2c_master_buf_busy_b0 &= 0xFE;

			if (i2c_master_buf_callback_f_b0 != 0)											// Если задана Callback-функция
				i2c_master_buf_callback_f_b0();												// вызываем ее
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
				UC1IE &= ~(UCB1TXIE);														// Запрет прерываний
				UC1IFG &= ~UCB1TXIFG;														// Сброс флага прерывания по TX
				UCB1CTL1 |= UCTXSTP;														// Отправляем стоп-условие
				
				while (UCB1CTL1 & UCTXSTP);													// Ожидание окончания стоп-условия
				
				i2c_master_buf_busy_b1 &= 0xFE;
				
				if (i2c_master_buf_callback_f_b1 != 0)										// Если задана Callback-функция
					i2c_master_buf_callback_f_b1();											// вызываем ее
			}
		}
		
		if ((UC1IE & UCB1RXIE) && (UC1IFG & UCB1RXIFG))
		{
			if (i2c_master_buf_pos_b1 < i2c_master_buf_len_b1)
				i2c_master_buf_str_b1[i2c_master_buf_pos_b1++] = UCB1RXBUF;
			else
			{
				UC1IE &= ~(UCB1RXIE);														// Запрет прерываний от RX
				UCB1CTL1 |= UCTXSTP;														// Отправляем стоп-условие
				
				while (UCB1CTL1 & UCTXSTP);													// Ожидание окончания стоп-условия
				
				i2c_master_buf_busy_b1 &= 0xFE;
				
				if (i2c_master_buf_callback_f_b1 != 0)										// Если задана Callback-функция
					i2c_master_buf_callback_f_b1();											// вызываем ее
			}
		}
	}
	
	//--------------------------------------------------------------------------------------
	void I2C_MASTER_RX_Service_B1()
	{
		if (UCB1STAT & UCNACKIFG)															// Отправить стоп-условие, если ведомый отправил NACK
		{
			UCB1STAT &= ~UCNACKIFG;
			
			UCB1CTL1 |= UCTXSTP;															// Отправляем стоп-условие
			while (UCB1CTL1 & UCTXSTP);														// Ожидание окончания стоп-условия
			
			i2c_master_buf_busy_b1 &= 0xFE;
			
			if (i2c_master_buf_callback_f_b1 != 0)											// Если задана Callback-функция
				i2c_master_buf_callback_f_b1();												// вызываем ее
		}
	}
#endif