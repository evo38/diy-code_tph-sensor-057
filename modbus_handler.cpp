#include <msp430.h>
//------------------------------------------------------------------------------------------------------------

#include "settings.h"
#include "crc16_modbus_table.h"
#include "modbus_rtu.h"
#include "bme280.h"
#include "main.h"

//------------------------------------------------------------------------------------------------------------
#include "modbus_handler.h"

//------------------------------------------------------------------------------------------------------------
#define RS_INIT				{ RS_DIR |= RS_BIT; }
#define RS_READ				{ RS_OUT &= ~RS_BIT; }
#define RS_WRITE			{ RS_OUT |= RS_BIT; }

//------------------------------------------------------------------------------------------------------------
void modbus_handler_settings();
void modbus_handler_listen();
void modbus_handler_answer();
void modbus_handler_read();
void modbus_handler_write06(bool NoAnswer = false);

//------------------------------------------------------------------------------------------------------------
extern unsigned int			MODBUS_TMR_INT_LONG_A0, MODBUS_TMR_INT_SHORT_A0;
extern char					MODBUS_A0_BR0, MODBUS_A0_BR1, MODBUS_A0_MCTL, MODBUS_A0_PEN, MODBUS_A0_PAR, MODBUS_A0_MSB, MODBUS_A0_SPB;

extern struct BME_Result 	gRes;
extern bool					bme_error;


//------------------------------------------------------------------------------------------------------------
void modbus_handler_init()
{
	RS_INIT
	
	MODBUS_StopTransaction();
	modbus_handler_settings();
	MODBUS_Init((char) settings_get_modbus_address(), &FAST_CRC16);
	
	modbus_handler_listen();
} // modbus_handler_init()

//------------------------------------------------------------------------------------------------------------
void modbus_handler_settings()
{
	uint8_t bits = 10;

	MODBUS_A0_PEN = 0;

	if (settings_get_modbus_parity() == MB_PARITY_ODD || settings_get_modbus_parity() == MB_PARITY_EVEN)
	{
		MODBUS_A0_PEN = 1;
		MODBUS_A0_PAR = 0;

		if (settings_get_modbus_parity() == MB_PARITY_EVEN) {
			MODBUS_A0_PAR = 1;
		}

		bits++;
	}


	MODBUS_A0_SPB = 1;

	if (settings_get_modbus_stop() == MB_STOP_2)
	{
		MODBUS_A0_SPB = 2;
		bits++;
	}


	MODBUS_A0_MSB = 0;

	//if (settings[MBSET_MSB] == MB_MSB_BG_ENDIAN)
		//MODBUS_A0_MSB = 1;


	switch (settings_get_modbus_speed())
	{
		case MB_SPEED_1200:
			MODBUS_A0_BR0 = 0x41;
			MODBUS_A0_BR1 = 0x03;
			MODBUS_A0_MCTL = 0x00;
			MODBUS_TMR_INT_LONG_A0 = 365*bits;
			MODBUS_TMR_INT_SHORT_A0 = 156*bits;
		break;
		case MB_SPEED_2400:
			MODBUS_A0_BR0 = 0xA1;
			MODBUS_A0_BR1 = 0x01;
			MODBUS_A0_MCTL = 0x00;
			MODBUS_TMR_INT_LONG_A0 = 182*bits;
			MODBUS_TMR_INT_SHORT_A0 = 78*bits;
		break;
		case MB_SPEED_4800:
			MODBUS_A0_BR0 = 0xD0;
			MODBUS_A0_BR1 = 0x00;
			MODBUS_A0_MCTL = 0x00;
			MODBUS_TMR_INT_LONG_A0 = 91*bits;
			MODBUS_TMR_INT_SHORT_A0 = 39*bits;
		break;
		case MB_SPEED_9600:
			MODBUS_A0_BR0 = 0x68;
			MODBUS_A0_BR1 = 0x00;
			MODBUS_A0_MCTL = 0x0C;
			MODBUS_TMR_INT_LONG_A0 = 46*bits;
			MODBUS_TMR_INT_SHORT_A0 = 23*bits;
			//MODBUS_A0_BR0 = 0x41;
			//MODBUS_A0_BR1 = 0x03;
			//MODBUS_TMR_INT_LONG_A0 = 365*bits;
			//MODBUS_TMR_INT_SHORT_A0 = 156*bits;
		break;
		case MB_SPEED_19200:
			MODBUS_A0_BR0 = 0x34;
			MODBUS_A0_BR1 = 0x00;
			MODBUS_A0_MCTL = 0x04;
			MODBUS_TMR_INT_LONG_A0 = 23*bits;
			MODBUS_TMR_INT_SHORT_A0 = 10*bits;
		break;
		case MB_SPEED_38400:
			MODBUS_A0_BR0 = 0x1A;
			MODBUS_A0_BR1 = 0x00;
			MODBUS_A0_MCTL = 0x0C;
			MODBUS_TMR_INT_LONG_A0 = 219;
			MODBUS_TMR_INT_SHORT_A0 = 94;
		break;
		case MB_SPEED_56000:
			MODBUS_A0_BR0 = 0x12;
			MODBUS_A0_BR1 = 0x00;
			MODBUS_A0_MCTL = 0x0C;
			MODBUS_TMR_INT_LONG_A0 = 219;
			MODBUS_TMR_INT_SHORT_A0 = 94;
		break;
		case MB_SPEED_115200:
			MODBUS_A0_BR0 = 0x08;
			MODBUS_A0_BR1 = 0x00;
			MODBUS_A0_MCTL = 0x0E;
			MODBUS_TMR_INT_LONG_A0 = 219;
			MODBUS_TMR_INT_SHORT_A0 = 84;
		break;
		case MB_SPEED_256000:
			MODBUS_A0_BR0 = 0x04;
			MODBUS_A0_BR1 = 0x00;
			MODBUS_A0_MCTL = 0x08;
			MODBUS_TMR_INT_LONG_A0 = 219;
			MODBUS_TMR_INT_SHORT_A0 = 84;
		break;
	}
} // modbus_handler_settings()

//------------------------------------------------------------------------------------------------------------
void modbus_handler_listen()
{
	RS_READ
	MODBUS_Listen(&modbus_handler_answer, true);
} // modbus_handler_listen()

//------------------------------------------------------------------------------------------------------------
void modbus_handler_answer()
{
	if (MODBUS_GetAddress() && MODBUS_GetAddress() != (char) settings_get_modbus_address())
	{
		modbus_handler_listen();
		return ;
	}
	
	if (!MODBUS_GetAddress())	// Общая для всех устройств команда
	{
		/*Не обрабатываем*/
		modbus_handler_listen();
		return ;
	}
	
	
	char* buffer = MODBUS_GetAnswer();
	
	if (MODBUS_Error() > 0)
	{	
		RS_WRITE
		
		*buffer = MODBUS_Error();
		MODBUS_Send((char) settings_get_modbus_address(), MODBUS_GetFunction() | 0x80, 0, 1, &modbus_handler_listen);
		return ;
	}
	
	// Команды
	if (MODBUS_GetFunction() == 0x03)																// Функция чтения из нескольких регистров (3)
	{
		if (MODBUS_GetLength() < 4)
			goto _goListen;

		modbus_handler_read();
		return ;
	}
	else if (MODBUS_GetFunction() == 0x06)															// Функция записи в несколько регистров (16)
	{
		if (MODBUS_GetLength() < 4)
			goto _goListen;

		modbus_handler_write06();
		return ;
	}
	
	_goListen:
		modbus_handler_listen();
} // modbus_handler_answer()

//------------------------------------------------------------------------------------------------------------
void modbus_handler_read()
{
	char* buffer = MODBUS_GetAnswer() + 1;
	char* ptr;
	char bufCount = 1,
		 bufFunction = MODBUS_GetFunction();
	
	unsigned int 	RegisterAddress,															// Адрес регистра
					RegisterNumber;																// Количество регистров для действий
	
	int buf;
	
	// Загоняем принятый адрес регистра в буфер
	RegisterAddress = MODBUS_GetAnswer()[0];
	RegisterAddress <<= 8;
	RegisterAddress |= MODBUS_GetAnswer()[1];

	// Загоняем принятое количество регистров в буфер
	RegisterNumber = MODBUS_GetAnswer()[2];
	RegisterNumber <<= 8;
	RegisterNumber |= MODBUS_GetAnswer()[3];
	
	if (RegisterAddress >= 0x1000 && RegisterAddress <= 0x100F)
	{
		RegisterNumber += RegisterAddress;
		
		for (; RegisterAddress < RegisterNumber; RegisterAddress++)
		{
			switch (RegisterAddress)
			{
				case 0x1000:
					*buffer++ = 0;
					*buffer++ = bme_error ? 1 : 0;
				break;
				case 0x1001:
					buf = (int) gRes.T;
					
					if (gRes.T - ((float) buf) >= 0.5)
						buf++;
					
					*buffer++ = (char) ((buf & 0xFF00) >> 8);
					*buffer++ = (char) (buf & 0x00FF);
				break;
				case 0x1002:
					buf = (unsigned int) gRes.H;
					
					if (gRes.H - ((float) buf) >= 0.5)
						buf++;
					
					*buffer++ = (char) ((buf & 0xFF00) >> 8);
					*buffer++ = (char) (buf & 0x00FF);
				break;
				case 0x1003:
					buf = (unsigned int) gRes.P;
					
					if (gRes.P - ((float) buf) >= 0.5)
						buf++;
					
					*buffer++ = (char) ((buf & 0xFF00) >> 8);
					*buffer++ = (char) (buf & 0x00FF);
				break;
				
				// T
				case 0x1004:
					ptr = (char*) &(gRes.T);
					*buffer++ = *(ptr + 1);
					*buffer++ = *(ptr + 0);
				break;
				case 0x1005:
					ptr = (char*) &(gRes.T);
					*buffer++ = *(ptr + 3);
					*buffer++ = *(ptr + 2);
				break;
				
				// H
				case 0x1006:
					ptr = (char*) &(gRes.H);
					*buffer++ = *(ptr + 1);
					*buffer++ = *(ptr + 0);
				break;
				case 0x1007:
					ptr = (char*) &(gRes.H);
					*buffer++ = *(ptr + 3);
					*buffer++ = *(ptr + 2);
				break;
				
				// P
				case 0x1008:
					ptr = (char*) &(gRes.P);
					*buffer++ = *(ptr + 1);
					*buffer++ = *(ptr + 0);
				break;
				case 0x1009:
					ptr = (char*) &(gRes.P);
					*buffer++ = *(ptr + 3);
					*buffer++ = *(ptr + 2);
				break;
				
				// P_Pa
				case 0x100A:
					ptr = (char*) &(gRes.P_Pa);
					*buffer++ = *(ptr + 1);
					*buffer++ = *(ptr + 0);
				break;
				case 0x100B:
					ptr = (char*) &(gRes.P_Pa);
					*buffer++ = *(ptr + 3);
					*buffer++ = *(ptr + 2);
				break;
			}
			
			bufCount += 2;
		}

		buffer = MODBUS_GetAnswer();
		*buffer = bufCount;																		// Устанавливаем 0-ой байт в буфере ответа = bufCount (количество байт ответа)
	}
	else if (RegisterAddress >= 0x2005 && RegisterAddress <= 0x200B)									// Если начальный адрес регистра лежит в области настроек
	{
		RegisterNumber += RegisterAddress;														// Количество регистров теперь обозначает конечный элемент массива
		
		for (; RegisterAddress < RegisterNumber; RegisterAddress++)								// Перебираем все элементы массива (диапазон адресов)
		{
			if (RegisterAddress < 0x200A)
				*buffer++ = 0;
			
			switch (RegisterAddress)
			{
				case 0x2005: *buffer++ = (char) settings_get_modbus_address(); break;
				case 0x2006: *buffer++ = (char) settings_get_modbus_speed(); break;
				case 0x2007: *buffer++ = (char) settings_get_modbus_parity(); break;
				case 0x2008: *buffer++ = (char) settings_get_modbus_stop(); break;
				case 0x2009: *buffer++ = (char) settings_get_interval(); break;
			}
			
			bufCount += 2;
		}

		buffer = MODBUS_GetAnswer();
		*buffer = bufCount;																		// Устанавливаем 0-ой байт в буфере ответа = bufCount (количество байт ответа)
	}
	else																					// Если неверно указан регистр
	{
		bufFunction |= 0x80;																// Устанавливаем флаг ошибки
		*buffer = 0x02;																		// Код ошибки: адрес данных, указанный в запросе не доступен данному подчиненному
		bufCount = 1;																		// Размер ответа = 1
	}

	RS_WRITE
	MODBUS_Send((char) settings_get_modbus_address(), bufFunction, 0, bufCount, &modbus_handler_listen, false);	// Отправляем ответ
} // modbus_handler_read()

//------------------------------------------------------------------------------------------------------------
void modbus_handler_write06(bool NoAnswer)
{
	char 	bufAnswer[4],																	// Буфер ответа
			bufFunction = MODBUS_GetFunction(),												// Загоняем код функции в буфер
			bufCount = 0;																	// Счетчик байт ответа
			
	unsigned int 	RegisterAddress;														// Адрес регистра
	
	if (!MODBUS_GetAddress())
		NoAnswer = true;
	
	if (!NoAnswer)
		RS_WRITE
	
	if (MODBUS_GetLength() != 4)													// Если количество принятых байт данных не равно 4
	{
		bufFunction |= 0x80;																// Устанавливаем флаг ошибки
		bufAnswer[0] = 0x03;																// Код ошибки: величина содержащаяся в поле данных запроса является не допустимой величиной для подчиненного
		bufCount = 1;																		// Размер буфера
	}
	else																					// Если количество принятых байт данных равно 4 (2 байта - адрес регистра, 2 байта - данные)
	{
		// Загоняем принятый адрес регистра в буфер
		RegisterAddress = MODBUS_GetAnswer()[0];
		RegisterAddress <<= 8;
		RegisterAddress |= MODBUS_GetAnswer()[1];
		
		bool	errFlag = false;															// Флаг ошибки несоотвествия принятых данных условиям
		
		char 	byte_1 = MODBUS_GetAnswer()[2],									// Загоняем первый байт данных в буфер
				byte_2 = MODBUS_GetAnswer()[3];									// Загоняем второй байт данных в буфер
		
		if (RegisterAddress >= 0x2005 && RegisterAddress <= 0x200B)							// Адрес регистра лежит в поле настроек по 0x2013
		{
			RegisterAddress -= 0x2005;														// Приводим адрес к счетчику массива
			
			uint8_t  settings_[] = { settings_get_modbus_address(), settings_get_modbus_speed(), settings_get_modbus_parity(), settings_get_modbus_stop(), settings_get_interval() };
			
			switch (RegisterAddress)														// Условие по адресу регистра
			{
				case 0x0:	if (byte_1 || !byte_2 || byte_2 > 247) errFlag = true; else settings_[0] = (uint8_t) byte_2; break;		// Адрес устройства
				case 0x1:	if (byte_1 || byte_2 > 8) errFlag = true; else settings_[1] = (uint8_t) byte_2; break;					// Скорость обмена с ПК
				case 0x2:	if (byte_1 || byte_2 > 2) errFlag = true; else settings_[2] = (uint8_t) byte_2; break;					// Паритет при обмене с ПК
				case 0x3:	if (byte_1 || byte_2 != 1 && byte_2 != 2) errFlag = true; else settings_[3] = (uint8_t) byte_2; break;	// Количество стоп-битов
				case 0x4:	if (byte_1 || !byte_2) errFlag = true; else settings_[4] = (uint8_t) byte_2; break;						// Период опроса датчиков температуры
			}
			
			if (errFlag)																	// Есть ошибка?
			{
				bufFunction |= 0x80;														// Устанавливаем флаг ошибки
				bufAnswer[0] = 0x03;														// Код ошибки: величина содержащаяся в поле данных запроса является не допустимой величиной для подчиненного
				bufCount = 1;																// Размер ответа = 1
			}
			else
			{
				bufAnswer[0] = MODBUS_GetAnswer()[0];								// В ответе повторяем
				bufAnswer[1] = MODBUS_GetAnswer()[1];								//					  адрес регистра
				bufAnswer[2] = byte_1;														// и записываемое значение
				bufAnswer[3] = byte_2;														//			  			   в регистра
				bufCount = 4;
				
				// Применение и сохранение настроек в ПЗУ
				settings_set(settings_[0], settings_[1], settings_[2], settings_[3], settings_[4]);
				
				return MODBUS_Send((char) settings_get_modbus_address(), bufFunction, bufAnswer, bufCount, &modbus_handler_init, false); 		// Отправляем ответ
			}
		}
		else if (RegisterAddress == 0x3002)
		{
			bufAnswer[0] = MODBUS_GetAnswer()[0];								// В ответе повторяем
			bufAnswer[1] = MODBUS_GetAnswer()[1];								//					  адрес регистра
			bufAnswer[2] = 0x00;														// и записываемое количество
			bufAnswer[3] = 0x01;														//			  			   примененных регистров
			bufCount = 4;
			
			return MODBUS_Send(settings_get_modbus_address(), bufFunction, bufAnswer, bufCount, &default_settings, false); 		// Отправляем ответ
		}
		else if (RegisterAddress == 0x3003)
		{
			bufAnswer[0] = MODBUS_GetAnswer()[0];								// В ответе повторяем
			bufAnswer[1] = MODBUS_GetAnswer()[1];								//					  адрес регистра
			bufAnswer[2] = 0x00;														// и записываемое количество
			bufAnswer[3] = 0x01;														//			  			   примененных регистров
			bufCount = 4;
			
			return MODBUS_Send((char) settings_get_modbus_address(), bufFunction, bufAnswer, bufCount, &reset_device, false); 		// Отправляем ответ
		}
		else
		{
			bufFunction |= 0x80;															// Устанавливаем флаг ошибки
			bufAnswer[0] = 0x02;															// Код ошибки: адрес данных, указанный в запросе не доступен данному подчиненному
			bufCount = 1;																	// Количество байт ответа = 1
		}
	}
	
	if (!NoAnswer)																			// Ответ нужен?
		MODBUS_Send((char) settings_get_modbus_address(), bufFunction, bufAnswer, bufCount, &modbus_handler_listen, false);
	else																					// Ответ не нужен?
		modbus_handler_listen();															// Ставим на прослушку
} // modbus_handler_write6()

//------------------------------------------------------------------------------------------------------------
