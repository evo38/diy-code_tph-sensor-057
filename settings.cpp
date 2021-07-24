#include <msp430.h>

//-------------------------------------------------------------------------------------------------------------
#include "settings.h"

//-------------------------------------------------------------------------------------------------------------
uint8_t settings_get_modbus_address()
{
	uint8_t*	flash_ptr = (uint8_t*) (0x1000);					// Segment C
	return *flash_ptr;
}

//-------------------------------------------------------------------------------------------------------------
uint8_t settings_get_modbus_speed()
{
	uint8_t*	flash_ptr = (uint8_t*) (0x1001);					// Segment C
	return *flash_ptr;
}

//-------------------------------------------------------------------------------------------------------------
uint8_t settings_get_modbus_parity()
{
	uint8_t*	flash_ptr = (uint8_t*) (0x1002);					// Segment C
	return *flash_ptr;
}

//-------------------------------------------------------------------------------------------------------------
uint8_t settings_get_modbus_stop()
{
	uint8_t*	flash_ptr = (uint8_t*) (0x1003);					// Segment C
	return *flash_ptr;
}

//-------------------------------------------------------------------------------------------------------------
uint8_t settings_get_interval()
{
	uint8_t*	flash_ptr = (uint8_t*) (0x1004);					// Segment C
	return *flash_ptr;
}

//-------------------------------------------------------------------------------------------------------------
void settings_set(uint8_t address, uint8_t speed, uint8_t parity, uint8_t stop, uint8_t interval)
{
	uint8_t* 	flash_ptr = (uint8_t*) (0x1000);					// Segment C
	
	FCTL3 = FWKEY;                            	// Clear Lock bit
	FCTL1 = FWKEY + ERASE;                      // Set Erase bit
	*flash_ptr = 0;                          	// Dummy write to erase Flash seg
	FCTL1 = FWKEY + WRT;                        // Set WRT bit for write operation
	
	*flash_ptr++ = address;
	*flash_ptr++ = speed;
	*flash_ptr++ = parity;
	*flash_ptr++ = stop;
	*flash_ptr++ = interval;
	
	FCTL1 = FWKEY;                            									// Clear WRT bit
	FCTL3 = FWKEY + LOCK;                       								// Set LOCK bit
}

//-------------------------------------------------------------------------------------------------------------
bool settings_is_init()
{
	FCTL2 = FWKEY + FSSEL_1 + FN0 + FN1 + FN2 + FN3 + FN4;             // MCLK/32 for Flash Timing Generator
	
	uint8_t address_ = *((uint8_t*) (0x1000));
	uint8_t speed_ = *((uint8_t*) (0x1001));
	
	if (speed_ > 8 || !address_ || address_ > 247)
	{
		settings_set_defaults();
		
		return false;
	}
	
	return true;
}

//-------------------------------------------------------------------------------------------------------------
void settings_set_defaults()
{
	settings_set(1, MODBUS_SPEED_9600, MODBUS_PARITY_NONE, MODBUS_STOP_1, 1);
}

//-------------------------------------------------------------------------------------------------------------
