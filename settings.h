#pragma once

//-------------------------------------------------------------------------------------------------------------
#include <stdint.h>

//-------------------------------------------------------------------------------------------------------------
#define MODBUS_SPEED_1200			0
#define MODBUS_SPEED_2400			1
#define MODBUS_SPEED_4800			2
#define MODBUS_SPEED_9600			3
#define MODBUS_SPEED_19200			4
#define MODBUS_SPEED_38400			5
#define MODBUS_SPEED_56000			6
#define MODBUS_SPEED_115200			7
#define MODBUS_SPEED_256000			8
		
#define MODBUS_PARITY_NONE			0
#define MODBUS_PARITY_ODD			1
#define MODBUS_PARITY_EVEN			2

#define MODBUS_STOP_1				1
#define MODBUS_STOP_2				2

//-------------------------------------------------------------------------------------------------------------
uint8_t settings_get_modbus_address();
uint8_t settings_get_modbus_speed();
uint8_t settings_get_modbus_parity();
uint8_t settings_get_modbus_stop();
uint8_t settings_get_interval();

void settings_set(uint8_t address, uint8_t speed, uint8_t parity, uint8_t stop, uint8_t interval);

bool settings_is_init();
void settings_set_defaults();
