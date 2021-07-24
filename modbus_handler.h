#pragma once

//------------------------------------------------------------------------------------------------------------
#define RS_BIT				BIT0
#define RS_DIR				P1DIR
#define RS_OUT				P1OUT

//------------------------------------------------------------------------------------------------------------
#define MBSET_ADDRESS		0
#define MBSET_SPEED			1
#define MBSET_PARITY		2
#define MBSET_STOP			3
#define MBSET_MSB			4								// Порядок битов

#define MB_SPEED_1200		0
#define MB_SPEED_2400		1
#define MB_SPEED_4800		2
#define MB_SPEED_9600		3
#define MB_SPEED_19200		4
#define MB_SPEED_38400		5
#define MB_SPEED_56000		6
#define MB_SPEED_115200		7
#define MB_SPEED_256000		8
#define MB_SPEED_MAX		MB_SPEED_256000

#define MB_PARITY_NONE		0
#define MB_PARITY_ODD		1
#define MB_PARITY_EVEN		2
#define MB_PARITY_MAX		MB_PARITY_EVEN

#define MB_STOP_1			1
#define MB_STOP_2			2
#define MB_STOP_MAX			MB_STOP_2

#define MB_MSB_LT_ENDIAN	0								// Младший - первый (по умолч.)
#define MB_MSB_BG_ENDIAN	1								// Старший - первый
#define MB_MSB_MAX			MB_MSB_BG_ENDIAN

//------------------------------------------------------------------------------------------------------------
void modbus_handler_init();