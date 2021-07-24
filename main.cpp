#include <msp430.h>

//-------------------------------------------------------------------------------------------------------------
#include "main.h"

//-------------------------------------------------------------------------------------------------------------
#include "i2c_master.h"
#include "bme280.h"
#include "settings.h"
#include "modbus_rtu.h"
#include "modbus_handler.h"

//-------------------------------------------------------------------------------------------------------------
void bme_inited(bool status);
void bme_measured_first(bool status);
void bme_measured(bool status);

//-------------------------------------------------------------------------------------------------------------
uint8_t period_counter = 0;
uint8_t half_second_counter = 0;
bool	bme_error;
struct 	BME_Result gRes;
bool	bme_try_to_init = false;
volatile uint8_t init_tryes = 3;
volatile uint8_t reset_btn_ticks = 0;
volatile uint8_t reset_btn_false_ticks = 0;

//-------------------------------------------------------------------------------------------------------------
void main(void)
{
	WDTCTL = WDTPW + WDTHOLD;                 	// Stop WDT

	BCSCTL1 = CALBC1_8MHZ;
	DCOCTL = CALDCO_8MHZ;
	
	P2DIR |= BIT4;
	
	P2DIR &= ~BIT3;
	P2REN |= BIT3;
	P2OUT |= BIT3;
	/*P2DIR |= BIT7;
	P2DIR &= ~BIT6;
	P1DIR |= BIT3;
	
	BCSCTL3 |= LFXT1S_0 + XCAP_0;	// Конфигурирование ACLK*/
	BCSCTL2 |= DIVS_3;
	
	
	_BIS_SR(GIE);
	
	settings_is_init();
	modbus_handler_init();
	
	TA1CCTL0 = CCIE;
	TA1CCR0 = 62499;					// 0,5 сек таймаут
	TA1CTL = TASSEL_2 + ID_3 + MC_1;
	
	BME_Init(&bme_inited);
	
	_BIS_SR(LPM0_bits + GIE);
}

//-------------------------------------------------------------------------------------------------------------
void bme_inited(bool status)
{
	TA1CTL &= ~MC_1;
	bme_try_to_init = true;
	
	if (status) {
		BME_Measure(&gRes, bme_measured_first);
	} else {
		bme_error = true;
	}
}

//-------------------------------------------------------------------------------------------------------------
void bme_measured_first(bool status)
{
	if (init_tryes-- > 0) {
		__delay_cycles(8000);
		bme_inited(true);
		return ;
	}
	/*if (gRes.P < 500)
	{
		bme_inited(true);
		return ;
	}*/
	
	led_blink();
	
	TA1CCTL0 = CCIE;
	TA1CCR0 = 62499;
	TA1CTL = TASSEL_2 + ID_3 + MC_1;
}

//-------------------------------------------------------------------------------------------------------------
void bme_measured(bool status)
{
	led_blink();
}

//-------------------------------------------------------------------------------------------------------------
void reset_device()
{
	WDTCTL = 0;
}

//-------------------------------------------------------------------------------------------------------------
void default_settings()
{
	settings_set_defaults();
	reset_device();
}

//-------------------------------------------------------------------------------------------------------------
void led_blink()
{
	if ((P2OUT & BIT4)) {
		return ;
	}
	
	P2OUT |= BIT4;
}

//-------------------------------------------------------------------------------------------------------------
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TA1_A0_ISR() {	
	if (!(P2IN & BIT3)) {
		reset_btn_ticks++;
	} else {
		reset_btn_false_ticks++;
	}
	
	if (reset_btn_ticks >= 7 && reset_btn_false_ticks <= 3) {
		uint8_t i = 4;
		
		settings_set_defaults();

		while (i--) {
			P2OUT |= BIT4;
			__delay_cycles(2000000);
			P2OUT &= ~BIT4;
			__delay_cycles(2000000);
		}

		__delay_cycles(64000000);

		reset_device();
		return;
	} else if (reset_btn_false_ticks > 3) {
		reset_btn_ticks = 0;
		reset_btn_false_ticks = 0;
	}
	
	if (!bme_try_to_init) {
		P2OUT ^= BIT4;
		return ;
	}
	
	if (half_second_counter < 30) {
		half_second_counter++;
		
		if ((P2OUT & BIT4)) {
			P2OUT &= ~BIT4;
		}
		
		return ;
	}
	
	half_second_counter = 0;
	
	if (period_counter >= settings_get_interval()) {
		BME_Measure(&gRes, bme_measured);
		period_counter = 0;
	} else {
		period_counter++;
	}
}

//-------------------------------------------------------------------------------------------------------------
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TA0I0(void)
{
	MODBUS_CCR0_Service_A0();
}

//-------------------------------------------------------------------------------------------------------------
#pragma vector=TIMER0_A1_VECTOR
__interrupt void TA0I1(void)
{
	MODBUS_CCR1_Service_A0(TA0IV);
}

//-------------------------------------------------------------------------------------------------------------
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
	MODBUS_TX_Service_A0();
	I2C_MASTER_TX_Service_B0();
	// ... Прочий код (например, для обработки прерываний от USCI A0)
}

//-------------------------------------------------------------------------------------------------------------
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)
{
	MODBUS_RX_Service_A0();
	I2C_MASTER_RX_Service_B0();
	// ... Прочий код (например, для обработки прерываний от USCI A0)
}
