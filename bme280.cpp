#include <msp430.h>

//-------------------------------------------------------------------------------------------------------------
#include "bme280.h"
#include "i2c_master.h"

//-------------------------------------------------------------------------------------------------------------
#define	BME280_CONTROL_MEAS			0xF4
#define	BME280_CONTROL_HUM			0xF2

#define	BME280_CONFIG				0xF5
#define	BME280_PRESSURE				0xF7
#define	BME280_TEMP					0xFA
#define	BME280_HUMIDITY				0xFD

#define BME280_OVERS_T1				0x20
#define BME280_OVERS_T2				0x40
#define BME280_OVERS_T4				0x60
#define BME280_OVERS_T8				0x80
#define BME280_OVERS_T16			0xA0

#define BME280_OVERS_P1				0x04
#define BME280_OVERS_P2				0x08
#define BME280_OVERS_P4				0x0C
#define BME280_OVERS_P8				0x10
#define BME280_OVERS_P16			0x14

#define BME280_OVERS_H1				0x01
#define BME280_OVERS_H2				0x02
#define BME280_OVERS_H4				0x03
#define BME280_OVERS_H8				0x04
#define BME280_OVERS_H16			0x05

#define BME280_SLEEP_MODE			0x00
#define BME280_FORCED_MODE			0x01
#define BME280_NORMAL_MODE			0x03

#define BME280_TSB_0_5				0x00
#define BME280_TSB_62_5				0x20
#define BME280_TSB_125				0x40
#define BME280_TSB_250				0x60
#define BME280_TSB_500				0x80
#define BME280_TSB_1000				0xA0
#define BME280_TSB_2000				0xC0
#define BME280_TSB_4000				0xE0

#define BME280_FILTER_OFF				0x00
#define BME280_FILTER_COEFFICIENT2		0x04
#define BME280_FILTER_COEFFICIENT4		0x08
#define BME280_FILTER_COEFFICIENT8		0x0C
#define BME280_FILTER_COEFFICIENT16		0x10

#define BME280_SPI_OFF				0x00
#define BME280_SPI_ON				0x01

#define BME280_CONTROL_MEAS_SET		(BME280_OVERS_T16 | BME280_OVERS_P16 | BME280_NORMAL_MODE)
#define BME280_CONTROL_HUM_SET		BME280_OVERS_H16	//H2
#define BME280_CONFIG_SET			(BME280_TSB_0_5 | BME280_FILTER_COEFFICIENT16 | BME280_SPI_OFF)


//-------------------------------------------------------------------------------------------------------------

#define PRESS_MSB_IN_BUFFER				1
#define PRESS_LSB_IN_BUFFER				2
#define PRESS_XLSB_IN_BUFFER			0
#define TEMP_MSB_IN_BUFFER				4
#define TEMP_LSB_IN_BUFFER				5
#define TEMP_XLSB_IN_BUFFER				3
#define HUM_MSB_IN_BUFFER				7
#define HUM_LSB_IN_BUFFER				6

//-------------------------------------------------------------------------------------------------------------
uint8_t buffer[2] = {0};
	
uint8_t 	bme_id = 0;

uint16_t	 	dig_T1;		// 0x88[7:0]/0x89[15:8]
int16_t 		dig_T2,		// 0x8A[7:0]/0x8B[15:8]
				dig_T3;		// 0x8C[7:0]/0x8D[15:8]
uint16_t		dig_P1;		// 0x8E[7:0]/0x8F[15:8]
int16_t			dig_P2,		// 0x90[7:0]/0x91[15:8]
				dig_P3,		// 0x92[7:0]/0x93[15:8]
				dig_P4,		// 0x94[7:0]/0x95[15:8]
				dig_P5,		// 0x96[7:0]/0x97[15:8]
				dig_P6,		// 0x98[7:0]/0x99[15:8]
				dig_P7,		// 0x9A[7:0]/0x9B[15:8]
				dig_P8,		// 0x9C[7:0]/0x9D[15:8]
				dig_P9;		// 0x9E[7:0]/0x9F[15:8]
uint8_t			dig_H1,		// 0xA1[7:0]
				dig_H3;		// 0xE3[7:0]
int16_t			dig_H2,		// 0xE1[7:0]/0xE2[15:8]
				dig_H4,		// 0xE4[11:4]/0xE5[3:0]
				dig_H5;		// 0xE5[7:4]/0xE6[11:4]
int8_t			dig_H6;		// 0xE7

//-------------------------------------------------------------------------------------------------------------
void (*buf_bool_callback)(bool);
struct BME_Result* buf_res;

//-------------------------------------------------------------------------------------------------------------
void bme_init0_callback();
void bme_init1_callback();
void bme_init2_callback();
void bme_init3_callback();
void bme_init4_callback();
void bme_init5_callback();
void bme_init6_callback();
void bme_init7_callback();
void bme_init8_callback();

void bme_init0_callback()					// Приём ChipID
{
	I2C_Master_Receive(BME_I2C_ADDR, 1, &bme_init1_callback);
}

void bme_init1_callback()					// ChipID принят
{
	bme_id = I2C_Master_Answer()[0];
	
	if (bme_id == 0x60 ||						// BME280
			bme_id >= 0x56 && bme_id <= 0x58) 		// BMP280
	{
		I2C_Master_ClearReceiveBuffer();
		
		buffer[0] = 0x88;
		
		I2C_Master_Transmit(BME_I2C_ADDR, buffer, 1, &bme_init2_callback);
	}
	else {
		buf_bool_callback(false);
	}
}

void bme_init2_callback()					// Приём 27 байт калибрационных данных
{
	I2C_Master_Receive(BME_I2C_ADDR, 27, &bme_init3_callback);
}

void bme_init3_callback()					// 27 байт калибрационных данных приняты
{
	uint8_t* b_ptr;
	
	b_ptr = I2C_Master_Answer();
	
	dig_T1 = (uint16_t) (*b_ptr++);						// 0x88
	dig_T1 |= ((uint16_t) (*b_ptr++) << 8);				// 0x89

	dig_T2 = (int16_t) (*b_ptr++);						// 0x8A
	dig_T2 |= ((int16_t) (*b_ptr++) << 8);				// 0x8B

	dig_T3 = (int16_t) (*b_ptr++);						// 0x8C
	dig_T3 |= ((int16_t) (*b_ptr++) << 8);				// 0x8D

	dig_P1 = (uint16_t) (*b_ptr++);						// 0x8E
	dig_P1 |= ((uint16_t) (*b_ptr++) << 8);				// 0x8F

	dig_P2 = (int16_t) (*b_ptr++);						// 0x90
	dig_P2 |= ((int16_t) (*b_ptr++) << 8);				// 0x91

	dig_P3 = (int16_t) *b_ptr++;						// 0x92
	dig_P3 |= ((int16_t) (*b_ptr++) << 8);				// 0x93

	dig_P4 = (int16_t) *b_ptr++;						// 0x94
	dig_P4 |= ((int16_t) (*b_ptr++) << 8);				// 0x95

	dig_P5 = (int16_t) *b_ptr++;						// 0x96
	dig_P5 |= ((int16_t) (*b_ptr++) << 8);				// 0x97

	dig_P6 = (int16_t) *b_ptr++;						// 0x98
	dig_P6 |= ((int16_t) (*b_ptr++) << 8);				// 0x99

	dig_P7 = (int16_t) *b_ptr++;						// 0x9A
	dig_P7 |= ((int16_t) (*b_ptr++) << 8);				// 0x9B

	dig_P8 = (int16_t) *b_ptr++;						// 0x9C
	dig_P8 |= ((int16_t) (*b_ptr++) << 8);				// 0x9D

	dig_P9 = (int16_t) *b_ptr++;						// 0x9E
	dig_P9 |= ((int16_t) (*b_ptr++) << 8);				// 0x9F
	b_ptr += 2;											// 0xA0
	dig_H1 = (uint8_t) *b_ptr;							// 0xA1
	
	//------------------
	buffer[0] = 0xE0;
	
	I2C_Master_Transmit(BME_I2C_ADDR, buffer, 1, &bme_init4_callback);
}

void bme_init4_callback()					// Приём 9 байт калибрационных данных
{
	I2C_Master_ClearReceiveBuffer();
	I2C_Master_Receive(BME_I2C_ADDR, 9, &bme_init5_callback);
}

void bme_init5_callback()					// 9 байт калибрационных данных приняты
{
	uint8_t* b_ptr;
	
	b_ptr = I2C_Master_Answer();
	
	dig_H2 = ((int16_t) (*b_ptr++) << 8);					// 1
	dig_H2 |= (int16_t) *b_ptr++;							// 0
	
	dig_H3 = (uint8_t) *b_ptr++;							// 2
	
	dig_H4 = (int16_t) ((*b_ptr++) << 4);					// 3
	dig_H4 |= (int16_t) ((*b_ptr) & 0x0F);					// 4
	
	dig_H5 = (int16_t) (((*b_ptr++) & 0xF0) >> 4);			// 4
	dig_H5 |= (int16_t) ((*b_ptr++) << 4);					// 5
	
	dig_H6 = (int8_t) *b_ptr;
	
	
	//------------------
	buffer[0] = BME280_CONFIG;
	buffer[1] = BME280_CONFIG_SET;
	
	I2C_Master_Transmit(BME_I2C_ADDR, buffer, 2, &bme_init6_callback);
}

void bme_init6_callback()					// Настройки BME280_CONFIG переданы
{
	buffer[0] = BME280_CONTROL_HUM;
	buffer[1] = BME280_CONTROL_HUM_SET;
	
	I2C_Master_Transmit(BME_I2C_ADDR, buffer, 2, &bme_init7_callback);
}

void bme_init7_callback()					// Настройки HUM переданы
{
	buffer[0] = BME280_CONTROL_MEAS;
	buffer[1] = BME280_CONTROL_MEAS_SET;
	
	I2C_Master_Transmit(BME_I2C_ADDR, buffer, 2, &bme_init8_callback);
}

void bme_init8_callback()					// Настройки MEAS переданы, инициализация прошла успешно
{
	buf_bool_callback(true);
}

//-------------------------------------------------------------------------------------------------------------
void BME_Init(void (*callback)(bool status))
{
	buf_bool_callback = callback;
	
	I2C_Master_Init(BME_I2C_DIV);
	
	buffer[0] = 0xD0;
	
	I2C_Master_Transmit(BME_I2C_ADDR, buffer, 1, &bme_init0_callback);
}

//-------------------------------------------------------------------------------------------------------------
uint8_t	BME_GetChipID()
{
	return bme_id;
}

void bme_measure0_callback();
void bme_measure1_callback();

void bme_measure0_callback()
{
	I2C_Master_ClearReceiveBuffer();
	I2C_Master_Receive(BME_I2C_ADDR, 9, &bme_measure1_callback);
}

void bme_measure1_callback()
{
	uint8_t* b_ptr = I2C_Master_Answer();
	
	if (!b_ptr) {
		return ;
	}
	
	uint32_t adc_T = (uint32_t) ((b_ptr[TEMP_XLSB_IN_BUFFER] & 0xF0) >> 4);
	adc_T |= ((uint32_t) b_ptr[TEMP_LSB_IN_BUFFER]) << 4;
	adc_T |= ((uint32_t) b_ptr[TEMP_MSB_IN_BUFFER]) << 12;

	int32_t adc_P = (int32_t) ((b_ptr[PRESS_XLSB_IN_BUFFER] & 0xF0) >> 4);
	adc_P |= ((int32_t) b_ptr[PRESS_LSB_IN_BUFFER]) << 4;
	adc_P |= ((int32_t) b_ptr[PRESS_MSB_IN_BUFFER]) << 12;
	
	int32_t adc_H = ((int32_t) b_ptr[HUM_LSB_IN_BUFFER]);
	adc_H |= ((int32_t) b_ptr[HUM_MSB_IN_BUFFER]) << 8;
	
	int32_t var1, var2, t, t_fine;
	var1 = ((((adc_T >> 3) - (((int32_t) dig_T1) << 1))) * ((int32_t) dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t) dig_T1)) * ((adc_T >> 4) - ((int32_t) dig_T1))) >> 12) * ((int32_t) dig_T3)) >> 14;
	t_fine = var1 + var2;
	t = (t_fine * 5 + 128) >> 8;
	
	int32_t v_x1_u32r;
	v_x1_u32r = (t_fine - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t) dig_H4) << 20) - (((int32_t) dig_H5) * v_x1_u32r)) +
	((int32_t) 16384)) >> 15) * (((((((v_x1_u32r * ((int32_t) dig_H6)) >> 10) * (((v_x1_u32r *
	((int32_t) dig_H3)) >> 11) + ((int32_t) 32768))) >> 10) + ((int32_t) 2097152)) *
	((int32_t) dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	uint32_t h = (uint32_t) (v_x1_u32r>>12);
	// need to divides on 1024 (result was in %)
	
	int64_t var3, var4, p;
	var3 = ((int64_t) t_fine) - 128000;
	var4 = var3 * var3 * (int64_t) dig_P6;
	var4 = var4 + ((var3 * (int64_t) dig_P5)<<17);
	var4 = var4 + (((int64_t) dig_P4) << 35);
	var3 = ((var3 * var3 * (int64_t) dig_P3) >> 8) + ((var3 * (int64_t) dig_P2) << 12);
	var3 = (((((int64_t)1) << 47) + var3)) * ((int64_t) dig_P1) >> 33;
	
	if (!var3) {
		(*buf_res).P = -1.0;
		(*buf_res).P_Pa = -1.0;
		p = 0;
		//return 0; // avoid exception caused by division by zero
	} else {
		p = 1048576 - adc_P;
		p = (((p << 31) - var4) * 3125) / var3;
		var3 = (((int64_t) dig_P9) * (p >> 13) * (p >> 13)) >> 25;
		var4 = (((int64_t) dig_P8) * p) >> 19;
		p = ((p + var3 + var4) >> 8) + (((int64_t) dig_P7) << 4);
		// need to divides on 256 (result was in Pa)
	}
	
	(*buf_res).T = (float) t / 100.0;
	(*buf_res).H = (float) h / 1024.0;
	(*buf_res).P_Pa = (*buf_res).P_Pa < -0.99995 && (*buf_res).P_Pa > -1.00005 ? 0 : ((float) (p / 256));
	(*buf_res).P = (*buf_res).P_Pa < -0.99995 && (*buf_res).P_Pa > -1.00005  ? 0 : (*buf_res).P_Pa / 133.3224;
	
	buf_bool_callback(true);
}

//-------------------------------------------------------------------------------------------------------------
void BME_Measure(struct BME_Result* res, void (*callback)(bool))
{
	if (!res)
	{
		callback(false);
		return ;
	}
	
	buf_bool_callback = callback;
	buf_res = res;
	
	buffer[0] = 0xF6;
	
	I2C_Master_Transmit(BME_I2C_ADDR, buffer, 1, &bme_measure0_callback);
}
