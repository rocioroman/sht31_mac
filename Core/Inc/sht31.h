/*
 * sht31.h
 *
 *  Created on: Aug 28, 2024
 *      Author: krea
 */

#ifndef INC_SHT31_H_
#define INC_SHT31_H_

#include <stdint.h>

#define SHT31_ADDRESS_A 0x44;

uint16_t command_Sht31	= 0X240B;				//0x2721 -- 0x240B;	// SINGE SHOT // NON STRETCH // MEDIUM REPEABILITY												//0x2721    // PERIODIC MEASUREMENT 10mps MEDIUM
uint8_t addressSht31	= 0x44;					// Address de I2C

//uint8_t buffer_SHT31_I2C_OUT[2]	= {0x24, 0x0B};	// BUFFER QUE ENVIA COMANDO (Repeatability Medium, Clock Stretching disabled)
uint8_t buffer_SHT31_I2C_OUT[2];
uint8_t buffer_SHT31_I2C_IN[6];					// Buffer que recibe respuesta

typedef struct SHT31_INFO{
	long long lastMeasureTime;
	uint16_t temperature_raw;
	uint16_t humidity_raw;
	float temperature;
	float humidity;
}sht31;



#endif /* INC_SHT31_H_ */
