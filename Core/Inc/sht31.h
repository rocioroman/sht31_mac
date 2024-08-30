#ifndef INC_SHT31_H_
#define INC_SHT31_H_


#include <stdint.h>
#include <stdbool.h> // Include for bool type
#include "stm32l4xx_hal.h"

// I2C Addresses
#define SHT31_ADDRESS_A 0x44 // Table 7 page 9 of the datasheet
#define SHT31_ADDRESS_B 0x45

// Commands
#define SHT31_READ_SERIAL_NUMBER 0x3780
// MEASUREMENT Single shot
#define SHT31_MEASUREMENT_STRETCH_HIGH 0x2C06
#define SHT31_MEASUREMENT_STRETCH_MEDIUM 0x2C0D
#define SHT31_MEASUREMENT_STRETCH_LOW 0x2C10

#define SHT31_MEASUREMENT_NOSTRETCH_HIGH 0x2400
#define SHT31_MEASUREMENT_NOSTRETCH_MEDIUM 0x240B
#define SHT31_MEASUREMENT_NOSTRETCH_LOW 0x2416

typedef enum {
    SHT31_OK,
    SHT31_ERROR,
	SHT31_TRANSMIT_ERROR,
	SHT31_RECEIVE_ERROR,
    SHT31_CRC_ERROR,
    SHT31_ID_MISMATCH,
} SHT31_Status;

// Function Prototypes
SHT31_Status SHT31_GetID(uint8_t* sensor_id);
SHT31_Status SHT31_Init(I2C_HandleTypeDef* hi2c, uint8_t address, uint8_t expected_id, uint16_t command, UART_HandleTypeDef* huart);
float SHT31_GetTemperature(void);
float SHT31_GetHumidity(void);
SHT31_Status SHT31_ReadTempHum(float *temperature_out, float *humidity_out);

#endif /* INC_SHT31_H_ */
