#include "sht31.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// Estrucutra para guardar información del sensor
typedef struct {
    uint8_t address;
    I2C_HandleTypeDef* hi2c;
    uint16_t command;
    float temperature;
    float humidity;
} SHT31_INFO;

static SHT31_INFO sht31_sensor;

//Envía comando al sensor
static SHT31_Status SHT31_SendCommand(uint16_t command) {
    uint8_t cmd[2];
    cmd[0] = (command >> 8) & 0xFF;
    cmd[1] = command & 0xFF;

    if (HAL_I2C_Master_Transmit(sht31_sensor.hi2c, (sht31_sensor.address << 1), cmd, 2, HAL_MAX_DELAY) != HAL_OK) {
        return SHT31_TRANSMIT_ERROR;
    }
    return SHT31_OK;
}


// CRC-8 calculation polynomial: x^8 + x^5 + x^4 + 1
static uint8_t SHT31_CRC8(uint8_t *data, int len) {
	uint8_t crc = 0xFF;
	const uint8_t poly = 0x31;

	for(uint8_t byte = len; byte; byte--) {
		crc ^= *(data++);
		for(uint8_t i = 8; i; i--) {
			crc = (crc & 0x80)? (crc<<1)^poly : (crc<<1);
		}
	}
	return crc;
}

// Internal function to get both temperature and humidity in a single I2C transaction
static SHT31_Status SHT31_ReadData(void) {
    uint8_t data[6];
    SHT31_Status status = SHT31_SendCommand(sht31_sensor.command);
    if (status != SHT31_OK) {
        return status;
    }

    HAL_Delay(15);  // Wait for measurement to complete

    if (HAL_I2C_Master_Receive(sht31_sensor.hi2c, (sht31_sensor.address << 1), data, 6, HAL_MAX_DELAY) != HAL_OK) {
        return SHT31_TRANSMIT_ERROR;
    }
    // Verify CRC for temperature
    if (SHT31_CRC8(data, 2) != data[2]) {
        return SHT31_CRC_ERROR;
    }

    // Verify CRC for humidity
    if (SHT31_CRC8(data + 3, 2) != data[5]) {
        return SHT31_CRC_ERROR;
    }

    uint16_t temperature_raw = (data[0] << 8) | data[1];
    uint16_t humidity_raw = (data[3] << 8) | data[4];

    sht31_sensor.temperature = -45 + 175 * (temperature_raw / 65535.0);
    sht31_sensor.humidity = 100 * (humidity_raw / 65535.0);

    return SHT31_OK;
}

// Function to get the sensor ID
SHT31_Status SHT31_GetID(uint8_t* serial_id, UART_HandleTypeDef* huart) {
    uint8_t serial_number[6];
    const uint8_t expected_serial[] = SHT31_SERIAL_NUMBER;
    if (SHT31_SendCommand(SHT31_READ_SERIAL_NUMBER) != SHT31_OK) {
        return SHT31_ERROR;
    }
    HAL_Delay(10);

    if (HAL_I2C_Master_Receive(sht31_sensor.hi2c, (sht31_sensor.address << 1), serial_number, 6, HAL_MAX_DELAY) != HAL_OK) {
        return SHT31_RECEIVE_ERROR;
    }
    memcpy(serial_id, serial_number, 6);
    char id_msg[100];
    sprintf(id_msg, "Retrieved Sensor ID: 0x%02X%02X%02X%02X%02X%02X\n",
               serial_id[0], serial_id[1], serial_id[2],
               serial_id[3], serial_id[4], serial_id[5]);

       // Send the ID over UART
    HAL_UART_Transmit(huart, (uint8_t*)id_msg, strlen(id_msg), HAL_MAX_DELAY);
    // Compare the retrieved serial number with the expected one
    if (memcmp(serial_id, expected_serial, 6) != 0) {
        return SHT31_ID_MISMATCH;  // Return error if they don't match
    }
     return SHT31_OK;
}

// Function to initialize the sensor
SHT31_Status SHT31_Init(I2C_HandleTypeDef* hi2c, uint8_t address, uint16_t command) {
    sht31_sensor.hi2c = hi2c;
    sht31_sensor.address = address;
    sht31_sensor.command = command;

    /*uint8_t serial_id[6]; //id obtenido por comando
    SHT31_Status status = SHT31_GetID(serial_id);
    if (status != SHT31_OK) {
        return status;
    }*/

    return SHT31_OK;
}


float SHT31_GetTemperature(void) {
    if (SHT31_ReadData() == SHT31_OK) {
        return sht31_sensor.temperature;
    } else {
        return -1.0f;  // Cuando hay error retorna una lectura invalida
    }
}

float SHT31_GetHumidity(void) {
    if (SHT31_ReadData() == SHT31_OK) {
        return sht31_sensor.humidity;
    } else {
        return -1.0f;  // Cuando hay error retorna una lectura invalida
    }
}

// Se obtiene temperatura y humedad de una lectura
SHT31_Status SHT31_ReadTempHum(float *temperature_out, float *humidity_out) {
    if (SHT31_ReadData() == SHT31_OK) {
        *temperature_out = sht31_sensor.temperature;
        *humidity_out = sht31_sensor.humidity;
        return SHT31_OK;
    } else {
        *temperature_out = *humidity_out = NAN;  // Si falla la operación
        return SHT31_ERROR;
    }
}
