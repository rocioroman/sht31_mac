#include "sht31.h"
#include <stdio.h>
#include <math.h> // For NAN

// Internal SHT31_INFO structure to hold sensor state
 struct {
    uint8_t address;
    I2C_HandleTypeDef* hi2c;
    uint16_t command;
    float temperature;
    float humidity;
} SHT31_INFO;

static SHT31_INFO sht31_sensor; // Static instance of the sensor structure

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

// CRC-8 calculation function
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
SHT31_Status SHT31_GetID(uint8_t* sensor_id) {
    uint8_t serial_number[6];

    if (SHT31_SendCommand(SHT31_READ_SERIAL_NUMBER) != SHT31_OK) {
        return SHT31_Error;
    }

    HAL_Delay(10);

    if (HAL_I2C_Master_Receive(sht31_sensor.hi2c, (sht31_sensor.address << 1), serial_number, 6, HAL_MAX_DELAY) != HAL_OK) {
        return SHT31_Error;
    }

    *sensor_id = serial_number[0];  // Assuming the ID is stored in the first byte of the serial number

    return SHT31_OK;
}

// Function to initialize the sensor
SHT31_Status SHT31_Init(I2C_HandleTypeDef* hi2c, uint8_t address, uint8_t expected_id, uint16_t command, UART_HandleTypeDef* huart) {
    sht31_sensor.hi2c = hi2c;
    sht31_sensor.address = address;
    sht31_sensor.command = command;

    uint8_t sensor_id;
    SHT31_Status status = SHT31_GetID(&sensor_id);
    if (status != SHT31_OK) {
        return status;
    }

    if (sensor_id != expected_id) {
        char error_msg[] = "Sensor ID mismatch!\n";
        HAL_UART_Transmit(huart, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
        return SHT31_ID_Mismatch;
    }

    char success_msg[] = "Sensor initialized successfully!\n";
    HAL_UART_Transmit(huart, (uint8_t*)success_msg, strlen(success_msg), HAL_MAX_DELAY);

    return SHT31_OK;
}

// Function to get the temperature value
float SHT31_GetTemperature(void) {
    if (SHT31_ReadData() == SHT31_OK) {
        return sht31_sensor.temperature;
    } else {
        return -1.0f;  // Return an invalid temperature on error
    }
}

// Function to get the humidity value
float SHT31_GetHumidity(void) {
    if (SHT31_ReadData() == SHT31_OK) {
        return sht31_sensor.humidity;
    } else {
        return -1.0f;  // Return an invalid humidity on error
    }
}

// Function to get both temperature and humidity in a single operation
SHT31_Status SHT31_ReadTempHum(float *temperature_out, float *humidity_out) {
    if (SHT31_ReadData() == SHT31_OK) {
        *temperature_out = sht31_sensor.temperature;
        *humidity_out = sht31_sensor.humidity;
        return SHT31_OK;
    } else {
        *temperature_out = *humidity_out = NAN;  // Return NAN if the operation fails
        return SHT31_ERROR;
    }
}
