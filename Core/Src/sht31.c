#include "sht31.h"
#include <string.h>
#include <math.h>

static SHT31_Status SHT31_SendCommand(sht31* sensor, uint16_t command) {
    uint8_t cmd[2];
    cmd[0] = (command >> 8) & 0xFF;
    cmd[1] = command & 0xFF;

    if (HAL_I2C_Master_Transmit(sensor->hi2c, (sensor->address << 1), cmd, 2, HAL_MAX_DELAY) != HAL_OK) {
        return SHT31_TRANSMIT_ERROR;
    }
    return SHT31_OK;
}

static uint8_t SHT31_CRC8(uint8_t *data, int len) {
    uint8_t crc = 0xFF;
    const uint8_t poly = 0x31;

    for (uint8_t byte = len; byte; byte--) {
        crc ^= *(data++);
        for (uint8_t i = 8; i; i--) {
            crc = (crc & 0x80) ? (crc << 1) ^ poly : (crc << 1);
        }
    }
    return crc;
}

// Function to get both temperature and humidity 
static SHT31_Status SHT31_ReadData(sht31* sensor) {
    uint8_t data[6];
    SHT31_Status status = SHT31_SendCommand(sensor, sensor->command);
    if (status != SHT31_OK) {
        return status;
    }

    HAL_Delay(15);  // Wait for measurement to complete

    if (HAL_I2C_Master_Receive(sensor->hi2c, (sensor->address << 1), data, 6, HAL_MAX_DELAY) != HAL_OK) {
        return SHT31_RECEIVE_ERROR;
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

    sensor->temperature = -45 + 175 * (temperature_raw / 65535.0);
    sensor->humidity = 100 * (humidity_raw / 65535.0);

    return SHT31_OK;
}

SHT31_Status SHT31_GetID(sht31* sensor, uint8_t* serial_id) {
    uint8_t serial_number[6];
    const uint8_t expected_serial[] = SHT31_SERIAL_NUMBER;
    if (SHT31_SendCommand(sensor, SHT31_READ_SERIAL_NUMBER) != SHT31_OK) {
        return SHT31_ERROR;
    }
    HAL_Delay(10);

    if (HAL_I2C_Master_Receive(sensor->hi2c, (sensor->address << 1), serial_number, 6, HAL_MAX_DELAY) != HAL_OK) {
        return SHT31_RECEIVE_ERROR;
    }
    memcpy(serial_id, serial_number, 6);
    // Compare the retrieved serial number with the expected one 
    //if (compare==1 & memcmp(serial_id, expected_serial, 6) != 0) {
    if (memcmp(serial_id, expected_serial, 6) != 0) {
        return SHT31_ID_MISMATCH;
    }
    return SHT31_OK;
}

SHT31_Status SHT31_Init(sht31* sensor, I2C_HandleTypeDef* hi2c, uint8_t address, uint16_t command) {
    sensor->hi2c = hi2c;
    sensor->address = address;
    sensor->command = command;
    return SHT31_OK;
}

SHT31_Status SHT31_ReadTempHum(sht31* sensor, float* temperature_out, float* humidity_out) {
    SHT31_Status status = SHT31_ReadData(sensor);
    if (status == SHT31_OK) {
        *temperature_out = sensor->temperature;
        *humidity_out = sensor->humidity;
    }
    return status;
}

float SHT31_GetTemperature(sht31* sensor) {
    SHT31_Status status = SHT31_ReadData(sensor);
    if (status == SHT31_OK) { 
        return sensor->temperature;
    } else {
        return -1.0f;  // Cuando hay error retorna una lectura invalida
    }
}

float SHT31_GetHumidity(sht31* sensor) {
    if (SHT31_ReadData() == SHT31_OK) {
        return sensor->humidity;
    } else {
        return -1.0f;  // Cuando hay error retorna una lectura invalida
    }
}

