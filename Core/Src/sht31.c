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

static SHT31_Status SHT31_ReadData(sht31* sensor) {
    uint8_t data[6];
    SHT31_Status status = SHT31_SendCommand(sensor, sensor->command);
    if (status != SHT31_OK) {
        return status;
    }

    HAL_Delay(15);  // Espera a que se complete la medicion

    if (HAL_I2C_Master_Receive(sensor->hi2c, (sensor->address << 1), data, 6, HAL_MAX_DELAY) != HAL_OK) {
        return SHT31_RECEIVE_ERROR;
    }
    // CRC temperatura
    if (SHT31_CRC8(data, 2) != data[2]) {
        return SHT31_CRC_ERROR;
    }
    // CRC humedad
    if (SHT31_CRC8(data + 3, 2) != data[5]) {
        return SHT31_CRC_ERROR;
    }

    uint16_t temperature_raw = (data[0] << 8) | data[1];
    uint16_t humidity_raw = (data[3] << 8) | data[4];

    sensor->temperature = -45 + 175 * (temperature_raw / 65535.0);
    sensor->humidity = 100 * (humidity_raw / 65535.0);

    return SHT31_OK;
}

SHT31_Status SHT31_GetID(sht31* sensor) {
    uint8_t serial_number[6];
    if (SHT31_SendCommand(sensor, SHT31_READ_SERIAL_NUMBER) != SHT31_OK) {
        return SHT31_SEND_COMMAND_ID_ERROR;
    }
    HAL_Delay(15);

    if (HAL_I2C_Master_Receive(sensor->hi2c, (sensor->address << 1), serial_number, 6, HAL_MAX_DELAY) != HAL_OK) {
        return SHT31_RECEIVE_ID_ERROR;
    }
    memcpy(sensor->id, serial_number, 6);
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
	SHT31_Status status = SHT31_ReadData(sensor);
    if (status == SHT31_OK) {
        return sensor->humidity;
    } else {
        return -1.0f;  // Cuando hay error retorna una lectura invalida
    }
}

////NON BLOCKING/////////
SHT31_Status SHT31_StartRead(sht31* sensor) {
    return SHT31_SendCommand(sensor, sensor->command);
}
// Revisa que la lectura este lista y obtiene los datos
SHT31_Status SHT31_ReadTempHum_NonBlocking(sht31* sensor, float* temperature_out, float* humidity_out) {
    uint8_t data[6];

    HAL_StatusTypeDef HAL_status= HAL_I2C_Master_Receive(sensor->hi2c, (sensor->address << 1), data, 6, 0);

    switch(HAL_status){
    	case HAL_OK:
    		break;
		case HAL_BUSY:
			return SHT31_RECEIVE_BUSY_ERROR;
		case HAL_TIMEOUT:
			return SHT31_RECEIVE_TIMEOUT_ERROR;
		case HAL_ERROR:
			return SHT31_RECEIVE_ERROR;
		default:
		     return SHT31_RECEIVE_UNKNOWN_ERROR;
    }

    if (SHT31_CRC8(data, 2) != data[2]) {
        return SHT31_CRC_ERROR;
    }

    if (SHT31_CRC8(data + 3, 2) != data[5]) {
        return SHT31_CRC_ERROR;
    }

    uint16_t temperature_raw = (data[0] << 8) | data[1];
    uint16_t humidity_raw = (data[3] << 8) | data[4];

    sensor->temperature = -45 + 175 * (temperature_raw / 65535.0);
    sensor->humidity = 100 * (humidity_raw / 65535.0);

    *temperature_out = sensor->temperature;
    *humidity_out = sensor->humidity;

    return SHT31_OK;
}

