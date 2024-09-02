// non blocking
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

// Comienza lectura paso 1
SHT31_Status SHT31_StartRead(sht31* sensor) {
    return SHT31_SendCommand(sensor, sensor->command);
}

// Revisa que la lectura este lista y obtiene los datos
SHT31_Status SHT31_ReadTempHum_NonBlocking(sht31* sensor, float* temperature_out, float* humidity_out) {
    uint8_t data[6];

    if (HAL_I2C_Master_Receive(sensor->hi2c, (sensor->address << 1), data, 6, 0) != HAL_OK) {
        return SHT31_RECEIVE_ERROR;
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
