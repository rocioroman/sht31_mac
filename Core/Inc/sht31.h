//header
#ifndef INC_SHT31_H_
#define INC_SHT31_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"

// I2C Addresses
#define SHT31_ADDRESS_A 0x44   ///< I2C address for SHT31 sensor (option A)
#define SHT31_ADDRESS_B 0x45   ///< I2C address for SHT31 sensor (option B)

// Serial Number Command
#define SHT31_READ_SERIAL_NUMBER 0x3780   ///< Command to read the sensor's serial number
#define SHT31_SERIAL_NUMBER {0x0C, 0x1F, 0x58, 0x97, 0x91, 0x6A}  ///< Numero serial 

// Measurement Commands
#define SHT31_MEASUREMENT_STRETCH_HIGH 0x2C06   ///< High repeatability with clock stretching
#define SHT31_MEASUREMENT_STRETCH_MEDIUM 0x2C0D ///< Medium repeatability with clock stretching
#define SHT31_MEASUREMENT_STRETCH_LOW 0x2C10    ///< Low repeatability with clock stretching
#define SHT31_MEASUREMENT_NOSTRETCH_HIGH 0x2400 ///< High repeatability without clock stretching
#define SHT31_MEASUREMENT_NOSTRETCH_MEDIUM 0x240B ///< Medium repeatability without clock stretching
#define SHT31_MEASUREMENT_NOSTRETCH_LOW 0x2416 ///< Low repeatability without clock stretching

/**
 * @brief Enumeration for SHT31 status codes.
 */
typedef enum {
    SHT31_OK,                ///< Operation was successful
    SHT31_ERROR,             ///< General error
    SHT31_TRANSMIT_ERROR,    ///< I2C transmission error
    SHT31_RECEIVE_ERROR,     ///< I2C reception error
    SHT31_CRC_ERROR,         ///< CRC check failed
    SHT31_ID_MISMATCH,       ///< Sensor ID does not match expected value
} SHT31_Status;

/**
 * @brief Structure to hold sensor information.
 */
typedef struct SHT31_INFO {
    uint8_t address;               ///< I2C address of the sensor
    I2C_HandleTypeDef* hi2c;       ///< Pointer to the I2C handle
    uint16_t command;              ///< Last command sent to the sensor
    float temperature;             ///< Last temperature reading
    float humidity;                ///< Last humidity reading
    uint8_t id[6];                 ///< Numero serial
} sht31;

/**
 * @brief Initializes the SHT31 sensor.
 * 
 * @param sensor Pointer to sht31 structure.
 * @param hi2c I2C handle to use for communication.
 * @param address I2C address of the sensor.
 * @param command Measurement command to be used.
 * @return SHT31_Status SHT31_OK if successful, error code otherwise.
 */
SHT31_Status SHT31_Init(sht31* sensor, I2C_HandleTypeDef* hi2c, uint8_t address, uint16_t command);

/**
 * @brief Retrieves the sensor ID (serial number) and stores it in the sensor struct.
 * 
 * @param sensor Pointer to sht31 structure.
 * @return SHT31_Status SHT31_OK if successful, error code otherwise.
 */
SHT31_Status SHT31_GetID(sht31* sensor);

/**
 * @brief Reads the temperature and humidity from the sensor (blocking).
 * 
 * @param sensor Pointer to sht31 structure.
 * @param temperature_out Pointer to store the temperature.
 * @param humidity_out Pointer to store the humidity.
 * @return SHT31_Status SHT31_OK if successful, error code otherwise.
 */
SHT31_Status SHT31_ReadTempHum(sht31* sensor, float* temperature_out, float* humidity_out);

/**
 * @brief Retrieves the last temperature reading.
 * 
 * @param sensor Pointer to sht31 structure.
 * @return float Last temperature reading.
 */
float SHT31_GetTemperature(sht31* sensor);

/**
 * @brief Retrieves the last humidity reading.
 * 
 * @param sensor Pointer to sht31 structure.
 * @return float Last humidity reading.
 */
float SHT31_GetHumidity(sht31* sensor);

/**
 * @brief Starts a non-blocking read of the temperature and humidity.
 * 
 * @param sensor Pointer to sht31 structure.
 * @return SHT31_Status SHT31_OK if the command was sent successfully, error code otherwise.
 */
SHT31_Status SHT31_StartRead(sht31* sensor);

/**
 * @brief Checks if the non-blocking read is complete and retrieves the data if available.
 * 
 * @param sensor Pointer to sht31 structure.
 * @param temperature_out Pointer to store the temperature.
 * @param humidity_out Pointer to store the humidity.
 * @return SHT31_Status SHT31_OK if data is available and successfully retrieved, error code otherwise.
 */
SHT31_Status SHT31_ReadTempHum_NonBlocking(sht31* sensor, float* temperature_out, float* humidity_out);

#endif /* INC_SHT31_H_ */
