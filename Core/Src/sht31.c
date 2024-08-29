/*
 * sht31.c
 *
 *  Created on: Aug 28, 2024
 *      Author: krea
 */


#include "sht31.h"

/*void SHT31_ReceiveI2C() {
	//HAL_StatusTypeDef ret = HAL_I2C_Master_Receive(sht31.hi2c, sht31.address<<1, buffer, num, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive_DMA(&hi2c1, addressSht31<<1, buffer_SHT31_I2C_IN, 6);
	//if(ret == HAL_OK) {
	// return SHT31_OK;
	//}
	//return SHT31_Transmit_Error;
}

void  SHT31_TransmitI2C() {
	buffer_SHT31_I2C_OUT[0]	= command_Sht31 >> 8;				// Copia primer byte del comando
	buffer_SHT31_I2C_OUT[1]	= uint8_t ( command_Sht31 & 0xFF );	// Copia segundo byte del comando
	//HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(sht31.hi2c, sht31.address<<1, buffer, 2, HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit_DMA(&hi2c1, addressSht31<<1, buffer_SHT31_I2C_OUT, 2);
	//if(ret == HAL_OK) {
	//	return SHT31_OK;
	//}
	//return SHT31_Transmit_Error;
}
*/
