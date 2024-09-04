
Ejemplo de uso (blocking): 
 ```C++
  int main (void){
    
    //( ...)

    // Inicializar instancia del sensor
    sht31 sensor;

    // Inicializar sensor con address SHT31_ADDRESS_A, una medicion nostretch_medium 
    SHT31_Status status_init = SHT31_Init(&sensor, &hi2c1, SHT31_ADDRESS_A, SHT31_MEASUREMENT_NOSTRETCH_MEDIUM);
    if (status_init != SHT31_OK) {
        char fail_msg[] = "Error inicializando sensor!\n";
        HAL_UART_Transmit(&huart3, (uint8_t*)fail_msg, strlen(fail_msg), HAL_MAX_DELAY);
        while (1); 
    }

    // Obtener ID del sensor
    SHT31_Status status_id = SHT31_GetID(&sensor);
    if (status_id == SHT31_OK) {
        char id_msg[100];
        sprintf(id_msg, "Sensor ID: 0x%02X%02X%02X%02X%02X%02X\n",
                sensor.id[0], sensor.id[1], sensor.id[2],
                sensor.id[3], sensor.id[4], sensor.id[5]);
        HAL_UART_Transmit(&huart3, (uint8_t*)id_msg, strlen(id_msg), HAL_MAX_DELAY);
    } else {
        char id_fail_msg[] = "Falla en lectura del ID sensor!\n";
        HAL_UART_Transmit(&huart3, (uint8_t*)id_fail_msg, strlen(id_fail_msg), HAL_MAX_DELAY);
        while (1);  
    }

    while (1) {
      float temperature, humidity;

      // Leer temperatura y humedad
      SHT31_Status status_read = SHT31_ReadTempHum(&sensor, &temperature, &humidity);
      if (status_read == SHT31_OK) {
          char data_msg[50];
          sprintf(data_msg, "Temp: %.2f C, Hum: %.2f %%\n", temperature, humidity);
          HAL_UART_Transmit(&huart3, (uint8_t*)data_msg, strlen(data_msg), HAL_MAX_DELAY);
      } else {
          char error_msg[] = "Error en lectura del sensor\n";
          HAL_UART_Transmit(&huart3, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
      }

      HAL_Delay(2000);  
  }
}

```

Ejemplo de uso (non-blocking): 

```C++  
int main(void) {
    // (...) Inicializaciones sistema

    // Crear instancia sensor
    sht31 sensor;

    // Inicializar sensor
    SHT31_Status status_init = SHT31_Init(&sensor, &hi2c1, SHT31_ADDRESS_A, SHT31_MEASUREMENT_NOSTRETCH_MEDIUM);
    if (status_init != SHT31_OK) {
        char fail_msg[] = "Error inicializando sensor!\n";
        HAL_UART_Transmit(&huart3, (uint8_t*)fail_msg, strlen(fail_msg), HAL_MAX_DELAY);
        while (1);  
    }

    // Retrieve and store the sensor ID
    SHT31_Status status_id = SHT31_GetID(&sensor);
    if (status_id == SHT31_OK) {
        char id_msg[100];
        sprintf(id_msg, "Sensor ID: 0x%02X%02X%02X%02X%02X%02X\n",
                sensor.id[0], sensor.id[1], sensor.id[2],
                sensor.id[3], sensor.id[4], sensor.id[5]);
        HAL_UART_Transmit(&huart3, (uint8_t*)id_msg, strlen(id_msg), HAL_MAX_DELAY);
    } else {
        char id_fail_msg[] = "Error en lectura del ID sensor!\n";
        HAL_UART_Transmit(&huart3, (uint8_t*)id_fail_msg, strlen(id_fail_msg), HAL_MAX_DELAY);
        while (1);  // Stay here in case of ID failure
    }

    // Comenzar lectura
    SHT31_Status status_start = SHT31_StartRead(&sensor);
    if (status_start != SHT31_OK) {
        char start_fail_msg[] = "Error iniciando lectura!\n";
        HAL_UART_Transmit(&huart3, (uint8_t*)start_fail_msg, strlen(start_fail_msg), HAL_MAX_DELAY);
        while (1);  
    }

    // Se revisa si la data esta lista
    while (1) {
        float temperature, humidity;

        SHT31_Status status_read = SHT31_ReadTempHum_NonBlocking(&sensor, &temperature, &humidity);
        if (status_read == SHT31_OK) {
            char data_msg[50];
            sprintf(data_msg, "Temp: %.2f C, Hum: %.2f %%\n", temperature, humidity);
            HAL_UART_Transmit(&huart3, (uint8_t*)data_msg, strlen(data_msg), HAL_MAX_DELAY);

            SHT31_StartRead(&sensor); //se comienza hace otra lectura si esta listo
        } else if (status_read == SHT31_RECEIVE_ERROR) {
            char error_msg[] = "Error reading from sensor\n"; //si no esta la lectura
            HAL_UART_Transmit(&huart3, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
        }

    }
}

```     
 