

 ```C++
/* USER CODE BEGIN 2 */
  SHT31_Init(&hi2c1, SHT31_ADDRESS_A, 0x0C, SHT31_MEASUREMENT_NOSTRETCH_MEDIUM, &huart3);
  /* USER CODE END 2 */ 
  
  /* USER CODE BEGIN WHILE */
  uint8_t sensor_id;
  SHT31_Status status = SHT31_GetID(&sensor_id);
  if (status == SHT31_OK) {
      char id_msg[50];
      sprintf(id_msg, "Sensor ID: 0x%02X\n", sensor_id);
      HAL_UART_Transmit(&huart3, (uint8_t*)id_msg, strlen(id_msg), HAL_MAX_DELAY);
  } else {
      char id_fail_msg[] = "Falla en lectura del ID\n";
      HAL_UART_Transmit(&huart3, (uint8_t*)id_fail_msg, strlen(id_fail_msg), HAL_MAX_DELAY);
      while (1); // Stay here in case of failure
  }


  while (1)
  {
    /* USER CODE END WHILE */
```

Para leer 

```C++  
  SHT31_Status status =   SHT31_Init(&hi2c1, SHT31_ADDRESS_A, 0x0C, SHT31_MEASUREMENT_NOSTRETCH_MEDIUM, &huart3);
  if (status != SHT31_OK) {
      char fail_msg[] = "Error inicializacion lector!\n";
      HAL_UART_Transmit(&huart3, (uint8_t*)fail_msg, strlen(fail_msg), HAL_MAX_DELAY);
      while (1); 
  }

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      float temperature, humidity;

      // Read temperature and humidity
      if (SHT31_ReadTempHum(&temperature, &humidity) == SHT31_OK) {
          char temp_msg[50];
          sprintf(temp_msg, "Temp: %.2f C\n", temperature);
          HAL_UART_Transmit(&huart3, (uint8_t*)temp_msg, strlen(temp_msg), HAL_MAX_DELAY);

          char hum_msg[50];
          sprintf(hum_msg, "Hum: %.2f %%\n", humidity);
          HAL_UART_Transmit(&huart3, (uint8_t*)hum_msg, strlen(hum_msg), HAL_MAX_DELAY);
      } else {
          char error_msg[] = "Error en la lectura\n";
          HAL_UART_Transmit(&huart3, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
      }

      // Delay between readings (e.g., 2 seconds)
      HAL_Delay(2000);
  }
```     
 