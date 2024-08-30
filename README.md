


    ``` c++
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
      char id_fail_msg[] = "Failed to read Sensor ID!\n";
      HAL_UART_Transmit(&huart3, (uint8_t*)id_fail_msg, strlen(id_fail_msg), HAL_MAX_DELAY);
      while (1); // Stay here in case of failure
  }


  while (1)
  {
    /* USER CODE END WHILE */
```
