#include "main.h"
#include"bme280.h"
#include<stdint.h>
#include<stdio.h>
#include <string.h>

// PB6 -> SCL
// PB7 -> SDA
// uint16_t dev_addr = 0x76; dev_address LEFT SHIFTED by 1 = 11101100
// 11101101 (ED) for read operation
// chip id = 0xD0 , 11010000
// SLAVE ADDRESS = 1110110 (0x76) when SDO is connected to GND
//                 1110111 (0x77) when SDO is conected to Vdd

I2C_HandleTypeDef hi2c1;


int8_t i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr); // to read the data from sensor
int8_t i2c_write(uint8_t reg_addr,  uint8_t *reg_data, uint32_t len, void *intf_ptr);  // to write data to sensor

int8_t i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    uint16_t dev_addr = (uint16_t)(uintptr_t)intf_ptr;   // attaching dev_addr to pointer for communication with bme280

    //  Send register address
    if (HAL_I2C_Master_Transmit(&hi2c1, dev_addr << 1, &reg_addr, 1, HAL_MAX_DELAY) != HAL_OK) {    // writing 0 on the 8th bit of dev_addr for read operation
        return -1; // Error in transmitting register address
    }
    	// Read data
    	if (HAL_I2C_Master_Receive(&hi2c1, dev_addr << 1, data, len, HAL_MAX_DELAY) != HAL_OK) {
    		return -1; // Error in receiving data
    }
    return 0; // Success
}


int8_t i2c_write(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    uint16_t dev_addr = (uint16_t)(uintptr_t)intf_ptr;
    uint8_t buffer[len + 1];

    // Preparing data with register address at the start
    buffer[0] = reg_addr;
    memcpy(&buffer[1], data, len);

    // Write data
    if (HAL_I2C_Master_Transmit(&hi2c1, dev_addr << 1, buffer, len + 1, HAL_MAX_DELAY) != HAL_OK) {
        return -1; // Error in transmitting data
    }
    return 0; // Success
}


void user_delay_us(uint32_t period, void *intf_ptr) {
	HAL_Delay(period / 1000);  //  for millisecond delay
}


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();

  	 struct bme280_dev dev;
     struct bme280_settings settings;
     struct bme280_data comp_data;

     int8_t rslt;

    // Initialize the sensor

	dev.intf_ptr = (void *)(uintptr_t)BME280_I2C_ADDR_PRIM;
	dev.intf = BME280_I2C_INTF;
	dev.read = (bme280_read_fptr_t)i2c_read;
	dev.write = (bme280_write_fptr_t)i2c_write;
	dev.delay_us = user_delay_us;

	rslt = bme280_init(&dev);
	// Check if the initialization was successful
	if (rslt == BME280_OK) {
		printf("BME280 initialization successful!\n");
	} else {
		printf("BME280 initialization failed with code: %d\n", rslt);
	return 0;
	}

	// settings configuration for BME280
	uint8_t settings_sel;
	settings.osr_h = BME280_OVERSAMPLING_16X;
	settings.osr_p = BME280_OVERSAMPLING_16X;
	settings.osr_p = BME280_OVERSAMPLING_16X;
	settings.filter = BME280_FILTER_COEFF_16;
	settings.standby_time = BME280_STANDBY_TIME_1000_MS;

	settings_sel = BME280_SEL_OSR_PRESS | BME280_SEL_OSR_TEMP| BME280_SEL_OSR_HUM | BME280_SEL_FILTER | BME280_SEL_STANDBY;
	bme280_set_sensor_settings(settings_sel, &settings, &dev); // applying setting configurations to sensor
	bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &dev); // applying sensor mode

	while (1)
    {
  	  rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev); // reading and compensating temperature,pressure and humidity values from the sensor
	  printf("Temperature: %0.2f °C\n", comp_data.temperature);
	  printf("Pressure: %0.2f hPa\n", comp_data.pressure);
	  printf("Humidity: %0.2f %%\n", comp_data.humidity);
	  HAL_Delay(1000);
    }
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
