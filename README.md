# BME280_I2C
Written a code to measure real time Temperature, Pressure and Humidity from BME280 sensor which is connected to STM32F407 discovery board via I2C.

The connections between STM32 and BME280 are described below: 

SCL -> PB6

SDA -> PB7

SDO -> GND ( When the SDO pin of the sensor is connected to the GND, the primary address for I2C communication is 0x76,
             if the SDO pin is connected to the Vdd, the address for I2C communication is 0x77.  )
             
For connection, two 4.7K Ohm resistors were connected to both SCL and SDA line to keep them HIGH during the communication.
