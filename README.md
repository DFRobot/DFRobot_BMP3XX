# DFRobot_BMP3XX
* [中文版](./README_CN.md)

This is a Library for BMP3XX, the function is to read temperature and pressure.
The BMP(390L/388) is a digital sensor with pressure and temperature measurement based on proven sensing principles. The 
sensor module is housed in an extremely compact 10-pin metal-lid LGA package with a footprint of only 2.0 × 2.0 mm² and 
max 0.8 mm package height. Its small dimensions and its low power consumption of 3.2 µA @1Hz allow the implementation 
in battery driven devices such as mobile phones, GPS modules or watches.

![BMP388(Gravity)产品实物图](./resources/images/BMP388.png)
![BMP390L(Fermion)产品实物图](./resources/images/BMP390L.png)


## Product Link (https://www.dfrobot.com/search-bmp3.html)
    SKU: SEN0423/SEN0371/SEN0251


## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)


## Summary
* BMP(390L/388) can read temperature and pressure.
* The library supports the SPI/I2C communication.
* BMP(390L/388) also includes FIFO functionality. This greatly improves ease of use.
* Interrupts than can be used in a power efficient manner without using software algorithms.
* BMP390L is more accurate than its predecessors, covering a wide measurement range from 300 hPa to 1250 hPa.
* This new barometric pressure(BMP390L) sensor exhibits an attractive price-performance ratio coupled with low power consumption.
* Due to the built-in hardware synchronization of the sensor(BMP390L) data and its ability to synchronize data from external devices.

## Installation

To use this library, first download the library file, paste it into the \Arduino\libraries directory, 
then open the examples folder and run the demo in the folder.


## Methods

```C++

  /**
   * @fn begin
   * @brief initialization function
   * @return int type, means returning initialization status
   * @retval 0 NO_ERROR
   * @retval -1 ERR_DATA_BUS
   * @retval -2 ERR_IC_VERSION
   */
  virtual int begin(void);

  /**
   * @fn setSamplingMode
   * @brief commonly used sampling modes that allows users to configure easily
   * @param mode:
   * @n       eUltraLowPrecision, Ultra-low precision, suitable for monitoring weather (lowest power consumption), the power is mandatory mode.
   * @n       eLowPrecision, Low precision, suitable for random detection, power is normal mode
   * @n       eNormalPrecision1, Normal precision 1, suitable for dynamic detection on handheld devices (e.g on mobile phones), power is normal mode
   * @n       eNormalPrecision2, Normal precision 2, suitable for drones, power is normal mode
   * @n       eHighPrecision, High precision, suitable for low-power handled devices (e.g mobile phones), power is normal mode
   * @n       eUltraPrecision, Ultra-high precision, suitable for indoor navigation, its acquisition rate will be extremely low, and the acquisition cycle is 1000 ms.
   * @return boolean, indicates configuration results
   * @retval True indicates configuration succeeds, successfully update the configuration
   * @retval False indicates configuration fails and remains its original state
   */
  bool setSamplingMode(ePrecisionMode_t mode);

  /**
   * @fn getSamplingPeriodUS
   * @brief Get the sampling period in the current mode
   * @return Return sampling period, unit: us
   */
  uint32_t getSamplingPeriodUS(void);

  /**
   * @fn readTempC
   * @brief Get temperature measurement value from register, working range (-40 ‒ +85 °C)
   * @return Return temperature measurements, unit: °C
   */
  float readTempC(void);

  /**
   * @fn readPressPa
   * @brief Get pressure measurement value from register, working range(300‒1250 hPa)
   * @return Return pressure measurements, unit: Pa
   * @attention If the reference value is provided before, the absolute value of the current position pressure is calculated according to the calibrated sea level atmospheric pressure
   */
  float readPressPa(void);

  /**
   * @fn calibratedAbsoluteDifference
   * @brief Take the given current location altitude as the reference value to eliminate the absolute difference for subsequent pressure and altitude data
   * @param altitude Altitude in current position
   * @return boolean, indicates whether the reference value is set successfully
   * @retval True indicates the reference value is set successfully
   * @retval False indicates fail to set the reference value
   */
  bool calibratedAbsoluteDifference(float altitude);

  /**
   * @fn readAltitudeM
   * @brief Calculate the altitude based on the atmospheric pressure measured by the sensor
   * @return Return altitude, unit: m
   * @attention If the reference value is provided before, the absolute value of the current sealevel is calculated according to the calibrated sea level atmospheric pressure
   */
  float readAltitudeM(void);

  /**
   * @fn getFIFOData
   * @brief Get the cached data in the FIFO
   * @param FIFOTemperatureC the variable for storing temperature measured data
   * @param FIFOPressurePa the variable for storing pressure measured data
   * @brief Temperature unit: °C; Pressure unit: Pa
   * @return None
   */
  void getFIFOData(float &FIFOTemperatureC, float &FIFOPressurePa);

  /**
   * @fn setCommand
   * @brief FIFO empty command and soft reset command of sensor
   * @param mode Basic sensor commands, three types of commands: 
   * @n       BMP3XX_CMD_NOP, Null command
   * @n       BMP3XX_CMD_FIFO_FLUSH, Clear all data in the FIFO without changing its settings  
   * @n       BMP3XX_CMD_SOFTRESET, Trigger a reset, all user configuration settings will be overwritten by their default state
   */
  void setCommand(uint8_t mode);

  /**
   * @fn setFIFOWTM
   * @brief FIFO water level settings configuration
   * @param WTMSetting FIFO water level (0-511) needs to be set. That FIFO fills up to the water level will trigger an interrupt
   */
  void setFIFOWTM(uint16_t WTMSetting);

  /**
   * @fn setFIFOMode1
   * @brief FIFO configuration 1 (FIFO1)
   * @param mode The FIFO mode needs to set, the following modes add up to mode:
   * @n       eFIFODIS: Disable FIFO , eFIFOEN: Enable FIFO
   * @n       eFIFOStopOnFullDIS: Continue writing when full , eFIFOStopOnFullEN: Stop writing when full
   * @n       eFIFOTimeDIS: Disable , eFIFOTimeEN: Enable return to the sensor time frame after the last valid data frame
   * @n       eFIFOPressDIS: Disable pressure data storage , eFIFOPressEN: Enable pressure data storage
   * @n       eFIFOTempDIS: Disable temperature data storage, eFIFOTempEN: Enable temperature data storage
   * @return None
   */
  void setFIFOMode1(uint8_t mode);

  /**
   * @fn setFIFOMode2
   * @brief FIFO Configuration 2 (FIFO2)
   * @param mode The FIFO mode needs to set, the following modes add up to mode:
   * @n       8 FIFO sampling options for pressure and temperature data (1-128), the coefficient is 2^fifo_subsampling(0-7): 
   * @n         eFIFOSubsampling0, eFIFOSubsampling1, eFIFOSubsampling2, eFIFOSubsampling3,
   * @n         eFIFOSubsampling4, eFIFOSubsampling5, eFIFOSubsampling6, eFIFOSubsampling7,
   * @n       eFIFODataSelectDIS: Unfiltered data (compensated or uncompensated) , eFIFODataSelectEN: Filtered data (compensated or uncompensated), plus two retention states: 
   * @n                            the same as "unfilt"
   * @return None
   */
  void setFIFOMode2(uint8_t mode);

  /**
   * @fn setINTMode
   * @brief Interrupt configuration(INT)
   * @param mode The interrupt mode needs to set. The following modes add up to mode: 
   * @n       Interrupt pin output mode: eINTPinPP: Push pull, eINTPinOD: Open drain
   * @n       Interrupt pin active level: eINTPinActiveLevelLow: Active low , eINTPinActiveLevelHigh: Active high
   * @n       Register interrupt latch: eINTLatchDIS: Disable, eINTLatchEN: Enable
   * @n       FIFO water level reached interrupt: eIntFWtmDis: Disable, eIntFWtmEn: Enable
   * @n       FIFO full interrupt: eINTFFullDIS: Disable, eINTFFullEN: Enable
   * @n       Interrupt pin initial (invalid, non-interrupt) level: eINTInitialLevelLOW: low , eINTInitialLevelHIGH: high
   * @n       Interrupt pin initial (invalid, non-interrupt) level:  eINTDataDrdyDIS: Disable , eINTDataDrdyEN: Enable
   * @return None
   */
  void setINTMode(uint8_t mode);

  /**
   * @fn setPWRMode
   * @brief Configure measurement mode and power mode 
   * @param mode The measurement mode and power mode that need to set. The following modes add up to mode:
   * @n       ePressDIS: Disable pressure measurement , ePressEN: Enable pressure measurement
   * @n       eTempDIS: Disable temperature measurement , eTempEN: Enable temperature measurement
   * @n       eSleepMode, eForcedMode, eNormalMode Three modes: 
   * @n         Sleep mode: It will be in sleep mode by default after power-on reset. In this mode, no measurement is performed and power consumption is minimal. 
   * @n                     All registers are accessible for reading the chip ID and compensation coefficient.
   * @n         Forced mode: In this mode, the sensor will take a single measurement according to the selected measurement and filtering options. After the measurement is 
   * @n                      completed, the sensor will return to sleep mode, and the measurement result can be obtained in the register.
   * @n         Normal mode: Continuously loop between the measurement period and the standby period. The output data rates are related to the ODR mode setting.
   * @return None
   */
  void setPWRMode(uint8_t mode);

  /**
   * @fn setOSRMode
   * @brief Configure the oversampling when measuring pressure and temperature (OSR:over-sampling register)
   * @param mode Oversampling mode of pressure and temperature measurement need to be set. The following modes add up to mode:
   * @n       6 pressure oversampling modes:
   * @n          ePressOSRMode1,  Pressure sampling×1, 16 bit / 2.64 Pa(Recommend temperature oversampling×1)
   * @n          ePressOSRMode2,  Pressure sampling×2, 16 bit / 2.64 Pa(Recommend temperature oversampling×1)
   * @n          ePressOSRMode4,  Pressure sampling×4, 18 bit / 0.66 Pa(Recommend temperature oversampling×1)
   * @n          ePressOSRMode8,  Pressure sampling×8, 19 bit / 0.33 Pa(Recommend temperature oversampling×2)
   * @n          ePressOSRMode16,  Pressure sampling×16, 20 bit / 0.17 Pa(Recommend temperature oversampling×2)
   * @n          ePressOSRMode32,  Pressure sampling×32, 21 bit / 0.085 Pa(Recommend temperature oversampling×2)
   * @n        6 temperature oversampling modes
   * @n          eTempOSRMode1,  Temperature sampling×1, 16 bit / 0.0050 °C
   * @n          eTempOSRMode2,  Temperature sampling×2, 16 bit / 0.0025 °C
   * @n          eTempOSRMode4,  Temperature sampling×4, 18 bit / 0.0012 °C
   * @n          eTempOSRMode8,  Temperature sampling×8, 19 bit / 0.0006 °C
   * @n          eTempOSRMode16,  Temperature sampling×16, 20 bit / 0.0003 °C
   * @n          eTempOSRMode32,  Temperature sampling×32, 21 bit / 0.00015 °C
   * @return None
  */
  void setOSRMode(uint8_t mode);

  /**
   * @fn setODRMode
   * @brief Set the output data rate setting in subdivision/sub-sampling mode(ODR:output data rates)
   * @param mode The output data rate needs to be set, configurable mode: 
   * @n       BMP3XX_ODR_200_HZ, BMP3XX_ODR_100_HZ, BMP3XX_ODR_50_HZ, BMP3XX_ODR_25_HZ, BMP3XX_ODR_12P5_HZ, 
   * @n       BMP3XX_ODR_6P25_HZ, BMP3XX_ODR_3P1_HZ, BMP3XX_ODR_1P5_HZ, BMP3XX_ODR_0P78_HZ, BMP3XX_ODR_0P39_HZ, 
   * @n       BMP3XX_ODR_0P2_HZ, BMP3XX_ODR_0P1_HZ, BMP3XX_ODR_0P05_HZ, BMP3XX_ODR_0P02_HZ, BMP3XX_ODR_0P01_HZ, 
   * @n       BMP3XX_ODR_0P006_HZ, BMP3XX_ODR_0P003_HZ, BMP3XX_ODR_0P0015_HZ
   * @return boolean, indicates configuration results
   * @retval True indicates configuration succeeds, successfully update the configuration
   * @retval False indicates configuration fails and remains its original state
   */
  bool setODRMode(uint8_t mode);

  /**
   * @fn setIIRMode
   * @brief IIR filter coefficient configuration (IIR filtering)
   * @param mode Set IIR filter coefficient, configurable mode: 
   * @n       BMP3XX_IIR_CONFIG_COEF_0, BMP3XX_IIR_CONFIG_COEF_1, BMP3XX_IIR_CONFIG_COEF_3, 
   * @n       BMP3XX_IIR_CONFIG_COEF_7, BMP3XX_IIR_CONFIG_COEF_15, BMP3XX_IIR_CONFIG_COEF_31, 
   * @n       BMP3XX_IIR_CONFIG_COEF_63, BMP3XX_IIR_CONFIG_COEF_127
   * @return None
   */
  void setIIRMode(uint8_t mode);

  /**
   * @fn getFIFOLength
   * @brief Get FIFO cached data size
   * @return Range of returned value: 0-511
   */
  uint16_t getFIFOLength(void);

```


## Compatibility

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | :---:
Arduino Uno        |      √       |              |             |
Arduino MEGA2560   |      √       |              |             |
Arduino Leonardo   |      √       |              |             |
FireBeetle-ESP8266 |      √       |              |             |
FireBeetle-ESP32   |      √       |              |             |
FireBeetle-M0      |      √       |              |             |
Micro:bit          |      √       |              |             |
Raspberry Pi       |      √       |              |             |


## History

- 2021/04/20 - Version 1.0.0 released.
- 2021/11/08 - Version 1.0.1 released.
- 2021/12/03 - Version 1.0.2 released.
- 2022/09/21 - Version 1.0.3 released.


## Credits

Written by qsjhyy(yihuan.huang@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))

