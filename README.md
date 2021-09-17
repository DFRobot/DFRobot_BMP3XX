# DFRobot_BMP3XX
* [中文版](./README_CN.md)

This is a Library for BMP3XX, the function is to read temperature and pressure.
The BMP(390L/388) is a digital sensor with pressure and temperature measurement based on proven sensing principles. The 
sensor module is housed in an extremely compact 10-pin metal-lid LGA package with a footprint of only 2.0 × 2.0 mm² and 
max 0.8 mm package height. Its small dimensions and its low power consumption of 3.2 µA @1Hz allow the implementation 
in battery driven devices such as mobile phones, GPS modules or watches.

<img src="https://ws.dfrobot.com.cn/FgGMuOYn58ZHD5s6jcOWoRUwVlOh" width="450" hegiht="" align=right/>


## Product Link (https://www.dfrobot.com.cn/goods-1392.html)
    SKU：SEN0423/SEN0251


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
* This new barometric pressure（BMP390L） sensor exhibits an attractive price-performance ratio coupled with low power consumption.
* Due to the built-in hardware synchronization of the sensor（BMP390L） data and its ability to synchronize data from external devices.

## Installation

To use this library, first download the library file, paste it into the \Arduino\libraries directory, 
then open the examples folder and run the demo in the folder.

## Methods

```C++

  /**
  * @brief Initialize function
  * @return Return 0 to indicate initialization succeed, return other values indicate failed and return an error code
  */
  virtual int begin(void);

  /**
  * @brief commonly used sampling modes that allows users to configure easily
  * @param mode:
  *        eUltraLowPrecision，Ultra-low precision, suitable for monitoring weather (lowest power consumption), the power is mandatory mode.
  *        eLowPrecision，Low precision, suitable for random detection, power is normal mode
  *        eNormalPrecision1，Normal precision 1, suitable for dynamic detection on handheld devices (e.g on mobile phones), power is normal mode
  *        eNormalPrecision2，Normal precision 2, suitable for drones, power is normal mode
  *        eHighPrecision，High precision, suitable for low-power handled devices （e.g mobile phones）, power is normal mode
  *        eUltraPrecision，Ultra-high precision, suitable for indoor navigation, its acquisition rate will be extremely low, and the acquisition cycle is 1000 ms.
  * @return Return True indicate configuration succeed, False indicate failed and remain its original state
  */
  bool setSamplingMode(ePrecisionMode_t mode);

  /**
  * @brief Get the sampling period in the current mode
  * @return Return sampling period, unit: us
  */
  uint32_t getSamplingPeriodUS(void);

  /**
  * @brief Get temperature measurement value from register, working range (-40 ‒ +85 °C)
  * @return Return temperature measurements, unit: °C
  */
  float readTempC(void);

  /**
  * @brief Get pressure measurement value from register, working range（300‒1250 hPa）
  * @brief If the reference value is provided before, the absolute value of the current position pressure is calculated according to the calibrated sea level 
  * @n     atmospheric pressure
  * @return Return pressure measurements, unit: Pa
  */
  float readPressPa(void);

  /**
  * @brief Take the given current location altitude as the reference value to eliminate the absolute difference for subsequent pressure and altitude data
  * @param altitude Altitude in current position
  * @return If pass in the reference value successfully, return turn. If failed, return false.
  */
  bool calibratedAbsoluteDifference(float altitude);

  /**
  * @brief Calculate the altitude based on the atmospheric pressure measured by the sensor
  * @brief If the reference value is provided before, the absolute value of the current sealevel is calculated according to the calibrated sea level atmospheric pressure
  * @return Return altitude, unit: m
  */
  float readAltitudeM(void);

  /**
  * @brief Get the cached data in the FIFO
  * @brief Temperature unit: °C; Pressure unit: Pa
  */
  void getFIFOData(float &FIFOTemperatureC, float &FIFOPressurePa);

  /**
  * @brief FIFO empty command and soft reset command of sensor
  * @param mode Basic sensor commands, three types of commands: 
  *        BMP3XX_CMD_NOP, Null command
  *        BMP3XX_CMD_FIFO_FLUSH, Clear all data in the FIFO without changing its settings  
  *        BMP3XX_CMD_SOFTRESET, Trigger a reset, all user configuration settings will be overwritten by their default state
  */
  void setCommand(uint8_t mode);

  /**
  * @brief FIFO water level settings configuration
  * @param WTMSetting FIFO water level (0-511) needs to be set. That FIFO fills up to the water level will trigger an interrupt
  */
  void setFIFOWTM(uint16_t WTMSetting);

  /**
  * @brief FIFO configuration 1 (FIFO1)
  * @param mode The FIFO mode needs to set, the following modes add up to mode:
  *        eFIFODIS： Disable FIFO ，eFIFOEN： Enable FIFO
  *        eFIFOStopOnFullDIS： Continue writing when full ，eFIFOStopOnFullEN：Stop writing when full
  *        eFIFOTimeDIS： Disable ，eFIFOTimeEN： Enable return to the sensor time frame after the last valid data frame
  *        eFIFOPressDIS： Disable pressure data storage ，eFIFOPressEN：Enable pressure data storage
  *        eFIFOTempDIS： Disable temperature data storage，eFIFOTempEN：Enable temperature data storage
  */
  void setFIFOMode1(uint8_t mode);

  /**
  * @brief FIFO Configuration 2(FIFO2)
  * @param mode The FIFO mode needs to set, the following modes add up to mode:
  *        8 FIFO sampling options for pressure and temperature data (1-128), the coefficient is 2^fifo_subsampling(0-7)：
  *          eFIFOSubsampling0, eFIFOSubsampling1, eFIFOSubsampling2, eFIFOSubsampling3,
  *          eFIFOSubsampling4, eFIFOSubsampling5, eFIFOSubsampling6, eFIFOSubsampling7,
  *        eFIFODataSelectDIS： Unfiltered data (compensated or uncompensated) ，eFIFODataSelectEN： Filtered data (compensated or uncompensated), plus two retention states: 
  *                             the same as "unfilt"
  */
  void setFIFOMode2(uint8_t mode);

  /**
  * @brief Interrupt configuration(INT)
  * @param mode The interrupt mode needs to set. The following modes add up to mode: 
  *        Interrupt pin output mode: eINTPinPP： Push pull，eINTPinOD：Open drain
  *        Interrupt pin active level: eINTPinActiveLevelLow： Active low ，eINTPinActiveLevelHigh：Active high
  *        Register interrupt latch: eINTLatchDIS： Disable，eINTLatchEN：Enable
  *        FIFO water level reached interrupt: eINTFWTMDIS： Disable，eINTFWTMEN：Enable
  *        FIFO full interrupt: eINTFFullDIS： Disable，eINTFFullEN： Enable
  *        Interrupt pin initial (invalid, non-interrupt) level: eINTInitialLevelLOW：low ，eINTInitialLevelHIGH：high
  *        Interrupt pin initial (invalid, non-interrupt) level:  eINTDataDrdyDIS： Disable ，eINTDataDrdyEN： Enable
  */
  void setINTMode(uint8_t mode);

  /**
  * @brief Configure measurement mode and power mode 
  * @param mode The measurement mode and power mode that need to set. The following modes add up to mode:
  *        ePressDIS：Disable pressure measurement ，ePressEN： Enable pressure measurement
  *        eTempDIS： Disable temperature measurement ，eTempEN： Enable temperature measurement
  *        eSleepMode, eForcedMode, eNormalMode Three modes：
  *          Sleep mode: It will be in sleep mode by default after power-on reset. In this mode, no measurement is performed and power consumption is minimal. 
  *                      All registers are accessible for reading the chip ID and compensation coefficient.
  *          Forced mode: In this mode, the sensor will take a single measurement according to the selected measurement and filtering options. After the measurement is 
  *                       completed, the sensor will return to sleep mode, and the measurement result can be obtained in the register.
  *          Normal mode: Continuously loop between the measurement period and the standby period. The output data rates are related to the ODR mode setting.
  */
  void setPWRMode(uint8_t mode);

  /**
  * @brief Configure the oversampling when measuring pressure and temperature (OSR:over-sampling register)
  * @param mode Oversampling mode of pressure and temperature measurement need to be set. The following modes add up to mode:
         6 pressure oversampling modes:
            ePressOSRMode1,  Pressure sampling×1，16 bit / 2.64 Pa（Recommend temperature oversampling×1）
            ePressOSRMode2,  Pressure sampling×2，16 bit / 2.64 Pa（Recommend temperature oversampling×1）
            ePressOSRMode4,  Pressure sampling×4，18 bit / 0.66 Pa（Recommend temperature oversampling×1）
            ePressOSRMode8,  Pressure sampling×8，19 bit / 0.33 Pa（Recommend temperature oversampling×2）
            ePressOSRMode16,  Pressure sampling×16，20 bit / 0.17 Pa（Recommend temperature oversampling×2）
            ePressOSRMode32,  Pressure sampling×32，21 bit / 0.085 Pa（Recommend temperature oversampling×2）
          6 temperature oversampling modes
            eTempOSRMode1,  Temperature sampling×1，16 bit / 0.0050 °C
            eTempOSRMode2,  Temperature sampling×2，16 bit / 0.0025 °C
            eTempOSRMode4,  Temperature sampling×4，18 bit / 0.0012 °C
            eTempOSRMode8,  Temperature sampling×8，19 bit / 0.0006 °C
            eTempOSRMode16,  Temperature sampling×16，20 bit / 0.0003 °C
            eTempOSRMode32,  Temperature sampling×32，21 bit / 0.00015 °C
  */
  void setOSRMode(uint8_t mode);

  /**
  * @brief Set the output data rate setting in subdivision/sub-sampling mode(ODR:output data rates)
  * @param mode The output data rate needs to be set, configurable mode：
  *        BMP3XX_ODR_200_HZ，BMP3XX_ODR_100_HZ，BMP3XX_ODR_50_HZ，BMP3XX_ODR_25_HZ，BMP3XX_ODR_12P5_HZ，
  *        BMP3XX_ODR_6P25_HZ，BMP3XX_ODR_3P1_HZ，BMP3XX_ODR_1P5_HZ，BMP3XX_ODR_0P78_HZ，BMP3XX_ODR_0P39_HZ，
  *        BMP3XX_ODR_0P2_HZ，BMP3XX_ODR_0P1_HZ，BMP3XX_ODR_0P05_HZ，BMP3XX_ODR_0P02_HZ，BMP3XX_ODR_0P01_HZ，
  *        BMP3XX_ODR_0P006_HZ，BMP3XX_ODR_0P003_HZ，BMP3XX_ODR_0P0015_HZ
  * @return Return True indicate configuration succeed, False indicate failed and remains its original state 
  bool setODRMode(uint8_t mode);

  /**
  * @brief IIR filter coefficient configuration (IIR filtering)
  * @param mode Set IIR filter coefficient, configurable mode：
  *        BMP3XX_IIR_CONFIG_COEF_0，BMP3XX_IIR_CONFIG_COEF_1，BMP3XX_IIR_CONFIG_COEF_3，
  *        BMP3XX_IIR_CONFIG_COEF_7，BMP3XX_IIR_CONFIG_COEF_15，BMP3XX_IIR_CONFIG_COEF_31，
  *        BMP3XX_IIR_CONFIG_COEF_63，BMP3XX_IIR_CONFIG_COEF_127
  */
  void setIIRMode(uint8_t mode);

  /**
  * @brief Get FIFO cached data size
  * @return Range of returned value: 0-511
  */
  uint16_t getFIFOLength(void);

  /**
  * @brief Get the water level set by FIFO
  * @return Range of returned value: 0-511
  */
  uint16_t getFIFOWTMValue(void);

```

## Compatibility

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | -----
Arduino Uno        |      √       |              |             | 
Firebeetle ESP8266 |      √       |              |             | 
Firebeetle ESP32-E |      √       |              |             | 
FireBeetle M0      |      √       |              |             | 
micro:bit          |      √       |              |             | 


## History

- data 2021-4-20
- version V0.1


## Credits

Written by qsj(qsj.huang@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))





