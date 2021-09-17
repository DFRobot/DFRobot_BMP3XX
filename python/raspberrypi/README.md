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
<snippet>
<content>


## Summary
* BMP(390L/388) can read temperature and pressure.
* The library supports the SPI/I2C communication.
* BMP(390L/388) also includes FIFO functionality. This greatly improves ease of use.
* Interrupts than can be used in a power efficient manner without using software algorithms.
* BMP390L is more accurate than its predecessors, covering a wide measurement range from 300 hPa to 1250 hPa.
* This new barometric pressure（BMP390L） sensor exhibits an attractive price-performance ratio coupled with low power consumption.
* Due to the built-in hardware synchronization of the sensor（BMP390L） data and its ability to synchronize data from external devices.


## Installation

To use the library, first download the library file, paste it into the directory you specified, then open the Examples folder and run the demo in that folder.


## Methods

```python

    '''
      @brief Initialize sensor
      @return Return True indicate iniatialization succeed, False indicate failed
    '''
    def begin(self)

    '''
      @brief Get pressure measurement value from register, working range (300‒1250 hPa)
             If the reference value is provided before, the absolute value of the current position pressure is calculated according to the calibrated sea level atmospheric pressure
      @return Return pressure measurement, unit: Pa
    '''
    @property
    def get_pressure(self)

    '''
      @brief Take the given current location altitude as the reference value to eliminate the absolute difference for subsequent pressure and altitude data
      @param altitude Altitude in current position
      @return false Return ture indicate pass in reference value succeed, false indicate faile
    '''
    def calibrated_absolute_difference(self, altitude)

    '''
      @brief Get temperature measurement value from register, working range(-40 ‒ +85 °C)
      @return Return temperature measurements, unit: ℃
    '''
    @property
    def get_temperature(self)

    '''
      @brief Calculate the altitude based on the atmospheric pressure measured by the sensor
             If the reference value is provided before, the absolute value of the current position pressure is calculated according to the calibrated sea level atmospheric pressure
      @return Return altitude, unit: m
    '''
    @property
    def get_altitude(self)

    '''
      @brief Commonly used sampling modes that allows users to configure easily
      @param mode:
             ULTRA_LOW_PRECISION，Ultra-low precision, suitable for monitoring weather (lowest power consumption), the power is mandatory mode.
             LOW_PRECISION，Low precision, suitable for random detection, power is normal mode
             NORMAL_PRECISION1，Normal precision 1, suitable for dynamic detection on handheld devices (e.g on mobile phones), power is normal mode
             NORMAL_PRECISION2，Normal precision 2, suitable for drones, power is normal mode
             HIGH_PRECISION，High precision, suitable for low-power handled devices （e.g mobile phones）, power is normal mode
             ULTRA_PRECISION，Ultra-high precision, suitable for indoor navigation, its acquisition rate will be extremely low, and the acquisition cycle is 1000 ms.
      @return Return True indicate configuration succeed, false indicate failed and remain its original state
    '''
    def set_common_sampling_mode(self, mode)

    '''
      @brief Enable or disable FIFO
      @param mode：
             True：Enable FIFO
             False：Disable FIFO
    '''
    def enable_fifo(self, mode)

    '''
      @brief Get the cached data in the FIFO
      @return Return the calibrated pressure data and the calibrated temperature data 
              Temperature unit: °C; Pressure unit: Pa
    '''
    def get_fifo_temp_press_data(self)

    '''
      @brief Get FIFO cached data size
      @return Range of return value: 0-511
    '''
    def get_fifo_length(self)

    '''
      @brief Empty cached data in the FIFO without changing its settings  
    '''
    def empty_fifo(self)

    '''
      @brief Reset and restart the sensor, restoring the sensor configuration to the default configuration
    '''
    def reset(self)

    '''
      @brief Enable interrupt of sensor data ready signal
             Note: As the interrupt pin is unique, the three interrupts are set to be used separately, please note the other two interrupt functions when using
    '''
    def enable_data_ready_int(self)

    '''
      @brief Enable the interrupt of the sensor FIFO reaching the water level signal
             Note: As the interrupt pin is unique, the three interrupts are set to be used separately, please note the other two interrupt functions when using
      @param wtm_value:设定FIFO的水位值（范围：0-511）
    '''
    def enable_fifo_wtm_int(self, wtm_value)

    '''
      @brief Enable the interrupt of the sensor FIFO reaching the water level signal
             Note: As the interrupt pin is unique, the three interrupts are set to be used separately, please note the other two interrupt functions when using
    '''
    def enable_fifo_full_int(self)

    '''
      @brief Configure measurement mode and power mode 
      @param mode The measurement mode and power mode that need to be set.:
             SLEEP_MODE(Sleep mode): It will be in sleep mode by default after power-on reset. In this mode, no measurement is performed and power consumption is minimal. 
                                     All registers are accessible for reading the chip ID and compensation coefficient.
             FORCED_MODE(Forced mode): In this mode, the sensor will take a single measurement according to the selected measurement and filtering options. After the 
                                       measurement is completed, the sensor will return to sleep mode, and the measurement result can be obtained in the register.
             NORMAL_MODE(Normal mode): Continuously loop between the measurement period and the standby period. The output data rates are related to the ODR mode setting.
    '''
    def set_power_mode(self, mode)

    '''
      @brief Configure the oversampling when measuring pressure and temperature (OSR:over-sampling register)
      @param mode Oversampling mode of pressure and temperature measurement need to be set:
             6 pressure oversampling modes:
               BMP3XX_PRESS_OSR_SETTINGS[0], Pressure sampling×1，16 bit / 2.64 Pa（Recommend temperature oversampling×1）
               BMP3XX_PRESS_OSR_SETTINGS[1], Pressure sampling×2，16 bit / 2.64 Pa（Recommend temperature oversampling×1）
               BMP3XX_PRESS_OSR_SETTINGS[2], Pressure sampling×4，18 bit / 0.66 Pa（Recommend temperature oversampling×1）
               BMP3XX_PRESS_OSR_SETTINGS[3], Pressure sampling×8，19 bit / 0.33 Pa（Recommend temperature oversampling×2）
               BMP3XX_PRESS_OSR_SETTINGS[4], Pressure sampling×16，20 bit / 0.17 Pa（Recommend temperature oversampling×2）
               BMP3XX_PRESS_OSR_SETTINGS[5], Pressure sampling×32，21 bit / 0.085 Pa（Recommend temperature oversampling×2）
             6 temperature oversampling modes
               BMP3XX_TEMP_OSR_SETTINGS[0], Temperature sampling×1，16 bit / 0.0050 °C
               BMP3XX_TEMP_OSR_SETTINGS[1], Temperature sampling×2，16 bit / 0.0025 °C
               BMP3XX_TEMP_OSR_SETTINGS[2], Temperature sampling×4，18 bit / 0.0012 °C
               BMP3XX_TEMP_OSR_SETTINGS[3], Temperature sampling×8，19 bit / 0.0006 °C
               BMP3XX_TEMP_OSR_SETTINGS[4], Temperature sampling×16，20 bit / 0.0003 °C
               BMP3XX_TEMP_OSR_SETTINGS[5], Temperature sampling×32，21 bit / 0.00015 °C
    '''
    def set_oversampling(self, press_osr_set, temp_osr_set)

    '''
      @brief IIR filter coefficient setting（IIR filtering）
      @param mode Set IIR filter coefficient, configurable modes：
             BMP3XX_IIR_CONFIG_COEF_0，BMP3XX_IIR_CONFIG_COEF_1，BMP3XX_IIR_CONFIG_COEF_3，
             BMP3XX_IIR_CONFIG_COEF_7，BMP3XX_IIR_CONFIG_COEF_15，BMP3XX_IIR_CONFIG_COEF_31，
             BMP3XX_IIR_CONFIG_COEF_63，BMP3XX_IIR_CONFIG_COEF_127
    '''
    def filter_coefficient(self, iir_config_coef)

    '''
      @brief Set output data rate in subdivision/sub-sampling mode(ODR:output data rates)
      @param mode The output data rate needs to be set, configurable mode：
             BMP3XX_ODR_200_HZ，BMP3XX_ODR_100_HZ，BMP3XX_ODR_50_HZ，BMP3XX_ODR_25_HZ，BMP3XX_ODR_12P5_HZ，
             BMP3XX_ODR_6P25_HZ，BMP3XX_ODR_3P1_HZ，BMP3XX_ODR_1P5_HZ，BMP3XX_ODR_0P78_HZ，BMP3XX_ODR_0P39_HZ，
             BMP3XX_ODR_0P2_HZ，BMP3XX_ODR_0P1_HZ，BMP3XX_ODR_0P05_HZ，BMP3XX_ODR_0P02_HZ，BMP3XX_ODR_0P01_HZ，
             BMP3XX_ODR_0P006_HZ，BMP3XX_ODR_0P003_HZ，BMP3XX_ODR_0P0015_HZ
      @return Return True indicate configuration succeed, False indicate failed and remain its original state
    '''
    def set_output_data_rates(self, odr_set)

```


## Compatibility

* RaspberryPi Version

| Board        | Work Well | Work Wrong | Untested | Remarks |
| ------------ | :-------: | :--------: | :------: | ------- |
| RaspberryPi2 |           |            |    √     |         |
| RaspberryPi3 |           |            |    √     |         |
| RaspberryPi4 |     √     |            |          |         |

* Python Version

| Python  | Work Well | Work Wrong | Untested | Remarks |
| ------- | :-------: | :--------: | :------: | ------- |
| Python2 |           |      √     |          |         |
| Python3 |     √     |            |          |         |


## History

- data 2021-03-10
- version V0.1


## Credits

Written by qsj(qsj.huang@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))
