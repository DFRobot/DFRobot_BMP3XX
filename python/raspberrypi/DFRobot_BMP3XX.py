# -*- coding: utf-8 -*
'''
  @file DFRobot_BMP3XX.py
  @brief Define the infrastructure of DFRobot_BMP3XX class
  @n This is a pressure and temperature sensor that can be controlled via IIC and SPI port.
  @n BMP(390L/388)has temperature compensation, data oversampling, IIR filter, binary sampling and other functions
  @n These functions improve the accuracy of data collected by the BMP (390L/388) sensor.
  @n BMP (390L/388) also has a 512-byte FIFO data buffer, which greatly improves its usability.
  @n Similarly, BMP (390L/388) has an interrupt pin, which can be used in a power-saving way without using software algorithms.
  @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @licence     The MIT License (MIT)
  @author [qsj.huang](li.feng@dfrobot.com)
  @version  V1.0
  @date  2021-5-6
  @get from https://www.dfrobot.com
  @url https://github.com/DFRobot/DFRobot_BMP3XX
'''
import sys
import time
import smbus
import spidev
import RPi.GPIO as GPIO
import logging
from ctypes import *

logger = logging.getLogger()
logger.setLevel(logging.INFO)  #Display all print information
#logger.setLevel(logging.FATAL)#If you don’t want to display too many prints, only print errors, please use this option
ph = logging.StreamHandler()
formatter = logging.Formatter("%(asctime)s - [%(filename)s %(funcName)s]:%(lineno)d - %(levelname)s: %(message)s")
ph.setFormatter(formatter) 
logger.addHandler(ph)

DFROBOT_BMP3XX_IIC_ADDR_SDO_GND = 0x76  # IIC communication address when SDO is grounded
DFROBOT_BMP3XX_IIC_ADDR_SDO_VDD = 0x77  # IIC communication address when SDO is connected to power

BMP388_CHIP_ID = 0x50      # BMP388 chip version
BMP390L_CHIP_ID = 0x60     # BMP390L chip version

# BMP3XX寄存器地址
BMP3XX_CHIP_ID = 0x00      # The “CHIP_ID” register contains the chip identification code.
BMP3XX_REV_ID = 0x01       # The “Rev_ID” register contains the mask revision of the ASIC.
BMP3XX_ERR_REG = 0x02      # Sensor Error conditions are reported in the “ERR_REG” register.
BMP3XX_STATUS = 0x03       # The Sensor Status Flags are stored in the “STATUS” register.
BMP3XX_P_DATA_PA = 0x04    # The 24Bit pressure data is split and stored in three consecutive registers.
BMP3XX_T_DATA_C = 0x07     # The 24Bit temperature data is split and stored in three consecutive registered.
BMP3XX_TIME = 0x0C         # The 24Bit sensor time is split and stored in three consecutive registers.
BMP3XX_EVENT = 0x10        # The “EVENT” register contains the sensor status flags.
BMP3XX_INT_STATUS = 0x11   # The “INT_STATUS” register shows interrupt status and is cleared after reading.
BMP3XX_FIFO_LENGTH = 0x12  # The FIFO byte counter indicates the current fill level of the FIFO buffer.
BMP3XX_FIFO_DATA = 0x14    # The “FIFO_DATA” is the data output register.
BMP3XX_FIFO_WTM_0 = 0x15   # The FIFO Watermark size is 9 Bit and therefore written to the FIFO_WTM_0 and FIFO_WTM_1 registers.
BMP3XX_FIFO_WTM_1 = 0x16   # The FIFO Watermark size is 9 Bit and therefore written to the FIFO_WTM_0 and FIFO_WTM_1 registers.
BMP3XX_FIFO_CONF_1 = 0x17  # The “FIFO_CONFIG_1” register contains the FIFO frame content configuration.
BMP3XX_FIFO_CONF_2 = 0x18  # The “FIFO_CONFIG_2” register extends the FIFO_CONFIG_1 register.
BMP3XX_INT_CTRL = 0x19     # Interrupt configuration can be set in the “INT_CTRL” register.
BMP3XX_IF_CONF = 0x1A      # The “IF_CONF” register controls the serial interface settings.
BMP3XX_PWR_CTRL = 0x1B     # The “PWR_CTRL” register enables or disables pressure and temperature measurement.
BMP3XX_OSR = 0x1C          # The “OSR” register controls the oversampling settings for pressure and temperature measurements.
BMP3XX_ODR = 0x1D          # The “ODR” register set the configuration of the output data rates by means of setting the subdivision/subsampling.
BMP3XX_IIR_CONFIG = 0x1F   # The “CONFIG” register controls the IIR filter coefficients
BMP3XX_CALIB_DATA = 0x31   # 0x31-0x45 is calibration data
BMP3XX_CMD = 0x7E          # Command register, can soft reset and clear all FIFO data

# Sensor configuration
SLEEP_MODE  = 0x00   # Sleep mode: It will be in sleep mode by default after power-on reset. In this mode, no measurement is performed and power consumption is minimal. 
                     #             All registers are accessible for reading the chip ID and compensation coefficient. */
FORCED_MODE = 0x10   # Forced mode：In this mode, the sensor will take a single measurement according to the selected measurement and filtering options. After the
                     #           measurement is completed, the sensor will return to sleep mode, and the measurement result can be obtained in the register. */
NORMAL_MODE = 0x30   # Normal mode: Continuously loop between the measurement period and the standby period. The output data rates are related to the ODR mode setting.*/

BMP3XX_PRESS_OSR_SETTINGS = (0x00, 0x01, 0x02, 0x03, 0x04, 0x05)  # pressure oversampling settings
BMP3XX_TEMP_OSR_SETTINGS = (0x00, 0x08, 0x10, 0x18, 0x20, 0x28)  # temperature oversampling settings

BMP3XX_IIR_CONFIG_COEF_0   = 0x00   # Filter coefficient is 0 ->Bypass mode
BMP3XX_IIR_CONFIG_COEF_1   = 0x02   # Filter coefficient is 1
BMP3XX_IIR_CONFIG_COEF_3   = 0x04   # Filter coefficient is 3
BMP3XX_IIR_CONFIG_COEF_7   = 0x06   # Filter coefficient is 7
BMP3XX_IIR_CONFIG_COEF_15  = 0x08   # Filter coefficient is 15
BMP3XX_IIR_CONFIG_COEF_31  = 0x0A   # Filter coefficient is 31
BMP3XX_IIR_CONFIG_COEF_63  = 0x0C   # Filter coefficient is 63
BMP3XX_IIR_CONFIG_COEF_127 = 0x0E   # Filter coefficient is 127

BMP3XX_ODR_200_HZ    = 0x00   # Prescale:1; ODR 200Hz; Sampling period:5 ms
BMP3XX_ODR_100_HZ    = 0x01   # Prescale:2; Sampling period:10 ms
BMP3XX_ODR_50_HZ     = 0x02   # Prescale:4; Sampling period:20 ms
BMP3XX_ODR_25_HZ     = 0x03   # Prescale:8; Sampling period:40 ms
BMP3XX_ODR_12P5_HZ   = 0x04   # Prescale:16; Sampling period:80 ms
BMP3XX_ODR_6P25_HZ   = 0x05   # Prescale:32; Sampling period:160 ms
BMP3XX_ODR_3P1_HZ    = 0x06   # Prescale:64; Sampling period:320 ms
BMP3XX_ODR_1P5_HZ    = 0x07   # Prescale:127; Sampling period:640 ms
BMP3XX_ODR_0P78_HZ   = 0x08   # Prescale:256; Sampling period:1.280 s
BMP3XX_ODR_0P39_HZ   = 0x09   # Prescale:512; Sampling period:2.560 s
BMP3XX_ODR_0P2_HZ    = 0x0A   # Prescale:1024 Sampling period:5.120 s
BMP3XX_ODR_0P1_HZ    = 0x0B   # Prescale:2048; Sampling period:10.24 s
BMP3XX_ODR_0P05_HZ   = 0x0C   # Prescale:4096; Sampling period:20.48 s
BMP3XX_ODR_0P02_HZ   = 0x0D   # Prescale:8192; Sampling period:40.96 s
BMP3XX_ODR_0P01_HZ   = 0x0E   # Prescale:16384; Sampling period:81.92 s
BMP3XX_ODR_0P006_HZ  = 0x0F   # Prescale:32768; Sampling period:163.84 s
BMP3XX_ODR_0P003_HZ  = 0x10   # Prescale:65536; Sampling period:327.68 s
BMP3XX_ODR_0P0015_HZ = 0x11   # Prescale:131072; ODR 25/16384Hz; Sampling period:655.36 s

# 6 commonly used sampling modes
ULTRA_LOW_PRECISION = 0   # Ultra-low precision, suitable for monitoring weather (lowest power consumption), the power is mandatory mode.
LOW_PRECISION = 1         # Low precision, suitable for casual detection, power is normal mode
NORMAL_PRECISION1 = 2     # Normal precision 1, suitable for dynamic detection on handheld devices (e.g on mobile phones), power is normal mode
NORMAL_PRECISION2 = 3     # Normal precision 2, suitable for drones, power is normal mode
HIGH_PRECISION = 4        # High precision, suitable for low-power handled devices （e.g mobile phones）, power is normal mode
ULTRA_PRECISION = 5       # Ultra-high precision, suitable for indoor navigation, its acquisition rate will be extremely low, and the acquisition cycle is 1000 ms.

# CMD(0x7E) Rigister command 
BMP3XX_CMD_NOP = 0x00         # reserved. No command.
BMP3XX_CMD_FIFO_FLUSH = 0xB0  # Clears all data in the FIFO, does not change FIFO_CONFIG registers.
BMP3XX_CMD_RESET = 0xB6       # Triggers a reset, all user configuration settings are overwritten with their default state.

# FIFO Header
BMP3_FIFO_TEMP_PRESS_FRAME = 0x94   # FIFO temperature pressure header frame
BMP3_FIFO_TEMP_FRAME = 0x90         # FIFO temperature header frame
BMP3_FIFO_PRESS_FRAME = 0x84        # FIFO pressure header frame
BMP3_FIFO_TIME_FRAME = 0xA0         # FIFO time header frame
BMP3_FIFO_CONFIG_CHANGE = 0x48      # FIFO configuration change header frame
BMP3_FIFO_ERROR_FRAME = 0x44        # FIFO error header frame

# Immediate data
BMP3XX_FIFO_DATA_FRAME_LENGTH = 7         # The byte length of each data in a frame of FIFO data is 3
STANDARD_SEA_LEVEL_PRESSURE_PA = 101325   # Standard sea level pressure, unit: pa


class DFRobot_BMP3XX:
    '''
      @brief Module init
    '''
    def __init__(self):
        '''Sea level pressure in Pa.'''
        self.sea_level_pressure = STANDARD_SEA_LEVEL_PRESSURE_PA

    '''
      @brief Initialize sensor
      @return Return True indicate initialization succeed, return False indicate failed
    '''
    def begin(self):
        ret = True
        chip_id = self._read_reg(BMP3XX_CHIP_ID, 1)
        logger.info(chip_id[0])
        if chip_id[0] not in (BMP388_CHIP_ID, BMP390L_CHIP_ID):
            ret = False
        self._get_coefficients()
        self.reset()
        return ret

    '''
      @brief Get pressure measurement value from register, working range (300‒1250 hPa)
             If the reference value is provided before, the absolute value of the current position pressure is calculated according to the calibrated sea level 
             atmospheric pressure
      @return Return pressure measurements, unit: Pa
    '''
    @property
    def get_pressure(self):
        adc_p, adc_t = self._get_reg_temp_press_data()
        return round(self._compensate_data(adc_p, adc_t)[0], 2)

    '''
      @brief Get pressure measurement value from register, working range (-40 ‒ +85 °C)
      @return Return temperature measurements, unit: °C
    '''
    @property
    def get_temperature(self):
        adc_p, adc_t = self._get_reg_temp_press_data()
        return round(self._compensate_data(adc_p, adc_t)[1], 2)

    '''
      @brief Take the given current location altitude as the reference value to eliminate the absolute difference for subsequent pressure and altitude data
      @param altitude Altitude in current position
      @return Pass the benchmark value successfully will return ture, if failed it will return false
    '''
    def calibrated_absolute_difference(self, altitude):
        '''The altitude in meters based on the currently set sea level pressure.'''
        ret = False
        if STANDARD_SEA_LEVEL_PRESSURE_PA == self.sea_level_pressure:
            self.sea_level_pressure = (self.get_pressure / pow(1.0 - (altitude / 44307.7), 5.255302))
            ret = True
        return ret

    '''
      @brief Calculate the altitude based on the atmospheric pressure measured by the sensor
             If the reference value is provided before, the absolute value of the current position pressure is calculated according to the calibrated sea 
             level atmospheric pressure
      @return Return altitude， unit: m
    '''
    @property
    def get_altitude(self):
        # see https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
        return round(44307.7 * (1 - (self.get_pressure / STANDARD_SEA_LEVEL_PRESSURE_PA) ** 0.190284), 2)

    '''
      @brief Configure measurement mode and power mode 
      @param mode The measurement mode and power mode that need to be set:
             SLEEP_MODE(Sleep mode): It will be in sleep mode by default after power-on reset. In this mode, no measurement is performed and power consumption is minimal. 
                                     All registers are accessible for reading the chip ID and compensation coefficient.
             FORCED_MODE(Forced mode): In this mode, the sensor will take a single measurement according to the selected measurement and filtering options. 
                                       After the measurement is completed, the sensor will return to sleep mode, and the measurement result can be obtained in the register.
             NORMAL_MODE(Normal mode): Continuously loop between the measurement period and the standby period. The output data rates are related to the ODR mode setting.
    '''
    def set_power_mode(self, mode):
        temp = self._read_reg(BMP3XX_PWR_CTRL, 1)[0]
        if (mode | 0x03) == temp:
            logger.info("Same configuration as before!")
        else:
            if mode != SLEEP_MODE:
                self._write_reg(BMP3XX_PWR_CTRL, (SLEEP_MODE & 0x30) | 0x03)
                time.sleep(0.02)
            self._write_reg(BMP3XX_PWR_CTRL, (mode & 0x30) | 0x03)
            time.sleep(0.02)

    '''
      @brief Enbale or disable FIFO
      @param mode：
             True: Enable FIFO
             False: Disable FIFO
    '''
    def enable_fifo(self, mode):
        if mode:
            '''Enable and initialize FIFO configuration'''
            self._write_reg(BMP3XX_FIFO_CONF_1, 0x1D)
            self._write_reg(BMP3XX_FIFO_CONF_2, 0x0C)
        else:
            '''关闭FIFO。'''
            self._write_reg(BMP3XX_FIFO_CONF_1, 0x1C)
            self._write_reg(BMP3XX_FIFO_CONF_2, 0x0C)
        time.sleep(0.02)

    '''
      @brief Configure the oversampling when measuring pressure and temperature (OSR:over-sampling register)
      @param mode Oversampling mode of pressure and temperature measurement need to be set:
             6 pressure oversampling mode:
               BMP3XX_PRESS_OSR_SETTINGS[0], Pressure sampling×1，16 bit / 2.64 Pa (Recommend temperature oversampling×1)
               BMP3XX_PRESS_OSR_SETTINGS[1], Pressure sampling×2，16 bit / 2.64 Pa (Recommend temperature oversampling×1)
               BMP3XX_PRESS_OSR_SETTINGS[2], Pressure sampling×4，18 bit / 0.66 Pa (Recommend temperature oversampling×1)
               BMP3XX_PRESS_OSR_SETTINGS[3], Pressure sampling×8，19 bit / 0.33 Pa (Recommend temperature oversampling×2)
               BMP3XX_PRESS_OSR_SETTINGS[4], Pressure sampling×16，20 bit / 0.17 Pa (Recommend temperature oversampling×2)
               BMP3XX_PRESS_OSR_SETTINGS[5], Pressure sampling×32，21 bit / 0.085 Pa (Recommend temperature oversampling×2)
             6 temperature oversampling mode
               BMP3XX_TEMP_OSR_SETTINGS[0], Temperature sampling×1，16 bit / 0.0050 °C
               BMP3XX_TEMP_OSR_SETTINGS[1], Temperature sampling×2，16 bit / 0.0025 °C
               BMP3XX_TEMP_OSR_SETTINGS[2], Temperature sampling×4，18 bit / 0.0012 °C
               BMP3XX_TEMP_OSR_SETTINGS[3], Temperature sampling×8，19 bit / 0.0006 °C
               BMP3XX_TEMP_OSR_SETTINGS[4], Temperature sampling×16，20 bit / 0.0003 °C
               BMP3XX_TEMP_OSR_SETTINGS[5], Temperature sampling×32，21 bit / 0.00015 °C
    '''
    def set_oversampling(self, press_osr_set, temp_osr_set):
        self._write_reg(BMP3XX_OSR, (press_osr_set + temp_osr_set) & 0x3F)

    '''
      @brief IIR filter coefficient setting（IIR filtering）
      @param mode Set IIR filter coefficient, configurable mode:
             BMP3XX_IIR_CONFIG_COEF_0，BMP3XX_IIR_CONFIG_COEF_1，BMP3XX_IIR_CONFIG_COEF_3，
             BMP3XX_IIR_CONFIG_COEF_7，BMP3XX_IIR_CONFIG_COEF_15，BMP3XX_IIR_CONFIG_COEF_31，
             BMP3XX_IIR_CONFIG_COEF_63，BMP3XX_IIR_CONFIG_COEF_127
    '''
    def filter_coefficient(self, iir_config_coef):
        '''The IIR filter coefficient.'''
        self._write_reg(BMP3XX_IIR_CONFIG, iir_config_coef & 0x0E)

    '''
      @brief Set output data rate in subdivision/sub-sampling mode (ODR:output data rates)
      @param mode The output data rate needs to be set, configurable mode:
             BMP3XX_ODR_200_HZ，BMP3XX_ODR_100_HZ，BMP3XX_ODR_50_HZ，BMP3XX_ODR_25_HZ，BMP3XX_ODR_12P5_HZ，
             BMP3XX_ODR_6P25_HZ，BMP3XX_ODR_3P1_HZ，BMP3XX_ODR_1P5_HZ，BMP3XX_ODR_0P78_HZ，BMP3XX_ODR_0P39_HZ，
             BMP3XX_ODR_0P2_HZ，BMP3XX_ODR_0P1_HZ，BMP3XX_ODR_0P05_HZ，BMP3XX_ODR_0P02_HZ，BMP3XX_ODR_0P01_HZ，
             BMP3XX_ODR_0P006_HZ，BMP3XX_ODR_0P003_HZ，BMP3XX_ODR_0P0015_HZ
      @return Return True indicate configuration succeed, false indicate failed and remain its original state
    '''
    def set_output_data_rates(self, odr_set):
        '''The IIR filter coefficient.'''
        ret = True
        self._write_reg(BMP3XX_ODR, odr_set & 0x1F)
        if (self._read_reg(BMP3XX_ERR_REG, 1)[0] & 0x04):
            logger.warning("Sensor configuration error detected!")
            ret = False
        return ret

    '''
      @brief Convert the incoming uint8 type data to int type
      @return data converted to int type
    '''
    def _uint8_to_int(self,num):
        if(num>127):
            num = num - 256
        return num

    '''
      @brief Convert the incoming uint16 type data to int type
      @return data converted to int type
    '''
    def _uint16_to_int(self,num):
        if(num>32767):
            num = num - 65536
        return num

    '''
      @brief   6 commonly used sampling modes that allows users to configure easily
      @param mode:
             ULTRA_LOW_PRECISION，Ultra-low precision, suitable for monitoring weather (lowest power consumption), the power is mandatory mode.
             LOW_PRECISION，Low precision, suitable for random detection, power is normal mode
             NORMAL_PRECISION1，Normal precision 1, suitable for dynamic detection on handheld devices (e.g on mobile phones), power is normal mode
             NORMAL_PRECISION2，Normal precision 2, suitable for drones, power is normal mode
             HIGH_PRECISION，High precision, suitable for low-power handled devices （e.g mobile phones）, power is normal mode
             ULTRA_PRECISION，Ultra-high precision, suitable for indoor navigation, its acquisition rate will be extremely low, and the acquisition cycle is 1000 ms.
      @return Return True indicate configuration succeed, false indicate failed and remains its original state.
    '''
    def set_common_sampling_mode(self, mode):
        ret = True
        if mode == ULTRA_LOW_PRECISION:
            self.set_power_mode(FORCED_MODE)
            self.set_oversampling(BMP3XX_PRESS_OSR_SETTINGS[0], BMP3XX_TEMP_OSR_SETTINGS[0])
            self.filter_coefficient(BMP3XX_IIR_CONFIG_COEF_0)
            self.set_output_data_rates(BMP3XX_ODR_0P01_HZ)
        elif mode == LOW_PRECISION:
            self.set_power_mode(NORMAL_MODE)
            self.set_oversampling(BMP3XX_PRESS_OSR_SETTINGS[1], BMP3XX_TEMP_OSR_SETTINGS[0])
            self.filter_coefficient(BMP3XX_IIR_CONFIG_COEF_0)
            self.set_output_data_rates(BMP3XX_ODR_100_HZ)
        elif mode == NORMAL_PRECISION1:
            self.set_power_mode(NORMAL_MODE)
            self.set_oversampling(BMP3XX_PRESS_OSR_SETTINGS[2], BMP3XX_TEMP_OSR_SETTINGS[0])
            self.filter_coefficient(BMP3XX_IIR_CONFIG_COEF_3)
            self.set_output_data_rates(BMP3XX_ODR_50_HZ)
        elif mode == NORMAL_PRECISION2:
            self.set_power_mode(NORMAL_MODE)
            self.set_oversampling(BMP3XX_PRESS_OSR_SETTINGS[3], BMP3XX_TEMP_OSR_SETTINGS[0])
            self.filter_coefficient(BMP3XX_IIR_CONFIG_COEF_1)
            self.set_output_data_rates(BMP3XX_ODR_50_HZ)
        elif mode == HIGH_PRECISION:
            self.set_power_mode(NORMAL_MODE)
            self.set_oversampling(BMP3XX_PRESS_OSR_SETTINGS[3], BMP3XX_TEMP_OSR_SETTINGS[0])
            self.filter_coefficient(BMP3XX_IIR_CONFIG_COEF_1)
            self.set_output_data_rates(BMP3XX_ODR_12P5_HZ)
        elif mode == ULTRA_PRECISION:
            self.set_power_mode(NORMAL_MODE)
            self.set_oversampling(BMP3XX_PRESS_OSR_SETTINGS[4], BMP3XX_TEMP_OSR_SETTINGS[1])
            self.filter_coefficient(BMP3XX_IIR_CONFIG_COEF_3)
            self.set_output_data_rates(BMP3XX_ODR_25_HZ)
        else:
            ret = False
        return ret

    '''
      @brief Enable interrupt of sensor data ready signal
             Note: As the interrupt pin is unique, the three interrupts are set to be used separately, please note the other two interrupt functions when using
    '''
    def enable_data_ready_int(self):
        self._write_reg(BMP3XX_INT_CTRL, 0x42)
        time.sleep(0.02)

    '''
      @brief Enable the interrupt of the sensor FIFO reaching the water level signal
             Note: As the interrupt pin is unique, the three interrupts are set to be used separately, please note the other two interrupt functions when using
      @param wtm_value: Set the water level value of FIFO (Range: 0-511)
    '''
    def enable_fifo_wtm_int(self, wtm_value):
        self._write_reg(BMP3XX_INT_CTRL, 0x0A)
        self._write_reg(BMP3XX_FIFO_WTM_0, wtm_value & 0xFF)
        self._write_reg(BMP3XX_FIFO_WTM_1, (wtm_value >> 8) & 0x01)
        time.sleep(0.02)

    '''
      @brief Enable the interrupt of the signal that the sensor FIFO is full
             Note: As the interrupt pin is unique, the three interrupts are set to be used separately, please note the other two interrupt functions when using
    '''
    def enable_fifo_full_int(self):
        self._write_reg(BMP3XX_INT_CTRL, 0x12)
        time.sleep(0.02)

    '''
      @brief Get the calibration data in the NVM register of the sensor
    '''
    def _get_coefficients(self):
        calib = self._read_reg(BMP3XX_CALIB_DATA, 21)
        self._data_calib = (
            ((calib[1] << 8) | calib[0]) / 2 ** -8.0,  # T1
            ((calib[3] << 8) | calib[2]) / 2 ** 30.0,  # T2
            self._uint8_to_int(calib[4]) / 2 ** 48.0,  # T3
            (self._uint16_to_int((calib[6] << 8) | calib[5]) - 2 ** 14.0) / 2 ** 20.0,  # P1
            (self._uint16_to_int((calib[8] << 8) | calib[7]) - 2 ** 14.0) / 2 ** 29.0,  # P2
            self._uint8_to_int(calib[9]) / 2 ** 32.0,  # P3
            self._uint8_to_int(calib[10]) / 2 ** 37.0,  # P4
            ((calib[12] << 8) | calib[11]) / 2 ** -3.0,  # P5
            ((calib[14] << 8) | calib[13]) / 2 ** 6.0,  # P6
            self._uint8_to_int(calib[15]) / 2 ** 8.0,  # P7
            self._uint8_to_int(calib[16]) / 2 ** 15.0,  # P8
            (self._uint16_to_int(calib[18] << 8) | calib[17]) / 2 ** 48.0,  # P9
            self._uint8_to_int(calib[19]) / 2 ** 48.0,  # P10
            self._uint8_to_int(calib[20]) / 2 ** 65.0,  # P11
        )

    '''
      @brief Use the obtained calibration data to calibrate and compensate the original value of the measured data
      @return Return the calibrated pressure data and the calibrated temperature data 
              Temperature unit: °C; Pressure unit: Pa
    '''
    def _compensate_data(self, adc_p, adc_t):
        # datasheet, p28，Trimming Coefficient listing in register map with size and sign attributes
        t1, t2, t3, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11 = self._data_calib
        #logger.info(self._data_calib)

        # Temperature compensation
        pd1 = adc_t - t1
        pd2 = pd1 * t2
        temperature = pd2 + (pd1 * pd1) * t3

        # Pressure compensation
        pd1 = p6 * temperature
        pd2 = p7 * temperature ** 2.0
        pd3 = p8 * temperature ** 3.0
        po1 = p5 + pd1 + pd2 + pd3
        pd1 = p2 * temperature
        pd2 = p3 * temperature ** 2.0
        pd3 = p4 * temperature ** 3.0
        po2 = adc_p * (p1 + pd1 + pd2 + pd3)
        pd1 = adc_p ** 2.0
        pd2 = p9 + p10 * temperature
        pd3 = pd1 * pd2
        pd4 = pd3 + p11 * adc_p ** 3.0
        pressure = po1 + po2 + pd4 - self.sea_level_pressure + STANDARD_SEA_LEVEL_PRESSURE_PA

        return round(pressure, 2), round(temperature, 2)

    '''
      @brief Obtain the raw measurement data of uncompensated and calibrated pressure and temperature from the register
      @return Return raw measurement data of pressure and temperature: adc_p, adc_t
    '''
    def _get_reg_temp_press_data(self):
        data = self._read_reg(BMP3XX_P_DATA_PA, 6)
        adc_p = data[2] << 16 | data[1] << 8 | data[0]
        adc_t = data[5] << 16 | data[4] << 8 | data[3]
        #logger.info(adc_p)
        #logger.info(adc_t)
        return adc_p, adc_t

    '''
      @brief Get the cached data in the FIFO
      @return Return the calibrated pressure data and the calibrated temperature data 
              Temperature unit: °C; Pressure unit: Pa
    '''
    def get_fifo_temp_press_data(self):
        data = self._read_reg(BMP3XX_FIFO_DATA, BMP3XX_FIFO_DATA_FRAME_LENGTH)
        adc_p, adc_t = 0, 0
        if data[0] == BMP3_FIFO_TEMP_PRESS_FRAME:
            adc_t = data[3] << 16 | data[2] << 8 | data[1]
            adc_p = data[6] << 16 | data[5] << 8 | data[4]
        elif data[0] == BMP3_FIFO_TEMP_FRAME:
            adc_t = data[3] << 16 | data[2] << 8 | data[1]
        elif data[0] == BMP3_FIFO_PRESS_FRAME:
            adc_p = data[3] << 16 | data[2] << 8 | data[1]
        elif data[0] == BMP3_FIFO_TIME_FRAME:
            logger.info(f"FIFO time:{data[3] << 16 | data[2] << 8 | data[1]}")
        elif data[0] == BMP3_FIFO_CONFIG_CHANGE:
            logger.warning("FIFO config change!!!")
        else:
            logger.warning("FIFO ERROR!!!")
        #logger.info(data[0])
        #logger.info(adc_p)
        #logger.info(adc_t)

        if (adc_p > 0):
            adc_p, adc_t = self._compensate_data(adc_p, adc_t)

        return adc_p, adc_t

    '''
      @brief Get FIFO cached data size
      @return The range of return value is: 0-511
    '''
    def get_fifo_length(self):
        len = self._read_reg(BMP3XX_FIFO_LENGTH, 2)
        return len[0] + (len[1] << 8)

    '''
      @brief Clear cached data in the FIFO without changing its settings  
    '''
    def empty_fifo(self):
        self._write_reg(BMP3XX_CMD, BMP3XX_CMD_FIFO_FLUSH)
        time.sleep(0.02)

    '''
      @brief Reset and restart the sensor, restoring the sensor configuration to the default configuration
    '''
    def reset(self):
        self._write_reg(BMP3XX_CMD, BMP3XX_CMD_RESET)

    '''
      @brief writes data to a register
      @param reg register address
             data written data
    '''
    def _write_reg(self, reg, data):
        '''Low level register writing, not implemented in base class'''
        raise NotImplementedError()

    '''
      @brief read the data from the register
      @param reg register address
             length read data length
    '''
    def _read_reg(self, reg, length):
        '''Low level register writing, not implemented in base class'''
        raise NotImplementedError()


'''@brief An example of an i2c interface module'''


class DFRobot_BMP3XX_I2C(DFRobot_BMP3XX):
    '''
      @brief Module I2C communication init
    '''
    def __init__(self, iic_addr=0x77, bus=1):
        self._addr = iic_addr
        self.i2c = smbus.SMBus(bus)
        super(DFRobot_BMP3XX_I2C, self).__init__()

    '''
      @brief writes data to a register
      @param reg register address
             data written data
    '''
    def _write_reg(self, reg, data):
        if isinstance(data, int):
            data = [data]
            #logger.info(data)
        self.i2c.write_i2c_block_data(self._addr, reg, data)

    '''
      @brief read the data from the register
      @param reg register address
             length read data length
    '''
    def _read_reg(self, reg, length):
        return self.i2c.read_i2c_block_data(self._addr, reg, length)


class DFRobot_BMP3XX_SPI(DFRobot_BMP3XX):
    '''
      @brief Module SPI communication init
    '''
    def __init__(self, cs=8, bus=0, dev=0, speed=8000000):
        self._cs = cs
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self._cs, GPIO.OUT, initial=1)
        GPIO.output(self._cs, GPIO.LOW)
        self.spi = spidev.SpiDev()
        self.spi.open(bus, dev)
        self.spi.no_cs = True
        self.spi.max_speed_hz = speed
        super(DFRobot_BMP3XX_SPI, self).__init__()

    '''
      @brief writes data to a register
      @param reg register address
             data written data
    '''
    def _write_reg(self, reg, data):
        if isinstance(data, int):
            data = [data]
            #logger.info(data)
        reg_addr = [reg & 0x7f]
        GPIO.output(self._cs, GPIO.LOW)
        self.spi.xfer(reg_addr)
        self.spi.xfer(data)
        GPIO.output(self._cs, GPIO.HIGH)

    '''
      @brief read the data from the register
      @param reg register address
             length read data length
    '''
    def _read_reg(self, reg, length):
        reg_addr = [reg | 0x80]
        GPIO.output(self._cs, GPIO.LOW)
        #logger.info(reg_addr)
        self.spi.xfer(reg_addr)
        time.sleep(0.01)
        self.spi.readbytes(1)
        rslt = self.spi.readbytes(length)
        GPIO.output(self._cs, GPIO.HIGH)
        return rslt
