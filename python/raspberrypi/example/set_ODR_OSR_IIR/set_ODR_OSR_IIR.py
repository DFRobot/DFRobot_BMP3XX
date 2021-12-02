# -*- coding: utf-8 -*
'''!
  @file  set_ODR_OSR_IIR.py
  @brief  Advanced data processing settings, configure more advanced data sampling and processing modes that meet your needs more
  @n  Configure measurement mode: sleep mode, enforcement mode, normal mode
  @n  Configure pressure and temperature over-sampling mode (increase sampling times)
  @n  Set the output data rate setting in subdivision/sub-sampling mode (set the data output rate, which must be less than the sampling frequency)
  @n  IIR filter coefficient setting (filter noise)
  @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license  The MIT License (MIT)
  @author  [qsjhyy](yihuan.huang@dfrobot.com)
  @version  V1.0
  @date  2021-05-06
  @url  https://github.com/DFRobot/DFRobot_BMP3XX
'''
from __future__ import print_function
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))))

from DFRobot_BMP3XX import *

'''
  # i2c_addr = 0x76: pin SDO is low
  # i2c_addr = 0x77: pin SDO is high
  # Note: Both the BMP390L and gravity version of the BMP388 sensor have pulled the SDO pin high by default, the address is 0x77; but the breakout version does not pull 
  #       the SDO high, and the I2C address is 0x76.
  # The following I2C and SPI communications support both BMP388 and BMP390L
'''
sensor = DFRobot_BMP3XX_I2C(i2c_addr = 0x77,bus = 1)
# sensor = DFRobot_BMP3XX_SPI(cs=8, bus=0, dev=0, speed=8000000)

def setup():
  while (sensor.begin() == False):
    print ('Please check that the device is properly connected')
    time.sleep(3)
  print("sensor begin successfully!!!")

  '''
    # Configure measurement mode and power mode 
    # mode The measurement mode and power mode that need to set:
    #   SLEEP_MODE(Sleep mode): It will be in sleep mode by default after power-on reset. In this mode, no measurement is performed and power consumption is minimal. 
    #                           All registers are accessible for reading the chip ID and compensation coefficient.
    #   FORCED_MODE(Forced mode): In this mode, the sensor will take a single measurement according to the selected measurement and filtering options. After the measurement 
                                  is completed, the sensor will return to sleep mode, and the measurement result can be obtained in the register.
    #   NORMAL_MODE(Normal mode): Continuously loop between the measurement period and the standby period. The output data rates are related to the ODR mode setting.
  '''
  sensor.set_power_mode(NORMAL_MODE)

  '''
    # Configure the oversampling when measuring pressure and temperature. (OSR:over-sampling register)
    # Oversampling mode of pressure and temperature measurement need to be set
    # press_osr_set 6 pressure oversampling mode:
    #   BMP3XX_PRESS_OSR_SETTINGS[0], Pressure sampling×1, 16 bit / 2.64 Pa (Recommend temperature oversampling×1)
    #   BMP3XX_PRESS_OSR_SETTINGS[1], Pressure sampling×2, 16 bit / 2.64 Pa (Recommend temperature oversampling×1)
    #   BMP3XX_PRESS_OSR_SETTINGS[2], Pressure sampling×4, 18 bit / 0.66 Pa (Recommend temperature oversampling×1)
    #   BMP3XX_PRESS_OSR_SETTINGS[3], Pressure sampling×8, 19 bit / 0.33 Pa (Recommend temperature oversampling×2)
    #   BMP3XX_PRESS_OSR_SETTINGS[4], Pressure sampling×16, 20 bit / 0.17 Pa (Recommend temperature oversampling×2)
    #   BMP3XX_PRESS_OSR_SETTINGS[5], Pressure sampling×32, 21 bit / 0.085 Pa (Recommend temperature oversampling×2)
    # temp_osr_set 6 temperature oversampling mode:
    #   BMP3XX_TEMP_OSR_SETTINGS[0], Temperature sampling×1, 16 bit / 0.0050 °C
    #   BMP3XX_TEMP_OSR_SETTINGS[1], Temperature sampling×2, 16 bit / 0.0025 °C
    #   BMP3XX_TEMP_OSR_SETTINGS[2], Temperature sampling×4, 18 bit / 0.0012 °C
    #   BMP3XX_TEMP_OSR_SETTINGS[3], Temperature sampling×8, 19 bit / 0.0006 °C
    #   BMP3XX_TEMP_OSR_SETTINGS[4], Temperature sampling×16, 20 bit / 0.0003 °C
    #   BMP3XX_TEMP_OSR_SETTINGS[5], Temperature sampling×32, 21 bit / 0.00015 °C
  '''
  sensor.set_oversampling(BMP3XX_PRESS_OSR_SETTINGS[1], BMP3XX_TEMP_OSR_SETTINGS[0])

  '''
    # IIR filter coefficient configuration (IIR filtering)
    # iir_config_coef Set IIR filter coefficient, configurable mode:
    #   BMP3XX_IIR_CONFIG_COEF_0, BMP3XX_IIR_CONFIG_COEF_1, BMP3XX_IIR_CONFIG_COEF_3, 
    #   BMP3XX_IIR_CONFIG_COEF_7, BMP3XX_IIR_CONFIG_COEF_15, BMP3XX_IIR_CONFIG_COEF_31, 
    #   BMP3XX_IIR_CONFIG_COEF_63, BMP3XX_IIR_CONFIG_COEF_127
  '''
  sensor.filter_coefficient(BMP3XX_IIR_CONFIG_COEF_1)

  '''
    # Set the output data rate setting in subdivision/sub-sampling mode (ODR:output data rates)
    # odr_set The output data rate needs to set, configurable mode:
    #   BMP3XX_ODR_200_HZ, BMP3XX_ODR_100_HZ, BMP3XX_ODR_50_HZ, BMP3XX_ODR_25_HZ, BMP3XX_ODR_12P5_HZ, 
    #   BMP3XX_ODR_6P25_HZ, BMP3XX_ODR_3P1_HZ, BMP3XX_ODR_1P5_HZ, BMP3XX_ODR_0P78_HZ, BMP3XX_ODR_0P39_HZ, 
    #   BMP3XX_ODR_0P2_HZ, BMP3XX_ODR_0P1_HZ, BMP3XX_ODR_0P05_HZ, BMP3XX_ODR_0P02_HZ, BMP3XX_ODR_0P01_HZ, 
    #   BMP3XX_ODR_0P006_HZ, BMP3XX_ODR_0P003_HZ, BMP3XX_ODR_0P0015_HZ
    # Return True indicate configure succeed, return False indicate failed, remain its original state
  '''
  while sensor.set_output_data_rates(BMP3XX_ODR_25_HZ) == False:
    print ('Set ODR mode fail! Please select lower frequency!')
    time.sleep(3)

  '''
    # Calibrate the sensor according to the current altitude
    # In this example, we use an altitude of 540 meters in Wenjiang District of Chengdu (China). Please change to the local altitude when using it.
    # If this interface is not called, the measurement data will not eliminate the absolute difference
    # Notice: This interface is only valid for the first call
    # If you do not need to eliminate the absolute difference of measurement, please comment the following two lines
  '''
  if( sensor.calibrated_absolute_difference(540.0) == True ):
    print("Absolute difference base value set successfully!")

def loop():
  # Read currently measured temperature date directly, unit: °C
  print("temperature : %.2f C" %(sensor.get_temperature))

  # Directly read the currently measured pressure data, unit: pa
  print("Pressure : %.2f Pa" %(sensor.get_pressure))

  # Read altitude, unit: m
  print("Altitude : %.2f m" %(sensor.get_altitude))

  print()
  time.sleep(5)

if __name__ == "__main__":
  setup()
  while True:
    loop()
