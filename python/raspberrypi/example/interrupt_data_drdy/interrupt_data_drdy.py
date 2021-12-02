# -*- coding: utf-8 -*
'''!
  @file  interruptDataDrdy.py
  @brief  Demonstrate ready data (temperature/pressure) interrupt
  @n  When measured data, the sensor will generate a 2.5 ms pulse signal by INT in the non-interrupt register locked state.
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

global flag
flag = 0

def int_callback(channel):
  global flag
  if flag == 0:
    flag = 1

# Use GPIO port to monitor sensor interrupt
gpio_int = 27
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(gpio_int, GPIO.IN)
GPIO.add_event_detect(gpio_int, GPIO.FALLING, callback=int_callback)

def setup():
  while (sensor.begin() == False):
    print ('Please check that the device is properly connected')
    time.sleep(3)
  print("sensor begin successfully!!!")

  '''
    # Configure measurement mode and power mode 
    # mode The measurement mode and power mode that need to set.
    #   SLEEP_MODE(Sleep mode): It will be in sleep mode by default after power-on reset. In this mode, no measurement is performed and power consumption is minimal. 
    #                           All registers are accessible for reading the chip ID and compensation coefficient.
    #   FORCED_MODE(Forced mode): In this mode, the sensor will take a single measurement according to the selected measurement and filtering options. After the measurement
    #                             is completed, the sensor will return to sleep mode, and the measurement result can be obtained in the register.
    #   NORMAL_MODE(Normal mode): Continuously loop between the measurement period and the standby period. The output data rates are related to the ODR mode setting.
  '''
  sensor.set_power_mode(NORMAL_MODE)

  '''
    # Enable interrupt of sensor data ready signal
    # Note: As the interrupt pin is unique, the three interrupts are set to be used separately
  '''
  sensor.enable_data_ready_int()

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
  global flag
  if(flag == 1):
    flag = 0
    # When data is ready and the interrupt is triggered, read altitude, unit: m
    print("Altitude : %.2f m" %(sensor.get_altitude))
    print()
    time.sleep(0.5)


if __name__ == "__main__":
  setup()
  while True:
    loop()
