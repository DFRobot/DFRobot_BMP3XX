# -*- coding: utf-8 -*
'''!
  @file  interruptUsingFIFO.py
  @brief  Demonstrate FIFO water level interrupt or FIFO full interrupt:
  @n  Empty the FIFO first, and then start to obtain the cached measurement data in it
  @n  When receiving FIFO water level interrupt signal generated by interrupt pin, read out all the data in it and calculate the average value before printing it out
  @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license  The MIT License (MIT)
  @author  [qsjhyy](yihuan.huang@dfrobot.com)
  @version  V1.0
  @date  2021-05-06
  @url  https://github.com/DFRobot/DFRobot_BMP3XX
'''
from __future__ import print_function
import sys
sys.path.append('../')
from DFRobot_BMP3XX import *

'''
  # iic_addr = 0x76: pin SDO is low
  # iic_addr = 0x77: pin SDO is high
  # Note: Both the BMP390L and gravity version of the BMP388 sensor have pulled the SDO pin high by default, the address is 0x77; but the breakout version does not pull 
  #       the SDO high, and the IIC address is 0x76.
  # The following IIC and SPI communications support both BMP388 and BMP390L
'''
sensor = DFRobot_BMP3XX_I2C(iic_addr = 0x77,bus = 1)
# sensor = DFRobot_BMP3XX_SPI(cs=8, bus=0, dev=0, speed=8000000)

global flag
flag = 0

def int_callback(channel):
  global flag
  if flag == 0:
    flag = 1

GPIO.setwarnings(False)
# Use GPIO port to monitor sensor interrupt
gpio_int = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(gpio_int, GPIO.IN)
GPIO.add_event_detect(gpio_int, GPIO.RISING, callback=int_callback)

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
    #                             is completed, the sensor will return to sleep mode, and the measurement result can be obtained in the register.
    #   NORMAL_MODE(Normal mode): Continuously loop between the measurement period and the standby period. The output data rates are related to the ODR mode setting.
  '''
  sensor.set_power_mode(NORMAL_MODE)

  '''
    # Enable or disable FIFO
    # mode：
    #   True: Enable FIFO
    #   False:Disable FIFO
  '''
  sensor.enable_fifo(True)

  '''
    # Enable the interrupt of the sensor FIFO reaching the water level signal
    # Note: As the interrupt pin is unique, the three interrupts are set to be used separately, please note the other two interrupt functions when using
    # wtm_value:设定FIFO的水位值（范围：0-511）
  '''
  sensor.enable_fifo_wtm_int(500)

  '''
    # Enable the interrupt of the signal that the sensor FIFO is full
    # Note: As the interrupt pin is unique, the three interrupts are set to be used separately, please note the other two interrupt functions when using
  '''
#  sensor.enable_fifo_full_int()

  '''
    # Calibrate the sensor according to the current altitude
    # In this example, we use an altitude of 540 meters in Wenjiang District of Chengdu (China). Please change to the local altitude when using it.
    # If this interface is not called, the measurement data will not eliminate the absolute difference
    # Notice: This interface is only valid for the first call
    # If you do not need to eliminate the absolute difference of measurement, please comment the following two lines
  '''
  if( sensor.calibrated_absolute_difference(540.0) == True ):
    print("Absolute difference base value set successfully!")

  # Empty data in FIFO, and its settings remains unchanged.
  sensor.empty_fifo()


def loop():
  global flag
  if(flag == 1):
    # When the water level interrupt is triggered, read the altitude, unit: m
    print("Altitude : %.2f m" %(sensor.get_altitude))

    # Read all the measurement data stored in the FIFO and sum them all
    fifo_temperature_sum, fifo_pressure_sum, count = 0, 0, 0
    while(sensor.get_fifo_length()):
      fifo_pressure, fifo_temperature = sensor.get_fifo_temp_press_data()
      if fifo_pressure == 0:
        continue
      fifo_temperature_sum += fifo_temperature
      fifo_pressure_sum += fifo_pressure
      count += 1

    print("The number of data read this time is：%d" %count)
    print("Below is the average of the results：")
    # At the same time, read and count the average temperature obtained in FIFO, unit: °C
    print("temperature : %.2f C" %(fifo_temperature_sum / count))
    # At the same time, read and count the average pressure value obtained in FIFO, unit: pa
    print("Pressure : %.2f Pa" %(fifo_pressure_sum / count))
    count = 0
    flag = 0

  '''
    # Obtain the cached measurement data in it
    # Return the calibrated pressure data and the calibrated temperature data 
    # Temperature unit: °C; Pressure unit: Pa
  '''
  fifo_pressure, fifo_temperature = sensor.get_fifo_temp_press_data()

  # Temperature data, unit: °C (When there is no temperature data in FIFO, its value will be 0)
  print("temperature : %.2f C" %fifo_temperature)

  # Pressure data, unit: pa (When there is no pressure data in FIFO, its value will be 0.)
  print("Pressure : %.2f Pa" %fifo_pressure)

  # The number of bytes of data stored in the FIFO (0-511)
  print("The amount of data the FIFO has cached : %d" %(sensor.get_fifo_length()))

  print()
  time.sleep(0.5)


if __name__ == "__main__":
  setup()
  flag = 0
  while True:
    loop()
