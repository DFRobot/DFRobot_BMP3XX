/*!
 * @file  setOdrOsrIir.ino
 * @brief  Advanced data processing settings, configure more advanced data sampling and processing modes that meet your needs more.
 * @details  Configure measurement mode: sleep mode, enforcement mode, normal mode
 * @n  Configure pressure and temperature over-sampling mode (increase sampling times)
 * @n  Set the output data rate setting in subdivision/sub-sampling mode (set the data output rate, which must be less than the sampling frequency)
 * @n  IIR filter coefficient setting (filter noise)
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date  2021-04-30
 * @url  https://github.com/DFRobot/DFRobot_BMP3XX
 */
#include <DFRobot_BMP3XX.h>

/**
 * Select the chip version BMP388/BMP390L
 * Select communication interface I2C, please comment out SPI interface.
 * I2C communication address settings: eSDOGND: connect SDO pin to GND, I2C address is 0×76 now.
 *                   eSDOVDD: Connect SDO pin to VDDIO (3v3), I2C address is 0×77 now
 * Notice: If using Gravity products, I2C communication address is 0×77 by default
 */
//DFRobot_BMP388_I2C sensor(&Wire, sensor.eSDOVDD);
DFRobot_BMP390L_I2C sensor(&Wire, sensor.eSDOVDD);

/**
 * Select the chip version BMP388/BMP390L
 * Select communication interface SPI, please comment out I2C interface.
 * Set up digital pin according to the on-board pin connected with SPI chip-select pin.
 * Notice: csPin used here is D3 digital pin on ESP32, other non-conflicting pins can also be selected as external interrupt pins
 */
// uint8_t csPin = D3;
// DFRobot_BMP388_SPI sensor(&SPI, csPin);
// DFRobot_BMP390L_SPI sensor(&SPI, csPin);


/* If you do not need to eliminate the absolute difference of measurement, please comment the following line */
#define CALIBRATE_ABSOLUTE_DIFFERENCE

void setup(void)
{
  Serial.begin(115200);

  int rslt;
  while( ERR_OK != (rslt = sensor.begin()) ){
    if(ERR_DATA_BUS == rslt){
      Serial.println("Data bus error!!!");
    }else if(ERR_IC_VERSION == rslt){
      Serial.println("Chip versions do not match!!!");
    }
    delay(3000);
  }
  Serial.println("Begin ok!");

  /**
   * Configure measurement mode and power mode 
   * mode The measurement mode and power mode that need to set. The following modes add up to mode:
   *      ePressDIS: Disable pressure measurement, ePressEN: Enable pressure measurement
   *      eTempDIS: Disable temperature measurement, eTempEN: Enable temperature measurement
   *      eSleepMode, eForcedMode/, eNormalMode Three modes:
   *        Sleep mode: It will be in sleep mode by default after power-on reset. In this mode, no measurement is performed and power consumption is minimal. All registers 
   *                    are accessible for reading the chip ID and compensation coefficient.
   *        Enforcement mode: In enforcement mode, the sensor will take a single measurement according to the selected measurement and filtering options. After the measurement 
   *                     is completed, the sensor will return to sleep mode, and the measurement result can be obtained in the register.
   *        Normal mode: Continuously loop between the measurement period and the standby period. The measurement rate can be set in the odrSel register, and you can choose 
   *                     the prescaler with different sampling frequency Fsampling=200Hz.
   */
  sensor.setPWRMode(sensor.ePressEN | 
                    sensor.eTempEN | 
                    sensor.eNormalMode);

  /**
   *  Configure the oversampling when measuring pressure and temperature  
   *  mode Oversampling mode of pressure and temperature measurement need to set. The following modes add up to mode:
   *       6 pressure oversampling mode:
   *         ePressOSRMode1, Pressure sampling × 1, 16 bit / 2.64 Pa (Recommend temperature oversampling × 1)
   *         ePressOSRMode2, Pressure sampling × 2, 16 bit / 2.64 Pa (Recommend temperature oversampling × 1)
   *         ePressOSRMode4, Pressure sampling × 4, 18 bit / 0.66 Pa (Recommend temperature oversampling × 1)
   *         ePressOSRMode8, Pressure sampling × 8, 19 bit / 0.33 Pa (Recommend temperature oversampling × 2)
   *         ePressOSRMode16, Pressure sampling × 16, 20 bit / 0.17 Pa (Recommend temperature oversampling × 2)
   *         ePressOSRMode32, Pressure sampling × 32, 21 bit / 0.085 Pa (Recommend temperature oversampling × 2)
   *       6 temperature oversampling mode
   *         eTempOSRMode1, Temperature sampling × 1, 16 bit / 0.0050 °C
   *         eTempOSRMode2,  Temperature sampling × 2, 16 bit / 0.0025 °C
   *         eTempOSRMode4,  Temperature sampling × 4, 18 bit / 0.0012 °C
   *         eTempOSRMode8,  Temperature sampling × 8, 19 bit / 0.0006 °C
   *         eTempOSRMode16,  Temperature sampling × 16, 20 bit / 0.0003 °C
   *         eTempOSRMode32,  Temperature sampling × 32, 21 bit / 0.00015 °C
   */
  sensor.setOSRMode(sensor.ePressOSRMode4 | 
                    sensor.eTempOSRMode1);

  /**
   * Configure output data rate in subdivision/sub-sampling mode
   * mode The output data rate needs to set, configurable mode
   *      BMP3XX_ODR_200_HZ, BMP3XX_ODR_100_HZ, BMP3XX_ODR_50_HZ, BMP3XX_ODR_25_HZ, BMP3XX_ODR_12P5_HZ, 
   *      BMP3XX_ODR_6P25_HZ, BMP3XX_ODR_3P1_HZ, BMP3XX_ODR_1P5_HZ, BMP3XX_ODR_0P78_HZ, BMP3XX_ODR_0P39_HZ, 
   *      BMP3XX_ODR_0P2_HZ, BMP3XX_ODR_0P1_HZ, BMP3XX_ODR_0P05_HZ, BMP3XX_ODR_0P02_HZ, BMP3XX_ODR_0P01_HZ, 
   *      BMP3XX_ODR_0P006_HZ, BMP3XX_ODR_0P003_HZ, BMP3XX_ODR_0P0015_HZ
   */
  while( !sensor.setODRMode(BMP3XX_ODR_50_HZ) ){
    Serial.println("Set ODR mode fail! Please select lower frequency!");
    delay(3000);
  }

  /**
   * IIR filter coefficient configuration
   * mode Set IIR filter coefficient, configurable mode:
   *      BMP3XX_IIR_CONFIG_COEF_0, BMP3XX_IIR_CONFIG_COEF_1, BMP3XX_IIR_CONFIG_COEF_3, 
   *      BMP3XX_IIR_CONFIG_COEF_7, BMP3XX_IIR_CONFIG_COEF_15, BMP3XX_IIR_CONFIG_COEF_31, 
   *      BMP3XX_IIR_CONFIG_COEF_63, BMP3XX_IIR_CONFIG_COEF_127
   */
  sensor.setIIRMode(BMP3XX_IIR_CONFIG_COEF_3);

  delay(100);
  #ifdef CALIBRATE_ABSOLUTE_DIFFERENCE
  /**
   * Calibrate the sensor according to the current altitude
   * In this example, we use an altitude of 540 meters in Wenjiang District of Chengdu (China). Please change to the local altitude when using it.
   * If this interface is not called, the measurement data will not eliminate the absolute difference
   * Note: This interface is only valid for the first call
   */
  if( sensor.calibratedAbsoluteDifference(540.0) ){
    Serial.println("Absolute difference base value set successfully!");
  }
  #endif
}

void loop()
{
  /* Read currently measured temperature date directly, unit: °C */
  float temperature = sensor.readTempC();
  Serial.print("temperature : ");
  Serial.print(temperature);
  Serial.println(" C");

  /* Directly read the currently measured pressure data, unit: pa */
  float Pressure = sensor.readPressPa();
  Serial.print("Pressure : ");
  Serial.print(Pressure);
  Serial.println(" Pa");

  Serial.println();
  delay(1000);
}
