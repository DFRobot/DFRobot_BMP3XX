/*!
 * @file DFRobot_BMP3XX.cpp
 * @brief Define the infrastructure DFRobot_BMP3XX class
 * @n This is a pressure and temperature sensor. I2C address cannot be changed. It can be controlled by I2C and SPI port.
 * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date 2021-04-1
 * @get from https://www.dfrobot.com
 * @url https://github.com/DFRobot/DFRobot_BMP3XX
 */
#include "DFRobot_BMP3XX.h"

DFRobot_BMP3XX::DFRobot_BMP3XX(uint8_t chipID)
{
  BMP3Info.chipID = chipID;
}

int DFRobot_BMP3XX::begin(void)
{
  uint8_t id;
  if(0 == readReg(BMP3XX_CHIP_ID, &id, sizeof(id)))   // Judge whether the data bus is successful
  {
    DBG("ERR_DATA_BUS");
    return ERR_DATA_BUS;
  }

  DBG("real sensor id=");DBG(id);
  if(BMP3Info.chipID != id)   // Judge whether the chip version matches
  {
    DBG("ERR_IC_VERSION");
    return ERR_IC_VERSION;
  }

  BMP3Info.seaLevelPressPa = STANDARD_SEA_LEVEL_PRESSURE_PA;
  setPWRMode(ePressEN | eTempEN | eNormalMode);   // Set normal aquisition mode, enable temperature and pressure aquisition.
  delay(50);
  getBMP3XXCalibData();
  delay(50);
  // cacheErrorStatus();
  // cacheSensorStatus();
  // cacheSensorEvent();
  // cacheINTStatus();
  DBG("begin ok!");
  return ERR_OK;
}

/*****************Integrated configuration of each register ******************************/

void DFRobot_BMP3XX::setFIFOMode1(uint8_t mode)
{
  writeReg(BMP3XX_FIFO_COFG_1, &mode, sizeof(mode));
}

void DFRobot_BMP3XX::setFIFOMode2(uint8_t mode)
{
  writeReg(BMP3XX_FIFO_COFG_2, &mode, sizeof(mode));
}

void DFRobot_BMP3XX::setINTMode(uint8_t mode)
{
  writeReg(BMP3XX_INT_CTRL, &mode, sizeof(mode));
}

void DFRobot_BMP3XX::setIFCONFMode(uint8_t mode)
{
  writeReg(BMP3XX_IF_CONF, &mode, sizeof(mode));
}

void DFRobot_BMP3XX::setPWRMode(uint8_t mode)
{
  readReg(BMP3XX_PWR_CTRL, &BMP3Info.PWRMode, sizeof(BMP3Info.PWRMode));
  if(*((uint8_t *)&BMP3Info.PWRMode) == mode){
    DBG("Same configuration as before！");
  }else{
    if(eSleepMode != BMP3Info.PWRMode.powerMode){   // You need to turn the device into sleep mode before changing its mode
      BMP3Info.PWRMode.powerMode = eSleepMode;
      writeReg(BMP3XX_PWR_CTRL, &BMP3Info.PWRMode, sizeof(mode));
      delay(20);   // Give it some time to enter sleep mode
    }
    memcpy(&BMP3Info.PWRMode, &mode, 1);
    writeReg(BMP3XX_PWR_CTRL, &BMP3Info.PWRMode, sizeof(mode));
    delay(20);   // Give it some time to switch mode
  }
}

void DFRobot_BMP3XX::setOSRMode(uint8_t mode)
{
  writeReg(BMP3XX_OSR, &mode, sizeof(mode));
}

bool DFRobot_BMP3XX::setODRMode(uint8_t mode)
{
  bool ret = true;

  writeReg(BMP3XX_ODR, &mode, sizeof(mode));
  uint32_t samplingPeriodus = getSamplingPeriodUS();   // Judge whether the ODR setting is resonable
  if(0 == samplingPeriodus){
    DBG("ODRSetting error!!!");
    ret = false;
  }

  return ret;
}

void DFRobot_BMP3XX::setIIRMode(uint8_t mode)
{
  writeReg(BMP3XX_IIR_CONFIG, &mode, sizeof(mode));
}

void DFRobot_BMP3XX::setCommand(uint8_t mode)
{
  writeReg(BMP3XX_CMD, &mode, sizeof(mode));
}

void DFRobot_BMP3XX::setFIFOWTM(uint16_t WTMSetting)
{
  uint8_t buf[2];
  buf[0] = WTMSetting;
  buf[1] = (WTMSetting >> 8) & 0x01;   // Range (0-511), only 9 bits are valid

  writeReg(BMP3XX_FIFO_WTM, &buf[0], 1);
  writeReg(BMP3XX_FIFO_WTM2, &buf[1], 1);
}

#define CASE_SAMPLING_MODE(MODE, PWR, OSR, ODR, IIR)   case MODE:\
                                                       setPWRMode(PWR);\
                                                       setOSRMode(OSR);\
                                                       setODRMode(ODR);\
                                                       setIIRMode(IIR);\
                                                       break;   // Simplify common mode configuration written macros
bool DFRobot_BMP3XX::setSamplingMode(ePrecisionMode_t mode)
{
  bool ret = true;
  switch (mode)
  {
    CASE_SAMPLING_MODE(eUltraLowPrecision, ePressEN | eTempEN | eForcedMode, ePressOSRMode1 | eTempOSRMode1, BMP3XX_ODR_0P01_HZ, BMP3XX_IIR_CONFIG_COEF_0)
    CASE_SAMPLING_MODE(eLowPrecision, ePressEN | eTempEN | eNormalMode, ePressOSRMode2 | eTempOSRMode1, BMP3XX_ODR_100_HZ, BMP3XX_IIR_CONFIG_COEF_0)
    CASE_SAMPLING_MODE(eNormalPrecision1, ePressEN | eTempEN | eNormalMode, ePressOSRMode4 | eTempOSRMode1, BMP3XX_ODR_50_HZ, BMP3XX_IIR_CONFIG_COEF_3)
    CASE_SAMPLING_MODE(eNormalPrecision2, ePressEN | eTempEN | eNormalMode, ePressOSRMode8 | eTempOSRMode1, BMP3XX_ODR_50_HZ, BMP3XX_IIR_CONFIG_COEF_1)
    CASE_SAMPLING_MODE(eHighPrecision, ePressEN | eTempEN | eNormalMode, ePressOSRMode8 | eTempOSRMode1, BMP3XX_ODR_12P5_HZ, BMP3XX_IIR_CONFIG_COEF_1)
    CASE_SAMPLING_MODE(eUltraPrecision, ePressEN | eTempEN | eNormalMode, ePressOSRMode16 | eTempOSRMode2, BMP3XX_ODR_25_HZ, BMP3XX_IIR_CONFIG_COEF_3)
    default:
      DBG("samping mode error!");
      ret = false;
  }
  delay(10);
  return ret;
}

/***************** Get and process data registers ******************************/

uint32_t DFRobot_BMP3XX::getSamplingPeriodUS(void)
{
  uint8_t ODRSetting;
  uint32_t samplingPeriodUS;

  readReg(BMP3XX_ODR, &ODRSetting, sizeof(ODRSetting));
  readReg(BMP3XX_PWR_CTRL, &BMP3Info.PWRMode, sizeof(BMP3Info.PWRMode));
  readReg(BMP3XX_OSR, &BMP3Info.overSamplingMode, sizeof(BMP3Info.overSamplingMode));

  samplingPeriodUS = 234 + BMP3Info.PWRMode.pressEN * (392 + pow(2, BMP3Info.overSamplingMode.OSRPress) * 2020) + 
                      BMP3Info.PWRMode.tempEN * (163 + pow(2, BMP3Info.overSamplingMode.OSRTemp) * 2020);

  return (pgm_read_dword(&correspondingSamplingPeriod[ODRSetting]) < samplingPeriodUS) ? 0:samplingPeriodUS;
}

float DFRobot_BMP3XX::readTempC(void)
{
  uint8_t buf[3] = {0};
  uint32_t uncompTemp;

  readReg(BMP3XX_T_DATA_C, &buf, sizeof(buf));
  uncompTemp = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16);

  return calibTemperatureC(uncompTemp);
}

float DFRobot_BMP3XX::readPressPa(void)
{
  uint8_t buf[6] = {0};
  uint32_t uncompPress, uncompTemp;

  readReg(BMP3XX_P_DATA_PA, &buf, sizeof(buf));
  uncompPress = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16);
  uncompTemp = (uint32_t)buf[3] | ((uint32_t)buf[4] << 8) | ((uint32_t)buf[5] << 16);

  /* Update the compensation temperature in the correction structure, which needs to be used to calculate the pressure */
  calibTemperatureC(uncompTemp);

  return calibPressurePa(uncompPress);
}

bool DFRobot_BMP3XX::calibratedAbsoluteDifference(float altitude)
{
  bool ret = false;
  if(STANDARD_SEA_LEVEL_PRESSURE_PA == BMP3Info.seaLevelPressPa){
    float pressure = readPressPa();
    BMP3Info.seaLevelPressPa = (pressure / pow(1.0 - (altitude / 44307.7), 5.255302));
    ret = true;
  }
  return ret;
}

float DFRobot_BMP3XX::readAltitudeM(void)
{
  float pressure = readPressPa();
  return (1.0 - pow(pressure / STANDARD_SEA_LEVEL_PRESSURE_PA, 0.190284)) * 44307.7;
}

void DFRobot_BMP3XX::getFIFOData(float &FIFOTemperatureC, float &FIFOPressurePa)
{
  uint16_t FIFODataLength = BMP3_FIFO_HEADER_LEN;   // Determine the length of a frame of FIFO data
  uint8_t FIFOData[10];

  readReg(BMP3XX_FIFO_COFG_1, &BMP3Info.FIFOMode1, sizeof(BMP3Info.FIFOMode1));
  if((eFIFOTempEN >> 4) == BMP3Info.FIFOMode1.FIFOTempEN){
    FIFODataLength += BMP3_FIFO_DATA_LEN;
  }
  if((eFIFOPressEN >> 3) == BMP3Info.FIFOMode1.FIFOPressEN){
    FIFODataLength += BMP3_FIFO_DATA_LEN;
  }
  readReg(BMP3XX_FIFO_DATA, &FIFOData, FIFODataLength);

  FIFOTemperatureC = 0;
  FIFOPressurePa = 0;
  switch (FIFOData[0])   // FIFO data frame header
  {
    case BMP3_FIFO_TEMP_PRESS_FRAME:   // Temperature and pressure are stored in FIFO
      FIFOTemperatureC = calibTemperatureC((uint32_t)FIFOData[1] | ((uint32_t)FIFOData[2] << 8) | ((uint32_t)FIFOData[3] << 16));
      FIFOPressurePa = calibPressurePa((uint32_t)FIFOData[4] | ((uint32_t)FIFOData[5] << 8) | ((uint32_t)FIFOData[6] << 16));
      break;
    case BMP3_FIFO_TEMP_FRAME:   // only temperature is stored in FIFO
      FIFOTemperatureC = calibTemperatureC((uint32_t)FIFOData[1] | ((uint32_t)FIFOData[2] << 8) | ((uint32_t)FIFOData[3] << 16));
      break;
    case BMP3_FIFO_PRESS_FRAME:   // only pressure is stored in FIFO
      FIFOPressurePa = calibPressurePa((uint32_t)FIFOData[1] | ((uint32_t)FIFOData[2] << 8) | ((uint32_t)FIFOData[3] << 16));
      break;
    case BMP3_FIFO_TIME_FRAME:   // FIFO time frame
      DBG("FIFO time:");
      DBG(((uint32_t)FIFOData[1] | ((uint32_t)FIFOData[2] << 8) | ((uint32_t)FIFOData[3] << 16))/25);
      DBG("ms");
      break;
    case BMP3_FIFO_CONFIG_CHANGE:
      DBG("FIFO config change!!!");
      break;
    case BMP3_FIFO_ERROR_FRAME:
      DBG("FIFO ERROR!!!");
      break;
    default:
      DBG("FIFO ERROR!!!");
      break;
  }

}


void DFRobot_BMP3XX::getBMP3XXCalibData(void)
{
  uint8_t regData[BMP3XX_CALIB_DATA_LEN] = {0};

  readReg(BMP3XX_CALIB_DATA, &regData, BMP3XX_CALIB_DATA_LEN);

  /* 1 / 2^8 */
  // 0.00390625f;
  BMP3Info.regCalibData.parT1 = BMP3XX_CONCAT_BYTES(regData[1], regData[0]);
  BMP3Info.quantizedCalibData.parT1 = ((float)BMP3Info.regCalibData.parT1 / pow(2, -8));
  // 1073741824.0f;
  BMP3Info.regCalibData.parT2 = BMP3XX_CONCAT_BYTES(regData[3], regData[2]);
  BMP3Info.quantizedCalibData.parT2 = ((float)BMP3Info.regCalibData.parT2 / pow(2, 30));
  // 281474976710656.0f;
  BMP3Info.regCalibData.parT3 = (int8_t)regData[4];
  BMP3Info.quantizedCalibData.parT3 = ((float)BMP3Info.regCalibData.parT3 / pow(2, 48));
  // 1048576.0f;
  BMP3Info.regCalibData.parP1 = (int16_t)BMP3XX_CONCAT_BYTES(regData[6], regData[5]);
  BMP3Info.quantizedCalibData.parP1 = ((float)(BMP3Info.regCalibData.parP1 - (16384)) / pow(2, 20));
  // 536870912.0f;
  BMP3Info.regCalibData.parP2 = (int16_t)BMP3XX_CONCAT_BYTES(regData[8], regData[7]);
  BMP3Info.quantizedCalibData.parP2 = ((float)(BMP3Info.regCalibData.parP2 - (16384)) / pow(2, 29));
  // 4294967296.0f;
  BMP3Info.regCalibData.parP3 = (int8_t)regData[9];
  BMP3Info.quantizedCalibData.parP3 = ((float)BMP3Info.regCalibData.parP3 / pow(2, 32));
  // 137438953472.0f;
  BMP3Info.regCalibData.parP4 = (int8_t)regData[10];
  BMP3Info.quantizedCalibData.parP4 = ((float)BMP3Info.regCalibData.parP4 / pow(2, 37));

  /* 1 / 2^3 */
  // 0.125f;
  BMP3Info.regCalibData.parP5 = BMP3XX_CONCAT_BYTES(regData[12], regData[11]);
  BMP3Info.quantizedCalibData.parP5 = ((float)BMP3Info.regCalibData.parP5 / pow(2, -3));
  // 64.0f;
  BMP3Info.regCalibData.parP6 = BMP3XX_CONCAT_BYTES(regData[14], regData[13]);
  BMP3Info.quantizedCalibData.parP6 = ((float)BMP3Info.regCalibData.parP6 / pow(2, 6));
  // 256.0f;
  BMP3Info.regCalibData.parP7 = (int8_t)regData[15];
  BMP3Info.quantizedCalibData.parP7 = ((float)BMP3Info.regCalibData.parP7 / pow(2, 8));
  // 32768.0f;
  BMP3Info.regCalibData.parP8 = (int8_t)regData[16];
  BMP3Info.quantizedCalibData.parP8 = ((float)BMP3Info.regCalibData.parP8 / pow(2, 15));
  // 281474976710656.0f;
  BMP3Info.regCalibData.parP9 = (int16_t)BMP3XX_CONCAT_BYTES(regData[18], regData[17]);
  BMP3Info.quantizedCalibData.parP9 = ((float)BMP3Info.regCalibData.parP9 / pow(2, 48));
  // 281474976710656.0f;
  BMP3Info.regCalibData.parP10 = (int8_t)regData[19];
  BMP3Info.quantizedCalibData.parP10 = ((float)BMP3Info.regCalibData.parP10 / pow(2, 48));
  // 36893488147419103232.0f;
  BMP3Info.regCalibData.parP11 = (int8_t)regData[20];
  BMP3Info.quantizedCalibData.parP11 = ((float)BMP3Info.regCalibData.parP11 / pow(2, 65));

}

float DFRobot_BMP3XX::calibTemperatureC(uint32_t uncompTemp)
{
  /* Temporary variable for compensation */
  float partialData1;
  float partialData2;

  partialData1 = (float)(uncompTemp - BMP3Info.quantizedCalibData.parT1);
  partialData2 = (float)(partialData1 * BMP3Info.quantizedCalibData.parT2);

  /* Update the compensation temperature in the correction structure, which needs to be used to calculate the pressure */
  BMP3Info.quantizedCalibData.tempLin = partialData2 + pow(partialData1, 2) * BMP3Info.quantizedCalibData.parT3;
  
  /* Return compensation temperature */
  return BMP3Info.quantizedCalibData.tempLin;
}

float DFRobot_BMP3XX::calibPressurePa(uint32_t uncompPress)
{
  /* Temporary variable for compensation */
  float partialData1;
  float partialData2;
  float partialData3;
  float partialData4;
  float partialOut1;
  float partialOut2;

  /* Variable to store compensation pressure */
  float compPress;

  /* calibration data */
  partialData1 = BMP3Info.quantizedCalibData.parP6 * BMP3Info.quantizedCalibData.tempLin;
  partialData2 = BMP3Info.quantizedCalibData.parP7 * pow(BMP3Info.quantizedCalibData.tempLin, 2);
  partialData3 = BMP3Info.quantizedCalibData.parP8 * pow(BMP3Info.quantizedCalibData.tempLin, 3);
  partialOut1 = BMP3Info.quantizedCalibData.parP5 + partialData1 + partialData2 + partialData3;

  partialData1 = BMP3Info.quantizedCalibData.parP2 * BMP3Info.quantizedCalibData.tempLin;
  partialData2 = BMP3Info.quantizedCalibData.parP3 * pow(BMP3Info.quantizedCalibData.tempLin, 2);
  partialData3 = BMP3Info.quantizedCalibData.parP4 * pow(BMP3Info.quantizedCalibData.tempLin, 3);
  partialOut2 = uncompPress * (BMP3Info.quantizedCalibData.parP1 + partialData1 + partialData2 + partialData3);

  partialData1 = (float)uncompPress * (float)uncompPress;
  partialData2 = BMP3Info.quantizedCalibData.parP9 + BMP3Info.quantizedCalibData.parP10 * BMP3Info.quantizedCalibData.tempLin;
  partialData3 = partialData1 * partialData2;
  partialData4 = partialData3 + pow((float)uncompPress, 3) * BMP3Info.quantizedCalibData.parP11;
  compPress = partialOut1 + partialOut2 + partialData4;

  return compPress - BMP3Info.seaLevelPressPa + STANDARD_SEA_LEVEL_PRESSURE_PA;
}

uint16_t DFRobot_BMP3XX::getFIFOLength(void)
{
  uint16_t FIFOLength;
  readReg(BMP3XX_FIFO_LENGTH, &FIFOLength, sizeof(FIFOLength));
  return FIFOLength;
}

uint16_t DFRobot_BMP3XX::getFIFOWTMValue(void)
{
  uint16_t FIFOWTMValue;
  readReg(BMP3XX_FIFO_WTM, &FIFOWTMValue, sizeof(FIFOWTMValue));
  return FIFOWTMValue;
}

/***************** Acquisition and processing of sensor status register ******************************/

void DFRobot_BMP3XX::cacheErrorStatus(void)
{
  readReg(BMP3XX_ERR_REG, &BMP3Info.errStatus, sizeof(BMP3Info.errStatus));

  if(BMP3Info.errStatus.fatalError){   // Fatal error, unrecoverable error
    DBG("Fatal error, unrecoverable error!");
  }
  if(BMP3Info.errStatus.CMDError){   // Command execution failed, clear after reading
    DBG("Command execution failed!");
  }
  if(BMP3Info.errStatus.configError){   // Sensor configuration error detected(work only under normal mode), clear after reading
    DBG("Sensor configuration error detected!");
  }
}

void DFRobot_BMP3XX::cacheSensorStatus(void)
{
  readReg(BMP3XX_ERR_REG, &BMP3Info.sensorStatus, sizeof(BMP3Info.sensorStatus));

  if(BMP3Info.sensorStatus.CMDReady){   // CMD decoder status
    DBG("The command decoder is ready to accept the new command!");
  }else{
    DBG("Executing command!");
  }
  if(BMP3Info.sensorStatus.pressDrdy){   // When a pressure data register is read out, it will be reset
    DBG("Pressure data ready!");
  }else{
    DBG("Pressure data is not ready!");
  }
  if(BMP3Info.sensorStatus.tempDrdy){   // When a temperature data register is read out, it will be reset
    DBG("Temperature data ready!");
  }else{
    DBG("Temperature data is not ready!");
  }
}

void DFRobot_BMP3XX::cacheSensorEvent(void)
{
  readReg(BMP3XX_EVENT, &BMP3Info.sensorEvent, sizeof(BMP3Info.sensorEvent));

  if(BMP3Info.sensorEvent.porDetected){   // “1”The device is powered on or soft reset, clear after reading
    DBG("The device is powered on or soft reset!");
  }
  if(BMP3Info.sensorEvent.itfActPt){   // “1”A serial interface transaction occurs during a pressure or temperature conversion, clear after reading
    DBG("A serial interface transaction occurs during a pressure or temperature conversion!");
  }
}

void DFRobot_BMP3XX::cacheINTStatus(void)
{
  readReg(BMP3XX_INT_STATUS, &BMP3Info.INTStatus, sizeof(BMP3Info.INTStatus));

  if(BMP3Info.INTStatus.fwtmINT){   // FIFO water level interrupt
    DBG("FIFO water level interrupt detected!");
  }
  if(BMP3Info.INTStatus.ffullINT){   // FIFO full interrupt
    DBG("FIFO full interrupt detected!");
  }
  if(BMP3Info.INTStatus.dataReady){   // Data ready interrupt
    DBG("Data detected ready for interrupt!");
  }
}

/***************** Initialization and reading and writing of I2C and SPI interfaces ******************************/

DFRobot_BMP3XX_I2C::DFRobot_BMP3XX_I2C(TwoWire *pWire, eSDOPinMode_t mode, uint8_t chipID)
  :DFRobot_BMP3XX(chipID)
{
  if(mode){
    _deviceAddr = DFROBOT_BMP3XX_I2C_ADDR_SDO_VDD;
  }else{
    _deviceAddr = DFROBOT_BMP3XX_I2C_ADDR_SDO_GND;
  }
  _pWire = pWire;
}

int DFRobot_BMP3XX_I2C::begin(void)
{
  _pWire->begin();   // Wire.h(I2C)library function initialize wire library
  return DFRobot_BMP3XX::begin();   // Use the initialization function of the parent class
}

void DFRobot_BMP3XX_I2C::writeReg(uint8_t reg, const void* pBuf, size_t size)
{
  if(pBuf == NULL){
    DBG("pBuf ERROR!! : null pointer");
  }
  uint8_t * _pBuf = (uint8_t *)pBuf;

  _pWire->beginTransmission(_deviceAddr);
  _pWire->write(reg);

  for(size_t i = 0; i < size; i++){
    _pWire->write(_pBuf[i]);
  }
  _pWire->endTransmission();
}

size_t DFRobot_BMP3XX_I2C::readReg(uint8_t reg, void* pBuf, size_t size)
{
  size_t count = 0;
  if(NULL == pBuf)
  {
    DBG("pBuf ERROR!! : null pointer");
  }
  uint8_t * _pBuf = (uint8_t*)pBuf;

  _pWire->beginTransmission(_deviceAddr);
  _pWire -> write(reg);
  if(0 != _pWire->endTransmission())   // Used Wire.endTransmission() to end a slave transmission started by beginTransmission() and arranged by write().
  {
    DBG("endTransmission ERROR!!");
  }else{
    _pWire->requestFrom(_deviceAddr, (uint8_t)size);   // Master device requests size bytes from slave device, which can be accepted by master device with read() or available()
    
    while (_pWire->available())
    {
      _pBuf[count++] = _pWire->read();   // Use read() to receive and put into buf
    }
    // _pWire->endTransmission();
  }
  return count;
}

DFRobot_BMP3XX_SPI::DFRobot_BMP3XX_SPI(SPIClass *pSpi, uint8_t csPin, uint8_t chipID)
  :DFRobot_BMP3XX(chipID)
{
  _pSpi = pSpi;
  _csPin = csPin;
}

int DFRobot_BMP3XX_SPI::begin(void)
{
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin,HIGH);
  _pSpi->begin();
  return DFRobot_BMP3XX::begin();
}

void DFRobot_BMP3XX_SPI::writeReg(uint8_t reg, const void* pBuf, size_t size)
{
  if(NULL == pBuf)
  {
    DBG("pBuf ERROR!! : null pointer");
  }
  uint8_t * _pBuf = (uint8_t *)pBuf;

  _pSpi->beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  _pSpi->transfer(reg & 0x7F);
  while(size--)
  {
    _pSpi->transfer(*_pBuf);
    _pBuf++;
  }
  digitalWrite(_csPin, HIGH);
  _pSpi->endTransaction();
}

size_t DFRobot_BMP3XX_SPI::readReg(uint8_t reg, void* pBuf, size_t size)
{
  if(NULL == pBuf)
  {
    DBG("pBuf ERROR!! : null pointer");
  }
  uint8_t * _pBuf = (uint8_t *)pBuf;

  size_t count = 0;
  _pSpi->beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  _pSpi->transfer(reg | 0x80);
  _pSpi->transfer(0x00);
  while(size--)
  {
    *_pBuf = _pSpi->transfer(0x00);
    _pBuf++;
    count++;
  }
  _pSpi->endTransaction();
  digitalWrite(_csPin, HIGH);
  return count;
}

/***************** BMP388 chip ******************************/
/***************** Initialization and reading and writing of I2C and SPI interfaces ******************************/

DFRobot_BMP388_I2C::DFRobot_BMP388_I2C(TwoWire *pWire, eSDOPinMode_t mode)
  :DFRobot_BMP3XX_I2C(pWire, mode, DFROBOT_BMP388_ID)
{

}

DFRobot_BMP388_SPI::DFRobot_BMP388_SPI(SPIClass *pSpi, uint8_t csPin)
  :DFRobot_BMP3XX_SPI(pSpi, csPin, DFROBOT_BMP388_ID)
{

}

/***************** BMP390L chip ******************************/
/***************** Initialization and reading and writing of I2C and SPI interfaces ******************************/

DFRobot_BMP390L_I2C::DFRobot_BMP390L_I2C(TwoWire *pWire, eSDOPinMode_t mode)
  :DFRobot_BMP3XX_I2C(pWire, mode, DFROBOT_BMP390L_ID)
{

}

DFRobot_BMP390L_SPI::DFRobot_BMP390L_SPI(SPIClass *pSpi, uint8_t csPin)
  :DFRobot_BMP3XX_SPI(pSpi, csPin, DFROBOT_BMP390L_ID)
{

}
