/*!
 * @file  DFRobot_BMP3XX.h
 * @brief  Define infrastructure of DFRobot_BMP3XX class
 * @details  This is a pressure and temperature sensor that can be controlled via IIC and SPI port.
 * @n  BMP(390L/388) has temperature compensation, data oversampling, IIR filter, binary sampling and other functions
 * @n  These functions improve the accuracy of data collected by the BMP (390L/388) sensor.
 * @n  BMP (390L/388) also has a 512-byte FIFO data buffer, which greatly improves its usability
 * @n  Similarly, BMP (390L/388) has an interrupt pin, which can be used in a power-saving way without using software algorithms.
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date  2021-04-01
 * @url  https://github.com/DFRobot/DFRobot_BMP3XX
 */
#ifndef __DFROBOT_BMP3XX_H__
#define __DFROBOT_BMP3XX_H__

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>


// #define ENABLE_DBG   //!< 打开这个宏, 可以看到程序的详细运行过程
#ifdef ENABLE_DBG
  #define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
  #define DBG(...)
#endif

#define STANDARD_SEA_LEVEL_PRESSURE_PA  101325   ///< Standard sea level pressure, unit: pa

#define DFROBOT_BMP3XX_IIC_ADDR_SDO_GND   uint8_t(0x76)   ///< IIC communication address when SDO is grounded
#define DFROBOT_BMP3XX_IIC_ADDR_SDO_VDD   uint8_t(0x77)   ///< IIC communication address when SDO is connected to power

#define DFROBOT_BMP388_ID 0x50             ///< BMP388 chip version
#define DFROBOT_BMP390L_ID 0x60            ///< BMP390L chip version

/* BMP3XX register address */
#define BMP3XX_CHIP_ID     uint8_t(0x00)   ///< The “CHIP_ID” register contains the chip identification code.
#define BMP3XX_REV_ID      uint8_t(0x01)   ///< The “Rev_ID” register contains the mask revision of the ASIC.
#define BMP3XX_ERR_REG     uint8_t(0x02)   ///< Sensor Error conditions are reported in the “ERR_REG” register.
#define BMP3XX_STATUS      uint8_t(0x03)   ///< The Sensor Status Flags are stored in the “STATUS” register.

#define BMP3XX_P_DATA_PA    uint8_t(0x04)   ///< The 24Bit pressure data is split and stored in three consecutive registers.
#define BMP3XX_T_DATA_C     uint8_t(0x07)   ///< The 24Bit temperature data is split and stored in three consecutive registersd.
#define BMP3XX_TIME        uint8_t(0x0C)    ///< The 24Bit sensor time is split and stored in three consecutive registers.

#define BMP3XX_EVENT       uint8_t(0x10)   ///< The “EVENT” register contains the sensor status flags.
#define BMP3XX_INT_STATUS  uint8_t(0x11)   ///< The “INT_STATUS” register shows interrupt status and is cleared after reading.
#define BMP3XX_FIFO_LENGTH uint8_t(0x12)   ///< The FIFO byte counter indicates the current fill level of the FIFO buffer.
#define BMP3XX_FIFO_DATA   uint8_t(0x14)   ///< The “FIFO_DATA” is the data output register.
#define BMP3XX_FIFO_WTM    uint8_t(0x15)   ///< The FIFO Watermark size is 9 Bit and therefore written to the FIFO_WTM_0 and FIFO_WTM_1 registers.
#define BMP3XX_FIFO_WTM2   uint8_t(0x16)   ///< The FIFO Watermark size is 9 Bit and therefore written to the FIFO_WTM_0 and FIFO_WTM_1 registers.
#define BMP3XX_FIFO_COFG_1 uint8_t(0x17)   ///< The “FIFO_CONFIG_1” register contains the FIFO frame content configuration.
#define BMP3XX_FIFO_COFG_2 uint8_t(0x18)   ///< The “FIFO_CONFIG_2” register extends the FIFO_CONFIG_1 register.

#define BMP3XX_INT_CTRL    uint8_t(0x19)   ///< Interrupt configuration can be set in the “INT_CTRL” register.
#define BMP3XX_IF_CONF     uint8_t(0x1A)   ///< The “IF_CONF” register controls the serial interface settings.
#define BMP3XX_PWR_CTRL    uint8_t(0x1B)   ///< The “PWR_CTRL” register enables or disables pressure and temperature measurement.
#define BMP3XX_OSR         uint8_t(0x1C)   ///< The “OSR” register controls the oversampling settings for pressure and temperature measurements.
#define BMP3XX_ODR         uint8_t(0x1D)   ///< The “ODR” register set the configuration of the output data rates by means of setting the subdivision/subsampling.
#define BMP3XX_IIR_CONFIG  uint8_t(0x1F)   ///< The “CONFIG” register controls the IIR filter coefficients
#define BMP3XX_CALIB_DATA  uint8_t(0x31)   ///< 0x31-0x45 is calibration data
#define BMP3XX_CMD         uint8_t(0x7E)   ///< Command register, can soft reset and clear all FIFO data

/* Set the constant of output data rate in subdivision/sub-sampling mode */
#define BMP3XX_ODR_200_HZ         uint8_t(0x00)   ///< Prescaler:1; ODR 200Hz; Sampling period:5 ms
#define BMP3XX_ODR_100_HZ         uint8_t(0x01)   ///< Prescaler:2; Sampling period:10 ms
#define BMP3XX_ODR_50_HZ          uint8_t(0x02)   ///< Prescaler:4; Sampling period:20 ms
#define BMP3XX_ODR_25_HZ          uint8_t(0x03)   ///< Prescaler:8; Sampling period:40 ms
#define BMP3XX_ODR_12P5_HZ        uint8_t(0x04)   ///< Prescaler:16; Sampling period:80 ms
#define BMP3XX_ODR_6P25_HZ        uint8_t(0x05)   ///< Prescaler:32; Sampling period:160 ms
#define BMP3XX_ODR_3P1_HZ         uint8_t(0x06)   ///< Prescaler:64; Sampling period:320 ms
#define BMP3XX_ODR_1P5_HZ         uint8_t(0x07)   ///< Prescaler:127; Sampling period:640 ms
#define BMP3XX_ODR_0P78_HZ        uint8_t(0x08)   ///< Prescaler:256; Sampling period:1.280 s
#define BMP3XX_ODR_0P39_HZ        uint8_t(0x09)   ///< Prescaler:512; Sampling period:2.560 s
#define BMP3XX_ODR_0P2_HZ         uint8_t(0x0A)   ///< Prescaler:1024 Sampling period:5.120 s
#define BMP3XX_ODR_0P1_HZ         uint8_t(0x0B)   ///< Prescaler:2048; Sampling period:10.24 s
#define BMP3XX_ODR_0P05_HZ        uint8_t(0x0C)   ///< Prescaler:4096; Sampling period:20.48 s
#define BMP3XX_ODR_0P02_HZ        uint8_t(0x0D)   ///< Prescaler:8192; Sampling period:40.96 s
#define BMP3XX_ODR_0P01_HZ        uint8_t(0x0E)   ///< Prescaler:16384; Sampling period:81.92 s
#define BMP3XX_ODR_0P006_HZ       uint8_t(0x0F)   ///< Prescaler:32768; Sampling period:163.84 s
#define BMP3XX_ODR_0P003_HZ       uint8_t(0x10)   ///< Prescaler:65536; Sampling period:327.68 s
#define BMP3XX_ODR_0P0015_HZ      uint8_t(0x11)   ///< Prescaler:131072; ODR 25/16384Hz; Sampling period:655.36 s

/* IIR filter coefficient setting constant */
#define BMP3XX_IIR_CONFIG_COEF_0           uint8_t(0x00)   ///< Filter coefficient is 0 -> bypass mode
#define BMP3XX_IIR_CONFIG_COEF_1           uint8_t(0x02)   ///< Filter coefficient is 1
#define BMP3XX_IIR_CONFIG_COEF_3           uint8_t(0x04)   ///< Filter coefficient is 3
#define BMP3XX_IIR_CONFIG_COEF_7           uint8_t(0x06)   ///< Filter coefficient is 7
#define BMP3XX_IIR_CONFIG_COEF_15          uint8_t(0x08)   ///< Filter coefficient is 15
#define BMP3XX_IIR_CONFIG_COEF_31          uint8_t(0x0A)   ///< Filter coefficient is 31
#define BMP3XX_IIR_CONFIG_COEF_63          uint8_t(0x0C)   ///< Filter coefficient is 63
#define BMP3XX_IIR_CONFIG_COEF_127         uint8_t(0x0E)   ///< Filter coefficient is 127

/* CMD(0x7E) register command */
#define BMP3XX_CMD_NOP         0x00    ///< reserved. No command.
#define BMP3XX_CMD_FIFO_FLUSH  0xB0    ///< Clears all data in the FIFO, does not change FIFO_CONFIG registers.
#define BMP3XX_CMD_SOFTRESET   0xB6    ///< Triggers a reset, all user configuration settings are overwritten with their default state.

/* FIFO Header */
///< FIFO temperature pressure header frame
#define BMP3_FIFO_TEMP_PRESS_FRAME              UINT8_C(0x94)
///< FIFO temperature header frame
#define BMP3_FIFO_TEMP_FRAME                    UINT8_C(0x90)
///< FIFO pressure header frame
#define BMP3_FIFO_PRESS_FRAME                   UINT8_C(0x84)
///< FIFO time header frame
#define BMP3_FIFO_TIME_FRAME                    UINT8_C(0xA0)
///< FIFO configuration change header frame
#define BMP3_FIFO_CONFIG_CHANGE                 UINT8_C(0x48)
///< FIFO error header frame
#define BMP3_FIFO_ERROR_FRAME                   UINT8_C(0x44)

#define BMP3_FIFO_HEADER_LEN                    UINT8_C(1)   ///< The byte length of the header in a frame of FIFO data is 1
#define BMP3_FIFO_DATA_LEN                      UINT8_C(3)   ///< The byte length of each data in a frame of FIFO data is 3


/* Convenience Macro */
#define BMP3XX_CALIB_DATA_LEN   (21)   ///< Number of calibration data bytes in the BMP3XX register
#define BMP3XX_CONCAT_BYTES(msb, lsb)   (((uint16_t)msb << 8) | (uint16_t)lsb)   ///< Macro combines two 8-bit data into one 16-bit data

/* Sampling period corresponding to ODR in microseconds */
static const uint32_t PROGMEM correspondingSamplingPeriod[] = {
  5000, 10000, 20000, 40000, 80000, 160000, 320000, 640000, 1280000, 2560000, 5120000, 10240000, 20480000,
  40960000, 81920000, 163840000, 327680000, 655360000
};

class DFRobot_BMP3XX
{
public:
#define ERR_OK             0   // No error
#define ERR_DATA_BUS      (-1)   // 数据总线错误
#define ERR_IC_VERSION    (-2)   // 芯片版本不匹配

/***************** 寄存器配置等结构体 ******************************/

  /**
   * @struct sFIFOMode1_t
   * @brief “FIFO_CONFIG_1”(0x17)寄存器包含FIFO帧内容配置。
   * @note 寄存器结构:
   * @n ----------------------------------------------------------------------------------------------------
   * @n |  b7  |  b6  |  b5  |      b4      |      b3       |      b2      |        b1         |    b0     |
   * @n ----------------------------------------------------------------------------------------------------
   * @n |      reserved      | fifo_temp_en | fifo_press_en | fifo_time_en | fifo_stop_on_full | fifo_mode |
   * @n ----------------------------------------------------------------------------------------------------
   */
  typedef struct
  {
    uint8_t   FIFOMode: 1; /**< 上电为0,  0: 禁用FIFO模式, 1: 启用FIFO模式 */
    uint8_t   FIFOStopOnFull: 1; /**< 上电为1,  0: 当FIFO已满时继续写入, 1: 当FIFO已满时停止写入 */
    uint8_t   FIFOTimeEN: 1; /**< 上电为0,  0: 禁用在最后一个有效数据帧之后返回传感器时间帧,  1: 启用返回传感器时间帧 */
    uint8_t   FIFOPressEN: 1; /**< 上电为0,  0: 禁用压力数据缓存, 1: 启用压力数据缓存 */
    uint8_t   FIFOTempEN: 1; /**< 上电为0,  0: 禁用温度数据缓存, 1: 启用温度数据缓存 */
    uint8_t   reserved: 3; /**< 保留位 */
  } __attribute__ ((packed)) sFIFOMode1_t; // 紧凑的结构体变量(知识点: __attribute__ ((packed))是避免字节对齐, 紧凑存放), 我们用来保存寄存器相关的内容。

  /**
   * @struct sFIFOMode2_t
   * @brief “FIFO_CONFIG_2”(0x18)寄存器扩展了FIFO_CONFIG_1寄存器。
   * @note 寄存器结构:
   * @n ------------------------------------------------------------------------------------------
   * @n |    b7    |   b6    |    b5    |    b4    |    b3    |    b2     |    b1     |    b0    |
   * @n ------------------------------------------------------------------------------------------
   * @n |           reserved            |     data_select     |         fifo_subsampling         |
   * @n ------------------------------------------------------------------------------------------
   */
  typedef struct
  {
    uint8_t   FIFOSubsampling: 3; /**< 压力和温度数据的FIFO下采样选择, 系数为2^FIFOSubsampling, 上电为2 */
    uint8_t   dataSelect: 2; /**< 对于压力和温度, 选择数据源, 上电为0,  0: 未过滤数据(补偿或未补偿),  1: 过滤数据(补偿或未补偿),  2or3: 保留, 与“unfilt”相同 */
    uint8_t   reserved: 3; /**< 保留位 */
  } __attribute__ ((packed)) sFIFOMode2_t;

  /**
   * @struct sIntMode_t
   * @brief 可以在“INT_CTRL”(0x19)寄存器中设置中断配置。
   * @details 它影响INT_STATUS寄存器和INT引脚。
   * @note 寄存器结构:
   * @n ------------------------------------------------------------------------------------------
   * @n |    b7    |   b6    |    b5    |    b4    |    b3    |    b2     |    b1     |    b0    |
   * @n ------------------------------------------------------------------------------------------
   * @n | reserved | drdy_en |  int_ds  | ffull_en | fwtm_en  | int_latch | int_level |  int_od  |
   * @n ------------------------------------------------------------------------------------------
   */
  typedef struct
  {
    uint8_t   INTOD: 1; /**< 引脚输出模式, 上电为0,  0: 推挽,  1: 开漏 */
    uint8_t   INTActiveLevel: 1; /**< 引脚电平, 上电为1,  0: 低电平有效,  1: 高电平有效 */
    uint8_t   INTLatch: 1; /**< 启用INT引脚和INT_STATUS寄存器锁定中断, 上电为0,  0: 禁用,  1: 启用 */
    uint8_t   INTFWTMEN: 1; /**< 启用INT引脚和INT_STATUS启用FIFO水位到达中断, 上电为0,  0: 禁用,  1: 启用 */
    uint8_t   INTFFullEN: 1; /**< 启用INT引脚和INT_STATUS启用FIFO全部中断, 上电为0,  0: 禁用,  1: 启用 */
    uint8_t   INTInitialLevel: 1; /**< 上电为0,  0: low,  1: high */
    uint8_t   INTDrdyEN: 1; /**< 启用INT引脚和INT_STATUS温度/压力数据准备中断, 上电为0,  0: 禁用,  1: 启用 */
    uint8_t   reserved: 1; /**< 保留位 */
  } __attribute__ ((packed)) sIntMode_t;

  /**
   * @struct sSerialMode_t
   * @brief “IF_CONF”(0x1A)寄存器控制串行接口设置。
   * @note 寄存器结构:
   * @n ------------------------------------------------------------------------------------------
   * @n |    b7   |    b6   |    b5   |    b4   |    b3   |      b2     |     b1      |    b0    |
   * @n ------------------------------------------------------------------------------------------
   * @n |                      reserved                   | i2c_wdt_sel | i2c_wdt_en  |   spi3   |
   * @n ------------------------------------------------------------------------------------------
   */
  typedef struct
  {
    uint8_t   SPI3: 1; /**< 上电为0,  0: SPI四线模式,  1: SPI三线模式 */
    uint8_t   I2CWDTEN: 1; /**< 上电为0,  0: 禁用I2C看门狗定时器,  1: 启用I2C看门狗定时器 */
    uint8_t   I2CWDTSel: 1; /**< 上电为0,  0: I2C看门狗定时器在1.25ms后超时,  1: I2C看门狗定时器在40ms后超时 */
    uint8_t   reserved: 5; /**< 保留位 */
  } __attribute__ ((packed)) sSerialMode_t;

  /**
   * @struct sPWRCTRL_t
   * @brief “PWR_CTRL”(0x1B)寄存器启用或禁用压力和温度测量。
   * @details 测量模式可以在这里设置: 
   * @n 睡眠模式: 上电复位后默认设置为睡眠模式。在睡眠模式下, 不执行任何测量, 并且功耗最少。所有寄存器均可访问；可以读取芯片ID和补偿系数。
   * @n 强制模式: 在强制模式下, 根据选择的测量和滤波选项进行单个测量。测量完成后, 传感器返回睡眠模式, 测量结果可从数据寄存器中获得。
   * @n 正常模式: 在测量周期和待机周期之间连续循环。测量速率在odr_sel寄存器中设置, 可以选择不同的采样频率Fsampling=200Hz的预分频器。
   * @note 寄存器结构:
   * @n ------------------------------------------------------------------------------------------
   * @n |    b7    |    b6    |    b5    |    b4    |    b3    |    b2    |    b1     |    b0    |
   * @n ------------------------------------------------------------------------------------------
   * @n |      reserved2      |     power_modes     |     reserved1       |  temp_en  | press_en |
   * @n ------------------------------------------------------------------------------------------
   */
  typedef struct
  {
    uint8_t   pressEN: 1; /**< 上电为0,  0: 禁用压力传感,  1: 启用压力传感 */
    uint8_t   tempEN: 1; /**< 上电为0,  0: 禁用温度传感,  1: 启用温度传感 */
    uint8_t   reserved1: 2; /**< 保留位 */
    uint8_t   powerMode: 2; /**< 上电为0,  0: 睡眠模式,  1or2: 强制模式,  3: 正常模式 */
    uint8_t   reserved2: 2; /**< 保留位 */
  } __attribute__ ((packed)) sPWRCTRL_t;

  /**
   * @struct sOverSamplingMode_t
   * @brief “OSR”(0x1C)寄存器控制压力和温度测量的过采样设置。
   * @details 温度压力过采样模式的6种配置:
   * @n ------------------------------------------------------------------------------------------
   * @n |   过采样设置   |   osr_p    |  压力过采样  |     典型压力解析度     |  推荐温度过采样  |
   * @n ------------------------------------------------------------------------------------------
   * @n |    超低功耗    |    000     |      ×1      |    16 bit / 2.64 Pa    |        ×1        |
   * @n ------------------------------------------------------------------------------------------
   * @n |     低功耗     |    001     |      ×2      |    16 bit / 2.64 Pa    |        ×1        |
   * @n ------------------------------------------------------------------------------------------
   * @n |   标准分辨率   |    010     |      ×4      |    18 bit / 0.66 Pa    |        ×1        |
   * @n ------------------------------------------------------------------------------------------
   * @n |    高分辨率    |    011     |      ×8      |    19 bit / 0.33 Pa    |        ×1        |
   * @n ------------------------------------------------------------------------------------------
   * @n |   超高分辨率   |    100     |      ×16     |    20 bit / 0.17 Pa    |        ×2        |
   * @n ------------------------------------------------------------------------------------------
   * @n |   最高分辨率   |    101     |      ×32     |    21 bit / 0.085 Pa   |        ×2        |
   * @n ------------------------------------------------------------------------------------------
   * @note 寄存器结构:
   * @n ------------------------------------------------------------------------------------------
   * @n |    b7    |    b6    |    b5    |    b4    |    b3    |    b2    |    b1     |    b0    |
   * @n ------------------------------------------------------------------------------------------
   * @n |      reserved       |             osr_t              |              osr_p              |
   * @n ------------------------------------------------------------------------------------------
   */
  typedef struct
  {
    uint8_t   OSRPress: 3; /**< 上电为0,  可设置6种压力过采样模式 */
    uint8_t   OSRTemp: 3; /**< 上电为0,  温度也有六种, 设置和压力类似, 但是建议使用表中推荐的温度过采样模式 */
    uint8_t   reserved: 2; /**< 保留位 */
  } __attribute__ ((packed)) sOverSamplingMode_t;

/***************** 校准补偿数据结构体 ******************************/

  /**
   * @struct sCalibData_t
   * @brief 缓存寄存器里面校准补偿数据的结构体
   */
  typedef struct
  {
    uint16_t parT1;
    uint16_t parT2;
    int8_t parT3;
    int16_t parP1;
    int16_t parP2;
    int8_t parP3;
    int8_t parP4;
    uint16_t parP5;
    uint16_t parP6;
    int8_t parP7;
    int8_t parP8;
    int16_t parP9;
    int8_t parP10;
    int8_t parP11;
    int64_t tempLin;
  } sCalibData_t;

  /**
   * @struct sQuantizedCalibData_t
   * @brief 量化调整后的补偿数据
   */
  typedef struct
  {
    float parT1;
    float parT2;
    float parT3;
    float parP1;
    float parP2;
    float parP3;
    float parP4;
    float parP5;
    float parP6;
    float parP7;
    float parP8;
    float parP9;
    float parP10;
    float parP11;
    float tempLin;
  } sQuantizedCalibData_t;

/***************** 设备状态信息结构体 ******************************/

  /**
   * @struct sSensorErrorStatus_t
   * @brief 传感器错误情况在“ERR_REG”寄存器中报告
   * @note 寄存器结构:
   * @n ------------------------------------------------------------------------------------------
   * @n |    b7   |    b6   |    b5   |    b4   |    b3   |      b2    |     b1    |     b0      |
   * @n ------------------------------------------------------------------------------------------
   * @n |                      reserved                   |  conf_err  |  cmd_err  |  fatal_err  |
   * @n ------------------------------------------------------------------------------------------
   */
  typedef struct
  {
    uint8_t   fatalError: 1; /**< 致命错误, 不可恢复的错误 */
    uint8_t   CMDError: 1; /**< 命令执行失败, 在阅读后清零 */
    uint8_t   configError: 1; /**< 检测到传感器配置错误(仅在正常模式下工作), 在阅读后清零 */
    uint8_t   reserved: 5; /**< 保留位 */
  } __attribute__ ((packed)) sSensorErrorStatus_t;

  /**
   * @struct sSensorStatus_t
   * @brief 传感器状态标志缓存在“STATUS”寄存器中
   * @note 寄存器结构:
   * @n ------------------------------------------------------------------------------------------
   * @n |      b7     |     b6    |     b5     |    b4   |    b3   |    b2   |    b1   |    b0   |
   * @n ------------------------------------------------------------------------------------------
   * @n |  reserved2  | drdy_temp | drdy_press | cmd_rdy |                reserved1              |
   * @n ------------------------------------------------------------------------------------------
   */
  typedef struct
  {
    uint8_t   reserved1: 4; /**< 保留位 */
    uint8_t   CMDReady: 1; /**< CMD解码器状态 */
    uint8_t   pressDrdy: 1; /**< 压力数据准备好 */
    uint8_t   tempDrdy: 1; /**< 温度数据准备好 */
    uint8_t   reserved2: 1; /**< 保留位 */
  } __attribute__ ((packed)) sSensorStatus_t;

  /**
   * @struct sSensorEvent_t
   * @brief “EVENT”寄存器包含传感器状态标志
   * @note 寄存器结构:
   * @n ------------------------------------------------------------------------------------------
   * @n |    b7   |    b6   |   b5   |   b4   |   b3   |   b2   |      b1      |       b0       |
   * @n ------------------------------------------------------------------------------------------
   * @n |                      reserved                         |  itf_act_pt  |  por_detected  |
   * @n ------------------------------------------------------------------------------------------
   */
  typedef struct
  {
    uint8_t   porDetected: 1; /**< 设备上电或软复位后被置“1”, 在阅读后清零 */
    uint8_t   itfActPt: 1; /**< 当在压力或温度转换期间发生串行接口事务时被置“1”, 在阅读后清零 */
    uint8_t   reserved: 6; /**< 保留位 */
  } __attribute__ ((packed)) sSensorEvent_t;

  /**
   * @struct sSensorINTStatus_t
   * @brief “INT_STATUS”寄存器显示中断状态, 并在读取后被清除。
   * @note 寄存器结构:
   * @n ------------------------------------------------------------------------------------------
   * @n |   b7  |   b6   |    b5   |    b4   |    b3   |      b2    |      b1     |      b0      |
   * @n ------------------------------------------------------------------------------------------
   * @n |             reserved2              |   drdy  | reserved1  |  ffull_int  |   fwtm_int   |
   * @n ------------------------------------------------------------------------------------------
   */
  typedef struct
  {
    uint8_t   fwtmINT: 1; /**< FIFO水位中断 */
    uint8_t   ffullINT: 1; /**< FIFO存满中断 */
    uint8_t   reserved1: 1; /**< 保留位 */
    uint8_t   dataReady: 1; /**< 数据准备好中断 */
    uint8_t   reserved2: 4; /**< 保留位 */
  } __attribute__ ((packed)) sSensorINTStatus_t;

  /**
   * @struct sBMP3XXDeviceInfo_t
   * @brief BMP3XX设备信息结构体
   */
  typedef struct
  {
    /* 芯片ID */
    uint8_t chipID;

    /* 测量数据的校准补偿系数 */
    sCalibData_t regCalibData;   /**< 储存寄存器读到的校准数据 */
    sQuantizedCalibData_t quantizedCalibData;   /**< 缓存量化后的校准数据 */
    float seaLevelPressPa;   /**< 计算海拔高度时使用的海平面大气压值 */

    /* 传感器配置数据 */
    sFIFOMode1_t FIFOMode1;   /**< 寄存器配置的相关结构体 */
    sFIFOMode2_t FIFOMode2;
    sIntMode_t intMode;
    sSerialMode_t serialMode;
    sPWRCTRL_t PWRMode;
    sOverSamplingMode_t overSamplingMode;

    /* 传感器状态数据 */
    sSensorErrorStatus_t errStatus;
    sSensorStatus_t sensorStatus;
    sSensorEvent_t sensorEvent;
    sSensorINTStatus_t INTStatus;

  } sBMP3XXDeviceInfo_t;

/***************** 寄存器每一位详细配置枚举数据类型 ******************************/

  /**
   * @enum  eFIFOMode_t
   * @brief  是否启用FIFO缓存
   */
  typedef enum
  {
    eFIFODIS = 0,   /**< 禁用FIFO */
    eFIFOEN,   /**< 启用FIFO */
  }eFIFOMode_t;

  /**
   * @enum  eFIFOStopOnFull_t
   * @brief  当FIFO已满时是否继续写入
   */
  typedef enum
  {
    eFIFOStopOnFullDIS = 0<<1,   /**< 禁用: 写满时继续写入 */
    eFIFOStopOnFullEN = 1<<1,   /**< 启用: 写满时停止写入 */
  }eFIFOStopOnFull_t;

  /**
   * @enum  eFIFOTime_t
   * @brief  是否在最后一个有效数据帧之后返回传感器时间帧
   */
  typedef enum
  {
    eFIFOTimeDIS = 0<<2,   /**< 禁用 */
    eFIFOTimeEN = 1<<2,   /**< 启用 */
  }eFIFOTime_t;

  /**
   * @enum  eFIFOPress_t
   * @brief  是否启用压力数据缓存
   */
  typedef enum
  {
    eFIFOPressDIS = 0<<3,   /**< 禁用压力数据缓存 */
    eFIFOPressEN = 1<<3,   /**< 启用压力数据缓存 */
  }eFIFOPress_t;

  /**
   * @enum  eFIFOTemp_t
   * @brief  是否启用温度数据缓存
   */
  typedef enum
  {
    eFIFOTempDIS = 0<<4,   /**< 禁用温度数据缓存 */
    eFIFOTempEN = 1<<4,   /**< 启用温度数据缓存 */
  }eFIFOTemp_t;

  /**
   * @enum  eFIFOSubsampling_t
   * @brief  压力和温度数据的FIFO下采样选择, 系数为2^FIFOSubsampling, 上电为2
   */
  typedef enum
  {
    eFIFOSubsampling0 = 0,
    eFIFOSubsampling1,
    eFIFOSubsampling2,
    eFIFOSubsampling3,
    eFIFOSubsampling4,
    eFIFOSubsampling5,
    eFIFOSubsampling6,
    eFIFOSubsampling7,
  }eFIFOSubsampling_t;

  /**
   * @enum  eFIFODataSelect_t
   * @brief  对于压力和温度, 选择数据源, 上电为0
   */
  typedef enum
  {
    eFIFODataSelectDIS = 0<<3,   /**< 未过滤数据(补偿或未补偿) */
    eFIFODataSelectEN = 1<<3,   /**< 过滤数据(补偿或未补偿) */
    eFIFODataSelectNO2 = 2<<3,    /**< 保留, 与“unfilt”相同 */
    eFIFODataSelectNO3 = 3<<3,    /**< 保留, 与“unfilt”相同 */
  }eFIFODataSelect_t;

  /**
   * @enum  eINTPinMode_t
   * @brief  中断引脚输出模式
   */
  typedef enum
  {
    eINTPinPP = 0,   /**< 推挽 */
    eINTPinOD,   /**< 开漏 */
  }eINTPinMode_t;

  /**
   * @enum  eINTPinActiveLevel_t
   * @brief  中断引脚信号电平
   */
  typedef enum
  {
    eINTPinActiveLevelLow = 0<<1,   /**< 低电平有效 */
    eINTPinActiveLevelHigh = 1<<1,   /**< 高电平有效 */
  }eINTPinActiveLevel_t;

  /**
   * @enum  eINTLatch_t
   * @brief  是否启用INT引脚和INT_STATUS寄存器锁定中断
   */
  typedef enum
  {
    eINTLatchDIS = 0<<2,   /**< 禁用 */
    eINTLatchEN = 1<<2,   /**< 启用 */
  }eINTLatch_t;

  /**
   * @enum  eINTFWTM_t
   * @brief  是否启用INT引脚和INT_STATUS启用FIFO水位到达中断
   */
  typedef enum
  {
    eINTFWTMDIS = 0<<3,   /**< 禁用 */
    eINTFWTMEN = 1<<3,   /**< 启用 */
  }eINTFWTM_t;

  /**
   * @enum  eINTFFull_t
   * @brief  是否启用INT引脚和INT_STATUS启用FIFO全部中断
   */
  typedef enum
  {
    eINTFFullDIS = 0<<4,   /**< 禁用 */
    eINTFFullEN = 1<<4,   /**< 启用 */
  }eINTFFull_t;

  /**
   * @enum  eINTInitialLevel_t
   * @brief  中断数据引脚电平
   */
  typedef enum
  {
    eINTInitialLevelLOW = 0<<5,   /**< 低电平 */
    eINTInitialLevelHIGH = 1<<5,   /**< 高电平 */
  }eINTInitialLevel_t;

  /**
   * @enum  eINTDataDrdy_t
   * @brief  是否启用INT引脚和INT_STATUS温度/压力数据准备中断
   */
  typedef enum
  {
    eINTDataDrdyDIS = 0<<6,   /**< 禁用 */
    eINTDataDrdyEN = 1<<6,   /**< 启用 */
  }eINTDataDrdy_t;

  /**
   * @enum  eSPISerialMode_t
   * @brief  SPI通信模式选择
   */
  typedef enum
  {
    eSerialModeSPI4 = 0,   /**< SPI四线模式 */
    eSerialModeSPI3,   /**< SPI三线模式 */
  }eSPISerialMode_t;

  /**
   * @enum  eI2CWDT_t
   * @brief  是否启用I2C看门狗定时器
   */
  typedef enum
  {
    eI2CWDTDIS = 0<<1,   /**< 禁用 */
    eI2CWDTEN = 1<<1,   /**< 启用 */
  }eI2CWDT_t;

  /**
   * @enum  eI2CWDTSel_t
   * @brief  配置I2C看门狗定时器
   */
  typedef enum
  {
    eI2CWDTSel1p25 = 0<<2,   /**< I2C看门狗定时器在1.25ms后超时 */
    eI2CWDTSel40 = 1<<2,   /**< I2C看门狗定时器在40ms后超时 */
  }eI2CWDTSel_t;

  /**
   * @enum  ePressMeasure_t
   * @brief  是否启用压力传感
   */
  typedef enum
  {
    ePressDIS = 0, /**< 禁用压力传感 */
    ePressEN,   /**< 启用压力传感 */
  }ePressMeasure_t;

  /**
   * @enum  eTempMeasure_t
   * @brief  是否启用温度传感
   */
  typedef enum
  {
    eTempDIS = 0<<1, /**< 禁用温度传感 */
    eTempEN = 1<<1,   /**< 启用温度传感 */
  }eTempMeasure_t;

  /**
   * @enum  ePowerMode_t
   * @brief  测量(电源)模式设置
   */
  typedef enum
  {
    eSleepMode = 0<<4,   /**< 睡眠模式: 上电复位后默认设置为睡眠模式。在睡眠模式下, 不执行任何测量, 并且功耗最少。所有寄存器均可访问；可以读取芯片ID和补偿系数。 */
    eForcedMode = 1<<4, /**< 强制模式: 在强制模式下, 根据选择的测量和滤波选项进行单个测量。测量完成后, 传感器返回睡眠模式, 测量结果可从数据寄存器中获得。 */
    eForcedMode2 = 2<<4, /**< 强制模式: 在强制模式下, 根据选择的测量和滤波选项进行单个测量。测量完成后, 传感器返回睡眠模式, 测量结果可从数据寄存器中获得。 */
    eNormalMode = 3<<4,  /**< 正常模式: 在测量周期和待机周期之间连续循环。输出数据率(output data rates)与ODR模式设置有关。 */
  }ePowerMode_t;

  /**
   * @enum  ePressOSRMode_t
   * @brief  6种压力过采样模式
   */
  typedef enum
  {
    ePressOSRMode1 = 0,   /**< 采样×1, 16 bit / 2.64 Pa(推荐温度过采样×1) */
    ePressOSRMode2,   /**< 采样×2, 16 bit / 2.64 Pa(推荐温度过采样×1) */
    ePressOSRMode4,   /**< 采样×4, 18 bit / 0.66 Pa(推荐温度过采样×1) */
    ePressOSRMode8,   /**< 采样×8, 19 bit / 0.33 Pa(推荐温度过采样×2) */
    ePressOSRMode16,   /**< 采样×16, 20 bit / 0.17 Pa(推荐温度过采样×2) */
    ePressOSRMode32,   /**< 采样×32, 21 bit / 0.085 Pa(推荐温度过采样×2) */
  }ePressOSRMode_t;

  /**
   * @enum  eTempOSRMode_t
   * @brief  6种温度过采样模式
   */
  typedef enum
  {
    eTempOSRMode1 = 0<<3,   /**< 采样×1, 16 bit / 0.0050 °C */
    eTempOSRMode2 = 1<<3,   /**< 采样×2, 16 bit / 0.0025 °C */
    eTempOSRMode4 = 2<<3,   /**< 采样×4, 18 bit / 0.0012 °C */
    eTempOSRMode8 = 3<<3,   /**< 采样×8, 19 bit / 0.0006 °C */
    eTempOSRMode16 = 4<<3,   /**< 采样×16, 20 bit / 0.0003 °C */
    eTempOSRMode32 = 5<<3,   /**< 采样×32, 21 bit / 0.00015 °C */
  }eTempOSRMode_t;

/***************** 方便用户选择的枚举数据类型 ******************************/

  /**
   * @enum  eSDOPinMode_t
   * @brief  SDO接线状态
   */
  typedef enum
  {
    eSDOGND = 0,   /**< SDO接GND */
    eSDOVDD,   /**< SDO接VDD */
  }eSDOPinMode_t;

  /**
   * @enum  ePrecisionMode_t
   * @brief  为了选择最佳设置, 建议使用的模式
   */
  typedef enum
  {
    eUltraLowPrecision = 0, /**< 超低精度, 适合天气监控(最低功耗), 电源模式为强制模式, IDD[µA]=4, RMSNoise[cm]=55 */
    eLowPrecision, /**< 低精度, 适合随意的检测, 电源模式为正常模式, IDD[µA]=358, RMSNoise[cm]=36 */
    eNormalPrecision1, /**< 标准精度1, 适合在手持式设备上动态检测(例如在手机上), 电源模式为正常模式, IDD[µA]=310, RMSNoise[cm]=10 */
    eNormalPrecision2, /**< 标准精度2, 适合无人机, 电源模式为正常模式, IDD[µA]=570, RMSNoise[cm]=11 */
    eHighPrecision, /**< 高精度, 适合在低功耗手持式设备上(例如在手机上), 电源模式为正常模式, IDD[µA]=145, RMSNoise[cm]=11 */
    eUltraPrecision, /**< 超高精度, 适合室内的导航, 电源模式为正常模式, IDD[µA]=560, RMSNoise[cm]=5 */
  }ePrecisionMode_t;

public:
  /**
   * @fn DFRobot_BMP3XX
   * @brief 构造函数
   * @param chipID 芯片ID
   * @return None
   */
  DFRobot_BMP3XX(uint8_t chipID);

  /**
   * @fn begin
   * @brief 初始化函数
   * @return int类型, 表示返回初始化的状态
   * @retval 0 NO_ERROR
   * @retval -1 ERR_DATA_BUS
   * @retval -2 ERR_IC_VERSION
   */
  virtual int begin(void);

/***************** 每个寄存器的整合配置 ******************************/

  /**
   * @fn setFIFOMode1
   * @brief FIFO配置一(FIFO1)
   * @param mode 需要设置的FIFO模式,下列模式相加为mode:
   * @n       eFIFODIS: 禁用FIFO, eFIFOEN: 启用FIFO
   * @n       eFIFOStopOnFullDIS: 写满时继续写入, eFIFOStopOnFullEN: 写满时停止写入
   * @n       eFIFOTimeDIS: 禁用, eFIFOTimeEN: 启用在最后一个有效数据帧之后返回传感器时间帧
   * @n       eFIFOPressDIS: 禁用压力数据缓存, eFIFOPressEN: 启用压力数据缓存
   * @n       eFIFOTempDIS: 禁用温度数据缓存, eFIFOTempEN: 启用温度数据缓存
   * @return None
   */
  void setFIFOMode1(uint8_t mode);

  /**
   * @fn setFIFOMode2
   * @brief FIFO配置二(FIFO2)
   * @param mode 需要设置的FIFO模式,下列模式相加为mode:
   * @n       8种压力和温度数据的FIFO下采样选择(1-128), 系数为2^fifo_subsampling(0-7): 
   * @n         eFIFOSubsampling0, eFIFOSubsampling1, eFIFOSubsampling2, eFIFOSubsampling3,
   * @n         eFIFOSubsampling4, eFIFOSubsampling5, eFIFOSubsampling6, eFIFOSubsampling7,
   * @n       eFIFODataSelectDIS:  未过滤数据(补偿或未补偿) , eFIFODataSelectEN:  过滤数据(补偿或未补偿), 外加两种保留状态: 与“unfilt”相同
   * @return None
   */
  void setFIFOMode2(uint8_t mode);

  /**
   * @fn setINTMode
   * @brief 中断配置(INT)
   * @param mode 需要设置的中断模式,下列模式相加为mode:
   * @n       中断引脚输出模式: eINTPinPP:  推挽 , eINTPinOD:  开漏
   * @n       中断引脚有效电平: eINTPinActiveLevelLow:  低电平有效 , eINTPinActiveLevelHigh:  高电平有效
   * @n       中断寄存器锁定: eINTLatchDIS:  禁用 , eINTLatchEN:  启用
   * @n       FIFO水位到达中断: eINTFWTMDIS:  禁用 , eINTFWTMEN:  启用
   * @n       FIFO存满中断: eINTFFullDIS:  禁用 , eINTFFullEN:  启用
   * @n       中断引脚初始(无效、无中断)电平: eINTInitialLevelLOW:  低电平 , eINTInitialLevelHIGH:  高电平
   * @n       温度/压力数据准备中断: eINTDataDrdyDIS:  禁用 , eINTDataDrdyEN:  启用
   * @return None
   */
  void setINTMode(uint8_t mode);

  /**
   * @fn setPWRMode
   * @brief 测量模式和电源模式的配置
   * @param mode 需要设置的测量模式和电源模式,下列模式相加为mode:
   * @n       ePressDIS:  禁用压力测量 , ePressEN:  启用压力测量
   * @n       eTempDIS:  禁用温度测量 , eTempEN:  启用温度测量
   * @n       eSleepMode, eForcedMode, eNormalMode 三种模式: 
   * @n         睡眠模式: 上电复位后默认设置为睡眠模式。在睡眠模式下, 不执行任何测量, 并且功耗最少。所有寄存器均可访问；可以读取芯片ID和补偿系数。
   * @n         强制模式: 在强制模式下, 根据选择的测量和滤波选项进行单个测量。测量完成后, 传感器返回睡眠模式, 测量结果可从数据寄存器中获得。
   * @n         正常模式: 在测量周期和待机周期之间连续循环, 输出数据率(output data rates)与ODR模式设置有关。
   * @return None
   */
  void setPWRMode(uint8_t mode);

  /**
   * @fn setOSRMode
   * @brief 压力和温度测量的过采样配置(OSR:over-sampling register)
   * @param mode 需要设置的压力和温度测量的过采样模式,下列模式相加为mode:
   * @n       6种压力过采样模式:
   * @n         ePressOSRMode1,  压力采样×1, 16 bit / 2.64 Pa(推荐温度过采样×1)
   * @n         ePressOSRMode2,  压力采样×2, 16 bit / 2.64 Pa(推荐温度过采样×1)
   * @n         ePressOSRMode4,  压力采样×4, 18 bit / 0.66 Pa(推荐温度过采样×1)
   * @n         ePressOSRMode8,  压力采样×8, 19 bit / 0.33 Pa(推荐温度过采样×2)
   * @n         ePressOSRMode16,  压力采样×16, 20 bit / 0.17 Pa(推荐温度过采样×2)
   * @n         ePressOSRMode32,  压力采样×32, 21 bit / 0.085 Pa(推荐温度过采样×2)
   * @n       6种温度过采样模式
   * @n         eTempOSRMode1,  温度采样×1, 16 bit / 0.0050 °C
   * @n         eTempOSRMode2,  温度采样×2, 16 bit / 0.0025 °C
   * @n         eTempOSRMode4,  温度采样×4, 18 bit / 0.0012 °C
   * @n         eTempOSRMode8,  温度采样×8, 19 bit / 0.0006 °C
   * @n         eTempOSRMode16,  温度采样×16, 20 bit / 0.0003 °C
   * @n         eTempOSRMode32,  温度采样×32, 21 bit / 0.00015 °C
   * @return None
   */
  void setOSRMode(uint8_t mode);

  /**
   * @fn setODRMode
   * @brief 细分/二次采样的方式设置输出数据率配置(ODR:output data rates)
   * @param mode 需要设置的输出数据率,可配置模式: 
   * @n       BMP3XX_ODR_200_HZ, BMP3XX_ODR_100_HZ, BMP3XX_ODR_50_HZ, BMP3XX_ODR_25_HZ, BMP3XX_ODR_12P5_HZ, 
   * @n       BMP3XX_ODR_6P25_HZ, BMP3XX_ODR_3P1_HZ, BMP3XX_ODR_1P5_HZ, BMP3XX_ODR_0P78_HZ, BMP3XX_ODR_0P39_HZ, 
   * @n       BMP3XX_ODR_0P2_HZ, BMP3XX_ODR_0P1_HZ, BMP3XX_ODR_0P05_HZ, BMP3XX_ODR_0P02_HZ, BMP3XX_ODR_0P01_HZ, 
   * @n       BMP3XX_ODR_0P006_HZ, BMP3XX_ODR_0P003_HZ, BMP3XX_ODR_0P0015_HZ
   * @return bool量, 表示配置结果
   * @retval True 表示配置成功, 成功更新配置
   * @retval False 表示配置失败, 保持原来的状态
   */
  bool setODRMode(uint8_t mode);

  /**
   * @fn setIIRMode
   * @brief IIR滤波系数配置(IIR filtering)
   * @param mode IIR滤波系数设置, 可配置模式: 
   * @n       BMP3XX_IIR_CONFIG_COEF_0, BMP3XX_IIR_CONFIG_COEF_1, BMP3XX_IIR_CONFIG_COEF_3, 
   * @n       BMP3XX_IIR_CONFIG_COEF_7, BMP3XX_IIR_CONFIG_COEF_15, BMP3XX_IIR_CONFIG_COEF_31, 
   * @n       BMP3XX_IIR_CONFIG_COEF_63, BMP3XX_IIR_CONFIG_COEF_127
   * @return None
   */
  void setIIRMode(uint8_t mode);

  /**
   * @fn setCommand
   * @brief 传感器FIFO清空命令和软复位命令
   * @param mode 传感器基本命令, 三种命令: 
   * @n       BMP3XX_CMD_NOP, 空命令
   * @n       BMP3XX_CMD_FIFO_FLUSH, 清除FIFO中的所有数据, 不改变FIFO配置
   * @n       BMP3XX_CMD_SOFTRESET, 触发重置, 所有用户配置设置将被其默认状态覆盖
   * @return None
   */
  void setCommand(uint8_t mode);

  /**
   * @fn setFIFOWTM
   * @brief FIFO水位设置配置
   * @param WTMSetting 需要设置的FIFO水位(0-511), FIFO填充达到水位值触发中断
   * @return None
   */
  void setFIFOWTM(uint16_t WTMSetting);

  /**
   * @fn setSamplingMode
   * @brief 让用户方便配置常用的采样模式
   * @param mode:
   * @n       eUltraLowPrecision, 超低精度, 适合天气监控(最低功耗), 电源模式为强制模式
   * @n       eLowPrecision, 低精度, 适合随意的检测, 电源模式为正常模式
   * @n       eNormalPrecision1, 标准精度1, 适合在手持式设备上动态检测(例如在手机上), 电源模式为正常模式
   * @n       eNormalPrecision2, 标准精度2, 适合无人机, 电源模式为正常模式
   * @n       eHighPrecision, 高精度, 适合在低功耗手持式设备上(例如在手机上), 电源模式为正常模式
   * @n       eUltraPrecision, 超高精度, 适合室内的导航, 采集速率会极低, 采集周期1000ms, 电源模式为正常模式
   * @return bool量, 表示配置结果
   * @retval True 表示配置成功, 成功更新配置
   * @retval False 表示配置失败, 保持原来的状态
   */
  bool setSamplingMode(ePrecisionMode_t mode);

/***************** 数据寄存器的获取和处理 ******************************/

  /**
   * @fn getSamplingPeriodUS
   * @brief 获取传感器当前采样模式下的采样周期
   * @return 返回采样周期, 单位us
   */
  uint32_t getSamplingPeriodUS(void);

  /**
   * @fn readTempC
   * @brief 从寄存器获取温度测量值, 工作范围(-40 ‒ +85 °C)
   * @return 返回温度测量值, 单位是℃
   */
  float readTempC(void);

  /**
   * @fn readPressPa
   * @brief 从寄存器获取压力测量值, 工作范围(300‒1250 hPa)
   * @return 返回压力测量值, 单位是Pa
   * @attention 若之前提供了基准值, 则根据校准的海平面大气压, 计算当前位置气压的绝对值
   */
  float readPressPa(void);

  /**
   * @fn calibratedAbsoluteDifference
   * @brief 以给定的当前位置海拔做为基准值, 为后续压力和海拔数据消除绝对差
   * @param altitude 当前位置海拔高度
   * @return bool量, 表示设置基准值是否成功
   * @retval True 表示设置基准值成功
   * @retval False 表示设置基准值失败
   */
  bool calibratedAbsoluteDifference(float altitude);

  /**
   * @fn readAltitudeM
   * @brief 根据传感器所测量大气压, 计算海拔高度
   * @return 返回海拔高度, 单位m
   * @attention 若之前提供了基准值, 则根据校准的海平面大气压, 计算当前位置海拔绝对高度
   */
  float readAltitudeM(void);

  /**
   * @fn getFIFOData
   * @brief 获取FIFO中缓存的数据
   * @param FIFOTemperatureC 用于存储温度测量数据的变量
   * @param FIFOPressurePa 用于存储压力测量数据的变量
   * @note 温度单位摄氏度, 压力单位帕
   * @return None
   */
  void getFIFOData(float &FIFOTemperatureC, float &FIFOPressurePa);

  /**
   * @fn getFIFOLength
   * @brief 获取FIFO已缓存数据大小
   * @return 返回值范围为: 0-511
   */
  uint16_t getFIFOLength(void);


protected:

  /**
   * @fn setIFCONFMode
   * @brief 串行接口配置
   * @param mode 需要设置的串行接口模式,下列模式相加为mode:
   * @n       eSerialModeSPI4:  SPI四线模式 , eSerialModeSPI3:  SPI三线模式
   * @n       eI2CWDTDIS:  禁用 , eI2CWDTEN:  启用I2C看门狗定时器
   * @n       eI2CWDTSel1p25:  I2C看门狗定时器在1.25ms后超时 , eI2CWDTSel40:  I2C看门狗定时器在40ms后超时
   * @return None
   */
  void setIFCONFMode(uint8_t mode);

  /**
   * @fn getFIFOWTMValue
   * @brief 获取FIFO设定的水位值
   * @return 返回值范围为: 0-511
   */
  uint16_t getFIFOWTMValue(void);

  /**
   * @fn getBMP3XXCalibData
   * @brief 获取sCalibData_t补偿校准数据
   * @return None
   */
  void getBMP3XXCalibData(void);

  /**
   * @fn calibTemperatureC
   * @brief 利用校准系数, 对原始数据进行补偿
   * @return 返回补偿后的温度测量值, 单位是摄氏度
   */
  float calibTemperatureC(uint32_t uncompTemp);

  /**
   * @fn calibPressurePa
   * @brief 利用校准系数, 对原始数据进行补偿
   * @return 返回补偿后的压力测量值, 单位是Pa
   */
  float calibPressurePa(uint32_t uncompPress);

/***************** 传感器状态寄存器的获取和处理 ******************************/

  /**
   * @fn cacheErrorStatus
   * @brief 这个API获取传感器发出的错误信息(致命的、命令和配置错误)。
   * @note 获取到的信息将存入到结构体BMP3Info._errStatus中: 
   * @n      BMP3Info.errStatus.fatalError:致命错误, 不可恢复的错误
   * @n      BMP3Info.errStatus.CMDError:命令执行失败, 在阅读后清零
   * @n      BMP3Info.errStatus.configError:检测到传感器配置错误(仅在正常模式下工作), 在阅读后清零
   * @return None
   */
  void cacheErrorStatus(void);

  /**
   * @fn cacheSensorStatus
   * @brief 这个API从传感器获取压力数据及温度数据是否准备好的状态, 和CMD解码器状态。
   * @note 获取到的信息将存入到结构体BMP3Info._sensorStatus中: 
   * @n      BMP3Info.sensorStatus.CMDReady:CMD解码器状态
   * @n      BMP3Info.sensorStatus.pressDrdy:当一个压力数据寄存器被读出时, 它会被重置
   * @n      BMP3Info.sensorStatus.tempDrdy:当一个温度数据寄存器被读出时, 它将被重置
   * @return None
   */
  void cacheSensorStatus(void);

  /**
   * @fn cacheSensorEvent
   * @brief 这个API从传感器获取传感器事件: 设备加电或软复位后为“ 1”, 阅读时清除；
   * @note 在压力或温度转换过程中发生接口事务时为“ 1”, 阅读时清除。
   * @n    获取到的信息将存入到结构体BMP3Info._sensorEvent中: 
   * @n      BMP3Info.sensorEvent.porDetected:“1”设备上电或软复位后, 在阅读后清零
   * @n      BMP3Info.sensorEvent.itfActPt:“1”当在压力或温度转换期间发生串行接口事务时, 在阅读后清零
   * @return None
   */
  void cacheSensorEvent(void);

  /**
   * @fn cacheINTStatus
   * @brief 这个API获取传感器中断的状态(FIFO水位, FIFO满, 数据准备好)。
   * @note 获取到的信息将存入到结构体BMP3Info._INTStatus中: 
   * @n      BMP3Info.INTStatus.fwtmINT:FIFO水位中断
   * @n      BMP3Info.INTStatus.ffullINT:FIFO存满中断
   * @n      BMP3Info.INTStatus.dataReady:数据准备好中断
   * @return None
   */
  void cacheINTStatus(void);

/***************** 寄存器读写接口 ******************************/

  /**
   * @fn writeReg
   * @brief 写寄存器函数, 设计为纯虚函数, 由派生类实现函数体
   * @param reg  寄存器地址 8bits
   * @param pBuf 要写入数据的存放缓存
   * @param size 要写入数据的长度
   * @return None
   */
  virtual void writeReg(uint8_t reg, const void* pBuf, size_t size)=0;

  /**
   * @fn readReg
   * @brief 读取寄存器函数, 设计为纯虚函数, 由派生类实现函数体
   * @param reg  寄存器地址 8bits
   * @param pBuf 要读取数据的存放缓存
   * @param size 要读取数据的长度
   * @return 返回读取的长度, 返回0表示读取失败
   */
  virtual size_t readReg(uint8_t reg, void* pBuf, size_t size)=0;

private:
  // 私有化定义的变量
  sBMP3XXDeviceInfo_t BMP3Info;
};

/***************** IIC和SPI接口的初始化和读写 ******************************/

class DFRobot_BMP3XX_IIC:public DFRobot_BMP3XX
{
public:
  /**
   * @fn DFRobot_BMP3XX_IIC
   * @brief 构造函数, 根据SDO引脚接线, 设置传感器IIC通信地址
   * @param pWire Wire.h里定义了Wire对象, 因此使用&Wire就能够指向并使用Wire中的方法
   * @param mode SDO引脚连接到GND,此时I2C地址为0x76;SDO引脚连接到VDDIO,此时I2C地址为0x77
   * @param chipID 芯片ID
   * @return None
   */
  DFRobot_BMP3XX_IIC(TwoWire *pWire, eSDOPinMode_t mode, uint8_t chipID);

  /**
   * @fn begin
   * @brief 子类初始化函数
   * @return int类型, 表示返回初始化的状态
   * @retval 0 NO_ERROR
   * @retval -1 ERR_DATA_BUS
   * @retval -2 ERR_IC_VERSION
   */
  virtual int begin(void);

protected:
  /**
   * @fn writeReg
   * @brief 通过IIC总线写入寄存器值
   * @param reg  寄存器地址 8bits
   * @param pBuf 要写入数据的存放缓存
   * @param size 要写入数据的长度
   * @return None
   */
  virtual void writeReg(uint8_t reg, const void* pBuf, size_t size);

  /**
   * @fn readReg
   * @brief 通过IIC总线读取寄存器值
   * @param reg  寄存器地址 8bits
   * @param pBuf 要读取数据的存放缓存
   * @param size 要读取数据的长度
   * @return 返回读取的长度, 返回0表示读取失败
   */
  virtual size_t readReg(uint8_t reg, void* pBuf, size_t size);

private:
  TwoWire *_pWire;   // IIC通信方式的指针
  uint8_t _deviceAddr;   // IIC通信的设备地址
};

class DFRobot_BMP3XX_SPI:public DFRobot_BMP3XX
{
public:
  /**
   * @fn DFRobot_BMP3XX_SPI
   * @brief 构造函数
   * @param pSpi SPI.h里定义了extern SPIClass SPI;因此取SPI对象的地址就能够指向并使用SPI中的方法
   * @param csPin 是指定cs接的数字引脚
   * @param chipID 芯片ID
   * @return None
   */
  DFRobot_BMP3XX_SPI(SPIClass *pSpi, uint8_t csPin, uint8_t chipID);

  /**
   * @fn DFRobot_BMP3XX_SPI
   * @brief 子类初始化函数
   * @return int类型, 表示返回初始化的状态
   * @retval 0 NO_ERROR
   * @retval -1 ERR_DATA_BUS
   * @retval -2 ERR_IC_VERSION
   */
  virtual int begin(void);

protected:
  /**
   * @fn writeReg
   * @brief 通过SPI总线写入寄存器值
   * @param reg  寄存器地址 8bits
   * @param pBuf 要写入数据的存放缓存
   * @param size 要写入数据的长度
   * @return None
   */
  virtual void writeReg(uint8_t reg, const void* pBuf, size_t size);

  /**
   * @fn readReg
   * @brief 通过SPI总线读取寄存器值
   * @param reg  寄存器地址 8bits
   * @param pBuf 要读取数据的存放缓存
   * @param size 要读取数据的长度
   * @return 返回读取的长度, 返回0表示读取失败
   */
  virtual size_t readReg(uint8_t reg, void* pBuf, size_t size);

private:
  SPIClass *_pSpi;   // SPI通信方式的指针
  uint8_t _csPin;   // SPI通信的片选引脚
};

/***************** BMP388芯片 ******************************/
/***************** IIC和SPI接口的初始化和读写 ******************************/

class DFRobot_BMP388_IIC:public DFRobot_BMP3XX_IIC
{
public:
  /**
   * @fn DFRobot_BMP388_IIC
   * @brief 构造函数, 根据SDO引脚接线, 设置传感器IIC通信地址
   * @param pWire Wire.h里定义了Wire对象, 因此使用&Wire就能够指向并使用Wire中的方法
   * @param mode SDO引脚连接到GND,此时I2C地址为0x76;SDO引脚连接到VDDIO,此时I2C地址为0x77
   * @return None
   */
  DFRobot_BMP388_IIC(TwoWire *pWire=&Wire, eSDOPinMode_t mode=eSDOVDD);

};

class DFRobot_BMP388_SPI:public DFRobot_BMP3XX_SPI
{
public:
  /**
   * @fn DFRobot_BMP388_SPI
   * @brief 构造函数, 根据SDO引脚接线, 设置传感器IIC通信地址
   * @param pWire Wire.h里定义了Wire对象, 因此使用&Wire就能够指向并使用Wire中的方法
   * @param csPin 是指定cs接的数字引脚
   * @return None
   */
  DFRobot_BMP388_SPI(SPIClass *pSpi=&SPI, uint8_t csPin=3);

};

/***************** BMP390L芯片 ******************************/
/***************** IIC和SPI接口的初始化和读写 ******************************/

class DFRobot_BMP390L_IIC:public DFRobot_BMP3XX_IIC
{
public:
  /**
   * @fn DFRobot_BMP390L_IIC
   * @brief 构造函数, 根据SDO引脚接线, 设置传感器IIC通信地址
   * @param pWire Wire.h里定义了Wire对象, 因此使用&Wire就能够指向并使用Wire中的方法
   * @param mode SDO引脚连接到GND,此时I2C地址为0x76;SDO引脚连接到VDDIO,此时I2C地址为0x77
   * @return None
   */
  DFRobot_BMP390L_IIC(TwoWire *pWire=&Wire, eSDOPinMode_t mode=eSDOVDD);

};

class DFRobot_BMP390L_SPI:public DFRobot_BMP3XX_SPI
{
public:
  /**
   * @fn DFRobot_BMP390L_SPI
   * @brief 构造函数, 根据SDO引脚接线, 设置传感器IIC通信地址
   * @param pWire Wire.h里定义了Wire对象, 因此使用&Wire就能够指向并使用Wire中的方法
   * @param csPin 是指定cs接的数字引脚
   * @return None
   */
  DFRobot_BMP390L_SPI(SPIClass *pSpi=&SPI, uint8_t csPin=3);

};

#endif
