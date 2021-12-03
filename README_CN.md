# DFRobot_BMP3XX
* [English Version](./README.md)

这是一个BMP3XX的库, 功能是读取温度和压力。
BMP(390L/388)是一款基于可靠传感原理的压力和温度测量数字传感器。
传感器模块安装在一个非常紧凑的10针金属盖LGA封装中, 占地面积仅为2.0 × 2.0 mm², 最大0.8 mm封装高度。
它的小尺寸和低功耗的3.2µA @1Hz允许实现在电池驱动的设备, 如手机, GPS模块或手表。

![产品实物图](./resources/images/BMP388.png)


## 产品链接 (https://www.dfrobot.com.cn/goods-1392.html)
    SKU: SEN0423/SEN0251


## 目录

* [概述](#概述)
* [库安装](#库安装)
* [方法](#方法)
* [兼容性](#兼容性)
* [历史](#历史)
* [创作者](#创作者)


## 概述

* BMP(390L/388)可读取温度和压力。
* 该库支持SPI/I2C通信。
* BMP(390L/388)也包括FIFO功能。这极大地提高了易用性。
* 中断可以在不使用软件算法的情况下以高效的方式使用。
* BMP390L比它的前辈更精确, 覆盖从300 hPa到1250 hPa的宽测量范围。
* 这款新的气压传感器(BMP390L)具有诱人的性价比和低功耗。
* 由于传感器(BMP390L)数据的内置硬件同步及其从外部设备同步数据的能力。


## 库安装

要使用这个库, 首先下载库文件, 将其粘贴到\Arduino\libraries目录中, 然后打开示例文件夹并在文件夹中运行演示。


## 方法

```C++

  /**
   * @fn begin
   * @brief 初始化函数
   * @return int类型, 表示返回初始化的状态
   * @retval 0 NO_ERROR
   * @retval -1 ERR_DATA_BUS
   * @retval -2 ERR_IC_VERSION
   */
  virtual int begin(void);

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
   * @fn setFIFOMode1
   * @brief FIFO配置一(FIFO1)
   * @param mode 需要设置的FIFO模式, 下列模式经过或运算后得到mode:
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
   * @param mode 需要设置的FIFO模式, 下列模式经过或运算后得到mode:
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
   * @param mode 需要设置的中断模式, 下列模式经过或运算后得到mode:
   * @n       中断引脚输出模式: eINTPinPP:  推挽 , eINTPinOD:  开漏
   * @n       中断引脚有效电平: eINTPinActiveLevelLow:  低电平有效 , eINTPinActiveLevelHigh:  高电平有效
   * @n       中断寄存器锁定: eINTLatchDIS:  禁用 , eINTLatchEN:  启用
   * @n       FIFO水位到达中断: eIntFWtmDis:  禁用 , eIntFWtmEn:  启用
   * @n       FIFO存满中断: eINTFFullDIS:  禁用 , eINTFFullEN:  启用
   * @n       中断引脚初始(无效、无中断)电平: eINTInitialLevelLOW:  低电平 , eINTInitialLevelHIGH:  高电平
   * @n       温度/压力数据准备中断: eINTDataDrdyDIS:  禁用 , eINTDataDrdyEN:  启用
   * @return None
   */
  void setINTMode(uint8_t mode);

  /**
   * @fn setPWRMode
   * @brief 测量模式和电源模式的配置
   * @param mode 需要设置的测量模式和电源模式, 下列模式经过或运算后得到mode:
   * @n       ePressDIS:  禁用压力测量 , ePressEN:  启用压力测量
   * @n       eTempDIS:  禁用温度测量 , eTempEN:  启用温度测量
   * @n       eSleepMode, eForcedMode, eNormalMode 三种模式: 
   * @n         睡眠模式: 上电复位后默认设置为睡眠模式。在睡眠模式下, 不执行任何测量, 并且功耗最少。所有寄存器均可访问;可以读取芯片ID和补偿系数。
   * @n         强制模式: 在强制模式下, 根据选择的测量和滤波选项进行单个测量。测量完成后, 传感器返回睡眠模式, 测量结果可从数据寄存器中获得。
   * @n         正常模式: 在测量周期和待机周期之间连续循环, 输出数据率(output data rates)与ODR模式设置有关。
   * @return None
   */
  void setPWRMode(uint8_t mode);

  /**
   * @fn setOSRMode
   * @brief 压力和温度测量的过采样配置(OSR:over-sampling register)
   * @param mode 需要设置的压力和温度测量的过采样模式, 下列模式经过或运算后得到mode:
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
   * @fn getFIFOLength
   * @brief 获取FIFO已缓存数据大小
   * @return 返回值范围为: 0-511
   */
  uint16_t getFIFOLength(void);

```


## 兼容性

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | -----
Arduino Uno        |      √       |              |             | 
Arduino MEGA2560   |      √       |              |             | 
Arduino Leonardo   |      √       |              |             | 
FireBeetle-ESP8266 |      √       |              |             | 
FireBeetle-ESP32   |      √       |              |             | 
FireBeetle-M0      |      √       |              |             | 
Micro:bit          |      √       |              |             | 
Raspberry Pi       |      √       |              |             | 


## 历史

- 2021/04/20 - 1.0.0 版本
- 2021/11/08 - 1.0.1 版本
- 2021/12/03 - 1.0.2 版本


## 创作者

Written by qsjhyy(yihuan.huang@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))

