# DFRobot_BMP3XX
* [English Version](./README.md)

这是一个BMP3XX的库，功能是读取温度和压力。
BMP(390L/388)是一款基于可靠传感原理的压力和温度测量数字传感器。
传感器模块安装在一个非常紧凑的10针金属盖LGA封装中，占地面积仅为2.0 × 2.0 mm²，最大0.8 mm封装高度。
它的小尺寸和低功耗的3.2µA @1Hz允许实现在电池驱动的设备，如手机，GPS模块或手表。

<img src="https://ws.dfrobot.com.cn/FgGMuOYn58ZHD5s6jcOWoRUwVlOh" width="450" hegiht="" align=right/>


## 产品链接 (https://www.dfrobot.com.cn/goods-1392.html)
    SKU：SEN0423/SEN0251


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
* BMP390L比它的前辈更精确，覆盖从300 hPa到1250 hPa的宽测量范围。
* 这款新的气压传感器(BMP390L)具有诱人的性价比和低功耗。
* 由于传感器(BMP390L)数据的内置硬件同步及其从外部设备同步数据的能力。


## 库安装

要使用这个库，首先下载库文件，将其粘贴到\Arduino\libraries目录中，然后打开示例文件夹并在文件夹中运行演示。


## 方法

```C++

  /**
  * @brief 初始化函数
  * @return 返回0表示初始化成功，返回其他值表示初始化失败，返回错误码
  */
  virtual int begin(void);

  /**
  * @brief 让用户方便配置常用的采样模式
  * @param mode:
  *        eUltraLowPrecision，超低精度，适合天气监控（最低功耗），电源模式为强制模式
  *        eLowPrecision，低精度，适合随意的检测，电源模式为正常模式
  *        eNormalPrecision1，标准精度1，适合在手持式设备上动态检测（例如在手机上），电源模式为正常模式
  *        eNormalPrecision2，标准精度2，适合无人机，电源模式为正常模式
  *        eHighPrecision，高精度，适合在低功耗手持式设备上（例如在手机上），电源模式为正常模式
  *        eUltraPrecision，超高精度，适合室内的导航，采集速率会极低，采集周期1000ms，电源模式为正常模式
  * @return 返回True表示配置成功，返回False表示配置失败，保持原来的状态
  */
  bool setSamplingMode(ePrecisionMode_t mode);

  /**
  * @brief 获取传感器当前采样模式下的采样周期
  * @return 返回采样周期，单位us
  */
  uint32_t getSamplingPeriodUS(void);

  /**
  * @brief 从寄存器获取温度测量值，工作范围（-40 ‒ +85 °C）
  * @return 返回温度测量值，单位是℃
  */
  float readTempC(void);

  /**
  * @brief 从寄存器获取压力测量值，工作范围（300‒1250 hPa）
  *        若之前提供了基准值，则根据校准的海平面大气压，计算当前位置气压的绝对值
  * @return 返回压力测量值，单位是Pa
  */
  float readPressPa(void);

  /**
  * @brief 以给定的当前位置海拔做为基准值，为后续压力和海拔数据消除绝对差
  * @param altitude 当前位置海拔高度
  * @return 传入基准值成功，返回ture，失败返回false
  */
  bool calibratedAbsoluteDifference(float altitude);

  /**
  * @brief 根据传感器所测量大气压，计算海拔高度
  *        若之前提供了基准值，则根据校准的海平面大气压，计算当前位置海拔绝对高度
  * @return 返回海拔高度，单位m
  */
  float readAltitudeM(void);

  /**
  * @brief 获取FIFO中缓存的数据
  *        温度单位摄氏度，压力单位帕
  */
  void getFIFOData(float &FIFOTemperatureC, float &FIFOPressurePa);

  /**
  * @brief 传感器FIFO清空命令和软复位命令
  * @param mode 传感器基本命令，三种命令：
  *        BMP3XX_CMD_NOP，空命令
  *        BMP3XX_CMD_FIFO_FLUSH，清除FIFO中的所有数据，不改变FIFO配置
  *        BMP3XX_CMD_SOFTRESET，触发重置，所有用户配置设置将被其默认状态覆盖
  */
  void setCommand(uint8_t mode);

  /**
  * @brief FIFO水位设置配置
  * @param WTMSetting 需要设置的FIFO水位（0-511），FIFO填充达到水位值触发中断
  */
  void setFIFOWTM(uint16_t WTMSetting);

  /**
  * @brief FIFO配置一(FIFO1)
  * @param mode 需要设置的FIFO模式,下列模式相加为mode:
  *        eFIFODIS： 禁用FIFO ，eFIFOEN： 启用FIFO
  *        eFIFOStopOnFullDIS： 写满时继续写入 ，eFIFOStopOnFullEN： 写满时停止写入
  *        eFIFOTimeDIS： 禁用 ，eFIFOTimeEN： 启用在最后一个有效数据帧之后返回传感器时间帧
  *        eFIFOPressDIS： 禁用压力数据缓存 ，eFIFOPressEN： 启用压力数据缓存
  *        eFIFOTempDIS： 禁用温度数据缓存 ，eFIFOTempEN： 启用温度数据缓存
  */
  void setFIFOMode1(uint8_t mode);

  /**
  * @brief FIFO配置二(FIFO2)
  * @param mode 需要设置的FIFO模式,下列模式相加为mode:
  *        8种压力和温度数据的FIFO下采样选择(1-128), 系数为2^fifo_subsampling(0-7)：
  *          eFIFOSubsampling0, eFIFOSubsampling1, eFIFOSubsampling2, eFIFOSubsampling3,
  *          eFIFOSubsampling4, eFIFOSubsampling5, eFIFOSubsampling6, eFIFOSubsampling7,
  *        eFIFODataSelectDIS： 未过滤数据(补偿或未补偿) ，eFIFODataSelectEN： 过滤数据(补偿或未补偿)，外加两种保留状态：与“unfilt”相同
  */
  void setFIFOMode2(uint8_t mode);

  /**
  * @brief 中断配置(INT)
  * @param mode 需要设置的中断模式,下列模式相加为mode:
  *        中断引脚输出模式: eINTPinPP： 推挽 ，eINTPinOD： 开漏
  *        中断引脚有效电平: eINTPinActiveLevelLow： 低电平有效 ，eINTPinActiveLevelHigh： 高电平有效
  *        中断寄存器锁定: eINTLatchDIS： 禁用 ，eINTLatchEN： 启用
  *        FIFO水位到达中断: eINTFWTMDIS： 禁用 ，eINTFWTMEN： 启用
  *        FIFO存满中断: eINTFFullDIS： 禁用 ，eINTFFullEN： 启用
  *        中断引脚初始(无效、无中断)电平: eINTInitialLevelLOW： 低电平 ，eINTInitialLevelHIGH： 高电平
  *        温度/压力数据准备中断: eINTDataDrdyDIS： 禁用 ，eINTDataDrdyEN： 启用
  */
  void setINTMode(uint8_t mode);

  /**
  * @brief 测量模式和电源模式的配置
  * @param mode 需要设置的测量模式和电源模式,下列模式相加为mode:
  *        ePressDIS： 禁用压力测量 ，ePressEN： 启用压力测量
  *        eTempDIS： 禁用温度测量 ，eTempEN： 启用温度测量
  *        eSleepMode, eForcedMode, eNormalMode 三种模式：
  *          睡眠模式：上电复位后默认设置为睡眠模式。在睡眠模式下，不执行任何测量，并且功耗最少。所有寄存器均可访问；可以读取芯片ID和补偿系数。
  *          强制模式：在强制模式下，根据选择的测量和滤波选项进行单个测量。测量完成后，传感器返回睡眠模式，测量结果可从数据寄存器中获得。
  *          正常模式：在测量周期和待机周期之间连续循环，输出数据率(output data rates)与ODR模式设置有关。
  */
  void setPWRMode(uint8_t mode);

  /**
  * @brief 压力和温度测量的过采样配置(OSR:over-sampling register)
  * @param mode 需要设置的压力和温度测量的过采样模式,下列模式相加为mode:
           6种压力过采样模式:
             ePressOSRMode1,  压力采样×1，16 bit / 2.64 Pa（推荐温度过采样×1）
             ePressOSRMode2,  压力采样×2，16 bit / 2.64 Pa（推荐温度过采样×1）
             ePressOSRMode4,  压力采样×4，18 bit / 0.66 Pa（推荐温度过采样×1）
             ePressOSRMode8,  压力采样×8，19 bit / 0.33 Pa（推荐温度过采样×2）
             ePressOSRMode16,  压力采样×16，20 bit / 0.17 Pa（推荐温度过采样×2）
             ePressOSRMode32,  压力采样×32，21 bit / 0.085 Pa（推荐温度过采样×2）
           6种温度过采样模式:
             eTempOSRMode1,  温度采样×1，16 bit / 0.0050 °C
             eTempOSRMode2,  温度采样×2，16 bit / 0.0025 °C
             eTempOSRMode4,  温度采样×4，18 bit / 0.0012 °C
             eTempOSRMode8,  温度采样×8，19 bit / 0.0006 °C
             eTempOSRMode16,  温度采样×16，20 bit / 0.0003 °C
             eTempOSRMode32,  温度采样×32，21 bit / 0.00015 °C
  */
  void setOSRMode(uint8_t mode);

  /**
  * @brief 细分/二次采样的方式设置输出数据率配置(ODR:output data rates)
  * @param mode 需要设置的输出数据率,可配置模式：
  *        BMP3XX_ODR_200_HZ，BMP3XX_ODR_100_HZ，BMP3XX_ODR_50_HZ，BMP3XX_ODR_25_HZ，BMP3XX_ODR_12P5_HZ，
  *        BMP3XX_ODR_6P25_HZ，BMP3XX_ODR_3P1_HZ，BMP3XX_ODR_1P5_HZ，BMP3XX_ODR_0P78_HZ，BMP3XX_ODR_0P39_HZ，
  *        BMP3XX_ODR_0P2_HZ，BMP3XX_ODR_0P1_HZ，BMP3XX_ODR_0P05_HZ，BMP3XX_ODR_0P02_HZ，BMP3XX_ODR_0P01_HZ，
  *        BMP3XX_ODR_0P006_HZ，BMP3XX_ODR_0P003_HZ，BMP3XX_ODR_0P0015_HZ
  * @return 返回True表示配置成功，返回False表示配置失败，保持原来的状态
  */
  bool setODRMode(uint8_t mode);

  /**
  * @brief IIR滤波系数配置（IIR filtering）
  * @param mode IIR滤波系数设置，可配置模式：
  *        BMP3XX_IIR_CONFIG_COEF_0，BMP3XX_IIR_CONFIG_COEF_1，BMP3XX_IIR_CONFIG_COEF_3，
  *        BMP3XX_IIR_CONFIG_COEF_7，BMP3XX_IIR_CONFIG_COEF_15，BMP3XX_IIR_CONFIG_COEF_31，
  *        BMP3XX_IIR_CONFIG_COEF_63，BMP3XX_IIR_CONFIG_COEF_127
  */
  void setIIRMode(uint8_t mode);

  /**
  * @brief 获取FIFO已缓存数据大小
  * @return 返回值范围为：0-511
  */
  uint16_t getFIFOLength(void);

  /**
  * @brief FIFO水位设置配置
  * @param WTMSetting 需要设置的FIFO水位（0-511），FIFO填充达到水位值触发中断
  */
  uint16_t getFIFOWTMValue(void);

```


## 兼容性

主板                | 通过    | 未通过   | 未测试 | 备注 
------------------ | :----------: | :----------: | :---------: | :---:
Arduino Uno        |      √       |              |             |
Arduino MEGA2560   |      √       |              |             |
Arduino Leonardo   |      √       |              |             |
FireBeetle-ESP8266 |      √       |              |             |
FireBeetle-ESP32   |      √       |              |             |
FireBeetle-M0      |      √       |              |             |
Micro:bit          |      √       |              |             |
Raspberry Pi       |      √       |              |             |


## 历史

- Data 2021-04-20
- Version V1.0.0


## 创作者

Written by qsjhyy(yihuan.huang@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))

