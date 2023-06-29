# DFRobot_MuVisionSensor
- [English Version](./README.md)

MU Vision 传感器是一个传感器模块，支持 Arduino、Microbit 和其他支持 UART 或 I2C 通信协议的 haredware 平台。

您可以使用这些库读取数据或设置 MU 视觉传感器的属性。

![产品效果图片](./resources/images/SEN0314.jpg)


## 产品链接 (https://www.dfrobot.com.cn/goods-2003.html)
 SKU: SEN0314


## Table of Contents

  * [概述](#概述)
  * [库安装](#库安装)
  * [方法](#方法)
  * [兼容性](#兼容性)
  * [历史](#历史)
  * [创作者](#创作者)


## Summary

* 能够做出丰富多样的视觉课程应用
   智能打印扫描小车
   智能无人车
   寻球投篮机器人等
* 体积小巧，功耗极低
* 易于使用：支持UART，I2C，WIFI通讯方式，适用于Arduino及micro:bit嵌入式平台
* 应用广泛
* 安全可靠，无需网络：所有视觉识别算法都于本地进行处理，无须联网，不受网络状况所限制，无需担忧隐私泄露


## Installation

这里有2种安装方法：

1. 使用此库前，请首先下载库文件，将其粘贴到\Arduino\libraries目录中，然后打开examples文件夹并在该文件夹中运行演示。
2. 直接在Arduino软件库管理中搜索下载 DFRobot_MuVisionSensor 库。


## 方法

```C++
  /**
    * @brief  MU视觉传感器初始化。
    * @param  communicationPort: 通信端口（MuVsUart或uart）。
    * @retval MU_OK: 初始化成功。
    *         其他：协议验证失败。
    */
  uint8_t begin(MuVsUart* communicationPort);

  /**
    * @brief  MU视觉传感器初始化。
    * @param  communicationPort: 通信端口（MuVsI2C或i2c）。
    * @retval MU_OK: 初始化成功。
    *         其他：协议验证失败。
    */
  uint8_t begin(MuVsI2C* communicationPort);

  // 基于接口
  /**
    * @brief  开始视觉识别。
    * @param  visionType: 视觉类型。
    * @retval MU_OK: 成功。
    *         其他：错误。
    */
  uint8_t visionBegin(muVisionType);

  /**
    * @brief  结束视觉识别。
    * @param  visionType: 视觉类型。
    * @retval MU_OK: 成功。
    *         其他：错误。
    */
  uint8_t visionEnd(muVisionType);

  /**
    * @brief  获取视觉识别结果数据，该函数将自动更新视觉结果。
    * @param  visionType: 视觉类型。
    * @param  objectInf:  对象信息。
    * @retval 信息值。
    */
  int getValue(muVisionType visionType, MuVsObjectInf objectInf);

  /**
    * @brief  写入视觉参数。
    * @param  visionType: 视觉类型。
    * @param  objectInf:  对象信息。
    * @param  value:  值。
    * @retval MU_OK: 成功。
    *         其他：错误。
    */
  int setValue(muVisionType visionType, MuVsObjectInf objectInf, uint8_t value) {
    return write(visionType, objectInf, value);
  }

  /**
    * @brief  获取视觉结果缓冲区指针。
    *         此函数不会更新视觉结果，请在使用该函数之前使用函数`updateResult([visionType])`或`getValue([visionType], kStatus)`更新视觉结果。
    * @param  visionType: 视觉类型。
    * @retval 视觉结果缓冲区指针，如果视觉类型不是`begin`或不受支持，则返回`nullptr`。
    */
  muVsVisionState* getVisionState(muVisionType visionType);

  // 高级接口
  /**
    * @brief  从视觉传感器更新结果数据，必须在visionBegin或visionSetStatus之后使用。
    * @param  visionType: 视觉类型。
    * @param  waitAllResult: true：获取所有输入视觉类型后返回。
    *                        false：获取输入视觉类型之一后返回。
    * @retval 已更新的视觉类型。
    */
  muVisionType updateResult(muVisionType visionType, bool waitAllResult = true);

  /**
    * @brief  写入视觉参数。
    * @param  visionType: 视觉类型。
    * @param  objectInf:  对象信息。
    * @param  value:  值。
    * @retval MU_OK:  成功。
    *         其他：错误。
    */
  uint8_t write(muVisionType, MuVsObjectInf, uint8_t value);

  /**
    * @brief  读取结果数据。
    * @param  visionType: 视觉类型。
    * @param  objectInf:  对象信息。
    * @retval 信息值。
    */
  uint8_t read(muVisionType visionType, MuVsObjectInf objectInf, uint8_t resultNum = 1);

  // 传感器功能
  // @brief  重启MU视觉传感器。
  uint8_t sensorSetRestart(void);
  // @brief  将所有寄存器设置为默认值（包括波特率）。
  uint8_t sensorSetDefault(void);

  // LED功能
  /**
    * @brief  设置LED。
    * @param  led: LED类型。
    * @param  manual: 手动模式。
    * @param  hold: 保持状态。
    * @retval MU_OK:  成功。
    *         其他：错误。
    */
  uint8_t ledSetMode(MuVsLed led, bool manual, bool hold);

  /**
    * @brief  设置LED颜色。
    * @param  led: LED类型。
    * @param  detectedColor: 传感器检测到目标时的LED颜色。
    * @param  undetectedColor: 传感器未检测到目标时的LED颜色。
    * @param  level: LED亮度，范围从0（关闭）到15。
    * @retval MU_OK:  成功。
    *         其他：错误。
    */
  uint8_t ledSetColor(MuVsLed led, MuVsLedColor detectedColor, MuVsLedColor undetectedColor, uint8_t level = 1);

  // 相机功能
  /**
    * @brief  设置相机缩放。
    * @param  相机缩放值。
  

  * @retval MU_OK:  成功。
    *         其他：错误。
    */
  uint8_t cameraSetZoom(muVsCameraZoom);

  /**
    * @brief  旋转相机。
    * @param  enable: true：旋转相机。
    *                 false：默认值。
    * @retval MU_OK:  成功。
    *         其他：错误。
    */
  uint8_t cameraSetRotate(bool enable);

  /**
    * @brief  设置相机帧率。
    * @param  相机帧率类型。
    * @retval MU_OK:  成功。
    *         其他：错误。
    */
  uint8_t cameraSetFPS(muVsCameraFPS);

  /**
    * @brief  设置相机白平衡。
    * @param  相机白平衡类型。
    * @retval MU_OK:  成功。
    *         其他：错误。
    */
  uint8_t cameraSetAwb(muVsCameraWhiteBalance);

  /**
    * @brief  获取相机缩放值。
    * @retval 相机缩放值。
    */
  muVsCameraZoom cameraGetZoom(void);

  /**
    * @brief  获取相机白平衡类型。
    * @retval 相机白平衡类型。
    */
  muVsCameraWhiteBalance cameraGetAwb(void);

  /**
    * @brief  获取相机旋转状态。
    * @retval 相机旋转状态。
    */
  bool cameraGetRotate(void);

  /**
    * @brief  获取相机帧率类型。
    * @retval 相机帧率类型。
    */
  muVsCameraFPS cameraGetFPS(void);

  // UART功能
  /**
    * @brief  设置UART波特率。
    * @param  UART波特率。
    * @retval MU_OK:  成功。
    *         其他：错误。
    */
  uint8_t uartSetBaudrate(muVsBaudrate baud);

  // 视觉功能
  /**
    * @brief  设置视觉状态。
    * @param  visionType: 视觉类型。
    * @param  enable: true：启用视觉。
    *                 false：禁用视觉。
    * @retval MU_OK:  成功。
    *         其他：错误。
    */
  uint8_t visionSetStatus(muVisionType visionType, bool enable);

  /**
    * @brief  设置视觉输出模式。
    * @param  visionType: 视觉类型。
    * @param  mode: 输出模式。
    * @retval MU_OK:  成功。
    *         其他：错误。
    */
  uint8_t visionSetOutputMode(muVsStreamOutputMode mode);

  /**
    * @brief  输出使能。
    * @param  visionType: 视觉类型。
    * @param  status: true：开始输出。
    *                 false：停止输出。
    * @retval MU_OK:  成功。
    *         其他：错误。
    */
  uint8_t visionSetOutputEnable(muVisionType visionType, bool status);

  /**
    * @brief  将视觉配置设置为默认值。
    * @param  visionType: 视觉类型。
    * @retval MU_OK:  成功。
    *         其他：错误。
    */
  uint8_t visionSetDefault(muVisionType visionType);

  /**
    * @brief  设置视觉等级。
    * @param  visionType: 视觉类型。
    * @param  level: 视觉等级。
    * @retval MU_OK:  成功。
    *         其他：错误。
    */
  uint8_t visionSetLevel(muVisionType visionType, muVsVisionLevel level);

  /**
    * @brief  获取视觉状态。
    * @param  visionType: 视觉类型。
    * @retval 视觉状态。
    */
  bool visionGetStatus(muVisionType visionType);

  /**
    * @brief  获取视觉等级。
    * @param  visionType: 视觉类型。
    * @retval 视觉等级。
    */
  muVsVisionLevel visionGetLevel(muVisionType visionType);

  /**
    * @brief  获取视觉输出模式。
    * @retval 视觉输出模式。
    */
  muVsStreamOutputMode visionGetOutputMode(void);

```


## 兼容性

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | -----
Arduino Uno        |      √       |              |             | 


## 历史

- 2023/06/29 - 1.1.7 版本


## 创作者

Written by DFRobot, 2022. (Welcome to our [website](https://www.dfrobot.com/))
