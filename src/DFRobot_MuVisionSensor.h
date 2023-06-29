/*!
 * @file DFRobot_MuVisionSensor.h
 * @brief Basic struct of DFRobot_MuVisionSensor class.
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT license (MIT)
 * @author DFRobot
 * @version  V1.0
 * @date  2023-06-28
 * @https://github.com/DFRobot/DFRobot_MuVisionSensor
 */
#ifndef MUVISIONSENSOR_SRC_MUVISIONSENSOR_H_
#define MUVISIONSENSOR_SRC_MUVISIONSENSOR_H_

#include "muVisionSensorInterface.h"
#include "muVisionSensorUartHwInterface.h"
#include "muVisionSensorI2cHwInterface.h"

/*
 * muVisionType
 * Bit: |          3         |          2         |            1             |           0         |
 *      | VISION_LINE_DETECT | VISION_BALL_DETECT | VISION_COLOR_RECOGNITION | VISION_COLOR_DETECT |
 *      |           7            |             6              |             5            |          4         |
 *      | VISION_NUM_CARD_DETECT | VISION_TRAFFIC_CARD_DETECT | VISION_SHAPE_CARD_DETECT | VISION_BODY_DETECT |
 */
typedef unsigned short muVisionType;

// muVisionType: Vision Type User Input
#define VISION_COLOR_DETECT           (muVisionType)visionTypeEnumToMacro(kVisionColorDetect)
#define VISION_COLOR_RECOGNITION      (muVisionType)visionTypeEnumToMacro(kVisionColorRecog)
#define VISION_BALL_DETECT            (muVisionType)visionTypeEnumToMacro(kVisionBall)
#define VISION_BODY_DETECT            (muVisionType)visionTypeEnumToMacro(kVisionBody)
#define VISION_SHAPE_CARD_DETECT      (muVisionType)visionTypeEnumToMacro(kVisionShapeCard)
#define VISION_TRAFFIC_CARD_DETECT    (muVisionType)visionTypeEnumToMacro(kVisionTrafficCard)
#define VISION_NUM_CARD_DETECT        (muVisionType)visionTypeEnumToMacro(kVisionNumberCard)
#define VISION_ALL                    (muVisionType)(visionTypeEnumToMacro(kVisionMaxType)-1)

// Card Type
// Vision Shape Card
#define MU_SHAPE_CARD_TICK            0x01U
#define MU_SHAPE_CARD_CROSS           0x02U
#define MU_SHAPE_CARD_CIRCLE          0x03U
#define MU_SHAPE_CARD_SQUARE          0x04U
#define MU_SHAPE_CARD_TRIANGLE        0x05U
// Vision Traffic Card
#define MU_TRAFFIC_CARD_FORWARD       0x01U
#define MU_TRAFFIC_CARD_LEFT          0x02U
#define MU_TRAFFIC_CARD_RIGHT         0x03U
#define MU_TRAFFIC_CARD_TURN_AROUND   0x04U
#define MU_TRAFFIC_CARD_PARK          0x05U
// Vision Color Type
#define MU_COLOR_UNKNOWN              0x00U
#define MU_COLOR_BLACK                0x01U
#define MU_COLOR_WHITE                0x02U
#define MU_COLOR_RED                  0x03U
#define MU_COLOR_YELLOW               0x04U
#define MU_COLOR_GREEN                0x05U
#define MU_COLOR_CYAN                 0x06U
#define MU_COLOR_BLUE                 0x07U
#define MU_COLOR_PURPLE               0x08U
// Vision Ball Type
#define MU_BALL_TABLE_TENNIS          0x01U
#define MU_BALL_TENNIS                0x02U


class DFRobot_MuVisionSensor {
 public:
  /**
    * @brief  construct class MU vision sensor.
    * @param  address: MU vision sensor device address.
    * @retval none
    */
  DFRobot_MuVisionSensor(uint32_t address = 0x60);
  virtual ~DFRobot_MuVisionSensor();

  /**
    * @ TODO WARNING this function may delete in later version, please use `begin(communicationPort)` instead.
    * @brief  MU vision sensor begin.
    * @param  communicationPort: MuVsI2C(i2c) or MuVsUart(uart).
    * @param  mode: kSerialMode
    *               kI2CMode
    * @retval MU_OK: begin success.
    *         other: protocol assert fail.
    */
  uint8_t __attribute__ ((deprecated("\n***WARNING*** function `begin(communicationPort, mode)` has been deprecated, and may delete in later version, please use `begin(communicationPort)` instead.")))
          begin(void* communicationPort,
                muVsMode mode);
  /**
    * @brief  MU vision sensor begin.
    * @param  communicationPort: MuVsUart(uart).
    * @retval MU_OK: begin success.
    *         other: protocol assert fail.
    */
  uint8_t begin(MuVsUart* communicationPort);
  /**
    * @brief  MU vision sensor begin.
    * @param  communicationPort: MuVsI2C(i2c).
    * @retval MU_OK: begin success.
    *         other: protocol assert fail.
    */
  uint8_t begin(MuVsI2C* communicationPort);

  // Based interface
  /**
    * @brief  begin vision.
    * @param  visionType: vision type.
    * @retval MU_OK:  success
    *         other:  error
    */
  uint8_t visionBegin(muVisionType);
  /**
    * @brief  end vision.
    * @param  visionType: vision type.
    * @retval MU_OK:  success
    *         other:  error
    */
  uint8_t visionEnd(muVisionType);
  /**
    * @brief  get vision result data, this function will update vision
    *         result automatically.
    * @param  visionType: vision type.
    * @param  objectInf:  object information
    * @retval information value
    */
  int getValue(muVisionType visionType,
               MuVsObjectInf objectInf);
  /**
    * @brief  write vision parameter.
    * @param  visionType: vision type.
    * @param  objectInf:  object information
    * @param  value:  value
    * @retval MU_OK:  success
    *         other:  error
    */
  int setValue(muVisionType visionType,
               MuVsObjectInf objectInf,
               uint8_t value) {
    return write(visionType, objectInf, value);
  }
  /**
    * @brief  get vision result buffer pointer.
    *         this function WILL NOT update vision result, please use
    *         function `updateResult([visionType])` or
    *         `getValue([visionType], kStatus)` to update vision result before this function
    * @param  visionType: vision type.
    * @retval vision result buffer pointer,
    *         return `nullptr` if the vision type is not `begin` or not supported
    */
  muVsVisionState* getVisionState(muVisionType visionType);

  // Advance interface
  /**
    * @brief  update result data from vision sensor, must used after
    *         visionBegin or visionSetStatus.
    * @param  visionType: vision type.
    * @param  waitAllResult:  true: return if get all input vision type
    *                           false: return if get one of the input vision type
    * @retval the vision type which have been updated
    */
  muVisionType updateResult(muVisionType visionType,
                            bool waitAllResult = true);
  /**
    * @brief  write vision parameter.
    * @param  visionType: vision type.
    * @param  objectInf:  object information
    * @param  value:  value
    * @retval MU_OK:  success
    *         other:  error
    */
  uint8_t write(muVisionType, MuVsObjectInf, uint8_t value);
  /**
    * @brief  read result data.
    * @param  visionType: vision type.
    * @param  objectInf:  object information
    * @retval information value
    */
  uint8_t read(muVisionType visionType,
               MuVsObjectInf objectInf,
               uint8_t resultNum = 1);

  // Sensor functions
  // @brief  restart MU vision sensor
  uint8_t sensorSetRestart(void);
  // @brief  set all register to default value(include baud rate)
  uint8_t sensorSetDefault(void);

  // LED functions
  /**
    * @brief  set led.
    * @param  led: led type.
    * @param  manual: vision type.
    * @param  hold:  object information
    * @retval MU_OK:  success
    *         other:  error
    */
  uint8_t ledSetMode(MuVsLed led, bool manual, bool hold);
  /**
    * @brief  set led color.
    * @param  led: led type.
    * @param  detectedColor: led color while sensor detected target.
    * @param  undetectedColor: led color while sensor undetected target.
    * @param  level:  led brightness, form 0(close) to 15
    * @retval MU_OK:  success
    *         other:  error
    */
  uint8_t ledSetColor(MuVsLed led,
                      MuVsLedColor detectedColor,
                      MuVsLedColor undetectedColor,
                      uint8_t level = 1);

  // Camera functions
  /**
    * @brief  set camera zoom.
    * @param  camera zoom value.
    * @retval MU_OK:  success
    *         other:  error
    */
  uint8_t cameraSetZoom(muVsCameraZoom);
  /**
    * @brief  rotate camera.
    * @param  enable: true: rotate camera.
    *                 false: default
    * @retval MU_OK:  success
    *         other:  error
    */
  uint8_t cameraSetRotate(bool enable);
  /**
    * @brief  set camera zoom.
    * @param  camera FPS type.
    * @retval MU_OK:  success
    *         other:  error
    */
  uint8_t cameraSetFPS(muVsCameraFPS);
  /**
    * @brief  set camera white balance.
    * @param  camera white balance type.
    * @retval MU_OK:  success
    *         other:  error
    */
  uint8_t cameraSetAwb(muVsCameraWhiteBalance);
  /**
    * @brief  get camera zoom value.
    * @retval camera zoom value
    */
  muVsCameraZoom cameraGetZoom(void);
  /**
    * @brief  get camera AWB type.
    * @retval camera AWB type
    */
  muVsCameraWhiteBalance cameraGetAwb(void);
  /**
    * @brief  get camera rotate state.
    * @retval camera rotate state
    */
  bool cameraGetRotate(void);
  /**
    * @brief  get camera FPS type.
    * @retval camera FPS type
    */
  muVsCameraFPS cameraGetFPS(void);

  // Uart functions
  /**
    * @brief  set UART baud rate.
    * @param  UART baud rate.
    * @retval MU_OK:  success
    *         other:  error
    */
  uint8_t uartSetBaudrate(muVsBaudrate baud);

  // Vision functions
  /**
    * @brief  set vision status.
    * @param  visionType: vision type
    * @param  enable: true: enable vision
    *                 false: disable vision
    * @retval MU_OK:  success
    *         other:  error
    */
  uint8_t visionSetStatus(muVisionType visionType, bool enable);
  /**
    * @brief  set vision status.
    * @param  visionType: vision type
    * @param  mode: output mode
    * @retval MU_OK:  success
    *         other:  error
    */
  uint8_t visionSetOutputMode(muVsStreamOutputMode mode);
  /**
    * @brief  output enable.
    * @param  visionType: vision type
    * @param  status: true: start output
    *                 false: stop output
    * @retval MU_OK:  success
    *         other:  error
    */
  uint8_t visionSetOutputEnable(muVisionType visionType, bool status);
  /**
    * @brief  set vision configure to default value.
    * @param  visionType: vision type
    * @retval MU_OK:  success
    *         other:  error
    */
  uint8_t visionSetDefault(muVisionType visionType);
  /**
    * @brief  set vision level.
    * @param  visionType: vision type
    * @param  level: vision level
    * @retval MU_OK:  success
    *         other:  error
    */
  uint8_t visionSetLevel(muVisionType visionType,
                         muVsVisionLevel level);
  /**
    * @brief  get vision status.
    * @retval vision status
    */
  bool visionGetStatus(muVisionType visionType);
  /**
    * @brief  get vision level.
    * @retval vision level
    */
  muVsVisionLevel visionGetLevel(muVisionType visionType);
  /**
    * @brief  get vision output mode.
    * @retval vision output mode
    */
  muVsStreamOutputMode visionGetOutputMode(void);


  DFRobot_MuVisionSensor(const DFRobot_MuVisionSensor&) = delete;
  DFRobot_MuVisionSensor& operator=(const DFRobot_MuVisionSensor &) = delete;

 private:
  uint8_t sensorLockReg(bool lock);
  muVisionType uartUpdateResult(muVisionType visionType, bool waitAllResult);
  bool mallocVisionBuffer(muVsMessageVisionType);
  bool freeVisionBuffer(muVsMessageVisionType);

  uint8_t _address = 0;
  muVsMode _mode = kSerialMode;
  muVsMethod* _muVsMethod = nullptr;
  muVsStreamOutputMode _outputMode = kCallBackMode;
  muVsVisionState *_visionState[kVisionMaxType-1] = {nullptr};
};


#endif /* ARDUINO_LIB_MUVISIONSENSOR_SRC_MUVISIONSENSOR_H_ */
