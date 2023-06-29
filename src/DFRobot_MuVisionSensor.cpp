/*!
 * @file DFRobot_MuVisionSensor.cpp
 * @brief Basic struct of DFRobot_MuVisionSensor class.
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT license (MIT)
 * @author DFRobot
 * @version  V1.0
 * @date  2023-06-28
 * @https://github.com/DFRobot/DFRobot_MuVisionSensor
 */
#include "DFRobot_MuVisionSensor.h"
#include <Arduino.h>

DFRobot_MuVisionSensor::DFRobot_MuVisionSensor(uint32_t address)
    : _address(address) {
}

DFRobot_MuVisionSensor::~DFRobot_MuVisionSensor() {
  if (_muVsMethod) {
    delete _muVsMethod;
  }
  for (int i = 1; i < kVisionMaxType; i++) {
    freeVisionBuffer(muVsMessageVisionType(i));
  }
}

uint8_t DFRobot_MuVisionSensor::begin(void* communicationPort,
                              muVsMode mode) {
  if (_muVsMethod) {
    delete _muVsMethod;
    _muVsMethod = nullptr;
  }
  _mode = mode;
  switch (mode) {
    case kSerialMode:
      if (_muVsMethod == nullptr) {
        _muVsMethod = new MuVisionSensorUart((MuVsUart *)communicationPort,
                                              _address);
      }
      break;
    case kI2CMode:
      if (_muVsMethod == nullptr) {
        _muVsMethod = new MuVisionSensorI2C((MuVsI2C *)communicationPort,
                                              _address);
      }
      break;
    default:
      return MU_ERROR_FAIL;
  }
  // check vs2 protocol version
  uint8_t protocol_version = 0;
  int err_count = 0;
  while (_muVsMethod->Get(kRegProtocolVersion, &protocol_version)
      || protocol_version != MU_PROTOCOL_VERSION) {
    ++err_count;
    if (err_count > 3) {
      delete _muVsMethod;
      _muVsMethod = nullptr;
      return MU_ERROR_UNSUPPROT_PROTOCOL;
    }
  }

  return MU_OK;
}
uint8_t DFRobot_MuVisionSensor::begin(MuVsUart* communicationPort) {
  if (_muVsMethod) {
    delete _muVsMethod;
    _muVsMethod = nullptr;
  }
  _mode = kSerialMode;
  _muVsMethod = new MuVisionSensorUart((MuVsUart *)communicationPort,
                                        _address);
  // check vs2 protocol version
  uint8_t protocol_version = 0;
  int err_count = 0;
  while (_muVsMethod->Get(kRegProtocolVersion, &protocol_version)
      || protocol_version != MU_PROTOCOL_VERSION) {
    ++err_count;
    if (err_count > 3) {
      delete _muVsMethod;
      _muVsMethod = nullptr;
      return MU_ERROR_UNSUPPROT_PROTOCOL;
    }
  }

  return MU_OK;
}
uint8_t DFRobot_MuVisionSensor::begin(MuVsI2C* communicationPort) {
  if (_muVsMethod) {
    delete _muVsMethod;
    _muVsMethod = nullptr;
  }
  _mode = kI2CMode;
  _muVsMethod = new MuVisionSensorI2C((MuVsI2C *)communicationPort,
                                        _address);
  // check vs2 protocol version
  uint8_t protocol_version = 0;
  int err_count = 0;
  while (_muVsMethod->Get(kRegProtocolVersion, &protocol_version)
      || protocol_version != MU_PROTOCOL_VERSION) {
    ++err_count;
    if (err_count > 3) {
      delete _muVsMethod;
      _muVsMethod = nullptr;
      return MU_ERROR_UNSUPPROT_PROTOCOL;
    }
  }

  return MU_OK;
}

//Advance interface
uint8_t DFRobot_MuVisionSensor::visionBegin(muVisionType visionType) {
  mu_err_t err;
  err = visionSetStatus(visionType, true);
  if (err) return err;
  delay(20);          // FIXME waiting for vision to initialize, may delete in later version
  err = visionSetOutputMode(kCallBackMode);
  if (err) return err;
  return MU_OK;
}

uint8_t DFRobot_MuVisionSensor::visionEnd(muVisionType visionType) {
  return visionSetStatus(visionType, false);
}

int DFRobot_MuVisionSensor::getValue(muVisionType visionType,
                             MuVsObjectInf objectInf) {
  if (objectInf == kStatus) {
    while((updateResult(visionType, true)&visionType) == 0);
  }
  return (int)read(visionType, objectInf);
}

muVsVisionState* DFRobot_MuVisionSensor::getVisionState(muVisionType visionType) {
  for (unsigned int i = 0; i < kVisionMaxType-1; ++i) {
    if (visionType & (0x01<<i)) {
      return _visionState[i];
    }
  }
  return nullptr;
}

uint8_t DFRobot_MuVisionSensor::visionSetStatus(muVisionType visionType, bool enable) {
  mu_err_t err;
  MuVsVisionConfig vision_config1;
  for (int i = 1; i < kVisionMaxType; i++) {
    if (visionType & visionTypeEnumToMacro(i)) {
      err = _muVsMethod->Set(kRegVisionId, i);
      if(err) return err;
      err = _muVsMethod->Get(kRegVisionConfig1, &vision_config1.vision_config_reg_value);
      if(err) return err;
      if (vision_config1.status != enable) {
        vision_config1.status = enable;
        err = _muVsMethod->Set(kRegVisionConfig1,
                                vision_config1.vision_config_reg_value);
        if (err) return err;
      }
      if (enable) {
        mallocVisionBuffer(muVsMessageVisionType(i));
      } else {
        freeVisionBuffer(muVsMessageVisionType(i));
      }
      _outputMode = vision_config1.output_mode;
    }
  }
  return MU_OK;
}

uint8_t DFRobot_MuVisionSensor::visionSetOutputMode(muVsStreamOutputMode mode) {
  mu_err_t err;
  MuVsVisionConfig vision_config1;
  _outputMode = mode;
  for (int i = 1; i < kVisionMaxType; ++i) {
    if (_visionState[i-1] != nullptr) {
      err = _muVsMethod->Set(kRegVisionId, i);
      if(err) return err;
      err = _muVsMethod->Get(kRegVisionConfig1, &vision_config1.vision_config_reg_value);
      if (err) return err;
      if (vision_config1.output_mode != mode) {
        vision_config1.output_mode = mode;
        err = _muVsMethod->Set(kRegVisionConfig1,
                                vision_config1.vision_config_reg_value);
        if (err) return err;
      }
    }
  }
  return MU_OK;
}

uint8_t DFRobot_MuVisionSensor::visionSetOutputEnable(muVisionType visionType, bool status) {
  mu_err_t err;
  MuVsVisionConfig vision_config1;
  for (int i = 1; i < kVisionMaxType; ++i) {
    if (visionType & visionTypeEnumToMacro(i)) {
      err = _muVsMethod->Set(kRegVisionId, i);
      if(err) return err;
      err = _muVsMethod->Get(kRegVisionConfig1, &vision_config1.vision_config_reg_value);
      if (err) return err;
      if (vision_config1.output_enable != status) {
        vision_config1.output_enable = status;
        err = _muVsMethod->Set(kRegVisionConfig1,
                                vision_config1.vision_config_reg_value);
        if (err) return err;
      }
    }
  }
  return MU_OK;
}

uint8_t DFRobot_MuVisionSensor::visionSetDefault(muVisionType visionType) {
  mu_err_t err;
  MuVsVisionConfig vision_config1;
  for (int i = 1; i < kVisionMaxType; ++i) {
    if (visionType & visionTypeEnumToMacro(i)) {
      err = _muVsMethod->Set(kRegVisionId, i);
      if(err) return err;
      err = _muVsMethod->Get(kRegVisionConfig1, &vision_config1.vision_config_reg_value);
      if (err) return err;
      vision_config1.default_setting = 1;
      err = _muVsMethod->Set(kRegVisionConfig1,
                              vision_config1.vision_config_reg_value);
      if (err) return err;
      while (vision_config1.default_setting) {
        err = _muVsMethod->Get(kRegVisionConfig1, &vision_config1.vision_config_reg_value);
        if (err) return err;
      }
    }
  }
  return MU_OK;
}

uint8_t DFRobot_MuVisionSensor::visionSetLevel(muVisionType visionType,
                                       muVsVisionLevel level) {
  mu_err_t err;
  MuVsVisionConfig vision_config1;
  for (int i = 1; i < kVisionMaxType; ++i) {
    if (visionType & visionTypeEnumToMacro(i)) {
      err = _muVsMethod->Set(kRegVisionId, i);
      if(err) return err;
      err = _muVsMethod->Get(kRegVisionConfig1, &vision_config1.vision_config_reg_value);
      if (err) return err;
      if (vision_config1.level != level) {
        vision_config1.level = level;
        err = _muVsMethod->Set(kRegVisionConfig1,
                                vision_config1.vision_config_reg_value);
        if (err) return err;
      }
    }
  }
  return MU_OK;
}

bool DFRobot_MuVisionSensor::visionGetStatus(muVisionType visionType) {
  uint8_t vision_status1 = 0;
  _muVsMethod->Get(kRegVisionConfig1, &vision_status1);
  return visionType&vision_status1;
}

muVsVisionLevel DFRobot_MuVisionSensor::visionGetLevel(muVisionType visionType) {
  MuVsVisionConfig vision_config1;
  for (int i = 1; i < kVisionMaxType; ++i) {
    if (visionType & visionTypeEnumToMacro(i)) {
      _muVsMethod->Set(kRegVisionId, i);
      _muVsMethod->Get(kRegVisionConfig1, &vision_config1.vision_config_reg_value);
      return vision_config1.level;
    }
  }
  return kLevelDefault;
}

muVsStreamOutputMode DFRobot_MuVisionSensor::visionGetOutputMode(void) {
  return _outputMode;
}

muVisionType DFRobot_MuVisionSensor::updateResult(muVisionType visionType,
                                          bool waitAllResult) {
  switch (_mode) {
    case kSerialMode:
      return uartUpdateResult(visionType, waitAllResult);
    case kI2CMode: {
      mu_err_t err;
      muVisionType vision_type_output = 0;
      muVsVisionState visionState;
      err = _muVsMethod->Get(kRegFrameCount, &visionState.frame);
      if (err) return vision_type_output;
      for (uint8_t i = 1; i < kVisionMaxType; ++i) {
        if ((visionType & visionTypeEnumToMacro(i)) && _visionState[i-1]) {
          if (visionState.frame != _visionState[i-1]->frame) {
            sensorLockReg(true);
            err = ((MuVisionSensorI2C *) _muVsMethod)->Read(
                (muVsMessageVisionType) i, &visionState);
            if (err) return vision_type_output;
            sensorLockReg(false);
            *_visionState[i-1] = visionState;
            vision_type_output = vision_type_output | visionTypeEnumToMacro(i);
          }
        }
      }
      return vision_type_output;
    }
    default:
      return uartUpdateResult(visionType, waitAllResult);
      break;
  }
}

muVisionType DFRobot_MuVisionSensor::uartUpdateResult(muVisionType visionType,
                                              bool waitAllResult) {
  muVisionType vision_detect = 0;
  muVsVisionState visionState;
  uint8_t mu_address;
  muVsMessageVisionType mu_vision_type;
  mu_err_t err;
  switch(_outputMode) {
    case kCallBackMode: {
      for (int i = 1; i < kVisionMaxType; ++i) {
        if ((visionType & visionTypeEnumToMacro(i)) && _visionState[i-1]) {
          ((MuVsUartMethod *)_muVsMethod)->GetMessage((muVsMessageVisionType)i);
          do {
            err = ((MuVsUartMethod *)_muVsMethod)->Read(&mu_address, &mu_vision_type, &visionState);
            if (err) return vision_detect;
            if (mu_address == _address
                && (visionType & visionTypeEnumToMacro(mu_vision_type))
                && _visionState[mu_vision_type-1]->frame != visionState.frame
                && mu_vision_type
                && mu_vision_type < kVisionMaxType) {
              *_visionState[mu_vision_type-1] = visionState;
              visionType = visionType&(~visionTypeEnumToMacro(mu_vision_type));
              vision_detect = vision_detect | visionTypeEnumToMacro(mu_vision_type);
              if (mu_vision_type == i && !waitAllResult) return vision_detect;
            }
          } while (mu_address != _address || mu_vision_type != i);
        }
      }
      break;
    }
    case kDataFlowMode:
    case kEventMode:
      while (visionType) {
        err = ((MuVsUartMethod *)_muVsMethod)->Read(&mu_address, &mu_vision_type, &visionState);
        if (err) return vision_detect;
        if (mu_address == _address
            && (visionType & visionTypeEnumToMacro(mu_vision_type))
            && mu_vision_type
            && mu_vision_type < kVisionMaxType
            && _visionState[mu_vision_type-1]) {
          _visionState[mu_vision_type-1]->detect = visionState.detect;
          _visionState[mu_vision_type-1]->frame = visionState.frame;
          for (int i = 0; i < visionState.detect; i++) {
            _visionState[mu_vision_type-1]->visionResult[i] = visionState.visionResult[i];
          }
          visionType = visionType&(~visionTypeEnumToMacro(mu_vision_type));
          vision_detect = vision_detect | visionTypeEnumToMacro(mu_vision_type);
          if (!waitAllResult) return vision_detect;
        }
      }
      break;
    default:
      return vision_detect;
  }
  return vision_detect;
}

uint8_t DFRobot_MuVisionSensor::write(muVisionType visionType,
                              MuVsObjectInf objectInf,
                              uint8_t value) {
  mu_err_t err;
  MuVsRegAddress address;
  uint8_t vs_type = 1;
  while ((visionType&0x01) == 0
      && vs_type < kVisionMaxType) {
    ++vs_type;
    visionType >>= 1;
  }
  switch(objectInf) {
    case kRValue:
    case kXValue:
      address = kRegParamValue1;
      break;
    case kGValue:
    case kYValue:
      address = kRegParamValue2;
      break;
    case kBValue:
    case kWidthValue:
      address = kRegParamValue3;
      break;
    case kHeightValue:
      address = kRegParamValue4;
      break;
    case kLabel:
      address = kRegParamValue5;
      break;
    default:
      return MU_ERROR_FAIL;
  }
  err = _muVsMethod->Set(kRegVisionId, vs_type);
  if (err) return err;
  return _muVsMethod->Set(address, value);
}

uint8_t DFRobot_MuVisionSensor::read(muVisionType visionType,
                             MuVsObjectInf objectInf,
                             uint8_t resultNum) {
  resultNum = resultNum ? (resultNum-1):1;
  resultNum = resultNum>MU_MAX_RESULT ? MU_MAX_RESULT:resultNum;
  uint8_t vision_pointer = 0;
  while ((visionType&0x01) == 0) {
    visionType = visionType>>1;
    vision_pointer++;
  }
  if (!_visionState[vision_pointer] || vision_pointer >= kVisionMaxType) return 0;
  switch(objectInf) {
    case kStatus:
      return _visionState[vision_pointer]->detect;
    case kXValue:
      return _visionState[vision_pointer]->visionResult[resultNum].xValue;
    case kYValue:
      return _visionState[vision_pointer]->visionResult[resultNum].yValue;
    case kWidthValue:
      return _visionState[vision_pointer]->visionResult[resultNum].width;
    case kHeightValue:
      return _visionState[vision_pointer]->visionResult[resultNum].height;
    case kLabel:
      return _visionState[vision_pointer]->visionResult[resultNum].lable;
    case kGValue:
      return _visionState[vision_pointer]->visionResult[resultNum].colorGValue;
    case kRValue:
      return _visionState[vision_pointer]->visionResult[resultNum].colorRValue;
    case kBValue:
      return _visionState[vision_pointer]->visionResult[resultNum].colorBValue;
    default:
      return 0;
  }
}

uint8_t DFRobot_MuVisionSensor::sensorSetRestart(void) {
  mu_err_t err;
  err = _muVsMethod->Set(kRegRestart, 1);
  return err;
}

uint8_t DFRobot_MuVisionSensor::sensorSetDefault(void) {
  MuVsSensorConfig1 sensor_config1;
  mu_err_t err;
  sensor_config1.default_setting = 1;
  err = _muVsMethod->Set(kRegSensorConfig1, sensor_config1.sensor_config_reg_value);
  return err;
}

uint8_t DFRobot_MuVisionSensor::sensorLockReg(bool lock) {
  mu_err_t err;
  err = _muVsMethod->Set(kRegLock, lock);
  return err;
}

//LED functions
uint8_t DFRobot_MuVisionSensor::ledSetMode(MuVsLed led, bool manual, bool hold) {
  MuVsLedConfig led_config;
  MuVsRegAddress address;
  mu_err_t err;
  switch(led) {
    case kLed1:
      address = kRegLed1;
      break;
    case kLed2:
      address = kRegLed2;
      break;
    default:
      return MU_ERROR_FAIL;
  }
  err = _muVsMethod->Get(address, &led_config.led_reg_value);
  if (err) return err;
  if (led_config.manual != manual
      || led_config.hold != hold) {
    led_config.manual = manual;
    led_config.hold = hold;
    err = _muVsMethod->Set(address, led_config.led_reg_value);
    if (err) return err;
  }

  return err;
}

uint8_t DFRobot_MuVisionSensor::ledSetColor(MuVsLed led,
                    MuVsLedColor detectedColor,
                    MuVsLedColor undetectedColor,
                    uint8_t level) {
  MuVsLedConfig led_config;
  MuVsRegAddress address;
  mu_err_t err;
  uint8_t led_level;
  _muVsMethod->Get(kRegLedLevel, &led_level);

  switch(led) {
    case kLed1:
      address = kRegLed1;
      led_level = (led_level&0xF0) | (level&0x0F);
      _muVsMethod->Set(kRegLedLevel, led_level);
      break;
    case kLed2:
      address = kRegLed2;
      led_level = (led_level&0x0F) | ((level&0x0F)<<4);
      _muVsMethod->Set(kRegLedLevel, led_level);
      break;
    default:
      return MU_ERROR_FAIL;
  }
  err = _muVsMethod->Get(address, &led_config.led_reg_value);
  if (err) return err;
  if (led_config.detectedColor != detectedColor
      || led_config.undetectedColor != undetectedColor) {
    led_config.detectedColor = detectedColor;
    led_config.undetectedColor = undetectedColor;
    err = _muVsMethod->Set(address, led_config.led_reg_value);
    if (err) return err;
  }

  return err;
}

//Camera functions
uint8_t DFRobot_MuVisionSensor::cameraSetZoom(muVsCameraZoom zoom) {
  MuVsCameraConfig cameraConfig;
  mu_err_t err;
  err = _muVsMethod->Get(kRegCameraConfig, &cameraConfig.cameraRegValue);
  if (cameraConfig.zoom != zoom) {
    cameraConfig.zoom = zoom;
    err = _muVsMethod->Set(kRegCameraConfig, cameraConfig.cameraRegValue);
  }
  return err;
}

uint8_t DFRobot_MuVisionSensor::cameraSetRotate(bool enable) {
  MuVsCameraConfig cameraConfig;
  mu_err_t err;
  err = _muVsMethod->Get(kRegCameraConfig, &cameraConfig.cameraRegValue);
  if (cameraConfig.rotate != enable) {
    cameraConfig.rotate = enable;
    err = _muVsMethod->Set(kRegCameraConfig, cameraConfig.cameraRegValue);
  }
  return err;
}

uint8_t DFRobot_MuVisionSensor::cameraSetFPS(muVsCameraFPS fps) {
  MuVsCameraConfig cameraConfig;
  mu_err_t err;
  err = _muVsMethod->Get(kRegCameraConfig, &cameraConfig.cameraRegValue);
  if (cameraConfig.fps != fps) {
    cameraConfig.fps = fps;
    err = _muVsMethod->Set(kRegCameraConfig, cameraConfig.cameraRegValue);
  }
  return err;
}

uint8_t DFRobot_MuVisionSensor::cameraSetAwb(muVsCameraWhiteBalance awb) {
  MuVsCameraConfig cameraConfig;
  mu_err_t err;
  err = _muVsMethod->Get(kRegCameraConfig, &cameraConfig.cameraRegValue);
  if (cameraConfig.white_balance != awb) {
    cameraConfig.white_balance = awb;
    err = _muVsMethod->Set(kRegCameraConfig, cameraConfig.cameraRegValue);
    // waiting for lock white balance
    if (awb == kLockWhiteBalance) {
      delay(1000);
    }
  }
  return err;
}

muVsCameraZoom DFRobot_MuVisionSensor::cameraGetZoom(void) {
  MuVsCameraConfig cameraConfig;
  _muVsMethod->Get(kRegCameraConfig, &cameraConfig.cameraRegValue);
  return cameraConfig.zoom;
}

muVsCameraWhiteBalance DFRobot_MuVisionSensor::cameraGetAwb(void) {
  MuVsCameraConfig cameraConfig;
  _muVsMethod->Get(kRegCameraConfig, &cameraConfig.cameraRegValue);
  return cameraConfig.white_balance;
}

bool DFRobot_MuVisionSensor::cameraGetRotate(void) {
  MuVsCameraConfig cameraConfig;
  _muVsMethod->Get(kRegCameraConfig, &cameraConfig.cameraRegValue);
  return cameraConfig.rotate;
}

muVsCameraFPS DFRobot_MuVisionSensor::cameraGetFPS(void) {
  MuVsCameraConfig cameraConfig;
  _muVsMethod->Get(kRegCameraConfig, &cameraConfig.cameraRegValue);
  return cameraConfig.fps;
}

//Uart functions
uint8_t DFRobot_MuVisionSensor::uartSetBaudrate(muVsBaudrate baud) {
  mu_err_t err;
  MuVsUartConfig uartConfig;
  err = _muVsMethod->Get(kRegUart, &uartConfig.uart_reg_value);
  if (uartConfig.baudrate != baud) {
    uartConfig.baudrate = baud;
    _muVsMethod->Set(kRegUart, uartConfig.uart_reg_value);
  }
  return err;
}

bool DFRobot_MuVisionSensor::mallocVisionBuffer(muVsMessageVisionType visionType) {
  if (visionType
      && visionType < kVisionMaxType
      && _visionState[visionType-1] == nullptr) {
    _visionState[visionType-1] = new muVsVisionState;
    _visionState[visionType-1]->detect = 0;
    _visionState[visionType-1]->frame = 0;
  }
  return true;
}

bool DFRobot_MuVisionSensor::freeVisionBuffer(muVsMessageVisionType visionType) {
  if (visionType
      && visionType < kVisionMaxType
      && _visionState[visionType-1]) {
    _visionState[visionType-1] = new muVsVisionState;
  }
  return true;
}



