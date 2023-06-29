/*!
 * @file muVisionSensorI2cHwInterface.cpp
 * @brief Basic struct of i2c HW interface class.
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT license (MIT)
 * @author DFRobot
 * @version  V1.0
 * @date  2023-06-28
 * @https://github.com/DFRobot/DFRobot_MuVisionSensor
 */
#include "muVisionSensorI2cHwInterface.h"
#include <Arduino.h>

MuVisionSensorI2C::MuVisionSensorI2C(MuVsI2C* i2cPort, uint32_t address)
    : MuVsI2CMethod(address),
      i2c_port_(i2cPort) {
}

MuVisionSensorI2C::~MuVisionSensorI2C() {}

uint32_t MuVisionSensorI2C::I2CRead(uint8_t regAddress, uint8_t* temp) {
  uint8_t ret = MU_OK;
  i2c_port_->beginTransmission((uint8_t)mu_address_);
  ret = i2c_port_->write(regAddress);
  if (!ret) return SERVER_RESPONSE_TIMEOUT;
  i2c_port_->endTransmission();
  //Debug Output
#if STREAM_DEBUG_ENABLE
  Serial.print("[R:");
  Serial.print(regAddress, HEX);
  Serial.print(',');
#endif
  if (i2c_port_->requestFrom(mu_address_, 1) != 1)
    return SERVER_RESPONSE_TIMEOUT;

  *temp = i2c_port_->read();
  //Debug Output
#if STREAM_DEBUG_ENABLE
  Serial.print(*temp, HEX);
  Serial.print("],");
#endif

  return MU_OK;
}

uint32_t MuVisionSensorI2C::I2CWrite(uint8_t regAddress, uint8_t value) {
  uint8_t ret = MU_OK;
  i2c_port_->beginTransmission((uint8_t)mu_address_);
  ret = i2c_port_->write(regAddress);
  if (!ret) return SERVER_RESPONSE_TIMEOUT;
  ret = i2c_port_->write(value);
  if (!ret) return SERVER_RESPONSE_TIMEOUT;
  i2c_port_->endTransmission();
  //Debug Output
#if STREAM_DEBUG_ENABLE
  Serial.print("[W:");
  Serial.print(regAddress, HEX);
  Serial.print(',');
  Serial.print(value, HEX);
  Serial.print("],");
#endif
  return MU_OK;
}
