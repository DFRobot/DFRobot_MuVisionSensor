/*!
 * @file muVisionSensorUartHwInterface.cpp
 * @brief Basic struct of uart HW interface class.
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT license (MIT)
 * @author DFRobot
 * @version  V1.0
 * @date  2023-06-28
 * @https://github.com/DFRobot/DFRobot_MuVisionSensor
 */
#include "muVisionSensorUartHwInterface.h"
#include <Arduino.h>

MuVisionSensorUart::MuVisionSensorUart(MuVsUart* uart,
                                       uint32_t address)
    : MuVsUartMethod(address),
      uart_(uart) {
}

MuVisionSensorUart::~MuVisionSensorUart() {}

uint32_t MuVisionSensorUart::UartRead(uint8_t* temp, uint8_t length) {
#if STREAM_DEBUG_ENABLE
  uint8_t ret = uart_->readBytes(temp, length);
  for (int i = 0; i < ret; i++) {
    Serial.print("0x");
    Serial.print(temp[i], HEX);
    Serial.print(',');
  }
  return ret;
#endif
  return uart_->readBytes(temp, length);
}

uint32_t MuVisionSensorUart::UartWrite(uint8_t* temp, uint8_t length) {
#if STREAM_DEBUG_ENABLE
  uint8_t ret = uart_->write(temp, length);
  for (int i = 0; i < ret; i++) {
    Serial.print(temp[i], HEX);
    Serial.print(',');
  }
  return ret;
#endif
  return uart_->write(temp, length);
}

