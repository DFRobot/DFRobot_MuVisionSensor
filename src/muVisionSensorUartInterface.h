/*!
 * @file muVisionSensorUartInterface.cpp
 * @brief Basic struct of uart interface class.
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT license (MIT)
 * @author DFRobot
 * @version  V1.0
 * @date  2023-06-28
 * @https://github.com/DFRobot/DFRobot_MuVisionSensor
 */
#ifndef ARDUINO_LIB_MUVISIONSENSOR_SRC_MU_VISION_SENSOR_UART_INTERFACE_H_
#define ARDUINO_LIB_MUVISIONSENSOR_SRC_MU_VISION_SENSOR_UART_INTERFACE_H_

#include "muVisionSensorInterface.h"

class MuVsUartMethod : public muVsMethod {
 public:
  MuVsUartMethod(uint32_t address);
  virtual ~MuVsUartMethod(void);
  MuVsUartMethod(const MuVsUartMethod&) = delete;
  MuVsUartMethod& operator=(const MuVsUartMethod &) = delete;

  virtual uint32_t UartRead(uint8_t* temp, uint8_t length) = 0;
  virtual uint32_t UartWrite(uint8_t* temp, uint8_t length) = 0;

  virtual mu_err_t Get(const uint8_t regAddress,
                       uint8_t* value) override;
  virtual mu_err_t Set(const uint8_t regAddress,
                       const uint8_t value) override;
  mu_err_t Read(uint8_t* mu_address,
                muVsMessageVisionType* visionType,
                muVsVisionState* visionState);
  mu_err_t GetMessage(muVsMessageVisionType);

 private:
 protected:
  mu_err_t GetProtocolHead(uint8_t* buf);
  mu_err_t GetProtocolBody(uint8_t* buf);
  uint8_t SumCheck(uint8_t* buf, uint8_t len);
};

#endif /* ARDUINO_LIB_MUVISIONSENSOR_SRC_MU_VISION_SENSOR_UART_INTERFACE_H_ */
