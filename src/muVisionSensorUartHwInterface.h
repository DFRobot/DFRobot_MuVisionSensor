/*!
 * @file muVisionSensorUartHwInterface.h
 * @brief Basic struct of uart HW interface class.
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT license (MIT)
 * @author DFRobot
 * @version  V1.0
 * @date  2023-06-28
 * @https://github.com/DFRobot/DFRobot_MuVisionSensor
 */
#ifndef ARDUINO_LIB_MUVISIONSENSOR_SRC_MU_VISION_SENSOR_UART_HW_INTERFACE_H_
#define ARDUINO_LIB_MUVISIONSENSOR_SRC_MU_VISION_SENSOR_UART_HW_INTERFACE_H_

#include "muVisionSensorUartInterface.h"
#include <Stream.h>

typedef Stream MuVsUart;

class MuVisionSensorUart: public MuVsUartMethod {
 public:
  MuVisionSensorUart(MuVsUart* uart,
                     uint32_t address);
  virtual ~MuVisionSensorUart();
  MuVisionSensorUart(const MuVisionSensorUart&) = delete;
  MuVisionSensorUart& operator=(const MuVisionSensorUart &) = delete;

  virtual uint32_t UartRead(uint8_t* temp, uint8_t length) override;
  virtual uint32_t UartWrite(uint8_t* temp, uint8_t length) override;

 private:
 protected:
  MuVsUart* uart_ = nullptr;
};

#endif /* ARDUINO_LIB_MUVISIONSENSOR_SRC_MU_VISION_SENSOR_UART_HW_INTERFACE_H_ */
