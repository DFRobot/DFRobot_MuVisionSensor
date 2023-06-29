/*!
 * @file muVisionSensorInterface.h
 * @brief Basic struct of sensor interface class.
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT license (MIT)
 * @author DFRobot
 * @version  V1.0
 * @date  2023-06-28
 * @https://github.com/DFRobot/DFRobot_MuVisionSensor
 */
#ifndef ARDUINO_LIB_MUVISIONSENSOR_SRC_MU_VISION_SENSOR_INTERFACE_H_
#define ARDUINO_LIB_MUVISIONSENSOR_SRC_MU_VISION_SENSOR_INTERFACE_H_

#include <stdint.h>
#include "muVisionSensorType.h"

class muVsMethod {
 public:
  muVsMethod(void) {}
  virtual ~muVsMethod(void) {}
  muVsMethod(const muVsMethod&) = delete;
  muVsMethod& operator=(const muVsMethod &) = delete;

  virtual mu_err_t Get(const uint8_t regAddress,
                       uint8_t* value) = 0;
  virtual mu_err_t Set(const uint8_t regAddress,
                       const uint8_t value) = 0;
 private:
 protected:
  uint32_t mu_address_ = 0;
};

#endif /* ARDUINO_LIB_MUVISIONSENSOR_SRC_MU_VISION_SENSOR_INTERFACE_H_ */
