/*!
 * @file muVisionSensorI2cInterface.h
 * @brief Basic struct of i2c interface class.
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT license (MIT)
 * @author DFRobot
 * @version  V1.0
 * @date  2023-06-28
 * @https://github.com/DFRobot/DFRobot_MuVisionSensor
 */
#ifndef ARDUINO_LIB_MUVISIONSENSOR_SRC_MU_VISION_SENSOR_I2C_INTERFACE_H_
#define ARDUINO_LIB_MUVISIONSENSOR_SRC_MU_VISION_SENSOR_I2C_INTERFACE_H_

#include "muVisionSensorInterface.h"

class MuVsI2CMethod : public muVsMethod {
 public:
  MuVsI2CMethod(uint32_t address);
  virtual ~MuVsI2CMethod(void);
  MuVsI2CMethod(const MuVsI2CMethod&) = delete;
  MuVsI2CMethod& operator=(const MuVsI2CMethod &) = delete;

  virtual uint32_t I2CRead(uint8_t regAddress, uint8_t* temp) = 0;
  virtual uint32_t I2CWrite(uint8_t regAddress, uint8_t value) = 0;

  virtual mu_err_t Get(const uint8_t regAddress,
                       uint8_t* value) override;
  virtual mu_err_t Set(const uint8_t regAddress,
                       const uint8_t value) override;
  mu_err_t Read(muVsMessageVisionType visionType,
                muVsVisionState* visionState);

 private:

 protected:
};

#endif /* ARDUINO_LIB_MUVISIONSENSOR_SRC_MU_VISION_SENSOR_UART_INTERFACE_H_ */
