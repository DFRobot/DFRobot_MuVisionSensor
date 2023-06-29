/*!
 * @file muVisionSensorI2cHwInterface.h
 * @brief Basic struct of i2c HW interface class.
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT license (MIT)
 * @author DFRobot
 * @version  V1.0
 * @date  2023-06-28
 * @https://github.com/DFRobot/DFRobot_MuVisionSensor
 */
#ifndef ARDUINO_LIB_MUVISIONSENSOR_SRC_MU_VISION_SENSOR_I2C_HW_INTERFACE_H_
#define ARDUINO_LIB_MUVISIONSENSOR_SRC_MU_VISION_SENSOR_I2C_HW_INTERFACE_H_

#include "muVisionSensorI2cInterface.h"
#include <Wire.h>
//@type define I2C type
typedef TwoWire MuVsI2C;

// if u want to use software wire in arduino, change include and MuVsI2C type here
//#include <SoftwareWire.h>
//@type define I2C type
//typedef SoftwareWire MuVsI2C;


// @Must public inheritance class MuVsI2CMethod
class MuVisionSensorI2C: public MuVsI2CMethod {
 public:
  MuVisionSensorI2C(MuVsI2C* i2cPort, uint32_t address);
  virtual ~MuVisionSensorI2C();
  MuVisionSensorI2C(const MuVisionSensorI2C&) = delete;
  MuVisionSensorI2C& operator=(const MuVisionSensorI2C &) = delete;

/**
  * @brief  I2C read byte.
  * @param  regAddress: register address.
  * @param  temp: register value.
  * @retval 0: read success
  *         not 0: error
  */
  virtual uint32_t I2CRead(uint8_t regAddress, uint8_t* temp) override;
/**
  * @brief  I2C write byte.
  * @param  regAddress: register address.
  * @param  value: the value write to register.
  * @retval 0: write success
  *         not 0: error
  */
  virtual uint32_t I2CWrite(uint8_t regAddress, uint8_t value) override;

 private:
  MuVsI2C* i2c_port_ = nullptr;

 protected:
};

#endif /* ARDUINO_LIB_MUVISIONSENSOR_SRC_MU_VISION_SENSOR_UART_HW_INTERFACE_H_ */
