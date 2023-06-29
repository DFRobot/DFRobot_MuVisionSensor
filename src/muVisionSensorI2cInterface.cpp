/*!
 * @file muVisionSensorI2cInterface.cpp
 * @brief Basic struct of i2c interface class.
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT license (MIT)
 * @author DFRobot
 * @version  V1.0
 * @date  2023-06-28
 * @https://github.com/DFRobot/DFRobot_MuVisionSensor
 */
#include "muVisionSensorI2cInterface.h"

MuVsI2CMethod::MuVsI2CMethod(uint32_t address)
    : muVsMethod() {
  mu_address_ = address;
}

MuVsI2CMethod::~MuVsI2CMethod() {}

mu_err_t MuVsI2CMethod::Get(const uint8_t regAddress,
                             uint8_t* value) {
	return I2CRead(regAddress, value);
}

mu_err_t MuVsI2CMethod::Set(const uint8_t regAddress,
                            const uint8_t value) {
  return I2CWrite(regAddress, value);
}

mu_err_t MuVsI2CMethod::Read(muVsMessageVisionType visionType,
                             muVsVisionState* visionState) {
  mu_err_t err;
  err = I2CWrite(kRegVisionId, visionType);
  if (err) return err;
  err = I2CRead(kRegResultNumber, &visionState->detect);
  if (err) return err;
  if (!visionState->detect) return MU_OK;
  err = I2CRead(kRegFrameCount, &visionState->frame);
  if (err) return err;
  visionState->detect = MU_MAX_RESULT<visionState->detect ?
      MU_MAX_RESULT:visionState->detect;
  for (uint32_t i = 0; i < visionState->detect; ++i) {
    err = I2CWrite(kRegResultId, i+1);
    if (err) return err;
    I2CRead(kRegResultData1, &visionState->visionResult[i].result_data1);
    I2CRead(kRegResultData2, &visionState->visionResult[i].result_data2);
    I2CRead(kRegResultData3, &visionState->visionResult[i].result_data3);
    I2CRead(kRegResultData4, &visionState->visionResult[i].result_data4);
    I2CRead(kRegResultData5, &visionState->visionResult[i].result_data5);
  }
  return MU_OK;
}





