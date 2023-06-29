/*!
 * @file 01_VisionBasic.ino
 * @brief Examples of vision settings.
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT license (MIT)
 * @author DFRobot
 * @version  V1.0
 * @date  2023-06-28
 * @https://github.com/DFRobot/DFRobot_MuVisionSensor
 */
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "DFRobot_MuVisionSensor.h"
#include <Wire.h>

/*
 * Vision Settings
 */
 
/*
 * Select Vision Type
 */
//#define VISION_TYPE     VISION_COLOR_DETECT // 01
//#define VISION_TYPE     VISION_COLOR_RECOGNITION // 02
//#define VISION_TYPE     VISION_BALL_DETECT // 03
//#define VISION_TYPE     VISION_BODY_DETECT // 05
#define VISION_TYPE     VISION_SHAPE_CARD_DETECT // 06
//#define VISION_TYPE     VISION_TRAFFIC_CARD_DETECT // 07
//#define VISION_TYPE     VISION_NUM_CARD_DETECT // 08

/*
 * Set Vision Level
 */
//#define VISION_LEVEL    kLevelSpeed // speed first
#define VISION_LEVEL    kLevelBalance // balance
//#define VISION_LEVEL    kLevelAccuracy // accuracy first

/*
 * Output Settings
 */
//#define OUTPUT_MODE_UART  1 // Uart/Serial - Select output switch to 00
#define OUTPUT_MODE_I2C   1 // I2C - Select output switch to 01

/*
 * Set soft serial pins
 */
#define SOFT_SERIAL_RX_PIN  2
#define SOFT_SERIAL_TX_PIN  3

/*
 * Functions
 */
DFRobot_MuVisionSensor MU(0x60); // 0x60 is device address

#ifdef OUTPUT_MODE_UART
SoftwareSerial SoftSerial(SOFT_SERIAL_RX_PIN, SOFT_SERIAL_TX_PIN);  // RX, TX	// The hardware serial port is used for log output, so we use soft serial port
#endif

void setup() {
  Serial.begin(115200);
  Serial.println("==========Start==========");

  Serial.println("MU begin...");
#ifdef OUTPUT_MODE_UART
  SoftSerial.begin(9600);
  MU.begin(&SoftSerial);   // Uart/Serial
#elif OUTPUT_MODE_I2C
  Wire.begin();
  MU.begin(&Wire);             //I2C
#endif
  Serial.println("Finised");

  Serial.println("Vision begin...");
  MU.visionBegin(VISION_TYPE);
#ifdef VISION_LEVEL
  MU.visionSetLevel(VISION_TYPE, VISION_LEVEL);
#endif
  Serial.println("Finished");
  Serial.println("MU Vision Sensor Processing...");
}
  
void loop() {
  muVsVisionState *visionState;
  muVisionType visionType;

  unsigned long frameBeginTime = 0;
  unsigned long frameProcessTime = 0;
  unsigned long totalProcessTime = 0;
  unsigned long detectedCount = 0;
  unsigned long totalProcessCount = 0;

  frameBeginTime = millis();
  
  while (1) {
    visionType = MU.updateResult(VISION_TYPE, false); // Update detection results
    if (visionType & VISION_TYPE) {
      frameProcessTime = millis() - frameBeginTime;
      frameBeginTime = millis();
      totalProcessTime += frameProcessTime;
      
      visionState = MU.getVisionState(VISION_TYPE);

      Serial.print("frame:");
      Serial.print(visionState->frame);
      totalProcessCount++;
      Serial.print("  || rate:");
      Serial.print((float)detectedCount / (float)totalProcessCount * 100);
      Serial.print("%");
      Serial.print("  time:");
      Serial.print(frameProcessTime);
      Serial.print("ms");
      Serial.print("  fps:");
      Serial.print(1000 / (totalProcessTime / totalProcessCount));      
      if (visionState->detect) {
        detectedCount++;
        Serial.print("  || detected");
        Serial.print("  label:");
        Serial.print(visionState->visionResult[0].lable);
        if (VISION_TYPE != VISION_COLOR_RECOGNITION) {
          Serial.print("  x:");
          Serial.print(visionState->visionResult[0].xValue);
          Serial.print("  y:");
          Serial.print(visionState->visionResult[0].yValue);
          Serial.print("  width:");
          Serial.print(visionState->visionResult[0].width);
          Serial.print("  height:");
          Serial.print(visionState->visionResult[0].height);
        } else {
          Serial.print("  r:");
          Serial.print(visionState->visionResult[0].colorRValue);
          Serial.print("  g:");
          Serial.print(visionState->visionResult[0].colorGValue);
          Serial.print("  b:");
          Serial.print(visionState->visionResult[0].colorBValue);
        }
      } else {
        Serial.print("  || undetected");
      }
      Serial.println("");
    }
  }
}
