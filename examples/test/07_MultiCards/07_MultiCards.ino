/*!
 * @file 07_MultiCards.ino
 * @brief Examples of multi cards.
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
 * Set Vision Type
 */
#define VISION_TYPE     VISION_SHAPE_CARD_DETECT | VISION_TRAFFIC_CARD_DETECT | VISION_NUM_CARD_DETECT

/*
 * Set Vision Level
 */
#define VISION_LEVEL    kLevelAccuracy // accuracy first

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
SoftwareSerial SoftSerial(SOFT_SERIAL_RX_PIN, SOFT_SERIAL_TX_PIN);  // RX, TX  // The hardware serial port is used for log output, so we use soft serial port
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
  
  while (1) {
    visionType = MU.updateResult(VISION_TYPE, false); // Update detection results
  
    if (visionType & VISION_TYPE) {
     visionState = MU.getVisionState(VISION_TYPE);
      Serial.print("=====frame:");
      Serial.print(visionState->frame);
      Serial.println("=====");
      
      Serial.print("shape card : ");
      if (MU.getValue(VISION_SHAPE_CARD_DETECT, kStatus)) {
        PrintShapeCardLabel(MU.getValue(VISION_SHAPE_CARD_DETECT, kLabel));
      } else {
        Serial.println("undetected");
      }
  
      Serial.print("traffic card : ");
      if (MU.getValue(VISION_TRAFFIC_CARD_DETECT, kStatus)) {
        PrintTrafficCardLabel(MU.getValue(VISION_TRAFFIC_CARD_DETECT, kLabel));
      } else {
        Serial.println("undetected");
      }
  
      Serial.print("number card : ");
      if (MU.getValue(VISION_NUM_CARD_DETECT, kStatus)) {
        Serial.println(MU.getValue(VISION_NUM_CARD_DETECT, kLabel));
      } else {
        Serial.println("undetected");
      }
    }
  }
}

void PrintShapeCardLabel(uint8_t label) {
  switch (label) {
    case 1:
      Serial.println("check");
      break;
    case 2:
      Serial.println("cross");
      break;
    case 3:
      Serial.println("circle");
      break;
    case 4:
      Serial.println("square");
      break;
    case 5:
      Serial.println("triangle");
      break;
    default:
      Serial.println("unknown");
      break;
  }
}

void PrintTrafficCardLabel(uint8_t label) {
  switch (label) {
    case 1:
      Serial.println("forward");
      break;
    case 2:
      Serial.println("left");
      break;
    case 3:
      Serial.println("right");
      break;
    case 4:
      Serial.println("turn around");
      break;
    case 5:
      Serial.println("park");
      break;
    default:
      Serial.println("unknown");
      break;
  }
}
