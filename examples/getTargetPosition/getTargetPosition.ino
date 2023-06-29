/*!
 * @file getTargetPosition.ino
 * @brief Examples of get target position.
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT license (MIT)
 * @author DFRobot
 * @version  V1.0
 * @date  2023-06-28
 * @https://github.com/DFRobot/DFRobot_MuVisionSensor
 */
#include <DFRobot_MuVisionSensor.h>
#include <Wire.h>
#include <SoftwareSerial.h>

/*
 * Choose communication mode define here:
 *    I2C_MODE    : I2C mode, default pin: MU_SDA <==> ARDUINO_SDA, MU_SCL <==> ARDUINO_SCL
 *    SERIAL_MODE : Serial mode, default pin: MU_TX <==> ARDUINO_PIN3, MU_RX <==> ARDUINO_PIN2
 */
#define I2C_MODE
//#define SERIAL_MODE

/*
 * Choose MU address here: 0x60, 0x61, 0x62, 0x63
 */
#define MU_ADDRESS    0x60

/*
 * Change vision type here, VISION_TYPE:VISION_COLOR_DETECT
 *                                      VISION_COLOR_RECOGNITION
 *                                      VISION_BALL_DETECT
 *                                      VISION_BODY_DETECT
 *                                      VISION_SHAPE_CARD_DETECT
 *                                      VISION_TRAFFIC_CARD_DETECT
 *                                      VISION_NUM_CARD_DETECT
 */
#define VISION_TYPE     VISION_BALL_DETECT

#ifdef SERIAL_MODE
#define TX_PIN 2
#define RX_PIN 3
SoftwareSerial mySerial(RX_PIN, TX_PIN);
#endif
DFRobot_MuVisionSensor Mu(MU_ADDRESS);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(500);

#ifdef I2C_MODE
  Wire.begin();
  Mu.begin(&Wire);              // initialized MU on I2C port
#elif defined SERIAL_MODE
  mySerial.begin(9600);
  Mu.begin(&mySerial);          // initialized MU on soft serial port
#endif

  Mu.visionBegin(VISION_TYPE);  // enable vision

  if (VISION_TYPE == VISION_COLOR_DETECT
      || VISION_TYPE == VISION_COLOR_RECOGNITION) {
    Mu.cameraSetAwb(kLockWhiteBalance); // lock AWB
    if (VISION_TYPE == VISION_COLOR_RECOGNITION) {
      Mu.write(VISION_TYPE, kXValue, 50);
      Mu.write(VISION_TYPE, kYValue, 50);
      Mu.write(VISION_TYPE, kWidthValue, 5);
      Mu.write(VISION_TYPE, kHeightValue, 5);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  long timeStart = millis();

  // read result
  if (Mu.getValue(VISION_TYPE, kStatus)) {
    Serial.println("vision detected:");
    switch (VISION_TYPE) {
      case VISION_BALL_DETECT:
      case VISION_BODY_DETECT:
      case VISION_SHAPE_CARD_DETECT:
      case VISION_TRAFFIC_CARD_DETECT:
      case VISION_NUM_CARD_DETECT:
      case VISION_COLOR_DETECT:
        Serial.print("x = ");
        Serial.println(Mu.getValue(VISION_TYPE, kXValue));
        Serial.print("y = ");
        Serial.println(Mu.getValue(VISION_TYPE, kYValue));
        Serial.print("width = ");
        Serial.println(Mu.getValue(VISION_TYPE, kWidthValue));
        Serial.print("height = ");
        Serial.println(Mu.getValue(VISION_TYPE, kHeightValue));
        if (VISION_TYPE != VISION_COLOR_DETECT) {
          Serial.print("label = ");
          Serial.println(Mu.getValue(VISION_TYPE, kLabel));
        } else {
          Serial.print("color = ");
          Serial.println(Mu.getValue(VISION_TYPE, kLabel));
        }
        break;
      case VISION_COLOR_RECOGNITION:
        Serial.print("r = ");
        Serial.println(Mu.getValue(VISION_TYPE, kRValue));
        Serial.print("g = ");
        Serial.println(Mu.getValue(VISION_TYPE, kGValue));
        Serial.print("b = ");
        Serial.println(Mu.getValue(VISION_TYPE, kBValue));
        Serial.print("color = ");
        Serial.println(Mu.getValue(VISION_TYPE, kLabel));
        break;
      default:
        break;
    }
  } else {
    Serial.println("vision undetected.");
  }
  Serial.print("fps = ");
  Serial.println(1000/(millis()-timeStart));
  Serial.println();
}




