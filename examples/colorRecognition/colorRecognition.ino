/*!
 * @file colorRecognition.ino
 * @brief Examples of recognition color.
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT license (MIT)
 * @author DFRobot
 * @version  V1.0
 * @date  2023-06-28
 * @https://github.com/DFRobot/DFRobot_MuVisionSensor
 */
#include <Arduino.h>
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
 *        default address: 0x60
 */
#define MU_ADDRESS    0x60

#ifdef SERIAL_MODE
#define TX_PIN 2
#define RX_PIN 3
SoftwareSerial mySerial(RX_PIN, TX_PIN);
#endif
DFRobot_MuVisionSensor Mu(MU_ADDRESS);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  uint8_t err = MU_ERROR_FAIL;
#ifdef I2C_MODE
  Wire.begin();
  // initialized MU on the I2C port
  err = Mu.begin(&Wire);
#elif defined SERIAL_MODE
  mySerial.begin(9600);
  // initialized MU on the soft serial port
  err = Mu.begin(&mySerial);
#endif
  if (err == MU_OK) {
    Serial.println("MU initialized.");
  } else {
    do {
      Serial.println("fail to initialize MU! Please check protocol "
                     "version or make sure MU is working on the "
                     "correct port with correct mode.");
      delay(5000);
    } while (1);
  }
  // enable vision: number card
  Mu.visionBegin(VISION_COLOR_RECOGNITION);
  Mu.write(VISION_COLOR_RECOGNITION, kXValue, 50);                  // set detect region center x value(0~100)
  Mu.write(VISION_COLOR_RECOGNITION, kYValue, 50);                  // set detect region center y value(0~100)
  Mu.write(VISION_COLOR_RECOGNITION, kWidthValue, 5);               // set detect region center width value(0~100)
  Mu.write(VISION_COLOR_RECOGNITION, kHeightValue, 5);              // set detect region center height value(0~100)
  Mu.cameraSetAwb(kLockWhiteBalance);                               // lock AWB
}

void loop() {
  // put your main code here, to run repeatedly:
  long timeStart = millis();

  // read result
  if (Mu.getValue(VISION_COLOR_RECOGNITION, kStatus)) {                    // update vision result and get status, 0: undetected, other: detected
    Serial.println("vision color detected:");
    Serial.print("label = ");
    switch(Mu.getValue(VISION_COLOR_RECOGNITION, kLabel)) {                // get vision result: label value
      case MU_COLOR_BLACK:
        Serial.println("black");
        break;
      case MU_COLOR_BLUE:
        Serial.println("blue");
        break;
      case MU_COLOR_CYAN:
        Serial.println("cyan");
        break;
      case MU_COLOR_GREEN:
        Serial.println("green");
        break;
      case MU_COLOR_PURPLE:
        Serial.println("purple");
        break;
      case MU_COLOR_RED:
        Serial.println("red");
        break;
      case MU_COLOR_WHITE:
        Serial.println("white");
        break;
      case MU_COLOR_YELLOW:
        Serial.println("yellow");
        break;
      default:
        Serial.print("unknow color type: ");
        Serial.println(Mu.getValue(VISION_COLOR_RECOGNITION, kLabel));
        break;
    }
    Serial.print("r = ");
    Serial.println(Mu.getValue(VISION_COLOR_RECOGNITION, kRValue));        // get vision result: x axes value
    Serial.print("g = ");
    Serial.println(Mu.getValue(VISION_COLOR_RECOGNITION, kGValue));        // get vision result: y axes value
    Serial.print("b = ");
    Serial.println(Mu.getValue(VISION_COLOR_RECOGNITION, kBValue));    // get vision result: width value
  } else {
    Serial.println("vision color undetected.");
  }
  Serial.print("fps = ");
  Serial.println(1000/(millis()-timeStart));
  Serial.println();
}


