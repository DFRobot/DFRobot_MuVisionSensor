/*!
 * @file colorDetect.ino
 * @brief Examples of color detect.
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT license (MIT)
 * @author DFRobot
 * @version  V1.0
 * @date  2023-06-28
 * @https://github.com/DFRobot/DFRobot_MuVisionSensor
 */
#include <Wire.h>

#define MU_ADDRESS    0x60
#define PROTOCOL_VER  0x03
#define VISION_ID     0x01

// register define
#define REG_PROTOCOL_VER  0x01
#define REG_LED1_CONF     0x06
#define REG_LED2_CONF     0x07
#define REG_LED_LEVEL     0x08
#define REG_CAMERA_CONF1  0x10
#define REG_FRAME_CNT     0x1F
#define REG_VISION_ID     0x20
#define REG_VISION_CONF1  0x21
#define REG_PARAM_VALUE5  0x29
#define RESULT_NUM        0x34
#define RESULT_DATA1      0x40
#define RESULT_DATA2      0x41
#define RESULT_DATA3      0x42
#define RESULT_DATA4      0x43
// color
#define MU_COLOR_BLACK                0x01U
#define MU_COLOR_WHITE                0x02U
#define MU_COLOR_RED                  0x03U
#define MU_COLOR_YELLOW               0x04U
#define MU_COLOR_GREEN                0x05U
#define MU_COLOR_CYAN                 0x06U
#define MU_COLOR_BLUE                 0x07U
#define MU_COLOR_PURPLE               0x08U

int i2cRead8(uint8_t reg) {
  Wire.beginTransmission(MU_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(MU_ADDRESS, 1);
  return Wire.read();
}
void i2cWrite8(const uint8_t reg, const uint8_t value) {
  Wire.beginTransmission(MU_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t reg[][2] = {
    { REG_LED_LEVEL,      0x00 }, // close led
//    { REG_LED1_CONF,      0x00 }, // LED1 color
//    { REG_LED2_CONF,      0x00 }, // LED2 color
    { REG_CAMERA_CONF1,   0x30 }, // lock AWB
    { REG_VISION_ID,      VISION_ID }, // set vision type = vision_detect
    { REG_PARAM_VALUE5,   MU_COLOR_RED }, // set vision parameter, color = red
    { REG_VISION_CONF1,   0x21 }, // vision begin
};
uint8_t frameCountLast = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  delay(500);

  if (i2cRead8(REG_PROTOCOL_VER) == PROTOCOL_VER) {
    Serial.println("device initialized.");
  } else {
    Serial.println("fail to initialize device! Please check protocol version.");
  }
  for (uint32_t i = 0; i < sizeof(reg)/2; ++i) {
    i2cWrite8(reg[i][0], reg[i][1]);
  }
  delay(1000); // waiting for AWB lock.
}

void loop() {
  // put your main code here, to run repeatedly:
  long timeStart = millis();
  int frameCount = 0;
  // waiting for update
  do {
    frameCount = i2cRead8(REG_FRAME_CNT);
  } while(frameCount == frameCountLast);
  frameCountLast = frameCount;

  i2cWrite8(REG_VISION_ID, VISION_ID);
  // read result
  if (i2cRead8(RESULT_NUM) > 0) {
    Serial.println("color detected:");
    Serial.print("x = ");
    Serial.println(i2cRead8(RESULT_DATA1));
    Serial.print("y = ");
    Serial.println(i2cRead8(RESULT_DATA2));
    Serial.print("width = ");
    Serial.println(i2cRead8(RESULT_DATA3));
    Serial.print("height = ");
    Serial.println(i2cRead8(RESULT_DATA4));
  } else {
    Serial.println("color undetected.");
  }
  Serial.print("fps = ");
  Serial.println(1000/(millis()-timeStart));
  Serial.println();
}





