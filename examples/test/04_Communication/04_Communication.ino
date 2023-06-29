/*!
 * @file 04_Communication.ino
 * @brief Examples of communication.
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
 * Select MU address
 */

#define MU_DEVICE_ADDRESS     0x60 // Select address switch to 00
//#define MU_DEVICE_ADDRESS     0x61 // Select address switch to 01
//#define MU_DEVICE_ADDRESS     0x62 // Select address switch to 10
//#define MU_DEVICE_ADDRESS     0x63 // Select address switch to 11

/*
 * Output Settings
 */

//#define OUTPUT_MODE_SOFT_SERIAL     1 // Uart/Serial - Select output switch to 00
//#define OUTPUT_MODE_HARD_SERIAL   1 // Uart/Serial - Select output switch to 00 // for Mega1280 or Mega2560
#define OUTPUT_MODE_I2C           1 // I2C - Select output switch to 01

/*
 * Set soft serial pins
 */
#define SOFT_SERIAL_RX_PIN  2
#define SOFT_SERIAL_TX_PIN  3

/*
 * Selet serial baudrate
 * Baud 230400,460800,921600 are not suitable for arduino
 */
#define BAUDRATE  kBaud9600
//#define BAUDRATE  kBaud19200
//#define BAUDRATE  kBaud38400
//#define BAUDRATE  kBaud57600
//#define BAUDRATE  kBaud115200
 
/*
 * Select I2c clock
 */
#define I2C_CLOCK  100000           //I2C
//#define I2C_CLOCK  400000

/*
 * Vision Settings
 * Take BODY vision as example
 */ 
#define VISION_TYPE     VISION_BODY_DETECT // 05

/*
 * Functions
 */
DFRobot_MuVisionSensor MU(MU_DEVICE_ADDRESS); // 0x60 is device address

#ifdef OUTPUT_MODE_SOFT_SERIAL
SoftwareSerial SoftSerial(SOFT_SERIAL_RX_PIN, SOFT_SERIAL_TX_PIN);  // RX, TX  // The hardware serial port is used for log output, so we use soft serial port
#endif

#ifdef OUTPUT_MODE_SOFT_SERIAL
MuVisionSensorUart Register(&SoftSerial, MU_DEVICE_ADDRESS);
#elif OUTPUT_MODE_HARD_SERIAL
MuVisionSensorUart Register(&Serial1, MU_DEVICE_ADDRESS);
#elif OUTPUT_MODE_I2C
MuVisionSensorI2C Register(&Wire,MU_DEVICE_ADDRESS);
#endif

void setup() {
  Serial.begin(115200);
  Serial.println("==========Start==========");

  Serial.println("MU begin...");
  
#ifdef OUTPUT_MODE_SOFT_SERIAL
  uint32_t BaudList[8] = {9600,19200,38400,57600,115200};
  SoftSerial.setTimeout(100);
  uint8_t dataRead = 0;
  for (int i = 0; i < 8; i++) {
    Serial.print("Baud Check:");
    Serial.println(BaudList[i]);
    SoftSerial.begin(BaudList[i]);
    delay(100);
    dataRead = 0;
    Register.Set(0x25, 0xAA);
    Register.Get(0x25, &dataRead);
    if (dataRead == 0xAA) {
      break;
    }
  }
  MU.begin(&SoftSerial);   // Soft Uart/Serial
  MU.uartSetBaudrate(BAUDRATE);
  delay(100);
  SoftSerial.begin(BaudList[BAUDRATE]);
  
#elif OUTPUT_MODE_HARD_SERIAL
  uint32_t BaudList[8] = {9600,19200,38400,57600,115200};
  SoftSerial.setTimeout(100);
  uint8_t dataRead = 0;
  for (int i = 0; i < 8; i++) {
    Serial.print("Baud Check:");
    Serial.println(BaudList[i]);
    Serial1.begin(BaudList[i]);
    delay(100);
    dataRead = 0;
    Register.Set(0x25, 0xAA);
    Register.Get(0x25, &dataRead);
    if (dataRead == 0xAA) {
      break;
    }
  }
  MU.begin(&Serial1);   // Hard Uart/Serial
  MU.uartSetBaudrate(BAUDRATE);
  delay(100);
  Serial1.begin(BaudList[BAUDRATE]);
  
#elif OUTPUT_MODE_I2C
  Wire.setClock(I2C_CLOCK);
  Wire.begin();
  MU.begin(&Wire);             //I2C
#endif
  Serial.println("Finised");

  Serial.println("Vision begin...");
  MU.visionBegin(VISION_TYPE);

  Serial.println("Finished");
  Serial.println("MU Vision Sensor Processing...");
}
  
void loop() {
  unsigned long timeStart = 0;
  float averageTime = 0.0;
  unsigned long successSend = 0;
  unsigned long totalSend = 0;
  uint8_t dataWrite = 0;
  uint8_t dataRead = 0;
  while (1) {
    timeStart = millis();
    for (uint8_t dataWrite = 0; dataWrite < 100; dataWrite++) {
      Register.Set(0x25, dataWrite);
      Register.Get(0x25, &dataRead);
      if (dataWrite == dataRead) {
        successSend++;
      }
      totalSend++;
    }
    averageTime = (float)(millis()-timeStart) / 100;
    
    Serial.print("total send:");
    Serial.print(totalSend);
    Serial.print("  successed:");
    Serial.print(successSend);
    Serial.print("  success rate:");
    Serial.print((float)successSend / (float)totalSend * 100);
    Serial.print("  r/w average time:");
    Serial.print(averageTime);
    Serial.println("ms");
  }
}
