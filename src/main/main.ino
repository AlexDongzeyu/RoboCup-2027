#include <Arduino.h>
#include "IR.h"
#include "movement.h"
#include "colorsensor.h"
#include "camera.h"
#include "kicker.h"
#include "tof.h"


Movement m;
IR ir;
ColorSensor c;
Camera cam(70.0);
Kicker kicker(3, 100);
TOF tof;


enum RobotState {
  SEARCH_BALL,
  ATTACK_BALL,
  AVOID_BORDER,
  ATTACK_GOAL,
  KICK
};

RobotState state = SEARCH_BALL;

// 非阻塞定时器
unsigned long lastLoopTime = 0;
const int LOOP_INTERVAL_MS = 20;   // 50Hz

// 串口打印节流
unsigned long lastPrintTime = 0;
const int PRINT_INTERVAL_MS = 200;

// 速度常量
const int SPEED_SLOW = 30;
const int SPEED_NORMAL = 50;
const int SPEED_FAST = 70;


void updateRobot() {
  c.updateReadings();
  float avoidAngle = c.getAvoidAngle();
  if (avoidAngle != -1) {
    state = AVOID_BORDER;
    m.basic_move_with_compass(avoidAngle, SPEED_SLOW);
    return;
  }

  ir.updateReadings();
  float ballAngle = ir.getBallAngle();

  switch (state) {
    case SEARCH_BALL:
      if (ballAngle != -1) {
        state = ATTACK_BALL;
      } else {
        m.rotate(SPEED_SLOW);
      }
      break;

    case ATTACK_BALL: {
      if (ballAngle == -1) {
        state = SEARCH_BALL;
        m.brake();
        break;
      }

      float camAngle = cam.calculateRotationAngle();
      int ballRange = tof.GetBallRange();

      if (ballRange > 0 && ballRange < 80 && camAngle != 999.0) {
        state = ATTACK_GOAL;
        break;
      }

      m.basic_move_with_compass(ballAngle, SPEED_NORMAL);
      break;
    }

    case ATTACK_GOAL: {
      float camAngle = cam.calculateRotationAngle();
      if (camAngle == 999.0) {
        state = ATTACK_BALL;
        break;
      }
      m.basic_move_with_compass_and_camera(ballAngle, SPEED_SLOW, camAngle);

      int ballRange = tof.GetBallRange();
      if (ballRange > 0 && ballRange < 50) {
        state = KICK;
      }
      break;
    }

    case KICK:
      kicker.performKick();
      delay(200);
      state = SEARCH_BALL;
      break;

    case AVOID_BORDER:
      if (avoidAngle == -1) {
        state = SEARCH_BALL;
      }
      break;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  ir.initIR();
  m.initMovement();
  c.init();
  cam.initialize();
  kicker.init();
  tof.initialize();

  Serial.println("Robot ready.");
}

void loop() {
  unsigned long now = millis();
  if (now - lastLoopTime < LOOP_INTERVAL_MS) return;
  lastLoopTime = now;

  updateRobot();

  if (now - lastPrintTime > PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    Serial.print("State: ");
    Serial.print(state);
    Serial.print(" | BallAngle: ");
    Serial.print(ir.getBallAngle());
    Serial.print(" | Avoid: ");
    Serial.println(c.getAvoidAngle());
  }
}

// #include <Wire.h>
// #include "Adafruit_TCS34725.h"

// #define TCA_ADDR 0x70
// #define TCS_ADDR 0x29
// #define NUM_SENSORS 8

// Adafruit_TCS34725 tcs = Adafruit_TCS34725(
//   TCS34725_INTEGRATIONTIME_50MS,
//   TCS34725_GAIN_4X
// );

// uint16_t greenValues[NUM_SENSORS];
// uint16_t clearValues[NUM_SENSORS];
// uint16_t redValues[NUM_SENSORS];
// uint16_t blueValues[NUM_SENSORS];

// void tcaSelect(uint8_t channel) {
//   if (channel > 7) return;

//   Wire1.beginTransmission(TCA_ADDR);
//   Wire1.write(1 << channel);
//   Wire1.endTransmission();
// }

// bool i2cDetect(uint8_t addr) {
//   Wire1.beginTransmission(addr);
//   return (Wire1.endTransmission() == 0);
// }

// void setup() {
//   Serial.begin(9600);
//   while (!Serial) {}

//   Wire1.begin();

//   Serial.println("=== COLOR SENSOR CHANNEL TEST ===");
//   Serial.println("This will print CH0 to CH7.");
//   Serial.println("Move a white object over one sensor at a time.");
//   Serial.println("Watch which channel changes most.");
//   Serial.println();

//   // Check multiplexer
//   Serial.print("Checking TCA9548A at 0x70... ");
//   if (i2cDetect(TCA_ADDR)) {
//     Serial.println("FOUND");
//   } else {
//     Serial.println("NOT FOUND");
//     while (1) {}
//   }

//   // Check each channel for a TCS3472
//   for (int i = 0; i < NUM_SENSORS; i++) {
//     tcaSelect(i);

//     Serial.print("Checking TCS3472 on CH");
//     Serial.print(i);
//     Serial.print("... ");

//     if (i2cDetect(TCS_ADDR)) {
//       Serial.println("FOUND");

//       if (!tcs.begin(TCS_ADDR, &Wire1)) {
//         Serial.print("BUT tcs.begin() FAILED on CH");
//         Serial.println(i);
//       }
//     } else {
//       Serial.println("NOT FOUND");
//     }
//   }

//   Serial.println();
//   Serial.println("Start testing...");
//   Serial.println();
// }

// void loop() {
//   uint16_t r, g, b, c;

//   for (int i = 0; i < NUM_SENSORS; i++) {
//     tcaSelect(i);

//     if (!i2cDetect(TCS_ADDR)) {
//       redValues[i] = 0;
//       greenValues[i] = 0;
//       blueValues[i] = 0;
//       clearValues[i] = 0;
//       continue;
//     }

//     tcs.getRawData(&r, &g, &b, &c);

//     redValues[i] = r;
//     greenValues[i] = g;
//     blueValues[i] = b;
//     clearValues[i] = c;
//   }

//   Serial.println("--------------------------------------------------");
//   for (int i = 0; i < NUM_SENSORS; i++) {
//     Serial.print("CH");
//     Serial.print(i);
//     Serial.print("   R: ");
//     Serial.print(redValues[i]);
//     Serial.print("   G: ");
//     Serial.print(greenValues[i]);
//     Serial.print("   B: ");
//     Serial.print(blueValues[i]);
//     Serial.print("   C: ");
//     Serial.println(clearValues[i]);
//   }

//   Serial.println();
//   delay(300);
// }