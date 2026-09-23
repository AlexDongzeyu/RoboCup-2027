#ifndef COLORSENSOR_H
#define COLORSENSOR_H

#include <Wire.h>
#include "Adafruit_TCS34725.h"

// ============================================================
//  ColorSensor — 8-channel TCS34725 via TCA9548A I2C mux
//  Hardware:  TCA9548A @ 0x70, A0/A1/A2 tied to GND
//             Uses Wire1: SCL = pin 16, SDA = pin 17
//  Sensor channels (clockwise from front, 0°):
//    CH0 = 0°   (front)
//    CH1 = 45°  (front-right)
//    CH2 = 90°  (right)
//    CH3 = 135° (back-right)
//    CH4 = 180° (back)
//    CH5 = 225° (back-left)
//    CH6 = 270° (left)
//    CH7 = 315° (front-left)
// ============================================================

class ColorSensor {
  public:
    // Call once in setup() — calibrates green-field baseline
    void init();

    // Call every loop iteration to refresh raw readings
    void updateReadings();

    // Returns the escape angle (opposite of sensor that hit white),
    // or -1.0f if no white is detected.
    float getAvoidAngle();

    // Debug helpers
    void printReadings();
    void printGreenValues();
    uint16_t* getAnalogValues();

  private:
    static const uint8_t TCA_ADDR   = 0x70;
    static const uint8_t TCS_ADDR   = 0x29;
    static const uint8_t NUM_SENSORS = 8;

    uint16_t analogValues[NUM_SENSORS];
    uint16_t greenValues[NUM_SENSORS];

    // How many counts above baseline counts as "white"
    int buffer = 500;

    Adafruit_TCS34725 tcs = Adafruit_TCS34725(
      TCS34725_INTEGRATIONTIME_50MS,
      TCS34725_GAIN_4X
    );

    void tcaSelect(uint8_t channel);
    bool i2cDetect(uint8_t addr);
};

#endif  // COLORSENSOR_H
