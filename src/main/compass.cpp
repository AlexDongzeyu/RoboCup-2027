#include <Arduino.h>
#include "compass.h"

void Compass::initialize() {
  Wire1.begin();
  if (!bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    return;
  }
  bno.setExtCrystalUse(true);
  bno.setMode(OPERATION_MODE_NDOF);
  delay(500);

  // uint8_t system, gyro, accel, mag;
  // bno.getCalibration(&system, &gyro, &accel, &mag);
  // Serial.print("CALIBRATION: Sys=");
  // Serial.print(system);
  // Serial.print(" Gyro=");
  // Serial.print(gyro);
  // Serial.print(" Accel=");
  // Serial.print(accel);
  // Serial.print(" Mag=");
  // Serial.println(mag);
}

float Compass::readCompass() {

  sensors_event_t event;
  bno.getEvent(&event);
  float angle = event.orientation.x;
  return (angle > 180) ? angle - 360 : angle;
}