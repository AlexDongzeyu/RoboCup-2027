#ifndef ROBOTMOTORS_H
#define ROBOTMOTORS_H

#include <Arduino.h>

class RobotMotors {
public:
  RobotMotors();
  void begin();

  // angle: 0=Forward, 90=Right (Clockwise)
  // speed: 0-255
  // turn: -255 to 255 (positive = Clockwise rotation)
  void drive(float angle, float speed, float turn);

  void brake();

private:
  // MDD3A Motor Driver Pins (Teensy 4.1)
  // Each motor has a PWM pin and a Direction pin.
  const int motor_FR_PWM = 1;
  const int motor_FR_Dir = 2;
  const int motor_BR_PWM = 4;
  const int motor_BR_Dir = 3;
  const int motor_BL_PWM = 5;
  const int motor_BL_Dir = 6;
  const int motor_FL_PWM = 7;
  const int motor_FL_Dir = 8;
};

#endif
