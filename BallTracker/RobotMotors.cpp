#include "RobotMotors.h"
#include <Arduino.h>
#include <math.h>

RobotMotors::RobotMotors() {}

void RobotMotors::begin() {
  // Direction pins — pure GPIO
  pinMode(motor_FR_Dir, OUTPUT);
  pinMode(motor_BR_Dir, OUTPUT);
  pinMode(motor_BL_Dir, OUTPUT);
  pinMode(motor_FL_Dir, OUTPUT);
  digitalWrite(motor_FR_Dir, LOW);
  digitalWrite(motor_BR_Dir, LOW);
  digitalWrite(motor_BL_Dir, LOW);
  digitalWrite(motor_FL_Dir, LOW);

  // PWM pins — prime the FlexPWM peripheral with duty=1 (0.4%).
  // This initialises the timer once; we NEVER call pinMode(OUTPUT)
  // on these pins again, to avoid a Teensy 4.1 FlexPWM re-init bug
  // that corrupts the duty cycle on certain channels.
  analogWrite(motor_FR_PWM, 1);
  analogWrite(motor_BR_PWM, 1);
  analogWrite(motor_BL_PWM, 1);
  analogWrite(motor_FL_PWM, 1);
}

/**
 * Commands the 4-wheel omni-drive robot.
 * angle: Direction to travel in degrees (0=Forward, 90=Right, 180=Back,
 * 270=Left) [CW] speed: Translation speed (0-255) turn: Rotational speed (-255
 * to 255, positive = Clockwise rotation)
 */
void RobotMotors::drive(float angle, float speed, float turn) {
  if (speed < 5 && abs(turn) < 5) {
    brake();
    return;
  }

  // Empirically verified: the wheel matrix is 90° offset from our CW system.
  // Motor test showed: N→W, E→N, W→S (all shifted -90°).
  // Fix: use 270 - angle instead of 360 - angle.
  float angle_ccw = 270.0f - angle;
  if (angle_ccw < 0.0f)
    angle_ccw += 360.0f;

  float rad = angle_ccw * PI / 180.0;

  // Wheel matrix using 40-degree offsets from the lateral axis (90 deg).
  // These provide the translational components for an X-drive.
  float fr_t = speed * sin(rad + (130.0 * PI / 180.0));
  float br_t = speed * sin(rad + (50.0 * PI / 180.0));
  float bl_t = speed * sin(rad - (50.0 * PI / 180.0));
  float fl_t = speed * sin(rad - (130.0 * PI / 180.0));

  // Mix translation and rotation.
  // 'turn' is positive for Clockwise rotation.
  // Since turn is added to all motors, and BL/FL are physically inverted
  // in their direction pins, this creates the necessary differential.
  float fr = fr_t + turn;
  float br = br_t + turn;
  float bl = bl_t + turn;
  float fl = fl_t + turn;

  // Constrain to PWM range
  float maxVal = abs(fr);
  if (abs(br) > maxVal)
    maxVal = abs(br);
  if (abs(bl) > maxVal)
    maxVal = abs(bl);
  if (abs(fl) > maxVal)
    maxVal = abs(fl);

  // Normalize motor speeds so the maximum commanding wheel matches
  // the peak desired magnitude (max of |speed| or |turn|).
  // This guarantees uniform translational speed in all directions.
  // Without this, cardinal directions only reached ~64% of requested speed!
  float targetMax = max(abs(speed), abs(turn));
  if (targetMax > 255.0f) targetMax = 255.0f;
  
  if (maxVal > 0.1f) {
    float scale = targetMax / maxVal;
    fr *= scale;
    br *= scale;
    bl *= scale;
    fl *= scale;
  }

  // Clamp minimum PWM to 1 so the FlexPWM stays active (never goes to 0)
  int fr_pwm = max((int)abs(fr), 1);
  int br_pwm = max((int)abs(br), 1);
  int bl_pwm = max((int)abs(bl), 1);
  int fl_pwm = max((int)abs(fl), 1);

  // Determine directions
  bool fr_dir = fr >= 0;    // HIGH if positive
  bool br_dir = br >= 0;    // HIGH if positive
  bool bl_dir = bl < 0;     // HIGH if negative (inverted logic)
  bool fl_dir = fl < 0;     // HIGH if negative (inverted logic)

  // Invert PWM for HIGH direction pins.
  // Because of how the motor driver bridging works, when DIR is HIGH,
  // the duty cycle actually runs in reverse (e.g. PWM=28 means 89% full throttle).
  // Therefore, whenever DIR is HIGH, we MUST send (255 - pwm) so the motor
  // actually gets the correct small duty cycle!
  int final_fr_pwm = fr_dir ? (255 - fr_pwm) : fr_pwm;
  int final_br_pwm = br_dir ? (255 - br_pwm) : br_pwm;
  int final_bl_pwm = bl_dir ? (255 - bl_pwm) : bl_pwm;
  int final_fl_pwm = fl_dir ? (255 - fl_pwm) : fl_pwm;

  // Debug: print motor values periodically
  static unsigned long lastMotorPrint = 0;
  if (millis() - lastMotorPrint >= 250) {
    lastMotorPrint = millis();
    Serial.print("[MOTOR] angle="); Serial.print(angle, 0);
    Serial.print(" FR:"); Serial.print(fr_pwm);
    Serial.print(" BR:"); Serial.print(br_pwm);
    Serial.print(" BL:"); Serial.print(bl_pwm);
    Serial.print(" FL:"); Serial.println(fl_pwm);
  }

  // Motor FR
  analogWrite(motor_FR_PWM, final_fr_pwm);
  digitalWrite(motor_FR_Dir, fr_dir ? HIGH : LOW);

  // Motor BR
  analogWrite(motor_BR_PWM, final_br_pwm);
  digitalWrite(motor_BR_Dir, br_dir ? HIGH : LOW);

  // Motor BL
  analogWrite(motor_BL_PWM, final_bl_pwm);
  digitalWrite(motor_BL_Dir, bl_dir ? HIGH : LOW);

  // Motor FL
  analogWrite(motor_FL_PWM, final_fl_pwm);
  digitalWrite(motor_FL_Dir, fl_dir ? HIGH : LOW);
}

void RobotMotors::brake() {
  // Use duty=1 (0.4%) instead of 0 to keep the FlexPWM peripheral alive.
  // The MDD3A motor driver cannot respond to 0.4% duty — motor is effectively stopped.
  // This avoids a Teensy 4.1 bug where analogWrite(pin, 0) tears down the FlexPWM
  // timer, and subsequent analogWrite(pin, N) re-initialises it with a corrupt
  // duty cycle on certain channels (pins 4 and 7 / FlexPWM2.0 and FlexPWM1.3).
  analogWrite(motor_FR_PWM, 1);
  analogWrite(motor_BR_PWM, 1);
  analogWrite(motor_BL_PWM, 1);
  analogWrite(motor_FL_PWM, 1);

  // Direction pins LOW for consistent idle state
  digitalWrite(motor_FR_Dir, LOW);
  digitalWrite(motor_BR_Dir, LOW);
  digitalWrite(motor_BL_Dir, LOW);
  digitalWrite(motor_FL_Dir, LOW);
}

