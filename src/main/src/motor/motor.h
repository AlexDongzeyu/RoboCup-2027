#pragma once

class Motors {
public:
  static constexpr int MAX_FORWARD_PWM = 255;
  static constexpr int MAX_TURN_PWM = 120;
  static constexpr int MAX_WHEEL_PWM = 255;
  static constexpr int RAMP_UP_STEP = 18;
  static constexpr int RAMP_DOWN_STEP = 25;
  static constexpr int TURN_RAMP_UP_STEP = 22;
  static constexpr int MIN_TURN_START_PWM = 58;
  static constexpr float DRIVE_TURN_RATIO = 0.55f;

  void begin();
  void drive(float targetForward, float targetClockwise);
  void holdStraight(int targetForward);
  void holdTurn();
  void testChannel(int index, int power);
  void stop();
  int maxAppliedPwm() const;
  float appliedForwardPwm() const;
  float appliedTurnPwm() const;
  void signedPwm(int (&commands)[4]) const;

private:
  int applied[4] = {};
  float forward = 0;
  float turn = 0;
  void write(int index, int power);
  void apply();
};
