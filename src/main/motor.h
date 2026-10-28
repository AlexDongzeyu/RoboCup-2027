#ifndef MOTOR_H
#define MOTOR_H

class Motor
{
public:
  /**
   * Initializes the motor pins to be used.
   *
   * @param `pin_c` the pin that, when held high and the other held low, roatates the motor clockwise.
   * @param `pin_cc` the pin that, when held high and the other held low, roatates the motor counter-clockwise.
   */
  Motor(unsigned int pin_c, unsigned int pin_cc);

  /**
   * Spins the wheel at a certain speed. If the integer for speed that is given is > 0, the wheel will spin clockwise. If the opposite, the wheel will spin counter-clockwise.
   */
  void spin(int speed);

  /**
   * Stops the spin of the wheel; brakes it.
   */
  void brake();

private:
  unsigned int pin_c;
  unsigned int pin_cc;
  int speed;
  bool is_breaking;
};

#endif