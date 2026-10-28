#include <Arduino.h>
#include "motor.h"

Motor::Motor(unsigned int pin_c, unsigned int pin_cc)
    : pin_c(pin_c), pin_cc(pin_cc)
{
    pinMode(pin_c, OUTPUT);
    pinMode(pin_cc, OUTPUT);
}

void Motor::spin(int s)
{
    if (s >= 0)
    {
        analogWrite(pin_c, s);
        analogWrite(pin_cc, 0);
    }
    else
    {
        analogWrite(pin_cc, -s);
        analogWrite(pin_c, 0);
    }
}

void Motor::brake()
{
    analogWrite(pin_c, 0);
    analogWrite(pin_cc, 0);
}