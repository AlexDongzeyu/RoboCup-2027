#include "components.h"
#include <Arduino.h>

Component::Component(int pin_numbers[], uint8_t modes[], int number_of_pins, String component_name)
{
    num_pins(number_of_pins);
    name(component_name);
    pin_nums(pin_numbers);
    mds(modes);
}

Component::setUp()
{
    Serial.print("Initializing ");
    Serial.print(name);
    Serial.println("...");
    Serial.println();

    for (int i = 0; i < num_pins; i++)
    {
        Serial.print("Initializing pin ");
        Serial.print(pin_nums[i]);
        Serial.println();

        pinMode(pin_nums[i], mds[i]);
    }
}