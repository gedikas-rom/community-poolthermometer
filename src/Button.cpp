#include "Button.h"
#include <Arduino.h>
#include <driver/rtc_io.h>

Button::Button(int pin)
{
    btnPin = pin;
    state = LOW;
    pinMode(btnPin, INPUT_PULLUP);
    rtc_gpio_hold_en((gpio_num_t)btnPin);
}

void Button::start()
{
}

int Button::checkBtn()
{
    // return 0 = nothing to do
    // return 1 = short klick
    // return 2 = long klick
    result = 0;

    int prevState = state;
    state = digitalRead(btnPin);

    if (prevState == LOW && state == HIGH)
    {
        buttonclickedMS = millis();
        result = 0;
    }
    else if (prevState == HIGH && state == LOW)
    {
        // here comes the logic

        if (millis() - buttonclickedMS < 50)
        {
            // debounce area - nothing to do
            result = 0;
        }
        else if (millis() - buttonclickedMS > 50 && millis() - buttonclickedMS < 1000)
        {
            // Serial.println("short click");
            result = 1;
        }
        else if (millis() - buttonclickedMS >= 1000)
        {
            result = 2;
        }

    }

    return result;
}