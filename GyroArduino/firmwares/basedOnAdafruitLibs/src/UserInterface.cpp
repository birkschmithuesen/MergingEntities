#include "UserInterface.hpp"
UserInterface::UserInterfaceBlinkModes UserInterface::currentBlinkMode = UserInterface::BlinkModeNormal;
int UserInterface::blinkFlashes = 0;

// for blink task
void UserInterface::blinkFunction(void *)
{
    pinMode(PIN_BLUELED, OUTPUT);
    pinMode(PIN_GREENLED, OUTPUT);
    while (true)
    {
        switch (UserInterface::currentBlinkMode)
        {

        case UserInterface::BlinkModeCalibrate:
            for (int i = 0; i < UserInterface::blinkFlashes; i++)
            {
                digitalWrite(PIN_GREENLED, HIGH);
                delay(200);
                digitalWrite(PIN_GREENLED, LOW);
            }
            delay(800);
            break;
        case UserInterface::BlinkModeNormal:
            digitalWrite(PIN_GREENLED, LOW);
            delay(100);
            break;
        }
    }
}