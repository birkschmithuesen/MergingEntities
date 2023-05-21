#pragma once

#define CALIBRATIONBUTON 23 /**< calibration button pin */
#define ID_PIN1 13          /**< 1st bit pin of ID DIP switch (D13) */
#define ID_PIN2 12          /**< 2nd bit pin of ID DIP switch (D12) */
#define ID_PIN3 14          /**< 3rd bit pin of ID DIP switch (D14) */
#define ID_PIN4 27          /**< 4rd bit pin of ID DIP switch (D27) */

class UserInterface
{
public:
    /**
     * Retrieve the (unique) ID configured for this controller.
     * This ID is used in the OSC messages to identify the sender.
     * Its value is between 0 and 15 inclusively.
     *
     * @return configured ID of the controller.
     * @see getControllerIdChars()
     */
    static uint8_t getControllerID()
    {
        uint8_t id = 0;

        // read the switch state from left to right and add value at position
        if (LOW == digitalRead(ID_PIN1))
        {
            id = 8;
        }
        if (LOW == digitalRead(ID_PIN2))
        {
            id += 4;
        }
        if (LOW == digitalRead(ID_PIN3))
        {
            id += 2;
        }
        if (LOW == digitalRead(ID_PIN4))
        {
            id += 1;
        }
        return id;
    }
};