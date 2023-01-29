/*
 * This example turns the ESP32 into a Bluetooth LE gamepad that presses buttons and moves axis
 *
 * At the moment we are using the default settings, but they can be canged using a BleGamepadConfig instance as parameter for the begin function.
 *
 * Possible buttons are:
 * BUTTON_1 through to BUTTON_16
 * (16 buttons by default. Library can be configured to use up to 128)
 *
 * Possible DPAD/HAT switch position values are:
 * DPAD_CENTERED, DPAD_UP, DPAD_UP_RIGHT, DPAD_RIGHT, DPAD_DOWN_RIGHT, DPAD_DOWN, DPAD_DOWN_LEFT, DPAD_LEFT, DPAD_UP_LEFT
 * (or HAT_CENTERED, HAT_UP etc)
 *
 * bleGamepad.setAxes sets all axes at once. There are a few:
 * (x axis, y axis, z axis, rx axis, ry axis, rz axis, slider 1, slider 2)
 *
 * Library can also be configured to support up to 5 simulation controls
 * (rudder, throttle, accelerator, brake, steering), but they are not enabled by default.
 *
 * Library can also be configured to support different function buttons
 * (start, select, menu, home, back, volume increase, volume decrease, volume mute)
 * start and select are enabled by default
 */

#include <Arduino.h>
#include <BleGamepad.h>
//#include <Wire.h>
//#include <Adafruit_I2CDevice.h>
#include <SPI.h>
//#include "Adafruit_seesaw.h"

BleGamepad bleGamepad("Shapeoko Pendant");

#include "Adafruit_seesaw.h"

// This can be enabled to get extra logging about the position of the controller relative to the correction values
//#define JOY_CALIBRATION_MODE

// This sketch requires that one of the optional interrupt pins on the Joy Featherwing is soldered. This value should match the
// complimentary GPIO pin on the ESP device. See: https://learn.adafruit.com/joy-featherwing/pinouts
#define IRQ_PIN 14 // Pin 14 is the pin directly to the left of the SCL pin on an ESP32

Adafruit_seesaw ss;

// GPIO pins on the Joy Featherwing for reading button presses. These should not be changed.
#define BUTTON_RIGHT 6
#define BUTTON_DOWN 7
#define BUTTON_LEFT 9
#define BUTTON_UP 10
#define BUTTON_SEL 14

// GPIO Analog pins on the Joy Featherwing for reading the analog stick. These should not be changed.
#define STICK_H 3
#define STICK_V 2

// When the analog stick is moved and returns to its center point there may be a deviation from the true center of 512. A calibration will 
// occur when the analog stick read task begins. Even after this calibration there may be some drift on the stick that can make determining 
// the center point error prone. In order to compensate for this, values can be specified to determine a reasonable center point. If you have 
// a use case where you don't care about drift or the center point of the stick, this can all be ignored entirely.
#ifndef STICK_CENTER_POINT
#define STICK_CENTER_POINT 512 // Analog stick will read 0...1024 along each axis
#endif
#ifndef STICK_L_CORRECTION
#define STICK_L_CORRECTION -55
#endif
#ifndef STICK_R_CORRECTION
#define STICK_R_CORRECTION 50
#endif
#ifndef STICK_U_CORRECTION
#define STICK_U_CORRECTION 20
#endif
#ifndef STICK_D_CORRECTION
#define STICK_D_CORRECTION -20
#endif

// Every time the analog values are read they will be slightly different. In order
// to only detect movement when the stick is actually moved, these values can tune
// the minimum amount of movement + or - before it is considered as moved.
#ifndef MIN_STICK_H_MOVE
#define MIN_STICK_H_MOVE 5
#endif
#ifndef MIN_STICK_V_MOVE
#define MIN_STICK_V_MOVE 5
#endif

#define BUTTON_Z_UP BUTTON_1
#define BUTTON_Z_DOWN BUTTON_2

#define BUTTON_FASTER BUTTON_3
#define BUTTON_SLOWER BUTTON_4

uint32_t button_mask = (1 << BUTTON_RIGHT) | (1 << BUTTON_DOWN) |
                       (1 << BUTTON_LEFT) | (1 << BUTTON_UP) | (1 << BUTTON_SEL);

QueueHandle_t buttonPressQueue;                      // Queue for notifying of button press changes
SemaphoreHandle_t i2cSem = xSemaphoreCreateBinary(); // This semaphore is used to synchronize calls to I2C to prevent concurrent operations

// ISR that gets triggered when a button is pressed.
void IRAM_ATTR onButtonPress()
{
    // The ISR just sends a signal to the queue. Value doesn't matter.
    uint8_t v = 0;
    if (!xQueueSend(buttonPressQueue, &v, portMAX_DELAY))
    {
        Serial.println("WARNING: Could not queue message because queue is full.");
    }
}


// Queue consumer for responding to button presses
void buttonPressConsumer(void *)
{
    Serial.println(F("buttonPressConsumer() begin"));
    uint32_t lastValue = 0;
    while (true)
    {
        void *p; // Don't care about this value, only that we get queued
        // This will yield until the queue gets signalled
        xQueueReceive(buttonPressQueue, &p, portMAX_DELAY);
        xSemaphoreTake(i2cSem, portMAX_DELAY);
        uint32_t v = ss.digitalReadBulk(button_mask);



        // Debounce by discarding duplicate reads
        if (lastValue != v)
        {
            Serial.println("Buttons changed");

            // Z Down
            int oldV = lastValue & (1 << BUTTON_DOWN);
            int newV = v & (1 << BUTTON_DOWN);
            if (oldV != newV)
            {
                Serial.println("\tBUTTON_DOWN changed");
                if (!(v & (1 << BUTTON_DOWN)))
                {
                    Serial.println("\t\tBUTTON_DOWN pressed");
                    bleGamepad.press(BUTTON_Z_DOWN);
                }
                else 
                {
                    Serial.println("\t\tBUTTON_DOWN released");
                    bleGamepad.release(BUTTON_Z_DOWN);
                }
            }


            // Z Up
            oldV = lastValue & (1 << BUTTON_UP);
            newV = v & (1 << BUTTON_UP);
            if (oldV != newV)
            {
                Serial.println("\tBUTTON_UP changed");
                if (!(v & (1 << BUTTON_UP)))
                {
                    Serial.println("\t\tBUTTON_UP pressed");
                    bleGamepad.press(BUTTON_Z_UP);
                }
                else 
                {
                    Serial.println("\t\tBUTTON_UPreleased");
                    bleGamepad.release(BUTTON_Z_UP);
                }
            }

            // Faster
            oldV = lastValue & (1 << BUTTON_RIGHT);
            newV = v & (1 << BUTTON_RIGHT);
            if (oldV != newV)
            {
                Serial.println("\tBUTTON_RIGHT changed");
                if (!(v & (1 << BUTTON_RIGHT)))
                {
                    Serial.println("\t\tBUTTON_RIGHT pressed");
                    bleGamepad.press(BUTTON_FASTER);
                }
                else 
                {
                    Serial.println("\t\tBUTTON_RIGHT released");
                    bleGamepad.release(BUTTON_FASTER);
                }
            }


            // Z Up
            oldV = lastValue & (1 << BUTTON_LEFT);
            newV = v & (1 << BUTTON_LEFT);
            if (oldV != newV)
            {
                Serial.println("\tBUTTON_LEFT changed");
                if (!(v & (1 << BUTTON_LEFT)))
                {
                    Serial.println("\t\tBUTTON_LEFT pressed");
                    bleGamepad.press(BUTTON_SLOWER);
                }
                else 
                {
                    Serial.println("\t\tBUTTON_LEFT");
                    bleGamepad.release(BUTTON_SLOWER);
                }
            }

            lastValue = v;
        }

        xSemaphoreGive(i2cSem);
    }

    vTaskDelete(NULL);
}

void analogStickTask(void *)
{
    Serial.println(F("analogStickTask() begin"));

    int16_t x_ctr, y_ctr;
    xSemaphoreTake(i2cSem, portMAX_DELAY);
    x_ctr = ss.analogRead(STICK_H);
    y_ctr = ss.analogRead(STICK_V);
    xSemaphoreGive(i2cSem);


    

    int16_t x = -1;
    int16_t y = -1;
    bool isCentered = true;
    while (true)
    {
        xSemaphoreTake(i2cSem, portMAX_DELAY);
        int16_t new_x = ss.analogRead(STICK_H);
        int16_t new_y = ss.analogRead(STICK_V);
        xSemaphoreGive(i2cSem);

        // Ignore minute position changes as the values will change slightly with
        // every read. This can be tuned with MIN_STICK_H_MOVE and MIN_STICK_V_MOVE
        if (new_x <= x - MIN_STICK_H_MOVE ||
            new_x >= x + MIN_STICK_H_MOVE ||
            new_y <= y - MIN_STICK_V_MOVE ||
            new_y >= y + MIN_STICK_V_MOVE)
        {


            // Make a best effort guess as to if the stick is centered or not based on
            // initial calibration and corrections
            isCentered = x_ctr >= max(0, new_x + STICK_L_CORRECTION) &&
                         x_ctr <= max(0, new_x + STICK_R_CORRECTION) &&
                         y_ctr <= max(0, new_y + STICK_U_CORRECTION) &&
                         y_ctr >= max(0, new_y + STICK_D_CORRECTION);

            // Ensure value is always 0...1024 and account for any corrections and/or calibrations to prevent over/underflows
            x = new_x < 0 ? 0 : new_x > 1024 ? 1024  : new_x;
            y = new_y < 0 ? 0 : new_y > 1024 ? 1024  : new_y;

            int pad_x = 32767 / 1024 * x;
            int pad_y = 32767 / 1024 * y;

            // Log the position of the analog stick in various ways for different kinds of application
            Serial.printf("Analog stick position change!\n\tIs centered: %s\n\tPosition: X=%d Y=%d\n\tPAD: X=%d Y=%d\n",
                          isCentered ? "true" : "false",
                          x, y, pad_x, pad_y);



            bleGamepad.setAxes(pad_x, pad_y, 0,0,0,0,0,0);

            //bleGamepad.setAxes(32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767);
        }

        // Tune this to be quick enough to read the controller position in a reasonable amount of time but not so fast that it 
        // saturates the I2C bus and delays or blocks other operations.
        delay(100);
           // bleGamepad.setAxes(32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767);

    }

    vTaskDelete(NULL);
}



void setup()
{
    Serial.begin(115200);


    Serial.println("Starting BLE work!");
    bleGamepad.begin();


      Serial.println("Joy FeatherWing example!");

    if (!ss.begin(0x49))
    {
        Serial.println("ERROR! seesaw not found");
        while (1)
        {
            delay(1);
        }
    }
    else
    {
        Serial.println("seesaw started");
        Serial.print("version: ");
        Serial.println(ss.getVersion(), HEX);
    }

    ss.pinModeBulk(button_mask, INPUT_PULLUP);
    ss.setGPIOInterrupts(button_mask, 1);
    pinMode(IRQ_PIN, INPUT);

    xSemaphoreGive(i2cSem); // Initialize the semaphore to 0 (default state is uninitialized which will cause a crash)
    buttonPressQueue = xQueueCreate(10, sizeof(uint8_t));

    // Task for listening to button presses
    xTaskCreate(
        buttonPressConsumer,
        "ButtonPressConsumer",
        10000, // Stack size -- too low and ESP will eventually crash within the task
        NULL,
        1,
        NULL);

    // Task for reading the analog stick value
    xTaskCreate(
        analogStickTask,
        "AnalogStickTask",
        10000, // Stack size -- too low and ESP will eventually crash within the task
        NULL,
        2,
        NULL);

    // Respond to changes from button presses
    attachInterrupt(IRQ_PIN, onButtonPress, FALLING);
/*
    Serial.println("JoyWing");
    joy.begin();
    Serial.println("JoyWing started");
    joy.registerJoystickCallback(joystickCallback);
    Serial.println("JoyWing stick");
    joy.registerButtonCallback(buttonCallback);
    Serial.println("JoyWing buttons");
*/
    // The default bleGamepad.begin() above enables 16 buttons, all axes, one hat, and no simulation controls or special buttons
}

void loop()
{
    /*
    if (bleGamepad.isConnected())
    {
        Serial.println("Press buttons 5, 16 and start. Move all enabled axes to max. Set DPAD (hat 1) to down right.");
        bleGamepad.press(BUTTON_5);
        bleGamepad.press(BUTTON_16);
        bleGamepad.pressStart();
        bleGamepad.setAxes(32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767);
        bleGamepad.setHat1(HAT_DOWN_RIGHT);
        // All axes, sliders, hats etc can also be set independently. See the IndividualAxes.ino example
        delay(500);

        Serial.println("Release button 5 and start. Move all axes to min. Set DPAD (hat 1) to centred.");
        bleGamepad.release(BUTTON_5);
        bleGamepad.releaseStart();
        bleGamepad.setHat1(HAT_CENTERED);
        bleGamepad.setAxes(0, 0, 0, 0, 0, 0, 0, 0);
        delay(500);
    }
    */
   //joy.update();
   delay(500);
}
