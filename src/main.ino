#include <BleKeyboard.h>
BleKeyboard bleKeyboard("Shapeoko Pendant Keyboard");

#include <Arduino.h>
#include <SPI.h>

#include "Adafruit_seesaw.h"

// This can be enabled to get extra logging about the position of the controller relative to the correction values
// #define JOY_CALIBRATION_MODE

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
#define STICK_CENTER_POINT 512 // Analog stick will read 0...1024 along each axis
#define STICK_L_CORRECTION -55
#define STICK_R_CORRECTION 50
#define STICK_U_CORRECTION 20
#define STICK_D_CORRECTION -20
#define D_THRESHOLD 200;

// Every time the analog values are read they will be slightly different. In order
// to only detect movement when the stick is actually moved, these values can tune
// the minimum amount of movement + or - before it is considered as moved.
#define MIN_STICK_H_MOVE 5
#define MIN_STICK_V_MOVE 5

#define BUTTON_Z_UP BUTTON_1
#define BUTTON_Z_DOWN BUTTON_2

#define BUTTON_FASTER BUTTON_3
#define BUTTON_SLOWER BUTTON_4

const int MAX_ANALOG_VAL = 4095;
const float MAX_BATTERY_VOLTAGE = 4.2; // Max LiPoly voltage of a 3.7 battery is 4.2

uint32_t button_mask = (1 << BUTTON_RIGHT) | (1 << BUTTON_DOWN) |
                       (1 << BUTTON_LEFT) | (1 << BUTTON_UP) | (1 << BUTTON_SEL);

QueueHandle_t buttonPressQueue;                      // Queue for notifying of button press changes
SemaphoreHandle_t i2cSem = xSemaphoreCreateBinary(); // This semaphore is used to synchronize calls to I2C to prevent concurrent operations

int currentSpeed = 4;

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
        Serial.println("Buttons test");

        // Debounce by discarding duplicate reads
        if (lastValue != v)
        {
            bleKeyboard.releaseAll();

            // Z Down
            if ((lastValue & (1 << BUTTON_DOWN)) != (v & (1 << BUTTON_DOWN)))
            {
                Serial.println("\tBUTTON_DOWN changed");
                if (!(v & (1 << BUTTON_DOWN)))
                {
                    Serial.println("\t\tBUTTON_DOWN pressed");
                    bleKeyboard.press(KEY_PAGE_DOWN);
                }
                else
                {
                    Serial.println("\t\tBUTTON_DOWN released");
                    // bleKeyboard.releaseAll();
                }
            }

            // Z Up
            if ((lastValue & (1 << BUTTON_UP)) != (v & (1 << BUTTON_UP)))
            {
                Serial.println("\tBUTTON_UP changed");
                if (!(v & (1 << BUTTON_UP)))
                {
                    Serial.println("\t\tBUTTON_UP pressed");
                    bleKeyboard.press(KEY_PAGE_UP);
                }
                else
                {
                    Serial.println("\t\tBUTTON_UPreleased");
                    // bleKeyboard.releaseAll();
                }
            }

            // Faster
            if ((lastValue & (1 << BUTTON_RIGHT)) != (v & (1 << BUTTON_RIGHT)))
            {
                // Serial.println("\tBUTTON_RIGHT changed");
                if (!(v & (1 << BUTTON_RIGHT)))
                {
                    Serial.println("\t\tBUTTON_RIGHT pressed");
                    currentSpeed = 4;
                    bleKeyboard.print("4");
                }
            }

            // Slow
            if ((lastValue & (1 << BUTTON_LEFT)) != (v & (1 << BUTTON_LEFT)))
            {
                if (!(v & (1 << BUTTON_LEFT)))
                {
                    if (currentSpeed == 3)
                        currentSpeed = 2;
                    else
                        currentSpeed = 3;
                    Serial.println("\t\tBUTTON_LEFT pressed");
                    bleKeyboard.print(String(currentSpeed));
                }
            }

            lastValue = v;
        }
        delay(100);
        xSemaphoreGive(i2cSem);
    }

    vTaskDelete(NULL);
}

enum JoyPosition
{
    Center = 0,
    Up = 1,
    Down = 2,
    Left = 3,
    Right = 4
};

void analogStickTask(void *)
{
    Serial.println(F("analogStickTask() begin"));

    bleKeyboard.releaseAll();

    int16_t x_ctr, y_ctr;
    xSemaphoreTake(i2cSem, portMAX_DELAY);
    x_ctr = ss.analogRead(STICK_H);
    y_ctr = ss.analogRead(STICK_V);
    xSemaphoreGive(i2cSem);

    bool wasLeft = false;
    bool wasRight = false;
    bool wasUp = false;
    bool wasDown = false;
    bool wasCenter = true;
    JoyPosition position = Center;
    bleKeyboard.releaseAll();

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
            x = new_x < 0 ? 0 : new_x > 1024 ? 1024
                                             : new_x;
            y = new_y < 0 ? 0 : new_y > 1024 ? 1024
                                             : new_y;

            bool isLeft = x < STICK_CENTER_POINT - D_THRESHOLD;
            bool isRight = x > STICK_CENTER_POINT + D_THRESHOLD;
            bool isUp = y < STICK_CENTER_POINT - D_THRESHOLD;
            bool isDown = y > STICK_CENTER_POINT + D_THRESHOLD;

            // Log the position of the analog stick in various ways for different kinds of application
            /*
            Serial.printf("Analog stick position change!\n\tIs centered: %s\n\tPosition: X=%d Y=%d\n",
                          isCentered ? "true" : "false",
                          x, y);
                          */

            JoyPosition newPosition = Center;
            if (isCentered)
                newPosition = Center;
            if (isLeft)
                newPosition = Left;
            else if (isRight)
                newPosition = Right;
            else if (isUp)
                newPosition = Up;
            else if (isDown)
                newPosition = Down;

            if (position != newPosition)
            {
                bleKeyboard.releaseAll();
                if (isLeft)
                {
                    bleKeyboard.press(KEY_LEFT_ARROW);
                    // Serial.println("Left");
                }
                else if (isRight)
                {
                    bleKeyboard.press(KEY_RIGHT_ARROW);
                    // Serial.println("Right");
                }
                else if (isUp)
                {
                    bleKeyboard.press(KEY_UP_ARROW);
                    // Serial.println("Up");
                }
                else if (isDown)
                {
                    bleKeyboard.press(KEY_DOWN_ARROW);
                    // Serial.println("Down");
                }
                else
                {
                    Serial.println("Center");
                }
                position = newPosition;

                UpdateBatteryPercent();
            }
        }

        delay(100);
    }

    vTaskDelete(NULL);
}

void setup()
{
    Serial.begin(115200);

    bleKeyboard.begin();

    if (!ss.begin(0x49))
    {
        Serial.println("ERROR! seesaw not found");
        while (1)
            delay(1);
    }

    ss.pinModeBulk(button_mask, INPUT_PULLUP);
    ss.setGPIOInterrupts(button_mask, 1);
    pinMode(IRQ_PIN, INPUT);

    xSemaphoreGive(i2cSem); // Initialize the semaphore to 0 (default state is uninitialized which will cause a crash)
    buttonPressQueue = xQueueCreate(10, sizeof(uint8_t));

    // Task for listening to button presses
    xTaskCreate(buttonPressConsumer, "ButtonPressConsumer", 10000, // Stack size -- too low and ESP will eventually crash within the task
                NULL, 1, NULL);

    // Task for reading the analog stick value
    xTaskCreate(analogStickTask, "AnalogStickTask", 10000, // Stack size -- too low and ESP will eventually crash within the task
                NULL, 2, NULL);

    // Respond to changes from button presses
    attachInterrupt(IRQ_PIN, onButtonPress, FALLING);
}

void loop()
{
    delay(1000);
}

void UpdateBatteryPercent()
{
    int rawValue = analogRead(A13);

    // Reference voltage on ESP32 is 1.1V
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html#adc-calibration
    // See also: https://bit.ly/2zFzfMT
    float voltageLevel = (rawValue / 4095.0) * 2 * 1.1 * 3.3; // calculate voltage level
    float batteryFraction = voltageLevel / MAX_BATTERY_VOLTAGE;

    Serial.println((String) "Raw:" + rawValue + " Voltage:" + voltageLevel + "V Percent: " + (batteryFraction * 100) + "%");
    bleKeyboard.setBatteryLevel(batteryFraction);
}