#include <Arduino.h>
#include "configuration.h"
#include "driver/mpu/mpu6050.h"
#include "service/ota/ota.h"
#include "tracker/Tracker.h"

#define WIRE_SPEED      400000
#define SERIAL_SPEED    500000

#ifdef DKVR_OVERRIDE_INT_PIN
#define INTERRUPT_PIN   DKVR_OVERRIDE_INT_PIN
#else
#define INTERRUPT_PIN   14
#endif

DKVR::Core::Tracker tracker;
bool error = false;
volatile bool interrupted = false;

void IRAM_ATTR onInterrupt()
{
    interrupted = true;
}

void setup()
{
    Wire.begin();
    Wire.setClock(WIRE_SPEED);
#ifdef DKVR_ENABLE_DEBUG
    Serial.begin(SERIAL_SPEED);
#endif

    if ((error = tracker.Initialize()))
    {
        PRINTLN("IMU error occured but just gonna do the work.");
    }
    else
    {
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), onInterrupt, RISING);
    }
}

void loop()
{
    tracker.Update();

    if (interrupted)
    {
        interrupted = false;
        tracker.OnInterrupt();
    }
}

