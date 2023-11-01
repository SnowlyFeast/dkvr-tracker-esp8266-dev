#ifndef DKVR_SERVICE_LED
#define DKVR_SERVICE_LED

#include "configuration.h"
#include <Arduino.h>

#ifdef DKVR_OVERRIDE_LED_PIN
#define LED_PIN_NUMB    DKVR_OVERRIDE_LED_PIN
#else
#define LED_PIN_NUMB    2
#endif

#define LED_SLOWEST_CYCLE_PERIOD    3000
#define LED_SLOWEST_ACTIVE_TIME     1500
#define LED_SLOWEST_REPEAT          1
#define LED_SLOWEST_DELAY           0

#define LED_NORMAL_CYCLE_PERIOD     2000
#define LED_NORMAL_ACTIVE_TIME      1000
#define LED_NORMAL_REPEAT           1
#define LED_NORMAL_DELAY            0

#define LED_FASTEST_CYCLE_PERIOD    500
#define LED_FASTEST_ACTIVE_TIME     250
#define LED_FASTEST_REPEAT          1
#define LED_FASTEST_DELAY           0

#define LED_DOUBLETAB_CYCLE_PERIOD  250
#define LED_DOUBLETAB_ACTIVE_TIME   125
#define LED_DOUBLETAB_REPEAT        2
#define LED_DOUBLETAB_DELAY         500

#define LED_TRIPLETAB_CYCLE_PERIOD  300
#define LED_TRIPLETAB_ACTIVE_TIME   180
#define LED_TRIPLETAB_REPEAT        3
#define LED_TRIPLETAB_DELAY         400

#define LED_LOCATE_DURATION         2900UL

namespace DKVR
{
    namespace Service
    {
        enum LEDMode
        {
            Manual,
            Slowest,
            Normal,
            Fastest,
            DoubleTab,
            TripleTab
        };

        class LEDService
        {
        public:
            LEDService();
            void SetLED(bool on);
            void SetMode(LEDMode mode);
            void Locate();
            void Update();

        private:
            unsigned long cycleBegin;
            unsigned short cyclePeriod;
            unsigned short activeTime;
            unsigned char repeat;
            unsigned short delay;
            bool ledOn;
            LEDMode mode;

            bool locate;
            unsigned long locateBegin;
            LEDMode memory;
        };

    } // namespace service

} // namespace dkvr

#endif // DKVR_SERVICE_LED
