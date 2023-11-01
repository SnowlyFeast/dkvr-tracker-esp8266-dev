#include "LED.h"

namespace DKVR
{
    namespace Service
    {

        LEDService::LEDService() : cycleBegin(millis()), ledOn(false)
        {
            SetMode(Manual);
            pinMode(LED_PIN_NUMB, OUTPUT);
        }
        

        void LEDService::SetLED(bool on)
        {
            if (on && !ledOn)
            {
                digitalWrite(LED_PIN_NUMB, LOW);
                ledOn = true;
            }
            else if (!on && ledOn)
            {
                digitalWrite(LED_PIN_NUMB, HIGH);
                ledOn = false;
            }
        }

        void LEDService::SetMode(LEDMode mode)
        {
            if (this->mode == mode)
                return;

            switch (mode)
            {
            default:
            case Manual:
                break;

            case Slowest:
                cyclePeriod = LED_SLOWEST_CYCLE_PERIOD;
                activeTime = LED_SLOWEST_ACTIVE_TIME;
                repeat = LED_SLOWEST_REPEAT;
                delay = LED_SLOWEST_DELAY;
                break;

            case Normal:
                cyclePeriod = LED_NORMAL_CYCLE_PERIOD;
                activeTime = LED_NORMAL_ACTIVE_TIME;
                repeat = LED_NORMAL_REPEAT;
                delay = LED_NORMAL_DELAY;
                break;

            case Fastest:
                cyclePeriod = LED_FASTEST_CYCLE_PERIOD;
                activeTime = LED_FASTEST_ACTIVE_TIME;
                repeat = LED_FASTEST_REPEAT;
                delay = LED_FASTEST_DELAY;
                break;

            case DoubleTab:
                cyclePeriod = LED_DOUBLETAB_CYCLE_PERIOD;
                activeTime = LED_DOUBLETAB_ACTIVE_TIME;
                repeat = LED_DOUBLETAB_REPEAT;
                delay = LED_DOUBLETAB_DELAY;
                break;

            case TripleTab:
                cyclePeriod = LED_TRIPLETAB_CYCLE_PERIOD;
                activeTime = LED_TRIPLETAB_ACTIVE_TIME;
                repeat = LED_TRIPLETAB_REPEAT;
                delay = LED_TRIPLETAB_DELAY;
                break;
            }

            this->mode = mode;
            locate = false;
            cycleBegin = millis();
        }

        void LEDService::Locate()
        {
            if (!locate)
            {
                memory = mode;
                SetMode(DoubleTab);
                locateBegin = millis();
                locate = true;
            }
        }

        void LEDService::Update()
        {
            // manual mode
            if (mode == Manual)
                return;

            // check locate mode ends
            if (locate && millis() > locateBegin + LED_LOCATE_DURATION)
            {
                locate = false;
                SetMode(memory);
            }

            unsigned long elapsed = millis() - cycleBegin;
            for (int i = 0; i < repeat; i++)
            {
                if (elapsed < cyclePeriod)
                {
                    SetLED((elapsed < activeTime));
                    return;
                }
                else
                {
                    elapsed -= cyclePeriod;
                }
            }

            if (elapsed < delay)
            {
                SetLED(false);
                return;
            }
            else
            {
                elapsed -= delay;
                cycleBegin = millis() - elapsed;
                SetLED((elapsed < activeTime));
            }
        }

    } // namespace service

} // namespace dkvr
