#ifndef DKVR_CORE_TOOLS
#define DKVR_CORE_TOOLS

#include <Arduino.h>

namespace DKVR
{
    namespace Utils
    {

        class SimpleTimer
        {
        public:
            void Set(unsigned long ms) { tick = millis() + ms; }
            bool IsOver() { return millis() > tick; }

        private:
            unsigned long tick = 0;
        };

    } // namespace utils
    
} // namespace dkvr


#endif // DKVR_CORE_TOOLS
