#pragma once
#include "main.h"

/*
Slew Header File
*/

namespace dace{

    class Slew{
        
        public:

            Slew(double maxRate);

            void reset(double value = 0);
            double calculate(double input);
            void setRate(double newRate);

        private:

            double maxRate;
            double lastValue;

    };

}//namespace dace