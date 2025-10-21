#include "Dace-Template/drive/slew.hpp"
#include "pros/rtos.hpp"
#include <cmath>

/*
Slew 
*/

namespace dace{
    dace::Slew::Slew(double maxRate)
        : maxRate(maxRate), lastValue(0) {}

    void Slew::reset(double value){
        lastValue = value;
    }

    void Slew::setRate(double newRate){
        maxRate = newRate;
    }

    double Slew::calculate(double input){
        double delta = input - lastValue;

        if(std::abs(delta) > maxRate){
            delta = (delta > 0) ? maxRate : -maxRate;
        }

        lastValue += delta;
        return lastValue;
    }
}//namespace dace