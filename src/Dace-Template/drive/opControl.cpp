#include "Dace-Template/drive/opControl.hpp"
#include <algorithm> 
#include <cmath>

extern dace::Drive chassis;

namespace {

    //Deadzone
    inline double dz(double v, int deadzone){
        return (std::abs(v) < deadzone) ? 0.0 : v;
    }

    inline double clamp127(double v){
        return std::clamp<double>(v, -127.0, 127.0);
    }

} //anon

namespace dace{

    void arcade_drive(int deadzone){
        
        //Left stick Y fwd/rev, right stick X lateral
        double forward = dz(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), deadzone);
        double turn = dz(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), deadzone);

        double left = clamp127(forward + turn);
        double right = clamp127(forward - turn);

        chassis.setTank(left, right);
    }

    void tank_drive(int deadzone){
        double left = dz(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), deadzone);
        double right = dz(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y), deadzone);

        chassis.setTank(clamp127(left), clamp127(right));
    }

    void single_stick_drive(Stick which, int deadzone){
        //one stick controls both fwd/rev (Y) and lateral (X)
        const bool useRight = (which == Stick::Right);

        double y = dz(master.get_analog(useRight
                      ? pros::E_CONTROLLER_ANALOG_RIGHT_Y
                      : pros::E_CONTROLLER_ANALOG_LEFT_Y), deadzone);
        
        double x = dz(master.get_analog(useRight
                      ? pros::E_CONTROLLER_ANALOG_RIGHT_X
                      : pros::E_CONTROLLER_ANALOG_LEFT_X), deadzone);
        
        double left = clamp127(y + x);
        double right = clamp127(y - x);

        chassis.setTank(left, right);
    }
} //namespace dace