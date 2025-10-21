#include "Dace-Template/drive/exit_conditions.hpp"
#include "pros/rtos.hpp"
#include "cmath"

/*
Exit Conditions
*/

namespace dace{

    ExitConditions exitConfig;

    void setExitConfig(double angleTol, double distTol, int timeoutMs){
        exitConfig = ExitConditions(angleTol, distTol, timeoutMs);
    }

    ExitConditions::ExitConditions(double angleTolerance,
                                   double distanceTolerance,
                                   int timeoutMs)
        : angleTol(angleTolerance), distTol(distanceTolerance),
          timeout(timeoutMs), targetTheta(0), startTime(0) {}
    
    void ExitConditions::setTarget(double heading) {
        targetTheta = heading;
        startTime = pros::millis();
    }

    bool ExitConditions::shouldExit(double currentHeading, double currentDistance, bool pidSettled){
        double angleError = std::fmod((targetTheta - currentHeading + 540.0), 360.0) - 180.0;

        bool distMet = std::abs(currentDistance) <= distTol;
        bool angleMet = std::abs(angleError) <= angleTol;
        bool timeoutMet = (pros::millis() - startTime) >= timeout;

        return pidSettled || (angleMet && distMet) || timeoutMet;
    }
}