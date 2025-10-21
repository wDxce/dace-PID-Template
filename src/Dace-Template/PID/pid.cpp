#include "main.h"

namespace dace {

    PID::PID(double kP, double kI, double kD, double tolerance, int settleTime)
        : kP(kP), kI(kI), kD(kD), tolerance(tolerance), settleTimeMs(settleTime),
          target(0), error(0), lastError(0), integral(0), derivative(0),
          settleTimer(0), headingMode(false) {}
    
    void PID::setTarget(double newTarget) {
        target = newTarget;
        error = 0;
        lastError = 0;
        integral = 0;
        derivative = 0;
        settleTimer = 0;
    }
    
    void PID::reset() {
        error = 0;
        lastError = 0;
        integral = 0;
        derivative = 0;
        settleTimer = 0;
    }
    
    void PID::setHeadingMode(bool enabled) {
        headingMode = enabled;
    }
    
    double PID::calculate(double current) {
        if (headingMode) {
            double diff = fmod(target - current + 540.0, 360.0) - 180.0;
            error = diff;
        } else {
            error = target - current;
        }
    
        integral += error;
        derivative = error - lastError;
        lastError = error;
    
        double output = kP * error + kI * integral + kD * derivative;
    
        if (std::abs(error) < tolerance) {
            settleTimer += 10;
        } else {
            settleTimer = 0;
        }
    
        return output;
    }
    
    bool PID::isSettled() const {
        return settleTimer >= settleTimeMs;
    }
    
    // ===== GLOBAL CONFIGURATION =====
    
    PIDConstants drivePIDVals;
    PIDConstants turnPIDVals;
    PIDConstants swingPIDVals;
    PIDConstants curvePIDVals;
    
    void PID_Values(
        double dP, double dI, double dD,
        double tP, double tI, double tD,
        double sP, double sI, double sD,
        double cP, double cI, double cD
    ) {
        drivePIDVals = {dP, dI, dD};
        turnPIDVals  = {tP,  tI,  tD};
        swingPIDVals = {sP,  sI,  sD};
        curvePIDVals = {cP,  cI,  cD};
    }

} // namespace dace
