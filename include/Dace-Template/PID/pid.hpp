#pragma once

/*
PID Header File
*/

namespace dace {
    class PID {
        public:
            PID(double kP, double kI, double kD, double tolerance = 1.0, int settleTime = 250);

            void setTarget(double newTarget);
            double calculate(double current);
            void reset();

            bool isSettled() const;
            void setHeadingMode(bool enabled);

        private:
            double kP, kI, kD;
            double target;
            double error;
            double lastError;
            double integral;
            double derivative;

            double tolerance;
            int settleTimeMs;
            int settleTimer;

            bool headingMode;
    };

    struct PIDConstants {
        double kP, kI, kD;
    };

    extern PIDConstants drivePIDVals;
    extern PIDConstants turnPIDVals;
    extern PIDConstants swingPIDVals;
    extern PIDConstants curvePIDVals;

    void PID_Values(
        double dP, double dI, double dD,
        double tP, double tI, double tD,
        double sP, double sI, double sD,
        double cP, double cI, double cD
    );
}//namespace dace