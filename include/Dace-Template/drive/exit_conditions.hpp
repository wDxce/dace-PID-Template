#pragma once

/*
Exit Conditions Header File
*/

namespace dace{

    class ExitConditions {
        public:
            ExitConditions(double angleTolerance = 1.0,
                           double distanceTolerance = 1.0,
                           int timeoutMs = 3000);
            
            void setTarget(double targetHeading);
            void reset();

            bool shouldExit(double currentHeading, double currentDistance, bool pidSettled);

        private:
            double targetTheta;
            double angleTol;
            double distTol;
            int timeout;
            int startTime;
    };

    extern ExitConditions exitConfig;
    void setExitConfig(double angleTol, double distTol, int timeoutMs);
}